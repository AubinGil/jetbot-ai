#!/usr/bin/env python3
"""
VILA-ROS2 Bridge with RPLidar Safety Layer - ENHANCED VERSION
- Vision: VILA analyzes camera for navigation
- Safety: RPLidar overrides if obstacles too close (no map needed)
- Intelligence: Detects dead-ends and performs smart turn-arounds

Parameters (ROS2):
- vila_url (string): VILA API base URL (default: http://localhost:5000)
- safety_distance (double): emergency stop if closer than this (m)
- warning_distance (double): slow down if closer than this (m)
- front_cone_deg (double): half-angle width of front cone around 0° (default 60)
- min_valid_range (double): ignore ranges below this (m) to avoid self-echo (default 0.2)
- exclude_sectors_deg (string): comma-separated degree ranges to ignore, e.g. "-90:-60,60:90"
- block_frames_threshold (integer): consecutive blocked frames to trigger stop (default 2)
- clear_frames_threshold (integer): consecutive clear frames to clear stop (default 2)
- stuck_threshold (integer): consecutive stuck cycles before initiating turnaround (default 4)
- turnaround_duration (double): how long to turn during turnaround maneuver (seconds, default 2.5)
- side_cone_deg (double): angle range for left/right side scanning (default 45.0)
- turnaround_angle_deg (double): if > 0, compute turnaround_duration from this angle and angular_speed (default 0 = disabled)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64MultiArray
from cv_bridge import CvBridge
import cv2
import base64
import requests
from requests.exceptions import RequestException
import time
import numpy as np
from enum import Enum

class NavState(Enum):
    """Navigation state machine"""
    NORMAL = 1          # Normal VILA-driven navigation
    STUCK_DETECTED = 2  # Detected we're stuck
    TURNING_AROUND = 3  # Executing turnaround maneuver
    RECOVERING = 4      # Just finished turnaround, recovering
    MUST_TURN = 5       # Must turn before allowing forward (after backing from obstacle)
    SCANNING = 6        # Performing 360° environmental scan

class VILARobotBridgeWithLidar(Node):
    @staticmethod
    def _param_to_bool(value):
        """Normalize parameter values into a bool (handles str/int/bool)."""
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        if value is None:
            return False
        try:
            return bool(int(value))
        except Exception:
            return False

    def __init__(self):
        super().__init__('vila_robot_bridge_lidar_enhanced')

        # Parameters - existing
        self.declare_parameter('vila_url', 'http://localhost:5000')
        self.declare_parameter('ollama_url', 'http://localhost:11434')
        self.declare_parameter('ollama_model', 'minicpm-v:latest')
        self.declare_parameter('safety_distance', 0.5)
        self.declare_parameter('warning_distance', 0.8)
        self.declare_parameter('front_cone_deg', 60.0)
        self.declare_parameter('min_valid_range', 0.2)
        self.declare_parameter('exclude_sectors_deg', '')
        self.declare_parameter('block_frames_threshold', 2)
        self.declare_parameter('clear_frames_threshold', 2)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('warning_speed_scale', 0.5)
        self.declare_parameter('backup_speed_scale', 0.5)  # Scale factor for gentle backups
        self.declare_parameter('smoothing_enabled', True)
        self.declare_parameter('smooth_linear_delta', 0.02)
        self.declare_parameter('smooth_angular_delta', 0.08)
        self.declare_parameter('smooth_timer_period', 0.1)
        self.declare_parameter('adaptive_smoothing', False)  # Enable adaptive transformer smoothing

        # Parameters - new for stuck detection and recovery
        self.declare_parameter('stuck_threshold', 4)  # cycles before declaring stuck
        self.declare_parameter('turnaround_duration', 2.5)  # seconds to turn around
        self.declare_parameter('side_cone_deg', 45.0)  # angle range for side scanning
        self.declare_parameter('turnaround_angle_deg', 0.0)  # if > 0, compute duration from angle

        # Parameters - periodic scanning and tight space detection
        self.declare_parameter('scan_interval_cycles', 10)  # perform 360° scan every N cycles (0 = disabled)
        self.declare_parameter('scan_duration', 4.0)  # seconds for 360° scan
        self.declare_parameter('corridor_width_threshold', 0.6)  # if L and R < this, we're in tight corridor
        self.declare_parameter('tight_space_boost', 0.65)  # scale factor (≤1) to calm navigation inside narrow corridors
        self.declare_parameter('wall_follow_enabled', False)
        self.declare_parameter('wall_follow_target_distance', 0.35)
        self.declare_parameter('wall_follow_activation_distance', 0.75)
        self.declare_parameter('wall_follow_exit_distance', 1.15)
        self.declare_parameter('wall_follow_gain', 0.45)
        self.declare_parameter('wall_follow_max_adjust', 0.30)

        # Parameters - scene narration for entertainment
        self.declare_parameter('narration_enabled', True)  # enable periodic scene narration
        self.declare_parameter('narration_interval', 15.0)  # seconds between narrations
        self.declare_parameter('narration_style', 'enthusiastic')  # style: enthusiastic, dramatic, casual, technical
        self.declare_parameter('narration_language', 'en')  # language: en (English) or fr (French)

        # Get all parameters
        self.vila_url = self.get_parameter('vila_url').value
        self.ollama_url = self.get_parameter('ollama_url').value
        self.ollama_model = self.get_parameter('ollama_model').value
        self.safety_distance = float(self.get_parameter('safety_distance').value)
        self.warning_distance = float(self.get_parameter('warning_distance').value)
        self.front_cone_deg = float(self.get_parameter('front_cone_deg').value)
        self.min_valid_range = float(self.get_parameter('min_valid_range').value)
        self.exclude_sectors_str = str(self.get_parameter('exclude_sectors_deg').value or '')
        self.block_frames_threshold = int(self.get_parameter('block_frames_threshold').value)
        self.clear_frames_threshold = int(self.get_parameter('clear_frames_threshold').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.warning_speed_scale = float(self.get_parameter('warning_speed_scale').value)
        self.backup_speed_scale = float(self.get_parameter('backup_speed_scale').value)
        self.smoothing_enabled = self._param_to_bool(self.get_parameter('smoothing_enabled').value)
        self.smooth_linear_delta = float(self.get_parameter('smooth_linear_delta').value)
        self.smooth_angular_delta = float(self.get_parameter('smooth_angular_delta').value)
        self.smooth_timer_period = float(self.get_parameter('smooth_timer_period').value)
        self.adaptive_smoothing = self._param_to_bool(self.get_parameter('adaptive_smoothing').value)
        self.smoothing_active = self.smoothing_enabled and self.smooth_timer_period > 0.0
        self.stuck_threshold = int(self.get_parameter('stuck_threshold').value)
        self.turnaround_duration = float(self.get_parameter('turnaround_duration').value)
        self.side_cone_deg = float(self.get_parameter('side_cone_deg').value)
        self.turnaround_angle_deg = float(self.get_parameter('turnaround_angle_deg').value)
        self.scan_interval_cycles = int(self.get_parameter('scan_interval_cycles').value)
        self.scan_duration = float(self.get_parameter('scan_duration').value)
        self.corridor_width_threshold = float(self.get_parameter('corridor_width_threshold').value)
        raw_tight_scale = float(self.get_parameter('tight_space_boost').value)
        self.tight_space_turn_scale = float(np.clip(raw_tight_scale, 0.45, 1.0))
        self.wall_follow_enabled = self._param_to_bool(self.get_parameter('wall_follow_enabled').value)
        self.wall_follow_target_distance = float(self.get_parameter('wall_follow_target_distance').value)
        self.wall_follow_activation_distance = float(self.get_parameter('wall_follow_activation_distance').value)
        self.wall_follow_exit_distance = float(self.get_parameter('wall_follow_exit_distance').value)
        self.wall_follow_gain = float(self.get_parameter('wall_follow_gain').value)
        self.wall_follow_max_adjust = float(self.get_parameter('wall_follow_max_adjust').value)
        self.narration_enabled = bool(self.get_parameter('narration_enabled').value)
        self.narration_interval = float(self.get_parameter('narration_interval').value)
        self.narration_style = str(self.get_parameter('narration_style').value)
        self.narration_language = str(self.get_parameter('narration_language').value)

        # Effective speed boost used during turnaround (keep single source of truth)
        # Lowered to avoid overly aggressive spins
        self._turn_boost = 1.0

        # If an angle is specified, compute the duration from angular speed and boost
        try:
            if self.turnaround_angle_deg and self.turnaround_angle_deg > 0.0 and self.angular_speed > 0.0:
                angle_rad = np.deg2rad(self.turnaround_angle_deg)
                effective_w = self.angular_speed * self._turn_boost
                computed = angle_rad / effective_w
                # Only override if computation is valid
                if np.isfinite(computed) and computed > 0:
                    self.turnaround_duration = float(computed)
                    self.get_logger().info(
                        f"Turnaround angle set: {self.turnaround_angle_deg:.1f}°, computed duration: {self.turnaround_duration:.2f}s (w_eff={effective_w:.2f} rad/s)"
                    )
        except Exception:
            # Fallback silently to user-provided duration
            pass

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vlm_text_pub = self.create_publisher(String, '/vila/response_text', 10)
        self.tts_pub = self.create_publisher(String, 'tts/speak', 10)
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Subscribe to adaptive smoothing parameters from transformer
        if self.adaptive_smoothing:
            self.adaptive_params_sub = self.create_subscription(
                Float64MultiArray, '/adaptive_smoothing_params',
                self.adaptive_params_callback, 10)

        self.bridge = CvBridge()
        self.target_twist = Twist()
        self.current_twist = Twist()
        self.latest_image = None
        self.latest_scan = None
        self.processing = False
        self.obstacle_detected = False
        self.min_distance = float('inf')
        self._block_count = 0
        self._clear_count = 0

        # Enhanced state tracking
        self.nav_state = NavState.NORMAL
        self.stuck_counter = 0
        self.last_action = "STOP"
        self.consecutive_stops = 0
        self.consecutive_backwards = 0
        self.turnaround_start_time = None
        self.turnaround_direction = 1  # 1 for left, -1 for right
        self.turnaround_scale = 1.0  # Locked scale value for current turnaround
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.wall_follow_side = None
        self._wall_follow_engaged = False

        # Anti-wall-charging protection
        self.just_backed_up = False
        self.turns_since_backup = 0
        self.required_turns_after_backup = 1  # Must turn at least once before allowing forward
        self.forward_cooldown = 0  # Cycles to wait before allowing forward again

        # Periodic scanning state
        self.cycle_counter = 0  # Count cycles since last scan
        self.scan_start_time = None
        self.scan_samples = []  # Store (angle, front_dist, left_dist, right_dist) during scan
        self.best_scan_direction = 0  # Direction with most clearance from last scan
        self.in_tight_space = False  # Flag for corridor/tight space detection
        self.tight_space_enter_count = 0  # Hysteresis counter for entering tight space
        self.tight_space_exit_count = 0  # Hysteresis counter for exiting tight space

        # Scene narration state
        self.narrating = False  # Flag to prevent overlapping narrations

        # Wait for vision model to be ready (Cosmos Reason or VILA endpoints)
        self.get_logger().info(f"Waiting for vision model at {self.vila_url}...")

        # Try health endpoint first, if it fails, try a simple request
        health_ok = False
        while True:
            try:
                # Try /health endpoint
                resp = requests.get(f'{self.vila_url}/health', timeout=2)
                if resp.status_code == 200:
                    self.get_logger().info("✓ Vision model is ready!")
                    health_ok = True
                    break
            except RequestException:
                pass

            # If /health doesn't exist, try a simple test request
            if not health_ok:
                try:
                    # Test if the server responds at all
                    resp = requests.post(
                        f'{self.vila_url}/generate',
                        json={'prompt': 'test'},
                        timeout=2
                    )
                    if resp.status_code in [200, 400]:  # 400 is ok, means server is up
                        self.get_logger().info("✓ Vision model is ready!")
                        break
                except RequestException:
                    pass

            time.sleep(1)

        # Create timer for periodic analysis (every 3 seconds)
        self.timer = self.create_timer(3.0, self.analyze_and_move)
        # Create high-frequency timer for turnaround execution (10Hz)
        self.maneuver_timer = self.create_timer(0.1, self.execute_maneuver)
        # Create timer for scene narration (if enabled)
        if self.narration_enabled and self.narration_interval > 0:
            self.narration_timer = self.create_timer(self.narration_interval, self.narrate_scene)

        if self.smoothing_active:
            self.smooth_timer = self.create_timer(self.smooth_timer_period, self._publish_smoothed_twist)
        else:
            self.smooth_timer = None

        self.get_logger().info("VILA Robot Bridge (Enhanced with Smart Navigation) ready!")
        self.get_logger().info(f"Safety: {self.safety_distance}m | Warning: {self.warning_distance}m | Front cone: ±{self.front_cone_deg}°")
        self.get_logger().info(f"Speeds: linear={self.linear_speed:.2f} m/s angular={self.angular_speed:.2f} rad/s")
        if self.smoothing_active:
            self.get_logger().info(
                f"Movement smoothing: Δlinear={self.smooth_linear_delta:.3f} m/s, "
                f"Δangular={self.smooth_angular_delta:.3f} rad/s @ {self.smooth_timer_period:.2f}s"
            )
        else:
            self.get_logger().info("Movement smoothing: disabled")
        self.get_logger().info(f"Stuck threshold: {self.stuck_threshold} cycles | Turnaround duration: {self.turnaround_duration:.2f}s")
        if self.scan_interval_cycles > 0:
            self.get_logger().info(f"Periodic scanning: every {self.scan_interval_cycles} cycles ({self.scan_duration:.1f}s per scan)")
        self.get_logger().info(f"Tight space detection: corridor width < {self.corridor_width_threshold}m")
        if self.narration_enabled:
            self.get_logger().info(f"🎤 Scene narration: enabled (every {self.narration_interval:.0f}s, style={self.narration_style}, language={self.narration_language})")

    def adaptive_params_callback(self, msg):
        """Update smoothing parameters from adaptive transformer"""
        if len(msg.data) >= 2:
            new_linear_delta = float(msg.data[0])
            new_angular_delta = float(msg.data[1])

            # Only update if values are reasonable
            if 0.005 <= new_linear_delta <= 0.1 and 0.02 <= new_angular_delta <= 0.2:
                self.smooth_linear_delta = new_linear_delta
                self.smooth_angular_delta = new_angular_delta
                self.get_logger().info(
                    f"🔄 Updated smoothing: Δlinear={new_linear_delta:.4f} Δangular={new_angular_delta:.4f}",
                    throttle_duration_sec=3.0
                )

    def image_callback(self, msg):
        """Store latest camera image"""
        if not self.processing:
            self.latest_image = msg

    def _parse_exclusions(self):
        sectors = []
        s = self.exclude_sectors_str.strip()
        if not s:
            return sectors
        for part in s.split(','):
            try:
                a, b = part.split(':')
                sectors.append((float(a), float(b)))
            except Exception:
                continue
        return sectors

    def scan_callback(self, msg):
        """Process lidar scan for obstacle detection AND side scanning"""
        self.latest_scan = msg

        ranges = np.array(msg.ranges, dtype=float)
        n = len(ranges)
        if n == 0:
            self.min_distance = float('inf')
            self.left_distance = float('inf')
            self.right_distance = float('inf')
            return

        # Build angles in degrees relative to forward (0 deg)
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        angles_deg = np.degrees(angles)

        # Valid ranges
        valid = np.isfinite(ranges)
        valid &= ranges >= self.min_valid_range
        if msg.range_max and np.isfinite(msg.range_max):
            valid &= ranges <= float(msg.range_max)

        # Front cone mask
        cone = np.abs(angles_deg) <= self.front_cone_deg

        # Left side cone: angles between 45° and 135° (or configurable)
        left_cone = (angles_deg >= self.side_cone_deg) & (angles_deg <= (90.0 + self.side_cone_deg))

        # Right side cone: angles between -135° and -45°
        right_cone = (angles_deg <= -self.side_cone_deg) & (angles_deg >= -(90.0 + self.side_cone_deg))

        # Exclusion mask
        excl = np.zeros(n, dtype=bool)
        for a, b in self._parse_exclusions():
            lo, hi = sorted((a, b))
            excl |= (angles_deg >= lo) & (angles_deg <= hi)

        # Front distance
        front_mask = valid & cone & (~excl)
        front_candidates = ranges[front_mask]
        if front_candidates.size > 0:
            self.min_distance = float(np.min(front_candidates))
        else:
            self.min_distance = float('inf')

        # Left side distance
        left_mask = valid & left_cone & (~excl)
        left_candidates = ranges[left_mask]
        if left_candidates.size > 0:
            self.left_distance = float(np.min(left_candidates))
        else:
            self.left_distance = float('inf')

        # Right side distance
        right_mask = valid & right_cone & (~excl)
        right_candidates = ranges[right_mask]
        if right_candidates.size > 0:
            self.right_distance = float(np.min(right_candidates))
        else:
            self.right_distance = float('inf')

        # Tight space / corridor detection with hysteresis (requires 3 consecutive frames)
        is_currently_tight = (
            self.left_distance < self.corridor_width_threshold and
            self.right_distance < self.corridor_width_threshold
        )

        if is_currently_tight:
            self.tight_space_enter_count += 1
            self.tight_space_exit_count = 0
            # Require 3 consecutive tight frames to enter tight space
            if not self.in_tight_space and self.tight_space_enter_count >= 3:
                self.in_tight_space = True
                self.get_logger().warn(f"🚧 Entering TIGHT SPACE (L:{self.left_distance:.2f}m R:{self.right_distance:.2f}m)")
        else:
            self.tight_space_exit_count += 1
            self.tight_space_enter_count = 0
            # Require 3 consecutive non-tight frames to exit tight space
            if self.in_tight_space and self.tight_space_exit_count >= 3:
                self.in_tight_space = False
                self.get_logger().info(f"✓ Exited tight space")

        # Debounced obstacle detection
        if self.min_distance < self.safety_distance:
            self._block_count += 1
            self._clear_count = 0
            if not self.obstacle_detected and self._block_count >= self.block_frames_threshold:
                self.obstacle_detected = True
                self.get_logger().warn(f"🛑 OBSTACLE DETECTED! Front: {self.min_distance:.2f}m L: {self.left_distance:.2f}m R: {self.right_distance:.2f}m")
                self.publish_stop()
        elif self.min_distance > self.warning_distance:
            self._clear_count += 1
            self._block_count = 0
            if self.obstacle_detected and self._clear_count >= self.clear_frames_threshold:
                self.obstacle_detected = False
                self.get_logger().info(f"✓ Path clear - {self.min_distance:.2f}m")
        else:
            self._block_count = 0
            self._clear_count = 0

    def publish_stop(self):
        """Emergency stop"""
        twist = Twist()
        self._apply_twist_immediately(twist)

    def _apply_twist_immediately(self, twist):
        """Set both current and target twist for immediate execution."""
        self.target_twist.linear.x = twist.linear.x
        self.target_twist.angular.z = twist.angular.z
        self.current_twist.linear.x = twist.linear.x
        self.current_twist.angular.z = twist.angular.z
        self.cmd_vel_pub.publish(twist)

    def _set_target_twist(self, twist):
        """Schedule a new target twist, let smoothing handle the publishing."""
        self.target_twist.linear.x = twist.linear.x
        self.target_twist.angular.z = twist.angular.z
        if self.smoothing_active and self.nav_state == NavState.NORMAL:
            self._publish_smoothed_twist()
        else:
            self.current_twist.linear.x = twist.linear.x
            self.current_twist.angular.z = twist.angular.z
            self.cmd_vel_pub.publish(twist)

    def _publish_smoothed_twist(self):
        """Periodic callback that ramps current twist toward the target."""
        if not self.smoothing_active or self.nav_state != NavState.NORMAL:
            return

        linear_target = self.target_twist.linear.x
        angular_target = self.target_twist.angular.z
        linear_current = self.current_twist.linear.x
        angular_current = self.current_twist.angular.z

        linear_diff = linear_target - linear_current
        if abs(linear_diff) <= self.smooth_linear_delta:
            linear_current = linear_target
        else:
            linear_current += self.smooth_linear_delta * np.sign(linear_diff)

        angular_diff = angular_target - angular_current
        if abs(angular_diff) <= self.smooth_angular_delta:
            angular_current = angular_target
        else:
            angular_current += self.smooth_angular_delta * np.sign(angular_diff)

        self.current_twist.linear.x = linear_current
        self.current_twist.angular.z = angular_current
        self.cmd_vel_pub.publish(self.current_twist)

    def _wall_follow_log_state(self):
        """Log when wall-following engagement changes."""
        engaged = self.wall_follow_side is not None
        if engaged == self._wall_follow_engaged:
            return
        self._wall_follow_engaged = engaged
        if engaged:
            self.get_logger().info(
                f"📐 Wall following engaged on {self.wall_follow_side.upper()} side "
                f"(target {self.wall_follow_target_distance:.2f}m)"
            )
        else:
            self.get_logger().info("📐 Wall following disengaged")

    def _select_wall_follow_side(self):
        """Choose which wall (left/right) to shadow based on lidar distances."""
        if not self.wall_follow_enabled:
            if self.wall_follow_side is not None:
                self.wall_follow_side = None
                self._wall_follow_log_state()
            return None

        side = self.wall_follow_side
        if side:
            dist = self.left_distance if side == 'left' else self.right_distance
            if not np.isfinite(dist) or dist > self.wall_follow_exit_distance:
                side = None

        if side is None:
            left_ready = self.left_distance < self.wall_follow_activation_distance
            right_ready = self.right_distance < self.wall_follow_activation_distance
            if left_ready and (not right_ready or self.left_distance <= self.right_distance):
                side = 'left'
            elif right_ready and (not left_ready or self.right_distance < self.left_distance):
                side = 'right'

        if side != self.wall_follow_side:
            self.wall_follow_side = side
            self._wall_follow_log_state()

        return side

    def _wall_follow_adjustment(self):
        """Compute angular correction to stay at the target wall distance."""
        if not self.wall_follow_enabled or self.nav_state != NavState.NORMAL:
            return 0.0

        side = self._select_wall_follow_side()
        if not side:
            return 0.0

        dist = self.left_distance if side == 'left' else self.right_distance
        if not np.isfinite(dist) or dist <= 0.0:
            return 0.0

        error = self.wall_follow_target_distance - dist
        correction = error * self.wall_follow_gain
        if side == 'left':
            correction = -correction

        correction = float(np.clip(correction, -self.wall_follow_max_adjust, self.wall_follow_max_adjust))
        return correction

    def ask_ollama(self, image_b64, prompt):
        """Fallback to local ollama model (minicpm-v)"""
        try:
            self.get_logger().info(f"Trying ollama fallback ({self.ollama_model})...")
            resp = requests.post(
                f'{self.ollama_url}/api/generate',
                json={
                    'model': self.ollama_model,
                    'prompt': prompt,
                    'images': [image_b64],
                    'stream': False
                },
                timeout=30
            )

            if resp.status_code == 200:
                result = resp.json()
                text = result.get('response', '')
                self.get_logger().info(f"Ollama response: {text[:100]}...")
                return text
            else:
                self.get_logger().error(f"Ollama error: {resp.status_code}")
                return None
        except Exception as e:
            self.get_logger().error(f"Ollama request failed: {e}")
            return None

    def check_if_stuck(self, action):
        """Determine if the robot is stuck in a dead-end"""
        # Count consecutive stops or backwards moves
        if action == "STOP" or self.obstacle_detected:
            self.consecutive_stops += 1
            self.consecutive_backwards = 0
        elif "BACKWARD" in action:
            self.consecutive_backwards += 1
            self.consecutive_stops = 0
        else:
            # Making progress, reset counters
            self.consecutive_stops = 0
            self.consecutive_backwards = 0
            self.stuck_counter = 0
            return False

        # In tight spaces, be more aggressive about detecting dead ends
        threshold = self.stuck_threshold
        if self.in_tight_space:
            threshold = max(2, self.stuck_threshold // 2)  # Half threshold in tight spaces
            if self.consecutive_stops >= 1 and self.min_distance < self.safety_distance:
                # Immediate dead-end in tight space with front blocked
                self.get_logger().warn(f"🚧 Dead-end in tight space detected immediately!")
                return True

        # Increment stuck counter if we're repeatedly stopping or going backward
        if self.consecutive_stops >= 2 or self.consecutive_backwards >= 2:
            self.stuck_counter += 1
        else:
            self.stuck_counter = max(0, self.stuck_counter - 1)

        return self.stuck_counter >= threshold

    def choose_turnaround_direction(self):
        """Choose which way to turn based on side distances"""
        # Turn toward the side with more clearance
        if self.left_distance > self.right_distance:
            self.turnaround_direction = 1  # Turn left (positive angular.z)
            self.get_logger().info(f"🔄 Choosing LEFT turnaround (L: {self.left_distance:.2f}m > R: {self.right_distance:.2f}m)")
        else:
            self.turnaround_direction = -1  # Turn right (negative angular.z)
            self.get_logger().info(f"🔄 Choosing RIGHT turnaround (R: {self.right_distance:.2f}m > L: {self.left_distance:.2f}m)")

    def _analyze_scan_results(self):
        """Analyze 360° scan data to find the best direction to move"""
        if not self.scan_samples:
            self.best_scan_direction = 0
            return

        # Find the direction with the most clearance
        best_score = -1
        best_angle = 0

        for sample in self.scan_samples:
            # Score based on front distance (most important) + side clearances
            score = sample['front'] * 2.0 + sample['left'] * 0.5 + sample['right'] * 0.5
            if score > best_score:
                best_score = score
                best_angle = sample['angle']

        self.best_scan_direction = best_angle
        self.get_logger().info(f"   Analyzed {len(self.scan_samples)} samples, best clearance at {best_angle:.0f}°")

    def narrate_scene(self):
        """Periodically describe what the robot sees for entertainment"""
        if self.narrating or self.latest_image is None:
            return

        self.narrating = True

        try:
            # Convert ROS image to base64
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
            _, buffer = cv2.imencode('.jpg', cv_image)
            image_b64 = base64.b64encode(buffer).decode('utf-8')

            # Create entertaining narration prompt based on style and language
            if self.narration_language == 'fr':
                # French prompts
                style_prompts = {
                    'enthusiastic': "Tu es un robot enthousiaste qui explore le monde ! Décris ce que tu vois en 1-2 phrases amusantes et énergiques. Sois excité par les choses ordinaires !",
                    'dramatic': "Tu es un narrateur dramatique. Décris ce que tu vois en 1-2 phrases théâtrales et exagérées, comme si tu narrais un film d'aventure !",
                    'casual': "Tu es un robot sympathique qui discute tranquillement. Décris ce que tu vois en 1-2 phrases détendues et conversationnelles.",
                    'technical': "Tu es un robot technique. Décris ce que tu vois en 1-2 phrases en te concentrant sur les objets, les couleurs et la disposition spatiale.",
                    'curious': "Tu es un robot curieux qui découvre le monde. Décris ce que tu vois en 1-2 phrases, en posant des questions ou en exprimant ton émerveillement.",
                }
            else:
                # English prompts (default)
                style_prompts = {
                    'enthusiastic': "You are an enthusiastic robot exploring the world! Describe what you see in 1-2 fun, energetic sentences. Be excited about ordinary things!",
                    'dramatic': "You are a dramatic narrator. Describe what you see in 1-2 theatrical, over-the-top sentences, like you're narrating an adventure movie!",
                    'casual': "You're a friendly robot casually chatting. Describe what you see in 1-2 relaxed, conversational sentences.",
                    'technical': "You are a technical robot. Describe what you see in 1-2 sentences focusing on objects, colors, and spatial layout.",
                    'curious': "You are a curious robot discovering the world. Describe what you see in 1-2 sentences, asking questions or expressing wonder.",
                }

            prompt = style_prompts.get(self.narration_style, style_prompts.get('enthusiastic', style_prompts['casual']))

            # Add context about robot state for more interesting narration
            context = ""
            if self.narration_language == 'fr':
                if self.in_tight_space:
                    context = " Tu es dans un couloir étroit."
                if self.obstacle_detected:
                    context += " Quelque chose bloque ton chemin."
                full_prompt = f"{prompt}{context} Reste bref et divertissant !"
            else:
                if self.in_tight_space:
                    context = " You are in a narrow corridor."
                if self.obstacle_detected:
                    context += " Something is blocking your path."
                full_prompt = f"{prompt}{context} Keep it short and entertaining!"

            self.get_logger().info(f"🎤 Generating scene narration...")

            # Try Cosmos Reason format first, fallback to VILA, then ollama
            narration = None
            narration_failed = False

            try:
                resp = requests.post(
                    f'{self.vila_url}/generate',
                    json={'image': image_b64, 'prompt': full_prompt},
                    timeout=15
                )

                if resp.status_code == 200:
                    result = resp.json()
                    narration = result.get('response', '')
                elif resp.status_code == 404:
                    # Fallback to VILA endpoints
                    resp = requests.post(
                        f'{self.vila_url}/describe',
                        json={'image': image_b64, 'prompt': full_prompt},
                        timeout=15
                    )

                    if resp.status_code == 404:
                        # Final fallback to analyze endpoint
                        resp = requests.post(
                            f'{self.vila_url}/analyze',
                            json={'image': image_b64, 'custom_prompt': full_prompt},
                            timeout=15
                        )

                    if resp.status_code == 200:
                        result = resp.json()
                        narration = result.get('response', '') or result.get('description', '')
                    else:
                        self.get_logger().warn(f"Narration server error: {resp.status_code}, trying ollama...")
                        narration_failed = True
                else:
                    self.get_logger().warn(f"Narration server error: {resp.status_code}, trying ollama...")
                    narration_failed = True
            except Exception as e:
                self.get_logger().warn(f'Narration server unreachable: {e}, trying ollama...')
                narration_failed = True

            # Fallback to ollama if remote server failed
            if narration_failed:
                narration = self.ask_ollama(image_b64, full_prompt)

            # Publish narration if we got one
            if narration and narration.strip():
                self.get_logger().info(f"🎤 Narration: {narration}")
                msg = String(data=narration)
                self.vlm_text_pub.publish(msg)
                self.tts_pub.publish(msg)
            else:
                self.get_logger().warn("Empty or failed narration response")

        except Exception as e:
            self.get_logger().error(f'Narration error: {e}')
        finally:
            self.narrating = False

    def execute_maneuver(self):
        """High-frequency timer to execute timed maneuvers like turnarounds and scans"""
        if self.nav_state == NavState.TURNING_AROUND:
            if self.turnaround_start_time is None:
                # Start the turnaround - lock the boost value at start to prevent jitter
                self.turnaround_start_time = time.time()
                self.turnaround_scale = self.tight_space_turn_scale if self.in_tight_space else self._turn_boost
                self.get_logger().info(
                    f"🔄 Starting turnaround for {self.turnaround_duration}s (scale={self.turnaround_scale:.2f})..."
                )

            elapsed = time.time() - self.turnaround_start_time

            if elapsed < self.turnaround_duration:
                # Still turning - use locked boost value to prevent jitter
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = self.turnaround_direction * self.angular_speed * self.turnaround_scale
                self.cmd_vel_pub.publish(twist)
            else:
                # Turnaround complete
                self.get_logger().info("✓ Turnaround complete! Resuming normal navigation...")
                self.publish_stop()
                self.nav_state = NavState.RECOVERING
                self.turnaround_start_time = None
                self.stuck_counter = 0
                self.consecutive_stops = 0
                self.consecutive_backwards = 0
                self.turnaround_scale = 1.0

        elif self.nav_state == NavState.SCANNING:
            if self.scan_start_time is None:
                # Start the scan
                self.scan_start_time = time.time()
                self.scan_samples = []
                self.get_logger().info(f"🔍 Starting 360° environmental scan ({self.scan_duration}s)...")

            elapsed = time.time() - self.scan_start_time

            if elapsed < self.scan_duration:
                # Still scanning - rotate slowly and collect data
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed * 0.8  # Slower rotation for better scanning
                self.cmd_vel_pub.publish(twist)

                # Collect sample every 0.1s
                angle_rotated = elapsed * twist.angular.z
                self.scan_samples.append({
                    'angle': np.degrees(angle_rotated),
                    'front': self.min_distance,
                    'left': self.left_distance,
                    'right': self.right_distance,
                    'timestamp': elapsed
                })
            else:
                # Scan complete - analyze results
                self.publish_stop()
                self._analyze_scan_results()
                self.get_logger().info(f"✓ Scan complete! Best direction: {self.best_scan_direction:.0f}°")
                self.nav_state = NavState.RECOVERING
                self.scan_start_time = None
                self.cycle_counter = 0  # Reset cycle counter

    def analyze_and_move(self):
        """Send image to VILA and execute movement (with enhanced stuck detection)"""
        # Increment cycle counter for periodic scanning
        if self.nav_state == NavState.NORMAL:
            self.cycle_counter += 1

        # Check if it's time for a periodic scan (only in NORMAL state, not stuck/recovering)
        if (self.scan_interval_cycles > 0 and
            self.cycle_counter >= self.scan_interval_cycles and
            self.nav_state == NavState.NORMAL and
            not self.obstacle_detected):
            self.get_logger().info(f"⏰ Time for periodic scan (cycle {self.cycle_counter})")
            self.nav_state = NavState.SCANNING
            return

        # Decrement forward cooldown
        if self.forward_cooldown > 0:
            self.forward_cooldown -= 1

        # Handle state machine
        if self.nav_state == NavState.TURNING_AROUND or self.nav_state == NavState.SCANNING:
            # Maneuver is being handled by execute_maneuver timer
            return

        if self.nav_state == NavState.RECOVERING:
            # Give one cycle to recover, then back to normal
            self.nav_state = NavState.NORMAL
            self.forward_cooldown = 2  # Wait 2 cycles before allowing forward
            return

        if self.nav_state == NavState.MUST_TURN:
            # Force a turn before allowing forward motion again
            if self.turns_since_backup >= self.required_turns_after_backup:
                self.get_logger().info("✓ Required turn completed, resuming normal navigation")
                self.nav_state = NavState.NORMAL
                self.just_backed_up = False
                self.turns_since_backup = 0
                self.forward_cooldown = 2  # Wait 2 more cycles
                return
            # Otherwise stay in MUST_TURN state and force a turn below

        # If obstacle too close, don't move (but check if stuck)
        if self.obstacle_detected:
            self.get_logger().warn(f"Obstacle detected - safety override (stuck_counter: {self.stuck_counter}/{self.stuck_threshold})")

            # Check if we're stuck
            if self.check_if_stuck("STOP"):
                self.get_logger().warn(f"🚨 DEAD-END DETECTED! Initiating turnaround...")
                self.nav_state = NavState.STUCK_DETECTED
                self.choose_turnaround_direction()
                self.nav_state = NavState.TURNING_AROUND
                return

            self.publish_stop()
            return

        if self.latest_image is None or self.processing:
            return

        self.processing = True

        try:
            # Convert ROS image to base64
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
            _, buffer = cv2.imencode('.jpg', cv_image)
            image_b64 = base64.b64encode(buffer).decode('utf-8')

            # Send to vision model (Cosmos Reason or VILA)
            self.get_logger().info(f"Asking vision model... (F: {self.min_distance:.2f}m L: {self.left_distance:.2f}m R: {self.right_distance:.2f}m)")

            # Create navigation prompt for Cosmos Reason
            nav_prompt = """You are a robot navigation assistant. Based on the camera image, decide the next action.

Available actions: FORWARD, LEFT, RIGHT, BACKWARD, STOP

Respond with ONLY the action word followed by a brief reason.
Example: "FORWARD - path is clear ahead"
"""

            # Try Cosmos Reason format first (/generate endpoint), fallback to VILA /analyze, then ollama
            action = 'STOP'
            raw_text = ''
            backend = 'unknown'
            vision_failed = False

            try:
                resp = requests.post(
                    f'{self.vila_url}/generate',
                    json={'prompt': nav_prompt, 'image': image_b64},
                    timeout=30
                )

                if resp.status_code == 200:
                    result = resp.json()
                    raw_text = result.get('response', '')
                    backend = 'cosmos-reason'
                elif resp.status_code == 404:
                    # Fallback to VILA format if /generate doesn't exist
                    resp = requests.post(
                        f'{self.vila_url}/analyze',
                        json={'image': image_b64},
                        timeout=30
                    )

                    if resp.status_code == 200:
                        result = resp.json()
                        action = result.get('action', 'STOP')
                        raw_text = result.get('response', '')
                        backend = result.get('backend', 'vila')
                    else:
                        self.get_logger().warn(f"Vision server error: {resp.status_code}, trying ollama...")
                        vision_failed = True
                else:
                    self.get_logger().warn(f"Vision server error: {resp.status_code}, trying ollama...")
                    vision_failed = True
            except Exception as e:
                self.get_logger().warn(f"Vision server unreachable: {e}, trying ollama...")
                vision_failed = True

            # Fallback to ollama if remote server failed
            if vision_failed:
                raw_text = self.ask_ollama(image_b64, nav_prompt)
                if raw_text is None:
                    self.get_logger().error("All vision backends failed!")
                    self.publish_stop()
                    self.processing = False
                    return
                backend = 'ollama'

            # Parse action from response text
            if raw_text:
                response_upper = raw_text.upper()
                if 'FORWARD' in response_upper:
                    action = 'FORWARD'
                elif 'LEFT' in response_upper:
                    action = 'LEFT'
                elif 'RIGHT' in response_upper:
                    action = 'RIGHT'
                elif 'BACKWARD' in response_upper:
                    action = 'BACKWARD'

            self.get_logger().info(f"Vision model says: {action} (backend={backend})")

            # Publish raw text for logging (TTS disabled to reduce chatter)
            try:
                if isinstance(raw_text, str) and raw_text.strip():
                    msg = String(data=raw_text)
                    self.vlm_text_pub.publish(msg)
                    # TTS disabled - robot was talking too much!
                    # self.tts_pub.publish(msg)
            except Exception as _:
                pass

            # Check if stuck BEFORE executing action
            if self.check_if_stuck(action):
                self.get_logger().warn(f"🚨 DEAD-END DETECTED! (action={action}, stuck_counter={self.stuck_counter})")
                self.nav_state = NavState.STUCK_DETECTED
                self.choose_turnaround_direction()
                self.nav_state = NavState.TURNING_AROUND
                self.processing = False
                return

            # Execute movement with lidar safety
            twist = Twist()
            speed_scale = 1.0

            if self.min_distance < self.warning_distance:
                speed_scale = max(0.0, min(1.0, self.warning_speed_scale))
                self.get_logger().info(f"⚠️  Slowing down (scale={speed_scale:.2f})")

            # Enhanced turning logic - if front is blocked, prefer sharper turns
            if self.min_distance < self.warning_distance and ("LEFT" in action or "RIGHT" in action):
                # Keep some boost when avoiding obstacles, but gentler minimum
                speed_scale = max(speed_scale, 0.6)
                self.get_logger().info(f"⚡ Boosting turn speed (gentle) to avoid obstacle")

            if self.in_tight_space:
                speed_scale = min(speed_scale, self.tight_space_turn_scale)

            # Override VILA's action if we must turn after backing up
            actual_action = action
            if self.nav_state == NavState.MUST_TURN and "FORWARD" in action:
                # Choose turn direction based on clearance
                if self.left_distance > self.right_distance:
                    actual_action = "LEFT"
                    self.get_logger().info(f"🚫 Blocking FORWARD (must turn first) - forcing LEFT (L:{self.left_distance:.2f}m > R:{self.right_distance:.2f}m)")
                else:
                    actual_action = "RIGHT"
                    self.get_logger().info(f"🚫 Blocking FORWARD (must turn first) - forcing RIGHT (R:{self.right_distance:.2f}m > L:{self.left_distance:.2f}m)")
            elif self.forward_cooldown > 0 and "FORWARD" in action:
                # Still in cooldown, prevent forward
                actual_action = "STOP"
                self.get_logger().info(f"⏸️  Forward on cooldown ({self.forward_cooldown} cycles remaining)")

            if "FORWARD" in actual_action:
                twist.linear.x = self.linear_speed * speed_scale
                twist.angular.z = 0.0
                wall_adjust = self._wall_follow_adjustment()
                if wall_adjust:
                    twist.angular.z += wall_adjust
                self.get_logger().info(f"→ Moving FORWARD (v={twist.linear.x:.2f})")
                # Reset backup tracking on successful forward
                if self.just_backed_up:
                    self.just_backed_up = False
                    self.turns_since_backup = 0
            elif "LEFT" in actual_action:
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed * speed_scale
                self.get_logger().info(f"→ Turning LEFT (w={twist.angular.z:.2f})")
                # Track turns after backing up
                if self.just_backed_up or self.nav_state == NavState.MUST_TURN:
                    self.turns_since_backup += 1
                    self.get_logger().info(f"   (turn {self.turns_since_backup}/{self.required_turns_after_backup} after backup)")
            elif "RIGHT" in actual_action:
                twist.linear.x = 0.0
                twist.angular.z = -self.angular_speed * speed_scale
                self.get_logger().info(f"→ Turning RIGHT (w={twist.angular.z:.2f})")
                # Track turns after backing up
                if self.just_backed_up or self.nav_state == NavState.MUST_TURN:
                    self.turns_since_backup += 1
                    self.get_logger().info(f"   (turn {self.turns_since_backup}/{self.required_turns_after_backup} after backup)")
            elif "BACKWARD" in actual_action:
                # Use gentle backup speed to avoid violent jerks
                twist.linear.x = -self.linear_speed * self.backup_speed_scale
                twist.angular.z = 0.0
                self.get_logger().info(f"→ Moving BACKWARD (v={twist.linear.x:.2f})")
                # Mark that we just backed up - must turn before going forward again
                self.just_backed_up = True
                self.turns_since_backup = 0
                self.nav_state = NavState.MUST_TURN
                self.get_logger().info("⚠️  Backed up from obstacle - will require turn before resuming forward")
            else:  # STOP
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info("→ STOPPING")

            self._set_target_twist(twist)
            self.last_action = actual_action

        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            self.processing = False

def main(args=None):
    rclpy.init(args=args)
    node = VILARobotBridgeWithLidar()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
