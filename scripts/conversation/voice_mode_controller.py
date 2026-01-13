#!/usr/bin/env python3
"""
Voice Command Controller for JetBot Modes
Listens for voice commands and controls robot modes:
- "follow me" -> Start gesture following
- "autopilot" -> Start obstacle avoidance
- "stop" -> Stop current mode
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import signal
import threading

class VoiceModeController(Node):
    def __init__(self):
        super().__init__('voice_mode_controller')

        # Subscribe to speech-to-text output
        self.subscription = self.create_subscription(
            String,
            '/stt/text',
            self.command_callback,
            10)

        # Publisher for TTS feedback
        self.feedback_pub = self.create_publisher(String, '/tts/speak', 10)

        # Track active processes
        self.active_process = None
        self.active_mode = None
        self.process_lock = threading.Lock()

        self.get_logger().info('Voice Mode Controller ready!')
        self.get_logger().info('Commands: "follow me", "autopilot", "start navigation", "stop"')

    def command_callback(self, msg):
        """Handle incoming voice commands"""
        command = msg.data.lower().strip()
        self.get_logger().info(f'Received command: {command}')

        # Check for commands
        if 'follow me' in command:
            self.start_follow_mode()
        elif 'autopilot' in command:
            self.start_autopilot_mode()
        elif 'start navigation' in command:
            self.start_navigation_mode()
        elif 'stop' in command:
            self.stop_current_mode()

    def start_follow_mode(self):
        """Start gesture following mode"""
        with self.process_lock:
            # Stop any running mode first
            if self.active_process is not None:
                self.get_logger().info('Stopping current mode before starting follow mode...')
                self._stop_process()

            self.get_logger().info('Starting follow me mode...')

            try:
                # Start gesture complete script with following enabled
                # Set FOLLOW_START_ENABLED=1 so robot starts following immediately
                # without requiring a wave gesture
                # Set TTS=0 to disable duplicate TTS (conversation system already has TTS)
                # Set ROS_DOMAIN_ID to match conversation system (37)
                env = os.environ.copy()
                env['FOLLOW_START_ENABLED'] = '1'
                env['TTS'] = '0'
                env['ROS_DOMAIN_ID'] = '37'

                self.active_process = subprocess.Popen(
                    ['bash', os.path.expanduser('~/start_gesture_complete.sh')],
                    stdout=None,  # Let output go to terminal for debugging
                    stderr=None,
                    env=env,
                    preexec_fn=os.setsid  # Create new process group
                )
                self.active_mode = 'follow'

                # Send feedback
                feedback = String()
                feedback.data = "Starting follow me mode!"
                self.feedback_pub.publish(feedback)

                self.get_logger().info('Follow mode started successfully')
                self.get_logger().info('Check terminal output for gesture system status')

            except Exception as e:
                self.get_logger().error(f'Failed to start follow mode: {e}')
                feedback = String()
                feedback.data = "Failed to start follow mode"
                self.feedback_pub.publish(feedback)

    def start_autopilot_mode(self):
        """Start obstacle avoidance autopilot mode"""
        with self.process_lock:
            # Stop any running mode first
            if self.active_process is not None:
                self.get_logger().info('Stopping current mode before starting autopilot...')
                self._stop_process()

            self.get_logger().info('Starting autopilot mode...')

            try:
                # Start obstacle avoidance script
                self.active_process = subprocess.Popen(
                    ['python3', os.path.expanduser('~/onnx_obstacle_avoidance_tight_spaces.py')],
                    stdout=None,  # Let output go to terminal for debugging
                    stderr=None,
                    preexec_fn=os.setsid  # Create new process group
                )
                self.active_mode = 'autopilot'

                # Send feedback
                feedback = String()
                feedback.data = "Starting autopilot mode!"
                self.feedback_pub.publish(feedback)

                self.get_logger().info('Autopilot mode started successfully')
                self.get_logger().info('Check terminal output for autopilot status')

            except Exception as e:
                self.get_logger().error(f'Failed to start autopilot mode: {e}')
                feedback = String()
                feedback.data = "Failed to start autopilot mode"
                self.feedback_pub.publish(feedback)

    def stop_current_mode(self):
        """Stop the currently active mode"""
        with self.process_lock:
            if self.active_process is None:
                self.get_logger().info('No active mode to stop')
                feedback = String()
                feedback.data = "No active mode running"
                self.feedback_pub.publish(feedback)
                return

            mode_name = self.active_mode or 'unknown'
            self.get_logger().info(f'Stopping {mode_name} mode...')

            self._stop_process()

            # Send feedback
            feedback = String()
            feedback.data = f"Stopped {mode_name} mode"
            self.feedback_pub.publish(feedback)

            self.get_logger().info(f'{mode_name} mode stopped')

    def _stop_process(self):
        """Internal method to stop the active process"""
        if self.active_process is not None:
            try:
                # Kill the entire process group
                os.killpg(os.getpgid(self.active_process.pid), signal.SIGTERM)

                # Wait for process to terminate
                try:
                    self.active_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    # Force kill if it doesn't stop gracefully
                    os.killpg(os.getpgid(self.active_process.pid), signal.SIGKILL)
                    self.active_process.wait()

            except Exception as e:
                self.get_logger().warn(f'Error stopping process: {e}')

            finally:
                self.active_process = None
                self.active_mode = None

    def start_navigation_mode(self):
        """Start navigation via the obstacle nav GPU2 script"""
        with self.process_lock:
            if self.active_process is not None:
                self.get_logger().info('Stopping current mode before starting navigation...')
                self._stop_process()

            self.get_logger().info('Starting navigation mode...')

            try:
                self.active_process = subprocess.Popen(
                    ['bash', os.path.expanduser('~/start_obstacle_nav_gpu2.sh')],
                    stdout=None,
                    stderr=None,
                    preexec_fn=os.setsid
                )
                self.active_mode = 'navigation'

                feedback = String()
                feedback.data = "Starting navigation mode!"
                self.feedback_pub.publish(feedback)

                self.get_logger().info('Navigation mode started successfully')
            except Exception as e:
                self.get_logger().error(f'Failed to start navigation mode: {e}')
                feedback = String()
                feedback.data = "Failed to start navigation mode"
                self.feedback_pub.publish(feedback)

    def cleanup(self):
        """Cleanup on shutdown"""
        self.get_logger().info('Cleaning up...')
        with self.process_lock:
            self._stop_process()

def main(args=None):
    rclpy.init(args=args)
    controller = VoiceModeController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.cleanup()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
