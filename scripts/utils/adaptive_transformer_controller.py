#!/usr/bin/env python3
"""
Adaptive Transformer Controller for Robot Navigation Smoothing
Uses IBM Granite TTM-R2 (TinyTimeMixer) for predictive velocity smoothing

This node:
- Subscribes to /cmd_vel and /scan to collect telemetry
- Maintains a sliding window of recent commands and sensor data
- Uses TTM-R2 transformer to predict optimal next velocities
- Publishes adaptive smoothing parameters or adjusted commands

Parameters (ROS2):
- transformer_enabled (bool): Enable transformer predictions (default: true)
- context_length (int): Historical window size for predictions (default: 64)
- prediction_horizon (int): How many steps ahead to predict (default: 10)
- model_path (string): Hugging Face model path (default: ibm-granite/granite-timeseries-ttm-r2)
- publish_mode (string): 'smoothing_params' or 'cmd_vel_filtered' (default: smoothing_params)
- update_rate (double): Frequency of predictions in Hz (default: 5.0)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
import numpy as np
from collections import deque
import threading

# Fallback frequency mapping used when resolution prefix tuning models expect `freq_token`
DEFAULT_FREQUENCY_MAPPING = {
    "oov": 0,
    "min": 1,
    "2min": 2,
    "5min": 3,
    "10min": 4,
    "15min": 5,
    "30min": 6,
    "h": 7,
    "H": 7,
    "d": 8,
    "D": 8,
    "W": 9,
}

# TTM imports (will be optional to allow running without transformer)
try:
    from tsfm_public.toolkit.get_model import get_model
    TTM_AVAILABLE = True
except ImportError:
    TTM_AVAILABLE = False
    print("⚠️  Warning: granite-tsfm not installed. Transformer predictions disabled.")
    print("   Install with: pip3 install granite-tsfm")


class AdaptiveTransformerController(Node):
    def __init__(self):
        super().__init__('adaptive_transformer_controller')

        # Parameters
        self.declare_parameter('transformer_enabled', True)
        self.declare_parameter('context_length', 64)
        self.declare_parameter('prediction_horizon', 10)
        self.declare_parameter('model_path', 'ibm-granite/granite-timeseries-ttm-r2')
        self.declare_parameter('publish_mode', 'smoothing_params')
        self.declare_parameter('update_rate', 5.0)
        self.declare_parameter('use_cpu', True)  # Force CPU on Jetson for stability
        self.declare_parameter('frequency', 'oov')

        self.transformer_enabled = self.get_parameter('transformer_enabled').value and TTM_AVAILABLE
        self.context_length = self.get_parameter('context_length').value
        self.prediction_horizon = self.get_parameter('prediction_horizon').value
        self.model_path = self.get_parameter('model_path').value
        self.publish_mode = self.get_parameter('publish_mode').value
        self.update_rate = self.get_parameter('update_rate').value
        self.use_cpu = self.get_parameter('use_cpu').value
        self.frequency = self.get_parameter('frequency').value
        self.freq_token_id = DEFAULT_FREQUENCY_MAPPING.get(self.frequency, DEFAULT_FREQUENCY_MAPPING["oov"])

        # Data buffers - sliding windows for time-series
        # Store: [timestamp, linear_x, angular_z, min_distance, left_distance, right_distance]
        self.data_window = deque(maxlen=self.context_length)
        self.lock = threading.Lock()

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Publishers
        if self.publish_mode == 'cmd_vel_filtered':
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_filtered', 10)
        else:  # smoothing_params mode
            self.smoothing_pub = self.create_publisher(
                Float64MultiArray, '/adaptive_smoothing_params', 10)

        # Latest sensor data
        self.latest_min_distance = float('inf')
        self.latest_left_distance = float('inf')
        self.latest_right_distance = float('inf')

        # Load transformer model
        self.model = None
        if self.transformer_enabled:
            self.get_logger().info(f"Loading TTM-R2 model: {self.model_path}...")
            try:
                import torch
                # Force CPU mode on Jetson to avoid CUDA issues
                if self.use_cpu:
                    self.device = torch.device('cpu')
                    self.get_logger().info("Using CPU for inference (Jetson-friendly)")
                else:
                    self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
                    self.get_logger().info(f"Using device: {self.device}")

                # Load model with appropriate context and prediction lengths
                self.model = get_model(
                    model_path=self.model_path,
                    context_length=self.context_length,
                    prediction_length=self.prediction_horizon,
                    freq_prefix_tuning=False,
                    prefer_longer_context=True
                )
                self.model = self.model.to(self.device)
                self.model.eval()

                self.get_logger().info("✓ TTM-R2 model loaded successfully!")
                self.get_logger().info(f"Freq token '{self.frequency}' ⇒ {self.freq_token_id}")
            except Exception as e:
                self.get_logger().error(f"Failed to load transformer: {e}")
                self.transformer_enabled = False
        else:
            if not TTM_AVAILABLE:
                self.get_logger().warn("Transformer disabled: granite-tsfm not installed")
            else:
                self.get_logger().info("Transformer disabled by configuration")

        # Create timer for periodic predictions
        if self.transformer_enabled:
            timer_period = 1.0 / self.update_rate
            self.prediction_timer = self.create_timer(timer_period, self.predict_and_publish)

        self.get_logger().info(f"Adaptive Transformer Controller ready!")
        self.get_logger().info(f"  Mode: {self.publish_mode}")
        self.get_logger().info(f"  Context: {self.context_length} samples")
        self.get_logger().info(f"  Prediction horizon: {self.prediction_horizon} steps")
        self.get_logger().info(f"  Update rate: {self.update_rate} Hz")

    def cmd_vel_callback(self, msg):
        """Store cmd_vel commands in sliding window"""
        timestamp = self.get_clock().now().nanoseconds / 1e9

        with self.lock:
            # Append [timestamp, linear_x, angular_z, min_dist, left_dist, right_dist]
            self.data_window.append([
                timestamp,
                float(msg.linear.x),
                float(msg.angular.z),
                float(self.latest_min_distance),
                float(self.latest_left_distance),
                float(self.latest_right_distance)
            ])

    def scan_callback(self, msg):
        """Update latest sensor distances from lidar"""
        ranges = np.array(msg.ranges, dtype=float)
        n = len(ranges)
        if n == 0:
            return

        # Build angles in degrees relative to forward (0 deg)
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        angles_deg = np.degrees(angles)

        # Valid ranges
        valid = np.isfinite(ranges)
        valid &= ranges >= 0.2  # min valid range
        if msg.range_max and np.isfinite(msg.range_max):
            valid &= ranges <= float(msg.range_max)

        # Front cone (±60°)
        front_cone = np.abs(angles_deg) <= 60.0
        front_mask = valid & front_cone
        front_candidates = ranges[front_mask]
        if front_candidates.size > 0:
            self.latest_min_distance = float(np.min(front_candidates))
        else:
            self.latest_min_distance = float('inf')

        # Left side cone (45° to 135°)
        left_cone = (angles_deg >= 45.0) & (angles_deg <= 135.0)
        left_mask = valid & left_cone
        left_candidates = ranges[left_mask]
        if left_candidates.size > 0:
            self.latest_left_distance = float(np.min(left_candidates))
        else:
            self.latest_left_distance = float('inf')

        # Right side cone (-135° to -45°)
        right_cone = (angles_deg <= -45.0) & (angles_deg >= -135.0)
        right_mask = valid & right_cone
        right_candidates = ranges[right_mask]
        if right_candidates.size > 0:
            self.latest_right_distance = float(np.min(right_candidates))
        else:
            self.latest_right_distance = float('inf')

    def _scale_context_features(self, features: np.ndarray):
        """Standardize each channel using the current context window."""
        mean = np.mean(features, axis=0)
        std = np.std(features, axis=0)
        std = np.where(std < 1e-3, 1.0, std)
        scaled = (features - mean) / std
        return scaled, mean, std

    def _inverse_scale_predictions(self, predictions: np.ndarray, mean: np.ndarray, std: np.ndarray):
        """Restore per-channel statistics to the model output."""
        mean = mean.reshape(-1, 1)
        std = std.reshape(-1, 1)
        return predictions * std + mean

    def predict_and_publish(self):
        """Use transformer to predict next velocities and publish results"""
        if not self.transformer_enabled or self.model is None:
            return

        with self.lock:
            if len(self.data_window) < self.context_length:
                # Not enough data yet
                return

            # Convert window to numpy array
            data = np.array(list(self.data_window), dtype=np.float32)

        try:
            # Extract features (skip timestamp column)
            # Features: [linear_x, angular_z, min_dist, left_dist, right_dist]
            features = data[:, 1:]  # Shape: (context_length, 5)

            # Replace inf values with large number for model stability
            features = np.nan_to_num(features, nan=0.0, posinf=10.0, neginf=-10.0)

            # Prepare input for transformer
            # TTM expects shape: (batch_size, sequence_length, num_input_channels)
            # We have: (context_length, num_channels)
            scaled_features, context_mean, context_std = self._scale_context_features(features)
            features_batched = np.expand_dims(scaled_features, axis=0)  # Shape: (1, context_length, 5)

            # Convert to torch tensor
            import torch
            input_tensor = torch.from_numpy(features_batched).float().to(self.device)
            freq_token = torch.full(
                (input_tensor.shape[0],),
                self.freq_token_id,
                dtype=torch.long,
                device=self.device,
            )

            # Run inference
            with torch.no_grad():
                model_output = self.model(input_tensor, freq_token=freq_token)

            # Extract tensor from model output (handles ModelOutput wrappers)
            if hasattr(model_output, "prediction_outputs") and model_output.prediction_outputs is not None:
                pred_tensor = model_output.prediction_outputs
            else:
                pred_tensor = model_output

            # Get predictions and convert back to numpy
            pred_numpy = pred_tensor.cpu().numpy()[0]  # Shape: (prediction_horizon, num_channels)
            pred_numpy = pred_numpy.T  # Shape: (num_channels, prediction_horizon)

            # Inverse transform to original scale
            pred_original_scale = self._inverse_scale_predictions(pred_numpy, context_mean, context_std)

            # Extract predicted velocities (first few steps)
            # pred_original_scale shape: (5, prediction_horizon)
            pred_velocities = pred_original_scale  # Shape: (5, prediction_horizon)

            # Take first prediction step for immediate action
            next_linear = float(pred_velocities[0, 0])  # linear_x prediction
            next_angular = float(pred_velocities[1, 0])  # angular_z prediction

            # Compute adaptive smoothing parameters based on predictions
            # Use variance in predictions to determine aggressiveness
            linear_variance = float(np.var(pred_velocities[0, :]))
            angular_variance = float(np.var(pred_velocities[1, :]))

            # Higher variance = more aggressive smoothing (smaller deltas)
            # Lower variance = less smoothing (larger deltas for responsiveness)
            adaptive_linear_delta = max(0.01, min(0.05, 0.02 / (1.0 + linear_variance)))
            adaptive_angular_delta = max(0.04, min(0.15, 0.08 / (1.0 + angular_variance)))

            if self.publish_mode == 'cmd_vel_filtered':
                # Publish filtered cmd_vel directly
                twist = Twist()
                twist.linear.x = next_linear
                twist.angular.z = next_angular
                self.cmd_vel_pub.publish(twist)

                self.get_logger().info(
                    f"Predicted: linear={next_linear:.3f} angular={next_angular:.3f}",
                    throttle_duration_sec=2.0
                )
            else:
                # Publish adaptive smoothing parameters
                msg = Float64MultiArray()
                msg.data = [adaptive_linear_delta, adaptive_angular_delta]
                self.smoothing_pub.publish(msg)

                self.get_logger().info(
                    f"Adaptive smoothing: Δlinear={adaptive_linear_delta:.4f} Δangular={adaptive_angular_delta:.4f}",
                    throttle_duration_sec=2.0
                )

        except Exception as e:
            self.get_logger().error(f"Prediction error: {e}", throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveTransformerController()

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
