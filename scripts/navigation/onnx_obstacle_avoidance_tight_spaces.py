#!/usr/bin/env python3
"""
ONNX-based Obstacle Avoidance Node - TIGHT SPACES VERSION
Optimized for tight indoor environments (living rooms, hallways)
- Gentler turning behavior
- Slower speeds for confined areas
- Better stuck detection
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import onnxruntime as ort
import numpy as np
from PIL import Image as PILImage
import time

class ONNXObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('onnx_obstacle_avoidance')

        # Parameters
        self.declare_parameter('model_path', '/home/gildas01/obstacle_models/best_model_20251211_115936.onnx')
        self.declare_parameter('forward_speed', 0.08)  # Slower for tight spaces
        self.declare_parameter('turn_speed', 0.18)      # Gentler turns
        self.declare_parameter('decision_interval', 1.5)  # More time to complete maneuvers
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('stuck_threshold', 5)    # After N blocked readings, try backing up

        self.model_path = self.get_parameter('model_path').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.decision_interval = self.get_parameter('decision_interval').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.stuck_threshold = self.get_parameter('stuck_threshold').value

        # Load ONNX model
        self.get_logger().info(f"Loading ONNX model from {self.model_path}...")
        self.session = ort.InferenceSession(
            self.model_path,
            providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
        )

        # Get input/output names
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name

        self.get_logger().info(f"Using providers: {self.session.get_providers()}")
        self.get_logger().info("Model loaded successfully!")

        # Image preprocessing (must match training)
        self.input_size = (224, 224)
        self.mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        self.std = np.array([0.229, 0.224, 0.225], dtype=np.float32)

        # ROS2 setup
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.bridge = CvBridge()
        self.latest_image = None
        self.processing = False

        # Statistics
        self.total_predictions = 0
        self.blocked_count = 0
        self.free_count = 0
        self.consecutive_blocked = 0  # Track consecutive blocked detections
        self.start_time = time.time()
        self.total_inference_time = 0.0

        # Create timer for periodic decisions
        self.timer = self.create_timer(self.decision_interval, self.make_decision)

        self.get_logger().info("="*60)
        self.get_logger().info("ONNX Obstacle Avoidance - TIGHT SPACES MODE")
        self.get_logger().info("="*60)
        self.get_logger().info(f"Model: {self.model_path}")
        self.get_logger().info(f"Forward speed: {self.forward_speed} m/s (slow for tight spaces)")
        self.get_logger().info(f"Turn speed: {self.turn_speed} rad/s (gentle turns)")
        self.get_logger().info(f"Decision interval: {self.decision_interval}s (more time)")
        self.get_logger().info(f"Stuck threshold: {self.stuck_threshold} detections")
        self.get_logger().info("="*60)

    def image_callback(self, msg):
        """Store latest camera image"""
        if not self.processing:
            self.latest_image = msg

    def preprocess_image(self, cv_image):
        """
        Preprocess image for ONNX model inference
        """
        # Convert BGR to RGB
        rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Resize
        resized = cv2.resize(rgb, self.input_size)

        # Convert to float and normalize to [0, 1]
        normalized = resized.astype(np.float32) / 255.0

        # Apply ImageNet normalization
        normalized = (normalized - self.mean) / self.std

        # Transpose to CHW format and add batch dimension
        transposed = np.transpose(normalized, (2, 0, 1))
        batched = np.expand_dims(transposed, axis=0)

        return batched.astype(np.float32)

    def predict_obstacle(self, cv_image):
        """
        Use trained ONNX model to predict if path is blocked
        Returns: (prediction, confidence, inference_time)
            prediction: 'free' or 'blocked'
            confidence: float between 0 and 1
            inference_time: inference time in milliseconds
        """
        try:
            # Preprocess
            input_data = self.preprocess_image(cv_image)

            # Run inference
            start_time = time.time()
            outputs = self.session.run([self.output_name], {self.input_name: input_data})
            inference_time = (time.time() - start_time) * 1000  # ms

            # Get logits and apply softmax
            logits = outputs[0][0]
            exp_logits = np.exp(logits - np.max(logits))  # Numerical stability
            probabilities = exp_logits / np.sum(exp_logits)

            # Get prediction
            predicted_class = np.argmax(probabilities)
            confidence = probabilities[predicted_class]

            # INVERTED: Model learned 0=free, 1=blocked (opposite of training script)
            # This is due to feedback loop in data collection
            prediction = 'blocked' if predicted_class == 1 else 'free'

            return prediction, confidence, inference_time

        except Exception as e:
            self.get_logger().error(f"Prediction error: {e}")
            return 'blocked', 0.0, 0.0  # Safe default: assume blocked

    def make_decision(self):
        """Main decision loop: capture image, predict, move"""
        if self.latest_image is None or self.processing:
            return

        self.processing = True

        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")

            # Get prediction
            prediction, confidence, inference_time = self.predict_obstacle(cv_image)

            # Update statistics
            self.total_predictions += 1
            self.total_inference_time += inference_time
            if prediction == 'blocked':
                self.blocked_count += 1
                self.consecutive_blocked += 1
            else:
                self.free_count += 1
                self.consecutive_blocked = 0  # Reset counter

            # Create movement command
            twist = Twist()

            # TIGHT SPACE LOGIC
            if self.consecutive_blocked >= self.stuck_threshold:
                # Stuck! Try backing up and turning
                twist.linear.x = -self.forward_speed * 0.5  # Backup slowly
                twist.angular.z = self.turn_speed * 1.2      # Turn while backing
                self.get_logger().warn(
                    f"STUCK ({self.consecutive_blocked} blocked) - BACKING UP & TURNING | "
                    f"Inference: {inference_time:.1f}ms"
                )
                # Reset counter after backing up
                if self.consecutive_blocked > self.stuck_threshold + 2:
                    self.consecutive_blocked = 0

            elif confidence < self.confidence_threshold:
                # Low confidence - move very cautiously
                twist.linear.x = self.forward_speed * 0.2
                twist.angular.z = self.turn_speed * 0.5
                self.get_logger().warn(
                    f"Low confidence ({confidence:.2f}) - Crawling cautiously | "
                    f"Inference: {inference_time:.1f}ms"
                )

            elif prediction == 'free':
                # Path is clear - move forward (but still slow for tight spaces)
                twist.linear.x = self.forward_speed
                twist.angular.z = 0.0
                self.get_logger().info(
                    f"Path CLEAR (conf: {confidence:.2f}) - Moving FORWARD | "
                    f"Inference: {inference_time:.1f}ms"
                )

            else:  # blocked
                # Obstacle detected - GENTLE arc turn (not spin in place)
                twist.linear.x = self.forward_speed * 0.3  # Keep some forward momentum
                twist.angular.z = self.turn_speed          # Turn gently
                self.get_logger().info(
                    f"OBSTACLE (conf: {confidence:.2f}) - GENTLE ARC TURN ({self.consecutive_blocked}/5) | "
                    f"Inference: {inference_time:.1f}ms"
                )

            # Publish movement command
            self.cmd_vel_pub.publish(twist)

            # Print statistics every 10 predictions
            if self.total_predictions % 10 == 0:
                elapsed = time.time() - self.start_time
                avg_inference = self.total_inference_time / self.total_predictions
                self.get_logger().info(f"\n{'='*60}")
                self.get_logger().info("Statistics:")
                self.get_logger().info(f"  Total predictions: {self.total_predictions}")
                self.get_logger().info(f"  Blocked: {self.blocked_count} ({100*self.blocked_count/self.total_predictions:.1f}%)")
                self.get_logger().info(f"  Free: {self.free_count} ({100*self.free_count/self.total_predictions:.1f}%)")
                self.get_logger().info(f"  Consecutive blocked: {self.consecutive_blocked}")
                self.get_logger().info(f"  Avg inference time: {avg_inference:.1f}ms")
                self.get_logger().info(f"  Rate: {self.total_predictions/elapsed:.2f} decisions/sec")
                self.get_logger().info(f"{'='*60}\n")

        except Exception as e:
            self.get_logger().error(f"Error in decision loop: {e}")
            # Safe default: stop
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
        finally:
            self.processing = False

    def destroy_node(self):
        """Cleanup: stop robot and print final statistics"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("Shutting down obstacle avoidance...")
        self.get_logger().info("="*60)

        # Stop robot
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

        # Print final statistics
        elapsed = time.time() - self.start_time
        avg_inference = self.total_inference_time / self.total_predictions if self.total_predictions > 0 else 0
        self.get_logger().info(f"Total predictions: {self.total_predictions}")
        self.get_logger().info(f"  Blocked: {self.blocked_count}")
        self.get_logger().info(f"  Free: {self.free_count}")
        self.get_logger().info(f"  Average inference: {avg_inference:.1f}ms")
        self.get_logger().info(f"Runtime: {elapsed:.1f}s")
        self.get_logger().info("="*60)

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    print("="*60)
    print("ONNX Obstacle Avoidance - TIGHT SPACES MODE")
    print("="*60)
    print("Optimized for confined indoor spaces:")
    print("  ✓ Slower speeds (safer)")
    print("  ✓ Gentler turning (arc turns, not spins)")
    print("  ✓ Stuck detection (backs up if needed)")
    print("  ✓ Longer decision intervals")
    print("="*60)
    print("Press Ctrl+C to stop")
    print("="*60)

    node = ONNXObstacleAvoidance()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopping autonomous navigation...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
