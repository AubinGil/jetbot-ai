#!/usr/bin/env python3
"""
Lightweight Wall Detection & Line Following Navigator
Combines line following with wall detection for indoor navigation
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class WallLineNavigator(Node):
    def __init__(self):
        super().__init__('wall_line_navigator')

        # Parameters
        self.declare_parameter('enable_line_following', True)
        self.declare_parameter('enable_wall_detection', True)
        self.declare_parameter('min_wall_distance', 30)  # pixels from edge
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.0)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_image_pub = self.create_publisher(Image, '/wall_line_nav/debug_image', 10)

        # CV Bridge
        self.bridge = CvBridge()

        # State variables
        self.last_line_error = 0
        self.wall_warning_left = False
        self.wall_warning_right = False

        self.get_logger().info('Wall & Line Navigator initialized')

    def detect_lines(self, image):
        """
        Detect colored lines (similar to your existing line following code)
        Returns: line_center_x, line_angle, line_detected
        """
        height, width = image.shape[:2]

        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define color range for line (adjust based on your line color)
        # Example for yellow line:
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        # You can add more colors here (white, red, etc.)
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Focus on lower portion of image (where line typically is)
        roi_height = int(height * 0.6)
        mask[:height - roi_height, :] = 0

        # Morphological operations to clean up
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None, None, False

        # Get largest contour
        largest_contour = max(contours, key=cv2.contourArea)

        if cv2.contourArea(largest_contour) < 100:  # Minimum area threshold
            return None, None, False

        # Calculate centroid
        M = cv2.moments(largest_contour)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # Calculate angle using fitLine
            [vx, vy, x, y] = cv2.fitLine(largest_contour, cv2.DIST_L2, 0, 0.01, 0.01)
            angle = np.arctan2(vy, vx)[0]

            return cx, angle, True

        return None, None, False

    def detect_walls(self, image):
        """
        Lightweight wall detection using edge detection
        Returns: wall_left, wall_right (True if wall too close)
        """
        height, width = image.shape[:2]

        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Edge detection
        edges = cv2.Canny(blurred, 50, 150)

        # Check left and right margins for walls
        margin = self.get_parameter('min_wall_distance').value

        # Left wall detection
        left_region = edges[:, :margin]
        left_edge_density = np.sum(left_region) / (height * margin * 255)

        # Right wall detection
        right_region = edges[:, width-margin:]
        right_edge_density = np.sum(right_region) / (height * margin * 255)

        # Threshold for wall detection (adjust based on testing)
        wall_threshold = 0.1

        wall_left = left_edge_density > wall_threshold
        wall_right = right_edge_density > wall_threshold

        return wall_left, wall_right, edges

    def image_callback(self, msg):
        """Process camera image for navigation"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Initialize command
            cmd = Twist()

            # Wall detection
            wall_left, wall_right, edges = self.detect_walls(cv_image)
            self.wall_warning_left = wall_left
            self.wall_warning_right = wall_right

            # Line detection
            line_x, line_angle, line_detected = self.detect_lines(cv_image)

            # Navigation logic
            if self.get_parameter('enable_line_following').value and line_detected:
                # Line following mode
                image_center = cv_image.shape[1] / 2
                error = (line_x - image_center) / image_center  # Normalized error

                # PID-like control (simplified)
                angular_speed = -error * self.get_parameter('max_angular_speed').value
                linear_speed = self.get_parameter('max_linear_speed').value

                # Reduce speed if turning
                if abs(error) > 0.3:
                    linear_speed *= 0.7

                cmd.linear.x = linear_speed
                cmd.angular.z = angular_speed

                self.get_logger().debug(f'Line following: error={error:.2f}, angular={angular_speed:.2f}')

            # Wall avoidance (overrides line following if walls too close)
            if self.get_parameter('enable_wall_detection').value:
                if wall_left and wall_right:
                    # Walls on both sides - stop or reverse
                    cmd.linear.x = -0.1
                    cmd.angular.z = 0.0
                    self.get_logger().warn('Walls detected on both sides!')
                elif wall_left:
                    # Wall on left - turn right
                    cmd.angular.z = -0.5
                    cmd.linear.x *= 0.5
                    self.get_logger().info('Wall detected on left, turning right')
                elif wall_right:
                    # Wall on right - turn left
                    cmd.angular.z = 0.5
                    cmd.linear.x *= 0.5
                    self.get_logger().info('Wall detected on right, turning left')

            # Publish command
            self.cmd_vel_pub.publish(cmd)

            # Publish debug image
            self.publish_debug_image(cv_image, line_x, line_detected, wall_left, wall_right, edges)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def publish_debug_image(self, image, line_x, line_detected, wall_left, wall_right, edges):
        """Create and publish debug visualization"""
        debug_img = image.copy()
        height, width = image.shape[:2]

        # Draw line center if detected
        if line_detected and line_x is not None:
            cv2.circle(debug_img, (int(line_x), height - 50), 10, (0, 255, 0), -1)
            cv2.line(debug_img, (int(line_x), 0), (int(line_x), height), (0, 255, 0), 2)

        # Draw image center
        cv2.line(debug_img, (width//2, 0), (width//2, height), (255, 0, 0), 2)

        # Draw wall warnings
        margin = self.get_parameter('min_wall_distance').value
        if wall_left:
            cv2.rectangle(debug_img, (0, 0), (margin, height), (0, 0, 255), 3)
            cv2.putText(debug_img, 'WALL LEFT', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        if wall_right:
            cv2.rectangle(debug_img, (width-margin, 0), (width, height), (0, 0, 255), 3)
            cv2.putText(debug_img, 'WALL RIGHT', (width-150, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Add status text
        status = "Line: " + ("DETECTED" if line_detected else "NOT DETECTED")
        cv2.putText(debug_img, status, (10, height-20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Publish
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, 'bgr8')
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing debug image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = WallLineNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
