#!/usr/bin/env python3
"""
Webots ROS2 Controller for Jetbot
Works with custom Jetbot models - auto-detects camera and motors
Subscribes to /cmd_vel and publishes camera to /camera/image_raw
"""

from controller import Robot
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np
import sys

class JetbotController(Node):
    def __init__(self, robot):
        super().__init__('jetbot_webots_controller')
        
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())
        
        # Auto-detect and setup motors
        self.setup_motors()
        
        # Auto-detect and setup camera
        self.setup_camera()
        
        # Robot parameters (tune these to match your real Jetbot)
        self.wheel_radius = 0.033  # meters (33mm typical for Jetbot)
        self.wheel_base = 0.10     # distance between wheels (100mm)
        
        # ROS2 subscriber for velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # ROS2 publisher for camera images
        if self.camera:
            self.image_pub = self.create_publisher(
                Image,
                '/camera/image_raw',
                10
            )
            self.create_timer(0.1, self.publish_camera_image)  # 10 Hz
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('Jetbot Webots ROS2 Controller Started')
        self.get_logger().info(f'Motors: {self.motor_names}')
        self.get_logger().info(f'Camera: {self.camera_name if self.camera else "None"}')
        self.get_logger().info(f'Subscribing to: /cmd_vel')
        self.get_logger().info(f'Publishing to: /camera/image_raw')
        self.get_logger().info('=' * 50)
    
    def setup_motors(self):
        """Auto-detect and configure motors"""
        # Common motor names
        motor_patterns = [
            ('left_wheel_hinge', 'right_wheel_hinge'),  # Your Jetbot!
            ('left_motor', 'right_motor'),
            ('left wheel motor', 'right wheel motor'),
            ('wheel_left_joint', 'wheel_right_joint'),
            ('motor_left', 'motor_right'),
            ('left', 'right')
        ]
        
        self.left_motor = None
        self.right_motor = None
        self.motor_names = []
        
        # Try to find motors
        for left_name, right_name in motor_patterns:
            try:
                left = self.robot.getDevice(left_name)
                right = self.robot.getDevice(right_name)
                if left and right:
                    self.left_motor = left
                    self.right_motor = right
                    self.motor_names = [left_name, right_name]
                    break
            except:
                continue
        
        if not self.left_motor or not self.right_motor:
            self.get_logger().error('Could not find motors! Check device names.')
            sys.exit(1)
        
        # Configure motors for velocity control
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
    
    def setup_camera(self):
        """Auto-detect and configure camera"""
        camera_patterns = ['camera', 'Camera', 'cam', 'webcam']
        
        self.camera = None
        self.camera_name = None
        
        for name in camera_patterns:
            try:
                cam = self.robot.getDevice(name)
                if cam:
                    self.camera = cam
                    self.camera_name = name
                    self.camera.enable(self.timestep)
                    break
            except:
                continue
        
        if not self.camera:
            self.get_logger().warn('No camera found - running without vision')
    
    def cmd_vel_callback(self, msg):
        """Convert Twist message to differential drive motor velocities"""
        linear_vel = msg.linear.x   # m/s
        angular_vel = msg.angular.z  # rad/s
        
        # Differential drive kinematics
        left_vel = (linear_vel - (angular_vel * self.wheel_base / 2.0)) / self.wheel_radius
        right_vel = (linear_vel + (angular_vel * self.wheel_base / 2.0)) / self.wheel_radius
        
        # Set motor velocities
        self.left_motor.setVelocity(left_vel)
        self.right_motor.setVelocity(right_vel)
    
    def publish_camera_image(self):
        """Publish camera image to ROS2"""
        if not self.camera:
            return
        
        # Get image from camera
        img_data = self.camera.getImage()
        if img_data is None:
            return
        
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        
        # Convert to numpy array
        img_array = np.frombuffer(img_data, np.uint8).reshape((height, width, 4))
        
        # Convert BGRA to RGB
        img_rgb = img_array[:, :, [2, 1, 0]]
        
        # Create ROS2 Image message
        img_msg = Image()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_link'
        img_msg.height = height
        img_msg.width = width
        img_msg.encoding = 'rgb8'
        img_msg.is_bigendian = False
        img_msg.step = width * 3
        img_msg.data = img_rgb.tobytes()
        
        self.image_pub.publish(img_msg)


def main(args=None):
    # Initialize Webots robot
    robot = Robot()
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create controller node
    controller = JetbotController(robot)
    
    # Main loop
    try:
        while robot.step(controller.timestep) != -1:
            rclpy.spin_once(controller, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop motors
        if controller.left_motor and controller.right_motor:
            controller.left_motor.setVelocity(0.0)
            controller.right_motor.setVelocity(0.0)
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
