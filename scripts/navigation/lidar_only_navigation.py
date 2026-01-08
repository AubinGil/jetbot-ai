#!/usr/bin/env python3
"""
Simple Lidar-Only Navigation
- Moves forward when path is clear
- Turns away from obstacles
- Stops when blocked
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class LidarNavigator(Node):
    def __init__(self):
        super().__init__('lidar_navigator')

        # Parameters
        self.declare_parameter('safety_distance', 0.5)  # meters
        self.declare_parameter('warning_distance', 1.0)  # meters
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('turn_speed', 0.5)

        self.safety_distance = self.get_parameter('safety_distance').value
        self.warning_distance = self.get_parameter('warning_distance').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.get_logger().info("Lidar Navigator ready!")
        self.get_logger().info(f"Safety distance: {self.safety_distance}m")
        self.get_logger().info(f"Warning distance: {self.warning_distance}m")

    def scan_callback(self, msg):
        """Process lidar scan and navigate"""
        ranges = np.array(msg.ranges)
        ranges[ranges == 0] = float('inf')  # Ignore zero readings
        ranges[ranges < 0.1] = float('inf')  # Ignore very close noise

        # Divide scan into sectors
        num_readings = len(ranges)
        sector_size = num_readings // 8

        # Front (0 degrees), Left (90 degrees), Right (270 degrees)
        front_start = num_readings - sector_size // 2
        front_end = sector_size // 2
        front_ranges = np.concatenate([ranges[front_start:], ranges[:front_end]])

        left_start = num_readings // 4 - sector_size // 2
        left_end = left_start + sector_size
        left_ranges = ranges[left_start:left_end]

        right_start = 3 * num_readings // 4 - sector_size // 2
        right_end = right_start + sector_size
        right_ranges = ranges[right_start:right_end]

        # Calculate minimum distances
        min_front = np.min(front_ranges) if len(front_ranges) > 0 else float('inf')
        min_left = np.min(left_ranges) if len(left_ranges) > 0 else float('inf')
        min_right = np.min(right_ranges) if len(right_ranges) > 0 else float('inf')

        # Decision logic
        twist = Twist()

        if min_front < self.safety_distance:
            # Obstacle ahead - turn towards clearer side
            if min_left > min_right:
                twist.linear.x = 0.0
                twist.angular.z = self.turn_speed
                self.get_logger().info(f"🛑 Obstacle {min_front:.2f}m ahead - Turning LEFT")
            else:
                twist.linear.x = 0.0
                twist.angular.z = -self.turn_speed
                self.get_logger().info(f"🛑 Obstacle {min_front:.2f}m ahead - Turning RIGHT")

        elif min_front < self.warning_distance:
            # Warning zone - slow down and prepare to turn
            speed_scale = (min_front - self.safety_distance) / (self.warning_distance - self.safety_distance)
            twist.linear.x = self.forward_speed * speed_scale * 0.5
            twist.angular.z = 0.0
            self.get_logger().info(f"⚠️  Obstacle {min_front:.2f}m - Slowing to {twist.linear.x:.2f}m/s")

        else:
            # Path clear - move forward
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0
            self.get_logger().info(f"✓ Clear {min_front:.2f}m - Moving FORWARD at {twist.linear.x:.2f}m/s")

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LidarNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop the robot
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        node.get_logger().info("Stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
