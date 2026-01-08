#!/usr/bin/env python3
"""
Simple Laser-based Obstacle Avoidance for Jetbot
Subscribes to /scan and publishes to /cmd_vel
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        # Parameters
        self.declare_parameter('safe_distance', 0.5)  # meters
        self.declare_parameter('forward_speed', 0.15)  # m/s
        self.declare_parameter('turn_speed', 0.3)     # rad/s
        
        self.safe_distance = self.get_parameter('safe_distance').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        
        # Subscribers and Publishers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Obstacle Avoidance started')
        self.get_logger().info(f'Safe distance: {self.safe_distance}m')
        self.get_logger().info(f'Forward speed: {self.forward_speed} m/s')
        
    def scan_callback(self, msg):
        """Process laser scan and decide movement"""
        
        # Divide laser scan into regions
        ranges = np.array(msg.ranges)
        ranges[ranges == 0] = float('inf')  # Replace 0 with infinity
        
        # Split into front, left, right regions
        num_readings = len(ranges)
        front_angle = 60  # degrees to consider as "front"
        front_readings = int((front_angle / 360.0) * num_readings)
        
        # Front region (centered)
        front_start = num_readings // 2 - front_readings // 2
        front_end = num_readings // 2 + front_readings // 2
        front = ranges[front_start:front_end]
        
        # Left region (0 to 90 degrees)
        left = ranges[0:num_readings//4]
        
        # Right region (270 to 360 degrees)
        right = ranges[3*num_readings//4:]
        
        # Find minimum distances
        front_min = np.min(front) if len(front) > 0 else float('inf')
        left_min = np.min(left) if len(left) > 0 else float('inf')
        right_min = np.min(right) if len(right) > 0 else float('inf')
        
        # Decision making
        cmd = Twist()
        
        if front_min > self.safe_distance:
            # Path is clear - move forward
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0
            self.get_logger().info(f'Clear ahead ({front_min:.2f}m) - Moving forward', 
                                 throttle_duration_sec=1.0)
        else:
            # Obstacle detected - stop and turn
            cmd.linear.x = 0.0
            
            # Turn away from obstacle
            if left_min > right_min:
                # Turn left (more space on left)
                cmd.angular.z = self.turn_speed
                self.get_logger().info(f'Obstacle! ({front_min:.2f}m) Turning LEFT', 
                                     throttle_duration_sec=0.5)
            else:
                # Turn right (more space on right)
                cmd.angular.z = -self.turn_speed
                self.get_logger().info(f'Obstacle! ({front_min:.2f}m) Turning RIGHT', 
                                     throttle_duration_sec=0.5)
        
        # Publish command
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot
        cmd = Twist()
        node.cmd_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
