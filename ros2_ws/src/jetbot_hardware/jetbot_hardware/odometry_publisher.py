#!/usr/bin/env python3
"""
Simple Odometry Publisher for JetBot
Integrates cmd_vel over time to publish /odom and TF
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import math

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Current velocities
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        # Timer for publishing odometry
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.publish_odometry)  # 50 Hz

        # Publish static transform from base_link to laser_frame
        self.publish_static_transforms()

        self.get_logger().info('Odometry publisher started')

    def publish_static_transforms(self):
        """Publish static TF from base_link to laser_frame"""
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'base_link'
        static_transform.child_frame_id = 'laser_frame'

        # Lidar position relative to base_link (adjust based on your robot)
        # x: 0.0 (centered), y: 0.0 (centered), z: 0.1 (10cm above base)
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.1

        # No rotation (lidar aligned with robot)
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform(static_transform)
        self.get_logger().info('Published static transform: base_link -> laser_frame')

    def cmd_vel_callback(self, msg):
        """Update current velocity from cmd_vel"""
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vth = msg.angular.z

    def publish_odometry(self):
        """Integrate velocity and publish odometry"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt <= 0:
            return

        # Update position (simple integration)
        delta_x = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
        delta_y = (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
        delta_theta = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Create quaternion from yaw
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        # Publish TF transform (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth

        # Covariance (uncertainty) - simple diagonal matrix
        # Position covariance
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y
        odom.pose.covariance[14] = 99999 # z (not used)
        odom.pose.covariance[21] = 99999 # roll (not used)
        odom.pose.covariance[28] = 99999 # pitch (not used)
        odom.pose.covariance[35] = 0.01  # yaw

        # Velocity covariance
        odom.twist.covariance[0] = 0.01   # vx
        odom.twist.covariance[7] = 0.01   # vy
        odom.twist.covariance[14] = 99999 # vz (not used)
        odom.twist.covariance[21] = 99999 # vroll (not used)
        odom.twist.covariance[28] = 99999 # vpitch (not used)
        odom.twist.covariance[35] = 0.01  # vyaw

        self.odom_pub.publish(odom)

        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
