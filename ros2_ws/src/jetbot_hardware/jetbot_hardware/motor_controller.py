#!/usr/bin/env python3
"""
Waveshare Jetbot Pro Motor Controller for ROS2 Humble
Subscribes to /cmd_vel and sends commands to motor controller
"""
import os
import time
from glob import glob

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class JetbotMotorController(Node):
    def __init__(self):
        super().__init__('jetbot_motor_controller')

        self.declare_parameter('port', '/dev/serial/by-id/usb-TinyUSB_TinyUSB_Device_123456-if00')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('max_accel', 1.5)  # m/s² max acceleration
        self.declare_parameter('max_angular_accel', 2.5)  # rad/s² max angular acceleration (smooth and graceful)
        self.declare_parameter('linear_deadband', 0.01)  # m/s - ignore commands below this
        self.declare_parameter('angular_deadband', 0.02)  # rad/s - ignore commands below this
        self.declare_parameter('enable_accel_smoothing', False)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.max_accel = self.get_parameter('max_accel').value
        self.max_angular_accel = self.get_parameter('max_angular_accel').value
        self.linear_deadband = self.get_parameter('linear_deadband').value
        self.angular_deadband = self.get_parameter('angular_deadband').value
        self.enable_smoothing = self.get_parameter('enable_accel_smoothing').value

        # Reconnection state
        self.serial_port = None
        self.failed_writes = 0
        self.max_failed_writes = 5

        self.open_serial()

        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.timer = self.create_timer(0.02, self.send_velocity)

        # Target velocities (from cmd_vel)
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0

        # Current smoothed velocities (sent to motors)
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0

        self.last_cmd_time = self.get_clock().now()
        self.dt = 0.02  # 50 Hz timer period

        self.get_logger().info('Motor controller node started')

    def open_serial(self):
        """Open or reopen the serial connection, trying any available ACM/USB device."""
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass

        candidates = self._serial_port_candidates()
        last_exception = None
        for candidate in candidates:
            try:
                self.serial_port = serial.Serial(candidate, self.baud, timeout=self.timeout)
                time.sleep(0.5)
                self.failed_writes = 0
                self.port = candidate
                self.get_logger().info(f'Connected to motor controller on {candidate} at {self.baud} baud')
                return
            except Exception as exc:
                last_exception = exc
                self.get_logger().warn(f'Failed to open serial port {candidate}: {exc}')

        if last_exception:
            self.get_logger().error('Unable to open any serial port for motor controller')
            raise last_exception
        raise RuntimeError('No serial port candidates available for motor controller')

    def reconnect_serial(self):
        """Attempt to reconnect the serial port"""
        self.get_logger().warn('Attempting to reconnect motor controller serial port...')
        try:
            time.sleep(0.3)
            self.open_serial()
            self.get_logger().info('Motor controller reconnected successfully!')
            return True
        except Exception as e:
            self.get_logger().error(f'Serial reconnection failed: {e}')
            return False

    def _serial_port_candidates(self):
        """Return serial port candidates, prioritizing the configured port."""
        seen = set()
        candidates = []
        if self.port:
            candidates.append(self.port)

        candidates.extend(sorted(glob('/dev/ttyACM*')))
        candidates.extend(sorted(glob('/dev/ttyUSB*')))

        filtered = []
        for port in candidates:
            if port not in seen and os.path.exists(port):
                filtered.append(port)
                seen.add(port)
        return filtered
    
    def cmd_vel_callback(self, msg):
        # Apply deadband to filter out tiny commands that cause jitter
        self.target_linear_x = msg.linear.x if abs(msg.linear.x) >= self.linear_deadband else 0.0
        self.target_linear_y = msg.linear.y if abs(msg.linear.y) >= self.linear_deadband else 0.0
        self.target_angular_z = msg.angular.z if abs(msg.angular.z) >= self.angular_deadband else 0.0
        self.last_cmd_time = self.get_clock().now()
        self.get_logger().info(f'Received cmd_vel: x={self.target_linear_x:.2f}, z={self.target_angular_z:.2f}', throttle_duration_sec=1.0)
    
    def send_velocity(self):
        current_time = self.get_clock().now()

        # Timeout safety: if no cmd_vel for 1 second, target zero velocity
        if (current_time - self.last_cmd_time).nanoseconds / 1e9 > 1.0:
            self.target_linear_x = 0.0
            self.target_linear_y = 0.0
            self.target_angular_z = 0.0

        # Apply acceleration limiting for smooth motion when explicitly enabled
        if self.enable_smoothing:
            self.current_linear_x = self._smooth_velocity(
                self.current_linear_x, self.target_linear_x, self.max_accel, self.dt)
            self.current_linear_y = self._smooth_velocity(
                self.current_linear_y, self.target_linear_y, self.max_accel, self.dt)
            self.current_angular_z = self._smooth_velocity(
                self.current_angular_z, self.target_angular_z, self.max_angular_accel, self.dt)
        else:
            self.current_linear_x = self.target_linear_x
            self.current_linear_y = self.target_linear_y
            self.current_angular_z = self.target_angular_z

        self.set_velocity(self.current_linear_x, self.current_linear_y, self.current_angular_z)

    def _smooth_velocity(self, current, target, max_accel, dt):
        """Apply acceleration limiting to velocity changes"""
        error = target - current
        max_change = max_accel * dt

        if abs(error) <= max_change:
            return target
        elif error > 0:
            return current + max_change
        else:
            return current - max_change
    
    def set_velocity(self, x, y, yaw):
        """Send velocity command to Waveshare motor controller"""
        if self.serial_port is None:
            return

        x_int = int(x * 1000)
        y_int = int(y * 1000)
        yaw_int = int(yaw * 1000)

        data = bytearray(11)
        data[0] = 0xAA  # head1
        data[1] = 0x55  # head2
        data[2] = 0x0B  # size
        data[3] = 0x11  # sendType_velocity
        data[4] = (x_int >> 8) & 0xFF
        data[5] = x_int & 0xFF
        data[6] = (y_int >> 8) & 0xFF
        data[7] = y_int & 0xFF
        data[8] = (yaw_int >> 8) & 0xFF
        data[9] = yaw_int & 0xFF
        data[10] = sum(data[:10]) & 0xFF  # checksum

        try:
            bytes_written = self.serial_port.write(data)
            if bytes_written != len(data):
                raise IOError(f'Incomplete write: {bytes_written}/{len(data)} bytes')

            # Reset failure counter on success
            self.failed_writes = 0
        except Exception as e:
            self.failed_writes += 1

            if self.failed_writes % 5 == 1:
                self.get_logger().error(f'Failed to write to serial port ({self.failed_writes} failures): {e}')

            # Attempt reconnection if threshold exceeded
            if self.failed_writes >= self.max_failed_writes:
                self.get_logger().warn('Max write failures reached, attempting reconnect...')
                self.reconnect_serial()
                self.failed_writes = 0  # Reset counter after reconnect attempt
    
    def destroy_node(self):
        self.set_velocity(0.0, 0.0, 0.0)
        if hasattr(self, 'serial_port'):
            self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = JetbotMotorController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
