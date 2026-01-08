#!/usr/bin/env python3
"""
Waveshare Jetbot Pro Motor Controller for ROS2 Humble with Encoder Feedback
Sends /cmd_vel commands AND reads encoder-based odometry from RP2040
"""
import os
import time
import struct
import threading
from glob import glob
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32
from tf2_ros import TransformBroadcaster
import serial
import math

class FrameState(Enum):
    HEAD1 = 1
    HEAD2 = 2
    SIZE = 3
    DATA = 4
    CHECKSUM = 5
    HANDLE = 6

class JetbotMotorControllerWithEncoders(Node):
    def __init__(self):
        super().__init__('jetbot_motor_controller_encoders')

        # Parameters
        self.declare_parameter('port', '/dev/serial/by-id/usb-TinyUSB_TinyUSB_Device_123456-if00')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('max_accel', 1.5)
        self.declare_parameter('max_angular_accel', 3.0)
        self.declare_parameter('linear_deadband', 0.01)
        self.declare_parameter('angular_deadband', 0.02)
        self.declare_parameter('publish_odom_transform', True)
        self.declare_parameter('linear_correction', 1.0)
        self.declare_parameter('angular_correction', 1.0)
        self.declare_parameter('enable_accel_smoothing', False)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.max_accel = self.get_parameter('max_accel').value
        self.max_angular_accel = self.get_parameter('max_angular_accel').value
        self.linear_deadband = self.get_parameter('linear_deadband').value
        self.angular_deadband = self.get_parameter('angular_deadband').value
        self.publish_odom_tf = self.get_parameter('publish_odom_transform').value
        self.linear_correction = self.get_parameter('linear_correction').value
        self.angular_correction = self.get_parameter('angular_correction').value
        self.enable_smoothing = self.get_parameter('enable_accel_smoothing').value

        # Serial port
        self.serial_port = None
        self.failed_writes = 0
        self.max_failed_writes = 5
        self.serial_lock = threading.Lock()

        self.open_serial()

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.lvel_pub = self.create_publisher(Int32, 'motor/lvel', 10)
        self.rvel_pub = self.create_publisher(Int32, 'motor/rvel', 10)
        self.lset_pub = self.create_publisher(Int32, 'motor/lset', 10)
        self.rset_pub = self.create_publisher(Int32, 'motor/rset', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Timers
        self.cmd_timer = self.create_timer(0.02, self.send_velocity)  # 50 Hz

        # Target velocities
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0

        # Current smoothed velocities
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0

        self.last_cmd_time = self.get_clock().now()
        self.dt = 0.02

        # Send robot parameters
        time.sleep(0.1)
        self.set_params(self.linear_correction, self.angular_correction)

        # Start serial receiving thread
        self.running = True
        self.serial_thread = threading.Thread(target=self.serial_receive_task, daemon=True)
        self.serial_thread.start()

        self.get_logger().info('Motor controller with encoder feedback started')

    def open_serial(self):
        """Open or reopen the serial connection"""
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
            with self.serial_lock:
                self.open_serial()
            self.get_logger().info('Motor controller reconnected successfully!')
            return True
        except Exception as e:
            self.get_logger().error(f'Serial reconnection failed: {e}')
            return False

    def _serial_port_candidates(self):
        """Return serial port candidates"""
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

    def checksum(self, data):
        """Calculate checksum"""
        return sum(data) & 0xFF

    def set_params(self, linear_corr, angular_corr):
        """Send robot calibration parameters"""
        data = bytearray(9)
        data[0] = 0xAA  # head1
        data[1] = 0x55  # head2
        data[2] = 0x09  # size
        data[3] = 0x13  # sendType_params
        lin_int = int(linear_corr * 1000)
        ang_int = int(angular_corr * 1000)
        data[4] = (lin_int >> 8) & 0xFF
        data[5] = lin_int & 0xFF
        data[6] = (ang_int >> 8) & 0xFF
        data[7] = ang_int & 0xFF
        data[8] = self.checksum(data[:8])

        with self.serial_lock:
            if self.serial_port:
                try:
                    self.serial_port.write(data)
                    self.get_logger().info('Sent robot parameters')
                except Exception as e:
                    self.get_logger().error(f'Failed to send params: {e}')

    def cmd_vel_callback(self, msg):
        """Handle cmd_vel messages"""
        self.target_linear_x = msg.linear.x if abs(msg.linear.x) >= self.linear_deadband else 0.0
        self.target_linear_y = msg.linear.y if abs(msg.linear.y) >= self.linear_deadband else 0.0
        self.target_angular_z = msg.angular.z if abs(msg.angular.z) >= self.angular_deadband else 0.0
        self.last_cmd_time = self.get_clock().now()

    def send_velocity(self):
        """Send velocity command at fixed rate"""
        current_time = self.get_clock().now()

        # Timeout safety
        if (current_time - self.last_cmd_time).nanoseconds / 1e9 > 1.0:
            self.target_linear_x = 0.0
            self.target_linear_y = 0.0
            self.target_angular_z = 0.0

        # Apply acceleration limiting when explicitly enabled
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
        """Apply acceleration limiting"""
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
        data[10] = self.checksum(data[:10])

        with self.serial_lock:
            try:
                bytes_written = self.serial_port.write(data)
                if bytes_written != len(data):
                    raise IOError(f'Incomplete write: {bytes_written}/{len(data)} bytes')
                self.failed_writes = 0
            except Exception as e:
                self.failed_writes += 1
                if self.failed_writes % 5 == 1:
                    self.get_logger().error(f'Failed to write to serial port ({self.failed_writes} failures): {e}')
                if self.failed_writes >= self.max_failed_writes:
                    self.get_logger().warn('Max write failures reached, attempting reconnect...')
                    self.reconnect_serial()
                    self.failed_writes = 0

    def serial_receive_task(self):
        """Serial port receiving task - runs in separate thread"""
        state = FrameState.HEAD1
        frame_size = 0
        frame_type = 0
        data = bytearray(50)
        last_time = self.get_clock().now()

        self.get_logger().info('Serial receive task started')

        while self.running and rclpy.ok():
            try:
                with self.serial_lock:
                    if not self.serial_port or not self.serial_port.is_open:
                        time.sleep(0.1)
                        continue

                    # State machine for parsing protocol
                    if state == FrameState.HEAD1:
                        byte_read = self.serial_port.read(1)
                        if len(byte_read) == 0:
                            continue
                        data[0] = byte_read[0]
                        if data[0] == 0xAA:
                            state = FrameState.HEAD2

                    elif state == FrameState.HEAD2:
                        byte_read = self.serial_port.read(1)
                        if len(byte_read) == 0:
                            state = FrameState.HEAD1
                            continue
                        data[1] = byte_read[0]
                        if data[1] == 0x55:
                            state = FrameState.SIZE
                        else:
                            state = FrameState.HEAD1

                    elif state == FrameState.SIZE:
                        byte_read = self.serial_port.read(1)
                        if len(byte_read) == 0:
                            state = FrameState.HEAD1
                            continue
                        data[2] = byte_read[0]
                        frame_size = data[2]
                        state = FrameState.DATA

                    elif state == FrameState.DATA:
                        bytes_to_read = frame_size - 4
                        bytes_read = self.serial_port.read(bytes_to_read)
                        if len(bytes_read) != bytes_to_read:
                            state = FrameState.HEAD1
                            continue
                        data[3:3+bytes_to_read] = bytes_read
                        frame_type = data[3]
                        state = FrameState.CHECKSUM

                    elif state == FrameState.CHECKSUM:
                        byte_read = self.serial_port.read(1)
                        if len(byte_read) == 0:
                            state = FrameState.HEAD1
                            continue
                        data[frame_size - 1] = byte_read[0]
                        calc_sum = self.checksum(data[:frame_size - 1])
                        if data[frame_size - 1] == calc_sum:
                            state = FrameState.HANDLE
                        else:
                            self.get_logger().debug(f'Checksum error: recv={data[frame_size-1]:02x} calc={calc_sum:02x}')
                            state = FrameState.HEAD1

                    elif state == FrameState.HANDLE:
                        # Process the received data
                        self.handle_received_data(data, frame_size, last_time)
                        last_time = self.get_clock().now()
                        state = FrameState.HEAD1

            except Exception as e:
                self.get_logger().error(f'Serial receive error: {e}')
                time.sleep(0.1)
                state = FrameState.HEAD1

    def handle_received_data(self, data, frame_size, last_time):
        """Process received sensor data from RP2040"""
        now_time = self.get_clock().now()

        try:
            # Parse IMU data (gyro: rad/s, accel: m/s², angles: degrees)
            gyro_x = struct.unpack('>h', data[4:6])[0] / 32768.0 * 2000.0 / 180.0 * math.pi
            gyro_y = struct.unpack('>h', data[6:8])[0] / 32768.0 * 2000.0 / 180.0 * math.pi
            gyro_z = struct.unpack('>h', data[8:10])[0] / 32768.0 * 2000.0 / 180.0 * math.pi
            accel_x = struct.unpack('>h', data[10:12])[0] / 32768.0 * 2.0 * 9.8
            accel_y = struct.unpack('>h', data[12:14])[0] / 32768.0 * 2.0 * 9.8
            accel_z = struct.unpack('>h', data[14:16])[0] / 32768.0 * 2.0 * 9.8
            roll = struct.unpack('>h', data[16:18])[0] / 10.0
            pitch = struct.unpack('>h', data[18:20])[0] / 10.0
            yaw_deg = struct.unpack('>h', data[20:22])[0] / 10.0

            # Publish IMU
            imu_msg = Imu()
            imu_msg.header.stamp = now_time.to_msg()
            imu_msg.header.frame_id = 'base_imu_link'
            imu_msg.angular_velocity.x = gyro_x
            imu_msg.angular_velocity.y = gyro_y
            imu_msg.angular_velocity.z = gyro_z
            imu_msg.linear_acceleration.x = accel_x
            imu_msg.linear_acceleration.y = accel_y
            imu_msg.linear_acceleration.z = accel_z

            # Orientation from yaw (simplified)
            yaw_rad = yaw_deg / 180.0 * math.pi
            imu_msg.orientation.z = math.sin(yaw_rad / 2.0)
            imu_msg.orientation.w = math.cos(yaw_rad / 2.0)

            self.imu_pub.publish(imu_msg)

            # Parse encoder-based odometry (x, y: meters, yaw: radians)
            odom_x = struct.unpack('>h', data[22:24])[0] / 1000.0
            odom_y = struct.unpack('>h', data[24:26])[0] / 1000.0
            odom_yaw = struct.unpack('>h', data[26:28])[0] / 1000.0

            # Parse velocities in base frame
            dx = struct.unpack('>h', data[28:30])[0] / 1000.0
            dy = struct.unpack('>h', data[30:32])[0] / 1000.0
            dyaw = struct.unpack('>h', data[32:34])[0] / 1000.0

            # Calculate velocities from deltas
            dt = (now_time - last_time).nanoseconds / 1e9
            if dt > 0:
                vx = dx / dt
                vy = dy / dt
                vyaw = dyaw / dt
            else:
                vx = vy = vyaw = 0.0

            # Publish TF transform
            if self.publish_odom_tf:
                t = TransformStamped()
                t.header.stamp = now_time.to_msg()
                t.header.frame_id = 'odom'
                t.child_frame_id = 'base_footprint'
                t.transform.translation.x = odom_x
                t.transform.translation.y = odom_y
                t.transform.translation.z = 0.0
                t.transform.rotation.z = math.sin(odom_yaw / 2.0)
                t.transform.rotation.w = math.cos(odom_yaw / 2.0)
                self.tf_broadcaster.sendTransform(t)

            # Publish Odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = now_time.to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_footprint'
            odom_msg.pose.pose.position.x = odom_x
            odom_msg.pose.pose.position.y = odom_y
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation.z = math.sin(odom_yaw / 2.0)
            odom_msg.pose.pose.orientation.w = math.cos(odom_yaw / 2.0)
            odom_msg.twist.twist.linear.x = vx
            odom_msg.twist.twist.linear.y = vy
            odom_msg.twist.twist.angular.z = vyaw

            # Covariance matrices (from original Waveshare code) - all values must be float
            odom_msg.pose.covariance = [
                1e-9, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1e-3, 1e-9, 0.0, 0.0, 0.0,
                0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 1e3
            ]
            odom_msg.twist.covariance = [
                1e-9, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1e-3, 1e-9, 0.0, 0.0, 0.0,
                0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.1
            ]

            self.odom_pub.publish(odom_msg)

            # Parse motor encoder velocities
            lvel = struct.unpack('>h', data[34:36])[0]
            rvel = struct.unpack('>h', data[36:38])[0]
            lset = struct.unpack('>h', data[38:40])[0]
            rset = struct.unpack('>h', data[40:42])[0]

            # Publish motor data
            self.lvel_pub.publish(Int32(data=lvel))
            self.rvel_pub.publish(Int32(data=rvel))
            self.lset_pub.publish(Int32(data=lset))
            self.rset_pub.publish(Int32(data=rset))

        except Exception as e:
            self.get_logger().error(f'Error parsing received data: {e}')

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.running = False
        self.set_velocity(0.0, 0.0, 0.0)
        if self.serial_thread.is_alive():
            self.serial_thread.join(timeout=1.0)
        if hasattr(self, 'serial_port') and self.serial_port:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JetbotMotorControllerWithEncoders()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
