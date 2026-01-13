#!/usr/bin/env python3
"""
Simple USB Camera Publisher for Jetbot
Publishes camera images to /camera/image_raw
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class JetbotCameraPublisher(Node):
    def __init__(self):
        super().__init__('jetbot_camera_publisher')
        
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15)
        
        device = self.get_parameter('device').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # Try multiple methods to open camera
        self.cap = None
        
        # Method 1: Try device string (e.g., /dev/video0)
        if isinstance(device, str) and device.startswith('/dev/video'):
            device_id = int(device.replace('/dev/video', ''))
            self.cap = cv2.VideoCapture(device_id)
        else:
            self.cap = cv2.VideoCapture(device)
        
        if self.cap is not None and self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, fps)
            
            # Test read
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn(f'Camera {device} opened but cannot read frames, trying next device...')
                self.cap.release()
                self.cap = None
        
        # Method 2: Try other camera indices
        if self.cap is None or not self.cap.isOpened():
            for i in range(4):
                self.get_logger().info(f'Trying camera index {i}...')
                self.cap = cv2.VideoCapture(i)
                if self.cap.isOpened():
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                    self.cap.set(cv2.CAP_PROP_FPS, fps)
                    ret, frame = self.cap.read()
                    if ret and frame is not None:
                        self.get_logger().info(f'Successfully opened camera at index {i}')
                        device = f'/dev/video{i}'
                        break
                    self.cap.release()
                self.cap = None
        
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().error(f'Failed to open any camera')
            raise RuntimeError(f'Cannot open camera')
        
        self.get_logger().info(f'Camera opened: {device} ({width}x{height} @ {fps}fps)')
        
        self.timer = self.create_timer(1.0 / fps, self.publish_image)
    
    def publish_image(self):
        try:
            ret, frame = self.cap.read()
            if ret:
                self.get_logger().info(f'Frame captured: {frame.shape}', throttle_duration_sec=5.0)
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_link'
                self.publisher.publish(msg)
                self.get_logger().info('Frame published', throttle_duration_sec=5.0)
            else:
                self.get_logger().warn('Failed to capture frame')
        except Exception as e:
            self.get_logger().error(f'Error in publish_image: {e}')
    
    def destroy_node(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = JetbotCameraPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
