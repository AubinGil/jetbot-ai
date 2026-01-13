#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import base64
import json
import requests
import numpy as np


class VLMVisionNode(Node):
    def __init__(self):
        super().__init__('vlm_vision_node')
        
        # Parameters
        self.declare_parameter('ollama_url', 'http://localhost:11434')
        self.declare_parameter('model', 'llava')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('processing_rate', 0.5)  # Hz (every 2 seconds due to VLM latency)
        
        self.ollama_url = self.get_parameter('ollama_url').value
        self.model = self.get_parameter('model').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.processing_rate = self.get_parameter('processing_rate').value
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        self.latest_image = None
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        
        # Publishers
        self.scene_description_pub = self.create_publisher(
            String,
            '/vlm/scene_description',
            10
        )
        
        self.obstacle_detection_pub = self.create_publisher(
            String,
            '/vlm/obstacle_detection',
            10
        )
        
        # Timer for periodic VLM queries
        self.timer = self.create_timer(
            1.0 / self.processing_rate,
            self.process_image_with_vlm
        )
        
        self.get_logger().info(f'VLM Vision Node started with model: {self.model}')
    
    def image_callback(self, msg):
        """Store the latest camera image"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
    
    def encode_image(self, cv_image):
        """Encode OpenCV image to base64"""
        _, buffer = cv2.imencode('.jpg', cv_image)
        return base64.b64encode(buffer).decode('utf-8')
    
    def query_ollama(self, prompt, image_b64):
        """Query Ollama VLM with image"""
        try:
            response = requests.post(
                f'{self.ollama_url}/api/generate',
                json={
                    'model': self.model,
                    'prompt': prompt,
                    'images': [image_b64],
                    'stream': False
                },
                timeout=90
            )
            
            if response.status_code == 200:
                return response.json()['response']
            else:
                self.get_logger().error(f'Ollama error: {response.status_code}')
                return None
                
        except Exception as e:
            self.get_logger().error(f'Error querying Ollama: {e}')
            return None
    
    def process_image_with_vlm(self):
        """Process latest image with VLM"""
        if self.latest_image is None:
            return
        
        # Encode image
        image_b64 = self.encode_image(self.latest_image)
        
        # Query 1: Scene description
        scene_prompt = "Describe what you see in this image. Focus on obstacles, paths, and navigation relevance. Be concise."
        scene_desc = self.query_ollama(scene_prompt, image_b64)
        
        if scene_desc:
            msg = String()
            msg.data = scene_desc
            self.scene_description_pub.publish(msg)
            self.get_logger().info(f'Scene: {scene_desc[:100]}...')
        
        # Query 2: Obstacle detection
        obstacle_prompt = "Are there any obstacles in front? Answer: YES with description, or NO and if path is clear left/right/center."
        obstacle_info = self.query_ollama(obstacle_prompt, image_b64)
        
        if obstacle_info:
            msg = String()
            msg.data = obstacle_info
            self.obstacle_detection_pub.publish(msg)
            self.get_logger().info(f'Obstacles: {obstacle_info[:100]}...')


def main(args=None):
    rclpy.init(args=args)
    node = VLMVisionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
