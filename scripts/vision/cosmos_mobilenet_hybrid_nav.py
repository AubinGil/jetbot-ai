#!/usr/bin/env python3
"""
Hybrid Navigation: Cosmos-Reason2-2B + MobileNet Fallback
Uses Cosmos-Reason2 when available, falls back to MobileNet if network fails

Architecture:
1. PRIMARY: Cosmos-Reason2-2B (remote on 5090) - Better reasoning
2. FALLBACK: MobileNet ONNX (local on Jetson) - Reliable backup
3. ENSEMBLE: Combine both when low confidence
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import requests
import onnxruntime as ort
import time
from collections import deque
import base64
import json


class HybridNavigationNode(Node):
    def __init__(self):
        super().__init__('hybrid_navigation')

        # Parameters
        self.declare_parameter('cosmos_url', 'http://192.168.2.16:8000')
        self.declare_parameter('mobilenet_path', '/home/gildas01/obstacle_models/best_model.onnx')
        self.declare_parameter('cosmos_timeout', 2.0)  # Network timeout
        self.declare_parameter('cosmos_enabled', True)
        self.declare_parameter('forward_speed', 0.12)
        self.declare_parameter('turn_speed', 0.30)
        self.declare_parameter('decision_interval', 1.0)

        # Get parameters
        self.cosmos_url = self.get_parameter('cosmos_url').value
        self.mobilenet_path = self.get_parameter('mobilenet_path').value
        self.cosmos_timeout = self.get_parameter('cosmos_timeout').value
        self.cosmos_enabled = self.get_parameter('cosmos_enabled').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.decision_interval = self.get_parameter('decision_interval').value

        # Setup
        self.bridge = CvBridge()
        self.last_decision_time = 0

        # Statistics
        self.stats = {
            'cosmos_success': 0,
            'cosmos_fail': 0,
            'mobilenet_used': 0,
            'ensemble_used': 0
        }

        # Load MobileNet (always available as fallback)
        self.get_logger().info(f'Loading MobileNet from {self.mobilenet_path}...')
        self.mobilenet_session = ort.InferenceSession(
            self.mobilenet_path,
            providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
        )
        self.get_logger().info('✅ MobileNet loaded (fallback ready)')

        # Test Cosmos connection
        if self.cosmos_enabled:
            if self.test_cosmos_connection():
                self.get_logger().info('✅ Cosmos-Reason2 connected (primary)')
            else:
                self.get_logger().warn('⚠️  Cosmos-Reason2 unavailable, using MobileNet only')
                self.cosmos_enabled = False

        # ROS2 setup
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Decision smoothing
        self.decision_history = deque(maxlen=5)

        self.get_logger().info('🚀 Hybrid Navigation Ready!')
        self.get_logger().info(f'   Primary: Cosmos-Reason2 ({"enabled" if self.cosmos_enabled else "disabled"})')
        self.get_logger().info(f'   Fallback: MobileNet (always available)')

    def test_cosmos_connection(self):
        """Test if Cosmos server is reachable"""
        try:
            response = requests.get(
                f'{self.cosmos_url}/health',
                timeout=2
            )
            return response.status_code == 200
        except:
            return False

    def classify_mobilenet(self, image):
        """
        Classify with MobileNet (local, fast, reliable)
        Returns: (classification, confidence)
        """
        # Preprocess for MobileNet
        img_resized = cv2.resize(image, (224, 224))
        img_normalized = (img_resized.astype(np.float32) / 255.0 - 0.5) / 0.5
        img_input = np.transpose(img_normalized, (2, 0, 1))
        img_input = np.expand_dims(img_input, axis=0)

        # Inference
        input_name = self.mobilenet_session.get_inputs()[0].name
        output = self.mobilenet_session.run(None, {input_name: img_input})[0]

        # Parse output (assuming binary classification: 0=blocked, 1=free)
        confidence = float(output[0][1])  # Confidence for "free"
        classification = "free" if confidence > 0.5 else "blocked"

        return classification, confidence

    def classify_cosmos(self, image):
        """
        Classify with Cosmos-Reason2 (remote, slow, smart)
        Returns: (classification, confidence, reasoning) or None if failed
        """
        try:
            # Encode image
            _, buffer = cv2.imencode('.jpg', image)
            image_b64 = base64.b64encode(buffer).decode('utf-8')

            # Improved navigation prompt with examples
            messages = [
                {
                    "role": "system",
                    "content": """You are a vision AI for a small wheeled robot (JetBot, 10cm tall, 15cm wide).
Your task: Determine if the robot can safely move forward.

BLOCKED: Large obstacles in the center path (walls, furniture, boxes, people, pets) that the robot cannot pass under or around within 1 meter.
FREE: Clear floor space ahead, or only small/flat objects the robot can drive over (papers, cables, small bumps).

The robot sees from ground level. Open floor = FREE. Large objects blocking path = BLOCKED."""
                },
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{image_b64}"
                            }
                        },
                        {
                            "type": "text",
                            "text": 'Analyze from robot perspective (ground level, small robot). Is the forward path clear? Respond ONLY with JSON: {"classification": "blocked"|"free", "confidence": 0.0-1.0}'
                        }
                    ]
                }
            ]

            # Call API with timeout
            response = requests.post(
                f'{self.cosmos_url}/v1/chat/completions',
                json={
                    'model': 'nvidia/Cosmos-Reason2-2B',
                    'messages': messages,
                    'max_tokens': 100,
                    'temperature': 0.0,  # Deterministic for navigation
                },
                timeout=self.cosmos_timeout
            )

            if response.status_code != 200:
                return None

            # Parse response
            result = response.json()
            content = result['choices'][0]['message']['content']

            # Extract JSON
            if '```json' in content:
                content = content.split('```json')[1].split('```')[0]
            elif '```' in content:
                content = content.split('```')[1].split('```')[0]

            data = json.loads(content.strip())
            classification = data.get('classification', 'blocked').lower()
            confidence = float(data.get('confidence', 0.5))

            return classification, confidence, "Cosmos reasoning"

        except Exception as e:
            self.get_logger().debug(f'Cosmos classification failed: {e}')
            return None

    def classify_hybrid(self, image):
        """
        Hybrid classification strategy:
        1. Try Cosmos first (better reasoning)
        2. Fallback to MobileNet if Cosmos fails
        3. Ensemble if low confidence
        """
        cosmos_result = None
        mobilenet_result = None

        # Try Cosmos first (if enabled)
        if self.cosmos_enabled:
            cosmos_start = time.time()
            cosmos_result = self.classify_cosmos(image)
            cosmos_time = time.time() - cosmos_start

            if cosmos_result is not None:
                classification, confidence, reasoning = cosmos_result
                self.stats['cosmos_success'] += 1

                # High confidence - use Cosmos result
                if confidence > 0.8:
                    self.get_logger().info(
                        f'🤖 Cosmos: {classification} ({confidence:.2f}) [{cosmos_time:.1f}s]'
                    )
                    return classification, confidence, 'cosmos'

                # Low confidence - get second opinion from MobileNet
                self.get_logger().info(
                    f'🤖 Cosmos uncertain ({confidence:.2f}), checking MobileNet...'
                )
            else:
                self.stats['cosmos_fail'] += 1
                self.get_logger().warn('⚠️  Cosmos failed, using MobileNet fallback')

        # Use MobileNet (fallback or ensemble)
        mobilenet_start = time.time()
        mobilenet_classification, mobilenet_confidence = self.classify_mobilenet(image)
        mobilenet_time = time.time() - mobilenet_start

        # If Cosmos succeeded with low confidence, ensemble
        if cosmos_result is not None and cosmos_result[1] < 0.8:
            cosmos_class, cosmos_conf, _ = cosmos_result

            # Both agree - high confidence
            if cosmos_class == mobilenet_classification:
                self.stats['ensemble_used'] += 1
                final_confidence = (cosmos_conf + mobilenet_confidence) / 2
                self.get_logger().info(
                    f'🤝 Ensemble: {cosmos_class} (Cosmos: {cosmos_conf:.2f}, MobileNet: {mobilenet_confidence:.2f})'
                )
                return cosmos_class, final_confidence, 'ensemble'

            # Disagree - be conservative (blocked)
            else:
                self.stats['ensemble_used'] += 1
                self.get_logger().warn(
                    f'⚠️  Disagreement! Cosmos: {cosmos_class}, MobileNet: {mobilenet_classification} → Using BLOCKED (safe)'
                )
                return 'blocked', 0.6, 'ensemble-conservative'

        # Pure MobileNet fallback
        self.stats['mobilenet_used'] += 1
        self.get_logger().info(
            f'📱 MobileNet: {mobilenet_classification} ({mobilenet_confidence:.2f}) [{mobilenet_time*1000:.0f}ms]'
        )
        return mobilenet_classification, mobilenet_confidence, 'mobilenet'

    def image_callback(self, msg):
        """Process camera images and navigate"""
        current_time = time.time()

        # Rate limiting
        if current_time - self.last_decision_time < self.decision_interval:
            return

        self.last_decision_time = current_time

        # Convert image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
            return

        # Classify with hybrid strategy
        classification, confidence, source = self.classify_hybrid(cv_image)

        # Add to history for smoothing
        self.decision_history.append(classification)

        # Smooth decision (majority vote)
        if len(self.decision_history) >= 3:
            blocked_count = self.decision_history.count('blocked')
            free_count = self.decision_history.count('free')
            smoothed_classification = 'blocked' if blocked_count > free_count else 'free'
        else:
            smoothed_classification = classification

        # Navigate
        self.navigate(smoothed_classification, confidence)

        # Print stats periodically
        total = sum(self.stats.values())
        if total > 0 and total % 10 == 0:
            self.get_logger().info(
                f'📊 Stats: Cosmos: {self.stats["cosmos_success"]} | '
                f'MobileNet: {self.stats["mobilenet_used"]} | '
                f'Ensemble: {self.stats["ensemble_used"]} | '
                f'Failures: {self.stats["cosmos_fail"]}'
            )

    def navigate(self, classification, confidence):
        """Control robot based on classification"""
        cmd = Twist()

        if classification == 'free' and confidence > 0.7:
            # Safe to move forward
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0
        elif classification == 'blocked':
            # Turn to find clear path
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed
        else:
            # Uncertain - slow down
            cmd.linear.x = self.forward_speed * 0.3
            cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = HybridNavigationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
