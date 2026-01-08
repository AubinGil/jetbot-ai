#!/usr/bin/env python3
"""
YOLOv11 Pose Gesture Node
Subscribes to /camera/image_raw, runs Ultralytics pose estimation, and publishes
wave gesture events for higher-level interaction nodes.
"""

from collections import deque
from pathlib import Path
import threading
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
except ImportError:  # pragma: no cover
    YOLO = None


class PoseGestureNode(Node):
    """Detect simple waving gestures using YOLOv11 pose keypoints."""

    def __init__(self) -> None:
        super().__init__('pose_gesture_node')

        if YOLO is None:
            self.get_logger().error(
                'Ultralytics is not installed. Install it or run inside the '
                'ultralytics/ultralytics:latest-jetson-jetpack6 container.'
            )
            raise RuntimeError('Missing Ultralytics dependency')

        self.bridge = CvBridge()
        self.frame_lock = threading.Lock()
        self.latest_frame: Optional[np.ndarray] = None

        # Parameters
        default_model_path = str(Path.home() / 'yolo11n-pose.onnx')
        self.camera_topic = self.declare_parameter('camera_topic', '/camera/image_raw').value
        self.model_path = Path(
            self.declare_parameter('model_path', default_model_path).value
        ).expanduser()
        self.device = self.declare_parameter('device', 'cuda:0').value
        self.imgsz = int(self.declare_parameter('image_size', 640).value)
        self.detection_rate = float(self.declare_parameter('detection_rate_hz', 8.0).value)
        self.conf_threshold = float(self.declare_parameter('confidence_threshold', 0.4).value)
        self.kp_conf_threshold = float(self.declare_parameter('keypoint_threshold', 0.35).value)
        self.raise_ratio = float(self.declare_parameter('wrist_raise_ratio', 0.08).value)
        self.wave_history = int(self.declare_parameter('wave_history', 12).value)
        self.wave_min_frames = int(self.declare_parameter('wave_min_frames', 6).value)
        self.wave_amplitude_px = float(self.declare_parameter('wave_amplitude_px', 60.0).value)
        self.wave_cooldown = float(self.declare_parameter('wave_cooldown_sec', 6.0).value)
        self.use_half = bool(self.declare_parameter('use_half_precision', True).value)
        self.publish_topic = self.declare_parameter('gesture_topic', 'gesture/events').value

        if not self.model_path.exists():
            raise FileNotFoundError(f'Model path not found: {self.model_path}')

        self.get_logger().info(f'Loading YOLO pose model from {self.model_path}')
        self.model = YOLO(str(self.model_path))

        self.gesture_pub = self.create_publisher(String, self.publish_topic, 10)
        self.create_subscription(Image, self.camera_topic, self.handle_image, 10)
        self.timer = self.create_timer(max(1.0 / self.detection_rate, 0.05), self.run_inference)

        self.left_history = deque(maxlen=self.wave_history)
        self.right_history = deque(maxlen=self.wave_history)
        self.last_wave_time = 0.0

        self.get_logger().info(
            f'✅ Pose gesture node ready (rate: {self.detection_rate} Hz, topic: {self.publish_topic})'
        )

    def handle_image(self, msg: Image) -> None:
        """Cache the latest image frame."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f'Failed to convert image: {exc}')
            return

        with self.frame_lock:
            self.latest_frame = frame.copy()

    def run_inference(self) -> None:
        """Run pose detection on the most recent frame."""
        with self.frame_lock:
            if self.latest_frame is None:
                return
            frame = self.latest_frame.copy()

        half_precision = self.use_half and (
            'cuda' in str(self.device).lower() or str(self.device).isdigit()
        )

        try:
            results = self.model.predict(
                frame,
                imgsz=self.imgsz,
                conf=self.conf_threshold,
                device=self.device,
                half=half_precision,
                verbose=False,
            )
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f'YOLO inference failed: {exc}')
            return

        if not results:
            self._clear_histories()
            return

        result = results[0]
        keypoints = getattr(result, 'keypoints', None)

        if keypoints is None or keypoints.data is None:
            self._clear_histories()
            return

        data = keypoints.data
        if hasattr(data, 'cpu'):
            data = data.cpu().numpy()
        else:
            data = np.asarray(data)

        height = frame.shape[0]
        wave_detected = False

        for person in data:
            left_wave = self._update_history(person, 'left', height)
            right_wave = self._update_history(person, 'right', height)
            wave_detected |= left_wave or right_wave

        if wave_detected:
            self._publish_wave()

    def _update_history(self, person: np.ndarray, hand: str, img_height: int) -> bool:
        """Update wrist histories and check if a wave gesture occurred."""
        key_indices = {
            'left': {'wrist': 9, 'shoulder': 5},
            'right': {'wrist': 10, 'shoulder': 6},
        }
        history = self.left_history if hand == 'left' else self.right_history
        indices = key_indices[hand]

        wrist = person[indices['wrist']]
        shoulder = person[indices['shoulder']]

        if wrist[2] < self.kp_conf_threshold or shoulder[2] < self.kp_conf_threshold:
            history.clear()
            return False

        wrist_above_shoulder = wrist[1] < (shoulder[1] - img_height * self.raise_ratio)

        if wrist_above_shoulder:
            history.append(wrist[0])
        else:
            history.clear()
            return False

        if len(history) < self.wave_min_frames:
            return False

        amplitude = max(history) - min(history)
        if amplitude >= self.wave_amplitude_px:
            history.clear()
            return True
        return False

    def _publish_wave(self) -> None:
        """Publish a wave gesture event while respecting cooldown."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_wave_time < self.wave_cooldown:
            return

        self.last_wave_time = now
        msg = String()
        msg.data = 'wave'
        self.gesture_pub.publish(msg)
        self.get_logger().info('👋 Wave detected - notifying interaction stack')

    def _clear_histories(self) -> None:
        self.left_history.clear()
        self.right_history.clear()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseGestureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
