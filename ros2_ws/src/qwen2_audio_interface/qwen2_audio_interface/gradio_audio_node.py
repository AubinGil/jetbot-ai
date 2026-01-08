#!/usr/bin/env python3
"""
ROS2 Node for Qwen2-Audio via Gradio Interface
Subscribes to audio, sends to Gradio server, publishes text intent
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gradio_client import Client
import base64
import io
import wave
import numpy as np
import soundfile as sf
import tempfile
import os


class GradioAudioNode(Node):
    def __init__(self):
        super().__init__('gradio_audio_node')

        # Parameters
        self.declare_parameter('gradio_url', 'https://4f127fdb7db57f9e2b.gradio.live')
        self.declare_parameter('audio_topic', '/mic/audio')
        self.declare_parameter('intent_topic', '/audio_intent')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('api_name', '/predict')

        self.gradio_url = self.get_parameter('gradio_url').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.api_name = self.get_parameter('api_name').value

        # Initialize Gradio client
        self.client = None
        self.connect_to_gradio()

        # Subscriber to audio data
        self.audio_sub = self.create_subscription(
            String,
            self.get_parameter('audio_topic').value,
            self.audio_callback,
            10
        )

        # Publisher for audio intent
        self.intent_pub = self.create_publisher(
            String,
            self.get_parameter('intent_topic').value,
            10
        )

        # Conversation history
        self.history = []

        self.get_logger().info('Gradio Audio Node started')
        self.get_logger().info(f'Gradio Server: {self.gradio_url}')
        self.get_logger().info(f'Listening on: {self.get_parameter("audio_topic").value}')
        self.get_logger().info(f'Publishing to: {self.get_parameter("intent_topic").value}')

    def connect_to_gradio(self):
        """Connect to the Gradio server"""
        try:
            self.get_logger().info(f'Connecting to Gradio server: {self.gradio_url}')
            self.client = Client(self.gradio_url)
            self.get_logger().info('✓ Connected to Gradio server successfully!')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Gradio: {e}')
            self.client = None
            return False

    def convert_to_16bit_mono_wav(self, audio_data, input_format='raw_pcm'):
        """
        Convert audio to 16-bit mono WAV format

        Args:
            audio_data: Raw audio bytes or numpy array
            input_format: 'raw_pcm', 'wav', or 'numpy'

        Returns:
            str: Path to temporary WAV file
        """
        try:
            if input_format == 'numpy':
                audio_array = audio_data
            elif input_format == 'wav':
                audio_array, sr = sf.read(io.BytesIO(audio_data))
                if sr != self.sample_rate:
                    self.get_logger().warn(f'Sample rate {sr} != {self.sample_rate}')
                    self.sample_rate = sr
            else:  # raw_pcm
                audio_array = np.frombuffer(audio_data, dtype=np.int16)

            # Convert to mono if stereo
            if len(audio_array.shape) > 1 and audio_array.shape[1] > 1:
                audio_array = np.mean(audio_array, axis=1)

            # Normalize to int16 range
            if audio_array.dtype != np.int16:
                if np.max(np.abs(audio_array)) <= 1.0:
                    audio_array = (audio_array * 32767).astype(np.int16)
                else:
                    audio_array = audio_array.astype(np.int16)

            # Create temporary WAV file
            temp_fd, temp_path = tempfile.mkstemp(suffix='.wav', prefix='jetbot_audio_')
            os.close(temp_fd)

            with wave.open(temp_path, 'wb') as wav_file:
                wav_file.setnchannels(1)  # Mono
                wav_file.setsampwidth(2)  # 16-bit
                wav_file.setframerate(self.sample_rate)
                wav_file.writeframes(audio_array.tobytes())

            return temp_path

        except Exception as e:
            self.get_logger().error(f'Error converting audio: {e}')
            return None

    def send_audio_to_gradio(self, audio_path, text_query="Transcribe and interpret this audio as a command."):
        """
        Send audio to Gradio server and get response

        Args:
            audio_path: Path to WAV file
            text_query: Text prompt for the model

        Returns:
            str: Response text or None on error
        """
        try:
            if not self.client:
                self.get_logger().error('Gradio client not initialized. Attempting to reconnect...')
                if not self.connect_to_gradio():
                    return None

            self.get_logger().info('Sending audio to Gradio server...')

            # Call Gradio API
            result = self.client.predict(
                text_query,
                audio_path,
                self.history,
                api_name=self.api_name
            )

            # Extract response
            if isinstance(result, tuple):
                response = result[0] if len(result) > 0 else ""
            else:
                response = str(result)

            self.get_logger().info(f'Received response: {response}')

            # Update history
            self.history.append([f"[Audio command]", response])

            # Keep history manageable (last 5 exchanges)
            if len(self.history) > 5:
                self.history = self.history[-5:]

            return response

        except Exception as e:
            self.get_logger().error(f'Error sending audio to Gradio: {e}')
            # Try to reconnect on error
            self.connect_to_gradio()
            return None

    def audio_callback(self, msg):
        """
        Callback for audio messages

        Args:
            msg: String message containing audio data (base64 or raw)
        """
        temp_path = None
        try:
            self.get_logger().info('Received audio message')

            # Try to decode as base64 first
            try:
                audio_data = base64.b64decode(msg.data)
                self.get_logger().debug('Decoded base64 audio')
            except:
                # Assume it's raw bytes
                audio_data = msg.data.encode() if isinstance(msg.data, str) else msg.data
                self.get_logger().debug('Using raw audio data')

            # Convert to 16-bit mono WAV file
            temp_path = self.convert_to_16bit_mono_wav(audio_data, input_format='wav')

            if temp_path is None:
                self.get_logger().error('Failed to convert audio')
                return

            # Send to Gradio server
            response_text = self.send_audio_to_gradio(temp_path)

            if response_text:
                # Publish intent
                intent_msg = String()
                intent_msg.data = response_text
                self.intent_pub.publish(intent_msg)
                self.get_logger().info(f'Published intent: {response_text}')

        except Exception as e:
            self.get_logger().error(f'Error in audio callback: {e}')

        finally:
            # Clean up temporary file
            if temp_path and os.path.exists(temp_path):
                try:
                    os.remove(temp_path)
                except:
                    pass


def main(args=None):
    rclpy.init(args=args)
    node = GradioAudioNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
