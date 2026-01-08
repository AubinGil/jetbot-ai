#!/usr/bin/env python3
"""
ROS2 Node for Qwen2-Audio Voice Interface
Subscribes to audio, sends to llama.cpp server, publishes text intent
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import base64
import io
import wave
import numpy as np
import soundfile as sf
import json


class AudioIntentNode(Node):
    def __init__(self):
        super().__init__('audio_intent_node')

        # Parameters
        self.declare_parameter('server_url', 'http://localhost:8080/v1/chat/completions')
        self.declare_parameter('model_name', 'qwen2audio')
        self.declare_parameter('audio_topic', '/mic/audio')
        self.declare_parameter('intent_topic', '/audio_intent')
        self.declare_parameter('sample_rate', 16000)

        self.server_url = self.get_parameter('server_url').value
        self.model_name = self.get_parameter('model_name').value
        self.sample_rate = self.get_parameter('sample_rate').value

        # Subscriber to audio data (expecting raw audio or String with base64)
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

        self.get_logger().info(f'Audio Intent Node started')
        self.get_logger().info(f'Server: {self.server_url}')
        self.get_logger().info(f'Listening on: {self.get_parameter("audio_topic").value}')
        self.get_logger().info(f'Publishing to: {self.get_parameter("intent_topic").value}')

    def convert_to_16bit_mono_wav(self, audio_data, input_format='raw_pcm'):
        """
        Convert audio to 16-bit mono WAV format

        Args:
            audio_data: Raw audio bytes or numpy array
            input_format: 'raw_pcm', 'wav', or 'numpy'

        Returns:
            bytes: WAV file data
        """
        try:
            if input_format == 'numpy':
                # audio_data is already numpy array
                audio_array = audio_data
            elif input_format == 'wav':
                # Parse WAV file
                audio_array, sr = sf.read(io.BytesIO(audio_data))
                if sr != self.sample_rate:
                    self.get_logger().warn(f'Sample rate {sr} != {self.sample_rate}, using input rate')
                    self.sample_rate = sr
            else:  # raw_pcm
                # Assume 16-bit PCM
                audio_array = np.frombuffer(audio_data, dtype=np.int16)

            # Convert to mono if stereo
            if len(audio_array.shape) > 1 and audio_array.shape[1] > 1:
                audio_array = np.mean(audio_array, axis=1)

            # Normalize to int16 range
            if audio_array.dtype != np.int16:
                if np.max(np.abs(audio_array)) <= 1.0:
                    # Float audio in range [-1, 1]
                    audio_array = (audio_array * 32767).astype(np.int16)
                else:
                    audio_array = audio_array.astype(np.int16)

            # Create WAV file in memory
            wav_buffer = io.BytesIO()
            with wave.open(wav_buffer, 'wb') as wav_file:
                wav_file.setnchannels(1)  # Mono
                wav_file.setsampwidth(2)  # 16-bit
                wav_file.setframerate(self.sample_rate)
                wav_file.writeframes(audio_array.tobytes())

            wav_buffer.seek(0)
            return wav_buffer.read()

        except Exception as e:
            self.get_logger().error(f'Error converting audio: {e}')
            return None

    def send_audio_to_server(self, wav_data):
        """
        Send audio to llama.cpp server and get response

        Args:
            wav_data: WAV file bytes

        Returns:
            str: Response text or None on error
        """
        try:
            # Base64 encode the WAV data
            audio_base64 = base64.b64encode(wav_data).decode('utf-8')

            # Prepare request payload (OpenAI-compatible format)
            payload = {
                "model": self.model_name,
                "messages": [
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "input_audio",
                                "audio_url": f"data:audio/wav;base64,{audio_base64}"
                            }
                        ]
                    }
                ]
            }

            # Send request
            self.get_logger().info('Sending audio to server...')
            response = requests.post(
                self.server_url,
                json=payload,
                headers={'Content-Type': 'application/json'},
                timeout=30
            )

            if response.status_code == 200:
                result = response.json()
                # Extract text from response
                if 'choices' in result and len(result['choices']) > 0:
                    message = result['choices'][0].get('message', {})
                    content = message.get('content', '')
                    self.get_logger().info(f'Received response: {content}')
                    return content
                else:
                    self.get_logger().error('Unexpected response format')
                    return None
            else:
                self.get_logger().error(f'Server error: {response.status_code} - {response.text}')
                return None

        except requests.exceptions.Timeout:
            self.get_logger().error('Request timeout')
            return None
        except Exception as e:
            self.get_logger().error(f'Error sending audio: {e}')
            return None

    def audio_callback(self, msg):
        """
        Callback for audio messages

        Args:
            msg: String message containing audio data (base64 or raw)
        """
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

            # Convert to 16-bit mono WAV
            wav_data = self.convert_to_16bit_mono_wav(audio_data, input_format='wav')

            if wav_data is None:
                self.get_logger().error('Failed to convert audio')
                return

            # Send to server
            response_text = self.send_audio_to_server(wav_data)

            if response_text:
                # Publish intent
                intent_msg = String()
                intent_msg.data = response_text
                self.intent_pub.publish(intent_msg)
                self.get_logger().info(f'Published intent: {response_text}')

        except Exception as e:
            self.get_logger().error(f'Error in audio callback: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = AudioIntentNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
