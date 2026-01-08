#!/usr/bin/env python3
"""
Text-to-Speech Node using Magpie (primary) and Piper (fallback)
Subscribes to: /tts/speak
Plays audio output
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import subprocess
import tempfile
import os
import threading
import queue
import sys
import socket
import time

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        
        # Parameters
        self.declare_parameter('piper_bin', 'piper')
        self.declare_parameter('piper_model', os.path.expanduser('~/en_US-lessac-medium.onnx'))
        self.declare_parameter('enable_tts', True)
        self.declare_parameter('speech_rate', 1.0)
        self.declare_parameter('magpie_script', os.path.expanduser('~/python-clients/scripts/tts/talk.py'))
        self.declare_parameter('magpie_voice', 'Magpie-Multilingual.EN-US.Aria')
        self.declare_parameter('magpie_language_code', 'en-US')
        self.declare_parameter('magpie_token', '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')
        self.declare_parameter('prefer_magpie', True)
        
        self.piper_bin = self.get_parameter('piper_bin').value
        self.piper_model = self.get_parameter('piper_model').value
        self.enable_tts = self.get_parameter('enable_tts').value
        self.speech_rate = self.get_parameter('speech_rate').value
        self.magpie_script = self.get_parameter('magpie_script').value
        self.magpie_voice = self.get_parameter('magpie_voice').value
        self.magpie_language_code = self.get_parameter('magpie_language_code').value
        self.magpie_token = self.get_parameter('magpie_token').value
        self.prefer_magpie = self.get_parameter('prefer_magpie').value
        
        # Check which TTS engines are available
        self.magpie_available = self.check_magpie()
        self.piper_available = self.check_piper()
        
        # Speech queue (to avoid overlapping speech)
        self.speech_queue = queue.Queue()
        self.is_speaking = False
        self.stop_playback_event = threading.Event()
        self.playback_lock = threading.Lock()
        self.current_playback = None

        # Publishers / Subscribers
        self.tts_state_pub = self.create_publisher(Bool, 'tts/playing', 10)
        self.create_subscription(String, 'tts/speak', self.handle_speak_request, 10)
        self.create_subscription(Bool, 'tts/interrupt', self.handle_interrupt, 10)
        
        # Start speech worker thread
        self.speech_thread = threading.Thread(target=self.speech_worker, daemon=True)
        self.speech_thread.start()
        
        self.get_logger().info('🔊 TTS Node initialized!')
        if self.magpie_available and self.prefer_magpie:
            self.get_logger().info(f'   Primary: Magpie ({self.magpie_voice})')
            if self.piper_available:
                self.get_logger().info(f'   Fallback: Piper')
        elif self.piper_available:
            self.get_logger().info(f'   Using: Piper')
        else:
            self.get_logger().warn('   No TTS available - TTS disabled')
    
    def check_magpie(self) -> bool:
        """Check if Magpie is available"""
        try:
            # Check if magpie conda env exists and script exists
            magpie_bin = os.path.expanduser('~/magpie/bin')
            activate_script = os.path.join(magpie_bin, 'activate')
            
            if not os.path.exists(activate_script):
                self.get_logger().info('Magpie conda environment not found')
                return False
            
            if not os.path.exists(self.magpie_script):
                self.get_logger().info(f'Magpie script not found: {self.magpie_script}')
                return False
            
            self.get_logger().info('✓ Magpie TTS available')
            return True
            
        except Exception as e:
            self.get_logger().info(f'Magpie check failed: {e}')
            return False
    
    def check_piper(self) -> bool:
        """Check if Piper is installed and model exists"""
        try:
            result = subprocess.run(
                [self.piper_bin, '--version'],
                capture_output=True,
                timeout=2
            )

            if result.returncode != 0:
                self.get_logger().warn('Piper binary not found')
                return False

            if not os.path.exists(self.piper_model):
                self.get_logger().warn(f'Piper model not found: {self.piper_model}')
                return False

            return True

        except Exception as e:
            self.get_logger().warn(f'Piper check failed: {e}')
            return False

    def check_internet_connectivity(self) -> bool:
        """Check if internet is available (for Magpie TTS)"""
        try:
            # Try to connect to a reliable DNS server with short timeout
            socket.create_connection(("8.8.8.8", 53), timeout=2)
            return True
        except (socket.timeout, socket.error, OSError):
            return False
    
    def handle_speak_request(self, msg: String):
        """Handle TTS request"""
        text = msg.data.strip()
        if not text:
            return
        
        if not self.enable_tts:
            self.get_logger().info(f'🔇 TTS disabled: "{text}"')
            return
        
        if not self.magpie_available and not self.piper_available:
            self.get_logger().warn(f'Cannot speak (no TTS available): "{text}"')
            return
        
        # Add to queue
        self.speech_queue.put(text)
        self.get_logger().info(f'📢 Queued: "{text}"')
    
    def speech_worker(self):
        """Worker thread to process speech queue"""
        while True:
            try:
                text = self.speech_queue.get(timeout=1.0)
                self.speak(text)
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Speech worker error: {e}')
    
    def speak(self, text: str):
        """Synthesize and play speech"""
        self.is_speaking = True
        self.stop_playback_event.clear()
        # Signal ASR to pause and give it a moment to flush
        self.publish_tts_state(True)
        try:
            # Increased delay from 0.15s to 0.3s to ensure ASR fully pauses
            time.sleep(0.3)
        except Exception:
            pass

        try:
            # Check if we should try Magpie (available, preferred, and online)
            use_magpie = False
            if self.magpie_available and self.prefer_magpie:
                if self.check_internet_connectivity():
                    use_magpie = True
                else:
                    self.get_logger().info('No internet - using Piper (offline TTS)')

            # Try Magpie if conditions are met
            if use_magpie:
                if self.speak_magpie(text):
                    self.get_logger().info(f'🔊 Spoke (Magpie): "{text}"')
                    return
                else:
                    self.get_logger().warn('Magpie failed, falling back to Piper')

            # Fall back to Piper
            if self.piper_available:
                self.speak_piper(text)
                self.get_logger().info(f'🔊 Spoke (Piper): "{text}"')

        except Exception as e:
            self.get_logger().error(f'TTS error: {e}')
        finally:
            # Keep ASR muted briefly after playback to avoid tail capture and echo
            # Increased from 0.25s to 0.5s to allow audio buffers to fully drain
            try:
                time.sleep(0.5)
            except Exception:
                pass
            self.stop_playback_event.clear()
            self.is_speaking = False
            self.publish_tts_state(False)

    def publish_tts_state(self, playing: bool):
        """Publish whether the TTS system is currently playing audio."""
        msg = Bool()
        msg.data = playing
        self.tts_state_pub.publish(msg)
    
    def speak_magpie(self, text: str) -> bool:
        """Synthesize speech using Magpie"""
        try:
            # Create temporary file for audio
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp_file:
                output_path = tmp_file.name
            
            # Build Magpie command
            magpie_conda = os.path.expanduser('~/magpie/bin/python')
            magpie_cmd = [
                magpie_conda,
                self.magpie_script,
                '--server', 'grpc.nvcf.nvidia.com:443',
                '--use-ssl',
                '--metadata', 'function-id', '877104f7-e885-42b9-8de8-f6e4c6303969',
                '--metadata', 'authorization', f'Bearer {self.magpie_token}',
                '--language-code', self.magpie_language_code,
                '--text', text,
                '--voice', self.magpie_voice,
                '--output', output_path
            ]
            
            # Synthesize
            result = subprocess.run(
                magpie_cmd,
                capture_output=True,
                timeout=60
            )
            
            if result.returncode != 0:
                self.get_logger().warn(f'Magpie synthesis failed: {result.stderr.decode()}')
                os.unlink(output_path)
                return False
            
            # Play audio
            self.play_audio(output_path)
            
            # Cleanup
            os.unlink(output_path)
            return True
            
        except Exception as e:
            self.get_logger().warn(f'Magpie error: {e}')
            return False
    
    def speak_piper(self, text: str):
        """Synthesize speech using Piper"""
        try:
            # Create temporary file for audio
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp_file:
                output_path = tmp_file.name
            
            # Run Piper
            piper_cmd = [
                self.piper_bin,
                '--model', self.piper_model,
                '--output_file', output_path,
            ]
            
            if self.speech_rate != 1.0:
                piper_cmd.extend(['--length_scale', str(1.0 / self.speech_rate)])
            
            # Synthesize
            result = subprocess.run(
                piper_cmd,
                input=text.encode('utf-8'),
                capture_output=True,
                timeout=30
            )
            
            if result.returncode != 0:
                self.get_logger().error(f'Piper synthesis failed: {result.stderr.decode()}')
                return
            
            # Play audio
            self.play_audio(output_path)
            
            # Cleanup
            os.unlink(output_path)
            
        except Exception as e:
            self.get_logger().error(f'Piper error: {e}')

    def handle_interrupt(self, msg: Bool):
        """Stop playback when a barge-in request arrives."""
        if not msg.data:
            return
        if not self.is_speaking:
            return
        self.get_logger().info('🛑 Barge-in request received, stopping TTS')
        self.stop_playback_event.set()
        self.terminate_playback()

    def terminate_playback(self, proc=None):
        """Terminate any running playback process."""
        with self.playback_lock:
            target = proc or self.current_playback
            if not target:
                return
            try:
                target.terminate()
                target.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                target.kill()
            except Exception as exc:
                self.get_logger().debug(f'Failed to stop playback: {exc}')
            finally:
                if self.current_playback is target:
                    self.current_playback = None

    def play_audio(self, audio_path: str):
        """Play audio file with interrupt support."""
        players = [
            (['aplay', '-q', audio_path], 'aplay'),
            (['paplay', audio_path], 'paplay'),
        ]

        for cmd, name in players:
            if self.stop_playback_event.is_set():
                break

            proc = None
            try:
                with self.playback_lock:
                    proc = subprocess.Popen(
                        cmd,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
                    self.current_playback = proc

                while proc.poll() is None:
                    if self.stop_playback_event.is_set():
                        self.terminate_playback(proc)
                        break
                    time.sleep(0.05)

                if self.stop_playback_event.is_set():
                    break

                if proc.returncode == 0:
                    return
            except FileNotFoundError:
                self.get_logger().warn(f'{name} not found, skipping playback')
            except Exception as exc:
                self.get_logger().error(f'Audio playback failed ({name}): {exc}')
            finally:
                with self.playback_lock:
                    if self.current_playback is proc:
                        self.current_playback = None

        self.stop_playback_event.clear()

def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
