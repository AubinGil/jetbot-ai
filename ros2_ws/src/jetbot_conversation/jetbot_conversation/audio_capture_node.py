#!/usr/bin/env python3
"""
Audio Capture Node with Wake Word Detection and VAD
Uses: Wake word → VAD recording → Whisper ASR → Publish text
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import numpy as np
import sounddevice as sd
import time
import queue
import threading
from typing import Optional

try:
    from faster_whisper import WhisperModel
    WHISPER_AVAILABLE = True
except ImportError:
    WHISPER_AVAILABLE = False

try:
    import webrtcvad
    VAD_AVAILABLE = True
except ImportError:
    VAD_AVAILABLE = False

class AudioCaptureNode(Node):
    def __init__(self):
        super().__init__('audio_capture_node')
        
        # Parameters
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('wake_word', 'hello')
        self.declare_parameter('awake_duration', 15.0)
        self.declare_parameter('vad_aggressiveness', 2)
        self.declare_parameter('whisper_model', 'tiny.en')
        self.declare_parameter('enable_wake_word', True)
        self.declare_parameter('max_recording_duration', 10.0)
        self.declare_parameter('barge_in_enabled', True)
        self.declare_parameter('barge_in_required_frames', 3)
        self.declare_parameter('barge_in_reset_silence', 12)
        self.declare_parameter('barge_in_adaptive_enabled', True)
        self.declare_parameter('barge_in_adaptive_delay', 5.0)
        self.declare_parameter('barge_in_adaptive_min_frames', 1)
        self.declare_parameter('barge_in_explicit_commands', True)

        self.sample_rate = self.get_parameter('sample_rate').value
        self.wake_word = self.get_parameter('wake_word').value.lower()
        self.awake_duration = self.get_parameter('awake_duration').value
        self.vad_aggressiveness = self.get_parameter('vad_aggressiveness').value
        self.whisper_model_name = self.get_parameter('whisper_model').value
        self.enable_wake_word = self.get_parameter('enable_wake_word').value
        self.max_recording_duration = self.get_parameter('max_recording_duration').value
        self.barge_in_enabled = self.get_parameter('barge_in_enabled').value
        self.barge_in_required_frames = self.get_parameter('barge_in_required_frames').value
        self.barge_in_reset_silence = self.get_parameter('barge_in_reset_silence').value
        self.barge_in_adaptive_enabled = self.get_parameter('barge_in_adaptive_enabled').value
        self.barge_in_adaptive_delay = self.get_parameter('barge_in_adaptive_delay').value
        self.barge_in_adaptive_min_frames = self.get_parameter('barge_in_adaptive_min_frames').value
        self.barge_in_explicit_commands = self.get_parameter('barge_in_explicit_commands').value

        # State
        self.is_awake = not self.enable_wake_word
        self.last_wake_time = 0
        self.audio_queue = queue.Queue()
        self.is_recording = False
        self.tts_active = False
        self.tts_resume_at = 0.0  # time until which we ignore mic after TTS
        self.barge_in_active_frames = 0
        self.barge_in_silence_frames = 0
        self.barge_in_requested = False
        self.tts_start_time = 0.0  # Track when TTS started for adaptive thresholding
        self.explicit_stop_buffer = []  # Buffer for explicit command detection
        self.frame_blocksize = int(self.sample_rate * 0.03)

        # Publishers
        self.text_pub = self.create_publisher(String, 'stt/text', 10)
        self.awake_pub = self.create_publisher(Bool, 'stt/awake', 10)
        self.interrupt_pub = self.create_publisher(Bool, 'tts/interrupt', 10)
        
        # Subscribers
        self.create_subscription(Bool, 'tts/playing', self.handle_tts_state, 10)
        
        # Initialize Whisper
        if WHISPER_AVAILABLE:
            self.get_logger().info(f'Loading Whisper model: {self.whisper_model_name}')
            self.whisper_model = WhisperModel(
                self.whisper_model_name,
                device='cpu',
                compute_type='int8',
                num_workers=1,
                cpu_threads=2
            )
            self.get_logger().info('✓ Whisper model loaded (CPU mode)')
        else:
            self.get_logger().error('faster-whisper not available! Install: pip3 install faster-whisper')
            self.whisper_model = None
        
        # Initialize VAD
        if VAD_AVAILABLE:
            self.vad = webrtcvad.Vad(self.vad_aggressiveness)
            self.get_logger().info(f'✓ VAD initialized (aggressiveness: {self.vad_aggressiveness})')
        else:
            self.get_logger().warn('webrtcvad not available. Install: pip3 install webrtcvad')
            self.vad = None
        
        # Start audio stream
        self.start_audio_stream()
        
        # Timer for wake word timeout
        self.create_timer(1.0, self.check_wake_timeout)
        
        self.get_logger().info('🎤 Audio Capture Node initialized!')
        if self.enable_wake_word:
            self.get_logger().info(f'   Wake word: "{self.wake_word}"')
        else:
            self.get_logger().info('   Always listening (wake word disabled)')
    
    def start_audio_stream(self):
        """Start audio input stream"""
        try:
            self.stream = sd.InputStream(
                channels=1,
                samplerate=self.sample_rate,
                dtype=np.int16,
                blocksize=self.frame_blocksize,
                callback=self.audio_callback
            )
            self.stream.start()
            self.get_logger().info('✓ Audio stream started')
        except Exception as e:
            self.get_logger().error(f'Failed to start audio stream: {e}')
            self.stream = None
    
    def audio_callback(self, indata, frames, time_info, status):
        """Audio stream callback"""
        if status:
            self.get_logger().warn(f'Audio status: {status}')
        
        # Add to processing queue (flatten to 1D)
        frame = indata.flatten().copy()
        self.audio_queue.put(frame)
        self.evaluate_barge_in(frame)

    def evaluate_barge_in(self, frame: np.ndarray) -> bool:
        """Run VAD while TTS plays to trigger a barge-in with adaptive thresholding."""
        if not (self.tts_active and self.barge_in_enabled and self.vad):
            return False
        if len(frame) != self.frame_blocksize:
            return False

        try:
            is_speech = self.vad.is_speech(frame.tobytes(), self.sample_rate)
        except Exception:
            return False

        if is_speech:
            self.barge_in_active_frames += 1
            self.barge_in_silence_frames = 0

            # Add audio to explicit command buffer
            if self.barge_in_explicit_commands:
                self.explicit_stop_buffer.append(frame)
                # Keep buffer at reasonable size (1.5 seconds of audio)
                max_buffer_frames = int(1.5 / 0.03)
                if len(self.explicit_stop_buffer) > max_buffer_frames:
                    self.explicit_stop_buffer.pop(0)
        else:
            self.barge_in_silence_frames += 1

        # Calculate adaptive threshold based on TTS duration
        required_frames = self.barge_in_required_frames
        if self.barge_in_adaptive_enabled and self.tts_start_time > 0:
            tts_duration = time.time() - self.tts_start_time
            if tts_duration >= self.barge_in_adaptive_delay:
                # Gradually reduce threshold for long responses
                progress = min(1.0, (tts_duration - self.barge_in_adaptive_delay) / 5.0)
                required_frames = int(
                    self.barge_in_required_frames -
                    (self.barge_in_required_frames - self.barge_in_adaptive_min_frames) * progress
                )
                required_frames = max(self.barge_in_adaptive_min_frames, required_frames)

        # Check if we've exceeded threshold
        if self.barge_in_active_frames >= required_frames:
            # Check for explicit stop commands if enabled
            if self.barge_in_explicit_commands and len(self.explicit_stop_buffer) > 5:
                if self.detect_explicit_stop_command():
                    self.get_logger().info('🛑 Explicit stop command detected')
                    self.reset_barge_in_state()
                    self.publish_barge_in_interrupt(reason='explicit_command')
                    return True

            self.reset_barge_in_state()
            tts_duration = time.time() - self.tts_start_time if self.tts_start_time > 0 else 0
            self.publish_barge_in_interrupt(reason=f'speech_detected_after_{tts_duration:.1f}s')
            return True

        # Reset counter after prolonged silence
        if self.barge_in_silence_frames >= self.barge_in_reset_silence:
            self.barge_in_active_frames = 0
            self.explicit_stop_buffer = []

        return False

    def detect_explicit_stop_command(self) -> bool:
        """Quick check for explicit stop commands like 'stop', 'enough', etc."""
        if not self.explicit_stop_buffer or not WHISPER_AVAILABLE:
            return False

        try:
            # Concatenate recent audio frames
            audio_data = np.concatenate(self.explicit_stop_buffer[-20:])  # Last ~0.6 seconds
            audio_data = audio_data.astype(np.float32)

            # Quick transcription with Whisper
            segments, _ = self.whisper_model.transcribe(
                audio_data,
                language='en',
                vad_filter=False,
                beam_size=1  # Fast, greedy decoding
            )

            text = ' '.join([seg.text.strip().lower() for seg in segments])

            # Check for stop phrases
            stop_phrases = ['stop', 'enough', 'that\'s enough', 'quiet', 'silence', 'shut up']
            for phrase in stop_phrases:
                if phrase in text:
                    self.get_logger().info(f'Detected stop phrase: "{text}"')
                    return True

        except Exception as e:
            self.get_logger().debug(f'Explicit command detection error: {e}')

        return False

    def publish_barge_in_interrupt(self, reason='speech_detected'):
        """Ask the TTS node to stop playback immediately."""
        if not (self.tts_active and self.barge_in_enabled):
            return

        if self.barge_in_requested:
            return

        msg = Bool()
        msg.data = True
        self.interrupt_pub.publish(msg)
        self.barge_in_requested = True
        self.get_logger().info(f'🛑 Barge-in triggered: {reason}')

    def reset_barge_in_state(self):
        """Reset counters used for barge-in detection."""
        self.barge_in_active_frames = 0
        self.barge_in_silence_frames = 0
        self.explicit_stop_buffer = []
    
    def handle_tts_state(self, msg: Bool):
        """Pause or resume ASR depending on TTS playback state."""
        playing = bool(msg.data)
        if playing == self.tts_active:
            return

        self.tts_active = playing
        if self.tts_active:
            self.tts_start_time = time.time()  # Track when TTS started
            self.get_logger().info('🔇 Pausing ASR during TTS playback')
            self.flush_audio_queue()
            self.reset_barge_in_state()
        else:
            # After TTS ends, wait a longer grace period to avoid capturing tail audio and echoes
            # Increased from 0.4s to 1.0s to allow speaker/microphone buffers to flush
            self.tts_resume_at = time.time() + 1.0
            if self.barge_in_requested:
                self.get_logger().info('⚡ Barge-in detected, resuming immediately')
            else:
                self.flush_audio_queue()
            self.barge_in_requested = False
            self.reset_barge_in_state()
            self.get_logger().info('🎙️ Resuming ASR after TTS playback')
    
    def check_wake_timeout(self):
        """Check if wake word has timed out"""
        if self.enable_wake_word and self.is_awake:
            if time.time() - self.last_wake_time > self.awake_duration:
                self.is_awake = False
                self.publish_awake_status(False)
                self.get_logger().info('😴 Going back to sleep (timeout)')
    
    def process_audio(self):
        """Process audio from queue"""
        frames = []
        silence_frames = 0
        max_silence_frames = 20  # ~0.6 seconds of silence
        
        recording_start = time.time()
        
        while time.time() - recording_start < self.max_recording_duration:
            if self.tts_active:
                return None
            try:
                frame = self.audio_queue.get(timeout=0.1)
                frames.append(frame)
                
                # Check for voice activity
                if self.vad and len(frame) == int(self.sample_rate * 0.03):
                    # Convert to bytes for VAD
                    frame_bytes = frame.tobytes()
                    is_speech = self.vad.is_speech(frame_bytes, self.sample_rate)
                    
                    if is_speech:
                        silence_frames = 0
                    else:
                        silence_frames += 1
                        
                    # Stop if long silence after speech
                    if len(frames) > 10 and silence_frames > max_silence_frames:
                        break
                
            except queue.Empty:
                silence_frames += 1
                if silence_frames > max_silence_frames:
                    break
        
        if frames:
            # Concatenate audio
            audio_data = np.concatenate(frames)
            
            # Transcribe with Whisper
            if self.whisper_model:
                text = self.transcribe_audio(audio_data)
                if text:
                    return text
        
        return None
    
    def transcribe_audio(self, audio_data: np.ndarray) -> Optional[str]:
        """Transcribe audio using Whisper"""
        try:
            # Convert to float32 and normalize
            audio_float = audio_data.astype(np.float32) / 32768.0
            
            # Transcribe
            segments, info = self.whisper_model.transcribe(
                audio_float,
                language='en',
                beam_size=1,
                vad_filter=True,
            )
            
            # Extract text
            text = ' '.join([segment.text for segment in segments]).strip()
            
            if text:
                self.get_logger().info(f'📝 Transcribed: "{text}"')
                return text
            
        except Exception as e:
            self.get_logger().error(f'Transcription failed: {e}')
        
        return None
    
    def check_for_wake_word(self, text: str) -> bool:
        """Check if text contains wake word"""
        return self.wake_word in text.lower()
    
    def publish_awake_status(self, awake: bool):
        """Publish awake status"""
        msg = Bool()
        msg.data = awake
        self.awake_pub.publish(msg)
    
    def publish_text(self, text: str):
        """Publish transcribed text"""
        msg = String()
        msg.data = text
        self.text_pub.publish(msg)
        self.get_logger().info(f'📤 Published: "{text}"')
    
    def flush_audio_queue(self):
        """Clear any buffered audio frames."""
        try:
            while True:
                self.audio_queue.get_nowait()
        except queue.Empty:
            pass
    
    def spin_once(self):
        """Custom spin to process audio"""
        # If TTS is active or we're in the post-TTS grace period, skip processing
        if self.tts_active or (self.tts_resume_at and time.time() < self.tts_resume_at):
            return
        # Check for audio to process
        if not self.audio_queue.empty():
            if not self.is_awake:
                # Listen for wake word
                text = self.process_audio()
                if text and self.check_for_wake_word(text):
                    self.is_awake = True
                    self.last_wake_time = time.time()
                    self.publish_awake_status(True)
                    self.get_logger().info(f'👂 Wake word detected: "{self.wake_word}"')
                    # Publish everything after wake word
                    remaining = text.split(self.wake_word, 1)[-1].strip()
                    if remaining:
                        self.publish_text(remaining)
            else:
                # Already awake, process command
                text = self.process_audio()
                if text:
                    self.publish_text(text)
                    self.last_wake_time = time.time()  # Reset timeout
    
    def destroy_node(self):
        """Cleanup"""
        if hasattr(self, 'stream') and self.stream:
            self.stream.stop()
            self.stream.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AudioCaptureNode()
    
    try:
        while rclpy.ok():
            node.spin_once()
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
