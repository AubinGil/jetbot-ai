# Conversation System Guide

Complete guide to voice control, speech recognition, and natural language interaction with JetBot AI.

## Overview

The conversation system enables natural voice interaction with the robot using:
- **Wake word detection** ("hello") - always listening
- **Speech-to-text** (Whisper) - converts voice to text
- **LLM reasoning** (Cosmos/Ollama) - understands commands and answers questions
- **Text-to-speech** (Magpie/Piper) - responds with voice

## Quick Start

### Basic Conversation Mode

```bash
cd ~/jetbot-ai
./launch/start_conversation_complete.sh
```

Say: **"hello, what do you see?"**

Robot will:
1. Detect wake word "hello"
2. Listen to your command
3. Process with vision (camera)
4. Respond: "I see a table and chairs in front of me"

### Available Commands

**Vision Commands:**
- "hello, what do you see?"
- "hello, describe the scene"
- "hello, what's in front of you?"

**Movement Commands:**
- "hello, move forward"
- "hello, turn left / turn right"
- "hello, stop"
- "hello, back up"

**Mode Switching:**
- "hello, follow me" → Starts gesture following
- "hello, autopilot" → Starts autonomous navigation
- "hello, stop mode" → Stops current mode

**Conversation:**
- "hello, what time is it?"
- "hello, tell me a joke"
- "hello, what's the weather?"

## Configuration

### Main Config File

Edit `ros2_ws/src/jetbot_conversation/config/conversation.yaml`:

```yaml
conversation_node:
  ros__parameters:
    # Primary LLM (Cosmos on remote GPU)
    primary_ollama_host: 'http://192.168.2.16:8000'
    primary_ollama_model: 'nvidia/Cosmos-Reason1-7B'
    primary_ollama_vision_model: 'nvidia/Cosmos-Reason1-7B'
    primary_api_type: 'openai'  # OpenAI-compatible API

    # Fallback (local Jetson)
    fallback_ollama_host: 'http://localhost:11434'
    fallback_ollama_model: 'granite4:latest'
    fallback_ollama_vision_model: 'granite3.2-vision:latest'

    # Conversation settings
    context_memory_size: 5  # Remember last 5 exchanges

    # Motion settings
    linear_speed: 0.25      # m/s
    angular_speed: 0.8      # rad/s
    command_timeout: 5.0    # Auto-stop after 5 seconds
```

### Audio Capture Settings

```yaml
audio_capture_node:
  ros__parameters:
    # Audio
    sample_rate: 16000

    # Wake word
    wake_word: 'hello'      # Change to your preference
    enable_wake_word: true
    awake_duration: 15.0    # Stay awake for 15 seconds after wake

    # VAD (Voice Activity Detection)
    vad_aggressiveness: 2   # 0-3, higher = more aggressive
    max_recording_duration: 10.0  # Maximum recording length

    # Whisper ASR
    whisper_model: 'tiny.en'  # tiny.en, base.en, small.en
```

### TTS Settings

```yaml
tts_node:
  ros__parameters:
    enable_tts: true
    speech_rate: 1.0        # 0.5-2.0

    # Magpie TTS (cloud, high quality)
    prefer_magpie: true
    magpie_voice: 'Magpie-Multilingual.EN-US.Aria'

    # Piper TTS (offline fallback)
    piper_bin: '/home/gildas01/piper_wrapper.sh'
    piper_model: '/home/gildas01/en_US-lessac-medium.onnx'
```

## Language Support

### English (Default)

```bash
export CONVERSATION_LANGUAGE=en
./launch/start_conversation_complete.sh
```

Wake word: **"hello"**
Voice: Aria (Magpie) or Lessac (Piper)

### French

```bash
export CONVERSATION_LANGUAGE=fr
./launch/start_conversation_complete.sh
```

Wake word: **"bonjour"**
Voice: Pascal (Magpie) or French Piper model

Config: `conversation_french.yaml`

## Speech Recognition (Whisper)

### Model Selection

Trade-off between speed and accuracy:

| Model | Speed | Accuracy | RAM Usage |
|-------|-------|----------|-----------|
| tiny.en | Fastest | Good | ~400MB |
| base.en | Fast | Better | ~500MB |
| small.en | Moderate | Best | ~1GB |

Change in `conversation.yaml`:
```yaml
whisper_model: 'base.en'  # or 'small.en'
```

### Wake Word Detection

Default wake word: **"hello"**

To change:
```yaml
wake_word: 'jarvis'  # or any word
```

Wake word alternatives:
- "hello" - most reliable
- "hey robot"
- "jarvis"
- "bonjour" (French)

### Voice Activity Detection (VAD)

Detects when you start/stop speaking.

Aggressiveness levels:
- **0** - Very lenient (captures more noise)
- **1** - Lenient
- **2** - Moderate (recommended)
- **3** - Aggressive (may cut off speech)

```yaml
vad_aggressiveness: 2
```

## Text-to-Speech (TTS)

### Magpie TTS (Primary)

Cloud-based, high-quality voices.

**Pros:**
- Natural, human-like voices
- Multi-language support
- Expressive intonation

**Cons:**
- Requires internet
- Requires API token

**Available Voices:**
- English: Aria, Danny, Judy, Stella
- French: Pascal
- See: [NVIDIA Magpie](https://build.nvidia.com/nvidia/magpie-multilingual-tts)

Setup:
```yaml
prefer_magpie: true
magpie_voice: 'Magpie-Multilingual.EN-US.Aria'
magpie_token: 'your-token-here'
```

Get free token at: https://build.nvidia.com

### Piper TTS (Fallback)

Offline, ONNX-based speech synthesis.

**Pros:**
- Works offline
- Fast inference
- No API required

**Cons:**
- Less natural than Magpie
- Limited voices

**Available Voices:**
Download from https://github.com/rhasspy/piper/releases
- en_US-lessac-medium
- en_US-amy-medium
- fr_FR-upmc-medium

Setup:
```bash
# Download model
wget https://github.com/rhasspy/piper/releases/download/v1.2.0/voice-en-us-lessac-medium.tar.gz
tar -xzf voice-en-us-lessac-medium.tar.gz

# Configure
piper_model: '/path/to/en_US-lessac-medium.onnx'
```

## LLM Models

### Cosmos (Recommended)

Best for vision-language reasoning.

**Models:**
- Cosmos-Reason1-7B (7B parameters)
- Cosmos-Reason2-8B (8B parameters)

Setup (remote GPU server):
```yaml
primary_ollama_host: 'http://192.168.2.16:8000'
primary_ollama_model: 'nvidia/Cosmos-Reason1-7B'
primary_api_type: 'openai'
```

### Ollama Models (Local)

Run on Jetson for offline operation.

**Text models:**
- granite4:latest (4B, fast, good quality)
- llama3.2 (3B, very fast)
- llama3.1:8b (8B, slower, better)

**Vision models:**
- granite3.2-vision:latest (lighter, faster)
- llava (heavier, better vision)

Pull models:
```bash
ollama pull granite4
ollama pull granite3.2-vision
```

### Model Selection Strategy

**Priority cascade:**
1. Try primary (Cosmos on remote GPU)
2. If failed → Fallback to local Ollama
3. If vision needed → Use vision model

This is automatic in the conversation node.

## Voice Mode Controller

Switches robot modes via voice commands.

### Enable Mode Controller

```bash
export MODE_CONTROL_ENABLED=1
./launch/start_conversation_complete.sh
```

### Available Mode Commands

**"follow me"**
- Starts gesture-based following
- Robot detects and follows person
- Maintains safe distance

**"autopilot"**
- Starts autonomous obstacle avoidance
- Robot navigates freely
- Avoids obstacles

**"stop" or "stop mode"**
- Halts active mode
- Returns to conversation mode

### How It Works

```
Voice: "hello, follow me"
    ↓
Conversation node detects command
    ↓
voice_mode_controller.py receives ROS message
    ↓
Launches start_gesture_complete.sh
    ↓
Robot enters following mode
```

## Advanced Features

### Context Memory

Robot remembers previous conversation:

```
You: "hello, what do you see?"
Robot: "I see a door in front of me"

You: "hello, is it open?"
Robot: "Yes, the door appears to be open"  # Remembers "door"
```

Controlled by:
```yaml
context_memory_size: 5  # Last 5 exchanges
```

### Tool Integration

Robot can use external tools:

**Time:**
```
You: "hello, what time is it?"
Robot: "It's 3:45 PM"
```

**Weather:**
```
You: "hello, what's the weather?"
Robot: "Currently 72°F and sunny"
```

**Web Search:**
```
You: "hello, who won the game yesterday?"
Robot: *searches web* "The Lakers won 105-98"
```

Enable in `conversation_node.py`:
```python
from jetbot_conversation.tools import ToolRegistry
tools = ToolRegistry()
```

### Barge-In (Disabled by Default)

Interrupt robot while it's speaking.

Currently disabled due to sensitivity. To re-enable:

```yaml
barge_in_enabled: true
barge_in_required_frames: 15  # Higher = less sensitive
```

## Monitoring & Debugging

### Check Audio Levels

```bash
# Record test
arecord -f S16_LE -r 16000 -d 5 test.wav
aplay test.wav

# Monitor input
alsamixer
# Press F4, adjust capture volume
```

### Monitor ROS Topics

```bash
# Audio capture
ros2 topic echo /audio/transcript

# Conversation responses
ros2 topic echo /conversation/response

# TTS output
ros2 topic echo /tts/speak
```

### Enable Debug Logging

Edit conversation node to add:
```python
self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
```

### Test Components Individually

**Test Whisper ASR:**
```bash
ros2 run jetbot_conversation audio_capture_node
# Speak into microphone, check /audio/transcript
```

**Test TTS:**
```bash
ros2 run jetbot_conversation tts_node
ros2 topic pub /tts/speak std_msgs/msg/String "{data: 'Hello world'}"
```

**Test LLM:**
```bash
ollama run granite4 "What do you see in this image?" < image.jpg
```

## Troubleshooting

### Wake word not detected

- Check microphone connection: `arecord -l`
- Test recording: `arecord -d 5 test.wav && aplay test.wav`
- Verify wake word in config
- Speak clearly, moderate pace
- Check microphone is default in `.asoundrc`

### Robot doesn't understand commands

- Use simple, direct commands
- Wait for wake word acknowledgment before speaking
- Check `whisper_model` - try `base.en` for better accuracy
- Verify LLM is running: `curl http://localhost:11434/api/tags`

### No TTS output

- Check speaker connection: `aplay -l`
- Test speaker: `aplay /usr/share/sounds/alsa/Front_Center.wav`
- Verify TTS node running: `ros2 node list`
- Check `enable_tts: true` in config
- If Magpie fails, ensure Piper fallback is configured

### Robot responds to background noise

- Increase `vad_aggressiveness` to 3
- Reduce microphone gain in `alsamixer`
- Use directional microphone
- Enable `enable_wake_word: true`

### Latency (slow responses)

- Use faster Whisper model (`tiny.en`)
- Use local Ollama instead of remote
- Reduce `context_memory_size`
- Use smaller LLM (granite4 vs llama3.1:8b)

### "Connection refused" to Ollama

```bash
# Check Ollama is running
curl http://localhost:11434/api/tags

# Start Ollama if needed
ollama serve &
```

## Performance Optimization

### Reduce Latency

1. **Use tiny Whisper model**: `whisper_model: 'tiny.en'`
2. **Local Ollama**: Avoid remote server delays
3. **Smaller LLM**: granite4 (4B) vs llama3.1:8b
4. **Disable tools**: Remove web search, weather if not needed

### Improve Accuracy

1. **Larger Whisper model**: `small.en` (requires more RAM)
2. **Better LLM**: llama3.1:8b or remote Cosmos
3. **Increase context**: `context_memory_size: 10`
4. **Better microphone**: Noise-canceling USB mic

## Example Conversations

### Vision Query
```
You: "hello, what do you see?"
Robot: "I see a living room with a couch, coffee table, and TV"

You: "hello, what color is the couch?"
Robot: "The couch appears to be gray or dark blue"
```

### Navigation Command
```
You: "hello, move forward slowly"
Robot: *starts moving forward*

You: "hello, stop"
Robot: *stops immediately*
```

### Mode Switching
```
You: "hello, follow me"
Robot: "Starting gesture following mode"
*Robot detects person and follows*

You: "hello, stop"
Robot: "Stopping following mode"
```

### General Conversation
```
You: "hello, tell me a joke"
Robot: "Why did the robot go to therapy? It had trouble processing its emotions!"

You: "hello, what's your favorite color?"
Robot: "As a robot, I don't have preferences, but I think blue is a nice color!"
```

## Next Steps

- Customize wake word and voices
- Add custom commands in conversation node
- Integrate with navigation modes
- Train custom Whisper model for your voice
- Add new tools (smart home, calendar, etc.)

---

For navigation integration, see [navigation.md](navigation.md).
