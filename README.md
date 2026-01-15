
<img width="1024" height="1536" alt="642497b0-943b-4307-8f24-38b6598c33a3" src="https://github.com/user-attachments/assets/ea923781-ba2e-4293-8b84-da4ffced13db" />



# JetBot AI - Advanced Autonomous Robot Platform

An intelligent mobile robot platform combining vision-language models, autonomous navigation, and natural voice interaction on NVIDIA Jetson.

## Features

- **Multi-Modal AI Conversation**: Voice-controlled interface with wake word detection, speech recognition (Whisper), and text-to-speech
- **Vision-Language Navigation**: Autonomous navigation using VILA/Cosmos models with real-time scene understanding
- **Advanced Obstacle Avoidance**: LiDAR fusion, YOLO object detection, and adaptive collision prevention
- **Gesture Recognition**: GPU-accelerated pose detection for human-following and gesture commands
- **SLAM Mapping**: Real-time mapping and localization for autonomous navigation
- **Multi-Language Support**: English and French conversation modes

## Quick Start

### Prerequisites
- NVIDIA Jetson Orin Nano (8GB)
- Ubuntu 22.04 + ROS2 Humble
- RPLidar A1/A2/A3
- USB camera, microphone, and speaker

### Basic Setup

```bash
# Clone repository
git clone https://github.com/AubinGil/jetbot-ai.git
cd jetbot-ai

# Build ROS2 workspace
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# Start conversation system
cd ..
./launch/start_conversation_complete.sh
```

### Available Launch Modes

```bash
# Voice conversation with camera
./launch/start_conversation_complete.sh

# Autonomous navigation with vision
./launch/start_vila_with_lidar_enhanced.sh

# Gesture-based following
./launch/start_gesture_complete.sh

# GPU-accelerated obstacle avoidance
./launch/start_obstacle_nav_gpu2.sh
```

## Architecture

```
┌─────────────────────────────────────────────────┐
│  Audio Input (Wake Word: "hello")              │
│  ↓                                              │
│  Whisper ASR → LLM (Cosmos/Ollama) → Commands  │
│  ↓                                              │
│  Magpie/Piper TTS → Audio Output               │
└─────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────┐
│  Camera → Vision Models (VILA/Cosmos/YOLO)      │
│  ↓                                              │
│  Navigation Decision Engine                     │
│  ↓                                              │
│  LiDAR Safety Layer → Motor Control (ROS2)     │
└─────────────────────────────────────────────────┘
```

## Hardware Requirements

- **Compute**: NVIDIA Jetson Orin Nano (8GB RAM minimum)
- **Sensors**:
  - RPLidar A1/A2/A3 (115200 baud, /dev/ttyUSB0)
  - USB Camera (640x480, /dev/video0)
  - USB Microphone
  - USB Speaker
- **Motors**: Waveshare motor controller (/dev/ttyACM0)
- **Network**: Optional remote GPU server for Cosmos models

See [HARDWARE.md](HARDWARE.md) for complete specifications and wiring.

## Software Stack

- **ROS2**: Humble
- **Vision**: VILA, Cosmos-Reason1/2, YOLO11, Qwen2-VL
- **LLM**: Ollama (local), OpenAI-compatible API (remote)
- **Speech**: Whisper (ASR), Magpie/Piper (TTS)
- **Navigation**: Nav2, SLAM Toolbox, LiDAR processing
- **Frameworks**: PyTorch, ONNX Runtime, TensorRT

See [SETUP.md](SETUP.md) for complete installation instructions.

## Documentation

- **[HARDWARE.md](HARDWARE.md)** - Hardware requirements and wiring guide
- **[SETUP.md](SETUP.md)** - Step-by-step installation instructions
- **[docs/navigation.md](docs/navigation.md)** - Autonomous navigation and SLAM
- **[docs/conversation.md](docs/conversation.md)** - Voice control and conversation setup
- **[docs/troubleshooting.md](docs/troubleshooting.md)** - Common issues and solutions

## Project Structure

```
jetbot-ai/
├── ros2_ws/          # ROS2 workspace with custom packages
├── scripts/          # Standalone Python scripts
│   ├── vision/       # VILA, Cosmos integration
│   ├── navigation/   # Obstacle avoidance, LiDAR
│   ├── conversation/ # Voice control, TTS/STT
│   └── utils/        # Utilities and tools
├── launch/           # Main startup scripts
├── models/           # Model configurations
└── docs/             # Detailed documentation
```

## Key ROS2 Packages

- **jetbot_conversation**: Multi-modal conversation pipeline
- **jetbot_hardware**: Motor control, camera, sensors
- **jetbot_vlm_nav**: Vision-language navigation
- **robot_face_display**: Animated robot face UI
- **jetbot_wall_line_nav**: Line-following navigation
- **jetbot_webrtc_teleop**: Remote teleoperation

## Configuration

Main configuration files:
- `ros2_ws/src/jetbot_conversation/config/conversation.yaml` - Conversation settings
- `ros2_ws/src/jetbot_hardware/config/slam_params.yaml` - SLAM parameters
- `ros2_ws/src/jetbot_vlm_nav/config/nav2_params.yaml` - Navigation tuning

## Environment Variables

```bash
# Optional: Remote Cosmos server
export COSMOS_HOST=http://192.168.2.16:8000
export COSMOS_MODEL=nvidia/Cosmos-Reason1-7B

# Optional: Remote Ollama server
export OLLAMA_REMOTE_HOST=http://192.168.2.29:11434

# Language selection
export CONVERSATION_LANGUAGE=en  # or 'fr' for French
```

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

## License

[Add your license here]

## Acknowledgments

- NVIDIA for Jetson platform and Isaac ROS
- Ollama for local LLM inference
- Open-source robotics community

## Author

[Aubin Gil](https://github.com/AubinGil)

---

**Status**: Active Development | **Platform**: Jetson Orin Nano | **ROS**: Humble
