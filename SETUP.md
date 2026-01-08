# Installation and Setup Guide

Step-by-step instructions to set up the JetBot AI platform on NVIDIA Jetson Orin Nano.

## Prerequisites

- NVIDIA Jetson Orin Nano with JetPack 6.x installed
- Ubuntu 22.04 (Jammy)
- Hardware assembled per [HARDWARE.md](HARDWARE.md)
- Internet connection
- 20GB+ free disk space

## Installation Steps

### 1. System Preparation

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install essential tools
sudo apt install -y git curl wget build-essential python3-pip

# Add user to required groups
sudo usermod -a -G dialout,video,audio $USER

# Reboot to apply group changes
sudo reboot
```

### 2. Install ROS2 Humble

```bash
# Set locale
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-dev-tools

# Source ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. Install ROS2 Dependencies

```bash
# Navigation and SLAM
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-common

# Sensors
sudo apt install -y \
  ros-humble-v4l2-camera \
  ros-humble-rplidar-ros \
  ros-humble-cv-bridge \
  ros-humble-image-transport

# Utilities
sudo apt install -y \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-xacro
```

### 4. Clone and Build JetBot AI Workspace

```bash
# Clone repository
cd ~
git clone https://github.com/AubinGil/jetbot-ai.git
cd jetbot-ai

# Build ROS2 workspace
cd ros2_ws
colcon build --symlink-install

# Source workspace
echo "source ~/jetbot-ai/ros2_ws/install/setup.bash" >> ~/.bashrc
source install/setup.bash
```

### 5. Install Python Dependencies

```bash
# Install pip packages
pip3 install --upgrade pip

# Core ML/AI packages
pip3 install \
  torch torchvision torchaudio \
  transformers \
  accelerate \
  opencv-python \
  numpy \
  pillow \
  scipy

# Speech and audio
pip3 install \
  openai-whisper \
  soundfile \
  pyaudio \
  webrtcvad

# Web and utilities
pip3 install \
  requests \
  gradio \
  fastapi \
  uvicorn

# Optional: TensorRT and ONNX (if not using JetPack defaults)
pip3 install onnxruntime-gpu
```

### 6. Install Ollama (Local LLM)

```bash
# Install Ollama
curl -fsSL https://ollama.com/install.sh | sh

# Start Ollama service
ollama serve &

# Pull required models
ollama pull granite4:latest
ollama pull granite3.2-vision:latest

# Optional: Additional models
ollama pull llama3.2
ollama pull llava
```

### 7. Download Model Files

```bash
# Create models directory
mkdir -p ~/jetbot-ai/models/onnx

# Download YOLO models (example)
cd ~/jetbot-ai/models/onnx
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolo11n.onnx
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolo11n-pose.onnx

# Download Piper TTS model (offline speech)
cd ~
wget https://github.com/rhasspy/piper/releases/download/v1.2.0/voice-en-us-lessac-medium.tar.gz
tar -xzf voice-en-us-lessac-medium.tar.gz
# Model files: en_US-lessac-medium.onnx, en_US-lessac-medium.onnx.json
```

### 8. Configure Audio

```bash
# Create ALSA configuration
cat > ~/.asoundrc << 'EOF'
pcm.!default {
    type asym
    playback.pcm "plughw:1,0"  # Adjust card number for your speaker
    capture.pcm "plughw:3,0"   # Adjust card number for your microphone
}
EOF

# Test audio
aplay -l  # List playback devices
arecord -l  # List capture devices

# Record test
arecord -f S16_LE -r 16000 -d 5 test.wav
aplay test.wav
```

### 9. Setup Swap (for larger models)

```bash
# Create 2GB swap file
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Make permanent
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

### 10. Environment Configuration

```bash
# Create environment variables file
cat > ~/jetbot-ai/.env << 'EOF'
# ROS2 Configuration
export ROS_DOMAIN_ID=37
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Local Ollama
export OLLAMA_HOST=http://localhost:11434

# Optional: Remote Cosmos server (if available)
# export COSMOS_HOST=http://192.168.2.16:8000
# export COSMOS_MODEL=nvidia/Cosmos-Reason1-7B

# Optional: Remote Ollama
# export OLLAMA_REMOTE_HOST=http://192.168.2.29:11434

# Language (en or fr)
export CONVERSATION_LANGUAGE=en

# Device paths
export CAMERA_DEV=/dev/video0
export MOTOR_DEV=/dev/ttyACM0
export LIDAR_DEV=/dev/ttyUSB0
EOF

# Source environment
echo "source ~/jetbot-ai/.env" >> ~/.bashrc
source ~/jetbot-ai/.env
```

## Optional: Remote GPU Server Setup

If you have a separate PC with GPU for running Cosmos models:

### On Remote PC (Windows/Linux with RTX GPU)

```bash
# Install vLLM
pip install vllm

# Start Cosmos server
vllm serve nvidia/Cosmos-Reason1-7B \
  --host 0.0.0.0 \
  --port 8000 \
  --max-model-len 4096
```

### On Jetson

```bash
# Test connection
curl http://192.168.2.16:8000/v1/models

# Set environment variable
export COSMOS_HOST=http://192.168.2.16:8000
```

## Verification

### Test ROS2 Installation

```bash
# Source workspace
cd ~/jetbot-ai/ros2_ws
source install/setup.bash

# List packages
ros2 pkg list | grep jetbot

# Expected output:
# jetbot_conversation
# jetbot_hardware
# jetbot_vlm_nav
# jetbot_wall_line_nav
# jetbot_webrtc_teleop
# qwen2_audio_interface
# robot_face_display
```

### Test Hardware Connections

```bash
# Test camera
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video0

# Test LiDAR (in another terminal)
ros2 run rplidar_ros rplidar_composition --ros-args \
  -p serial_port:=/dev/ttyUSB0

# Test motors (publishes cmd_vel)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}" --once
```

### Test Ollama

```bash
# Check Ollama is running
curl http://localhost:11434/api/tags

# Test model
ollama run granite4 "Hello, how are you?"
```

### Test Vision Models

```bash
# Test with image
ollama run granite3.2-vision "Describe this image" < test_image.jpg
```

## Running the System

### Full Conversation System

```bash
cd ~/jetbot-ai
./launch/start_conversation_complete.sh
```

Say "hello" (wake word) then give commands:
- "hello, what do you see?"
- "hello, move forward"
- "hello, turn left"

### Autonomous Navigation

```bash
cd ~/jetbot-ai
./launch/start_vila_with_lidar_enhanced.sh
```

Robot will navigate autonomously using vision and LiDAR.

### Gesture Following

```bash
cd ~/jetbot-ai
./launch/start_gesture_complete.sh
```

Wave at the camera, robot will follow.

### GPU Obstacle Avoidance

```bash
cd ~/jetbot-ai
./launch/start_obstacle_nav_gpu2.sh
```

Real-time obstacle detection and avoidance.

## Configuration

### Conversation Settings

Edit `ros2_ws/src/jetbot_conversation/config/conversation.yaml`:

```yaml
conversation_node:
  ros__parameters:
    # Primary LLM (Cosmos or local)
    primary_ollama_host: 'http://localhost:11434'
    primary_ollama_model: 'granite4:latest'

    # Motion speeds
    linear_speed: 0.25  # m/s
    angular_speed: 0.8  # rad/s

    # Wake word
    wake_word: 'hello'
```

### Navigation Settings

Edit `ros2_ws/src/jetbot_vlm_nav/config/nav2_params.yaml` for Nav2 parameters.

Edit launch scripts for obstacle avoidance tuning:
- `launch/start_vila_with_lidar_enhanced.sh` - LiDAR thresholds
- `launch/start_obstacle_nav_gpu2.sh` - Detection sensitivity

## Troubleshooting

### "Command not found" errors
```bash
# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source ~/jetbot-ai/ros2_ws/install/setup.bash
```

### Permission denied for /dev/ttyUSB0 or /dev/ttyACM0
```bash
sudo usermod -a -G dialout $USER
sudo reboot
```

### Ollama not responding
```bash
# Restart Ollama
pkill ollama
ollama serve &
```

### Camera not found
```bash
# Check device
ls -l /dev/video*

# Try different USB port
# Update CAMERA_DEV in .env file
```

### Out of memory errors
```bash
# Check swap is enabled
free -h

# Increase swap if needed
sudo swapoff /swapfile
sudo fallocate -l 4G /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

### Build errors
```bash
# Clean and rebuild
cd ~/jetbot-ai/ros2_ws
rm -rf build install log
colcon build --symlink-install
```

## Performance Optimization

### For faster inference:

1. **Use ONNX models** instead of PyTorch when available
2. **Enable TensorRT** for YOLO models
3. **Use remote GPU** for large models (Cosmos)
4. **Reduce camera resolution** if processing is slow (320x240)
5. **Lower LLM context length** in configs

### For better accuracy:

1. **Use larger models** (granite4 → llama3.2:8b)
2. **Increase camera resolution** (640x480 → 1280x720)
3. **Add remote Cosmos** for reasoning tasks
4. **Increase SLAM resolution** in slam_params.yaml

## Next Steps

1. Read [docs/conversation.md](docs/conversation.md) for voice control details
2. Read [docs/navigation.md](docs/navigation.md) for autonomous navigation
3. Customize configuration files for your setup
4. Train custom models for your environment

## Support

- Issues: https://github.com/AubinGil/jetbot-ai/issues
- Documentation: https://github.com/AubinGil/jetbot-ai/tree/main/docs

---

**Installation Complete!** Your JetBot AI is ready to use.
