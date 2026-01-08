# Launch Scripts

Main startup scripts for different JetBot AI modes.

## Available Scripts

### `start_conversation_complete.sh`
**Full voice conversation system**

- Wake word detection ("hello")
- Speech-to-text (Whisper)
- Vision-language understanding (Cosmos/Ollama)
- Text-to-speech (Magpie/Piper)
- Motor control for voice commands
- Camera integration

**Usage:**
```bash
./launch/start_conversation_complete.sh
```

**Options:**
```bash
# Enable voice mode controller (gesture, autopilot switching)
export MODE_CONTROL_ENABLED=1

# Change language
export CONVERSATION_LANGUAGE=fr  # French
export CONVERSATION_LANGUAGE=en  # English (default)

# Dry run (test without starting)
./launch/start_conversation_complete.sh --dry-run

# With teleop
./launch/start_conversation_complete.sh --teleop keyboard
```

**Example Commands:**
- "hello, what do you see?"
- "hello, move forward"
- "hello, follow me" (starts gesture mode)

---

### `start_vila_with_lidar_enhanced.sh`
**Autonomous navigation with vision AI and LiDAR safety**

- VILA vision-language model for scene understanding
- RPLidar obstacle detection
- Smart stuck detection and recovery
- Optional TTS narration
- Adaptive transformer controller
- Wall following mode

**Usage:**
```bash
./launch/start_vila_with_lidar_enhanced.sh
```

**Options:**
```bash
# Enable narration
export TTS=1
export NARRATION_LANGUAGE=en  # or 'fr'

# Tune navigation
export LIN_SPEED=0.20           # Linear speed (m/s)
export ANG_SPEED=0.45           # Angular speed (rad/s)
export SAFETY_DISTANCE=0.15     # Emergency stop distance
export WARNING_DISTANCE=0.25    # Slow down distance

# Enable advanced features
export ADAPTIVE_TRANSFORMER_ENABLED=true
export WALL_FOLLOW_ENABLED=true

# Use external server
export COSMOS_HOST=http://192.168.2.16:8000
```

**Features:**
- Automatic dead-end detection → 180° turnaround
- Tight space navigation
- Scene narration every 60 seconds (with TTS=1)
- Fallback to local Ollama if remote unavailable

---

### `start_gesture_complete.sh`
**Human-following mode with gesture recognition**

- YOLO11-pose detection (GPU accelerated)
- Person tracking and following
- Gesture recognition (wave to activate)
- Safe following distance
- Motor control with smooth velocity ramping

**Usage:**
```bash
./launch/start_gesture_complete.sh
```

**How to Use:**
1. Start script
2. Wave at camera
3. Robot will detect and follow you
4. Stop gesture or voice "stop" to halt

**Tuning:**
```bash
# Following distance
export TARGET_DISTANCE=1.0  # meters

# Speed limits
export MAX_LINEAR=0.3       # m/s
export MAX_ANGULAR=0.6      # rad/s
```

**Requirements:**
- Camera: /dev/video0
- Motors: /dev/ttyACM0
- GPU: For YOLO11-pose inference

---

### `start_obstacle_nav_gpu2.sh`
**GPU-accelerated obstacle avoidance**

- ONNX runtime for fast inference
- MobileNet obstacle detection
- Tight space navigation optimization
- Optional Cosmos reasoning layer
- Real-time collision prevention

**Usage:**
```bash
./launch/start_obstacle_nav_gpu2.sh
```

**Features:**
- ~15 FPS obstacle detection on Jetson
- Trained for indoor environments
- Reactive navigation (no map required)
- Confidence-based decision making

**Configuration:**
```bash
# Detection sensitivity
export CONFIDENCE_THRESHOLD=0.7

# Navigation behavior
export LIN_SPEED=0.20
export ANG_SPEED=0.40
```

---

## Environment Variables

All scripts support these common variables:

```bash
# Device paths
export CAMERA_DEV=/dev/video0
export MOTOR_DEV=/dev/ttyACM0
export LIDAR_DEV=/dev/ttyUSB0

# ROS2 settings
export ROS_DOMAIN_ID=37

# Remote servers (optional)
export COSMOS_HOST=http://192.168.2.16:8000
export OLLAMA_REMOTE_HOST=http://192.168.2.29:11434
```

Set in `.env` file or export before running.

## Stopping Scripts

All scripts can be stopped with:
- `Ctrl+C` in terminal
- Kill all: `pkill -f ros2`
- Emergency stop: `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once`

## Troubleshooting

### Script won't start
- Source ROS2: `source /opt/ros/humble/setup.bash`
- Source workspace: `source ~/jetbot-ai/ros2_ws/install/setup.bash`
- Check permissions: Device paths must be accessible

### Hardware not found
- Camera: `ls -l /dev/video*`
- LiDAR: `ls -l /dev/ttyUSB*`
- Motors: `ls -l /dev/ttyACM*`

### Poor performance
- Reduce camera resolution
- Use smaller models
- Close other applications

See [docs/troubleshooting.md](../docs/troubleshooting.md) for more help.

## Combining Modes

You can combine scripts by running in separate terminals:

**Example: Conversation + Gesture**
```bash
# Terminal 1: Start gesture following
./launch/start_gesture_complete.sh

# Terminal 2: Start conversation (without camera/hardware)
# Edit script to skip hardware launch
```

**Note**: Be careful not to launch conflicting hardware nodes (e.g., two camera nodes).

## Next Steps

- Tune parameters for your environment
- Customize wake words and voices
- Train custom models for better accuracy
- Create your own launch scripts combining features

---

For detailed documentation, see:
- [docs/navigation.md](../docs/navigation.md)
- [docs/conversation.md](../docs/conversation.md)
