# Jetbot VLM Navigation Package

ROS2 Humble package for vision-language model (VLM) based navigation using Ollama/LLaVA with behavior trees.

## Features

- **VLM Vision Node**: Processes camera images with LLaVA for scene understanding
- **Behavior Tree Navigation**: py_trees-based decision making
- **Webots Simulation**: Test before deploying to real Jetbot
- **Real-time Obstacle Detection**: VLM-powered navigation decisions

## Prerequisites

```bash
# Install ROS2 Humble dependencies
sudo apt update
sudo apt install ros-humble-cv-bridge ros-humble-vision-opencv

# Install Python dependencies
pip3 install opencv-python requests py_trees

# Install Webots (optional for simulation)
sudo snap install webots

# Ensure Ollama is running with LLaVA model
ollama pull llava
```

## Building

```bash
cd ~/jetbot_vlm_ws
colcon build --packages-select jetbot_vlm_nav
source install/setup.bash
```

## Usage

### 1. Start with camera (real Jetbot or USB camera)

```bash
# Terminal 1: Ensure Ollama is running
ollama serve

# Terminal 2: Launch navigation
ros2 launch jetbot_vlm_nav jetbot_navigation.launch.py
```

### 2. With Webots Simulation

```bash
# After creating Webots world file
ros2 launch jetbot_vlm_nav webots_simulation.launch.py
```

### 3. Manual node testing

```bash
# Terminal 1: VLM Vision Node
ros2 run jetbot_vlm_nav vlm_vision_node.py

# Terminal 2: Behavior Tree Node
ros2 run jetbot_vlm_nav behavior_tree_node.py

# Terminal 3: Monitor topics
ros2 topic echo /vlm/scene_description
ros2 topic echo /vlm/obstacle_detection
```

## Configuration

Edit parameters in launch files:
- `ollama_url`: Ollama server URL (default: http://localhost:11434)
- `model`: VLM model name (default: llava)
- `camera_topic`: Camera topic (default: /camera/image_raw)
- `processing_rate`: VLM query frequency in Hz (default: 2.0)

## Behavior Tree Logic

```
Navigation (Selector)
├── Avoid Obstacle (Sequence)
│   ├── Check Obstacle
│   ├── Stop
│   ├── Find Clear Path
│   └── Turn to Avoid
└── Move Forward (default)
```

## Topics

### Published
- `/cmd_vel` (geometry_msgs/Twist): Robot velocity commands
- `/vlm/scene_description` (std_msgs/String): Scene description from VLM
- `/vlm/obstacle_detection` (std_msgs/String): Obstacle detection info

### Subscribed
- `/camera/image_raw` (sensor_msgs/Image): Camera input

## Jetbot Deployment

1. Install ROS2 Humble on Jetbot
2. Copy this package to Jetbot
3. Ensure Ollama runs on Jetbot or configure remote URL
4. Update camera topic to match Jetbot camera
5. Launch navigation system

## Webots Setup

Create a Webots world file with:
1. Jetbot robot model with differential drive
2. Camera sensor
3. Environment with obstacles
4. ROS2 controller interface

## Next Steps

- [ ] Create Webots world file for Jetbot
- [ ] Fine-tune VLM prompts for better navigation
- [ ] Add more behavior tree conditions
- [ ] Implement SLAM integration
- [ ] Add goal-based navigation
- [ ] Test on physical Jetbot

## License

MIT
