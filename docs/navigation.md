# Navigation System Guide

Comprehensive guide to autonomous navigation, obstacle avoidance, and SLAM on JetBot AI.

## Navigation Modes

The JetBot supports multiple navigation modes:

1. **Vision-Language Navigation (VILA)** - AI-guided exploration
2. **Obstacle Avoidance** - Reactive collision prevention
3. **Gesture Following** - Human-following mode
4. **SLAM Navigation** - Map-based path planning
5. **Wall Following** - Line/wall-following navigation

## 1. Vision-Language Navigation (VILA)

Uses vision-language models to understand scenes and make navigation decisions.

### Start VILA Navigation

```bash
cd ~/jetbot-ai
./launch/start_vila_with_lidar_enhanced.sh
```

### Features

- **Scene Understanding**: Describes environment in natural language
- **LiDAR Safety Layer**: Prevents collisions even if vision fails
- **Smart Recovery**: Detects dead-ends and executes 180° turnarounds
- **TTS Narration**: Optional scene narration (set `TTS=1`)

### Configuration

Edit `launch/start_vila_with_lidar_enhanced.sh`:

```bash
# Speed settings
LIN_SPEED=0.20          # Forward speed (m/s)
ANG_SPEED=0.45          # Turn speed (rad/s)

# Safety distances
SAFETY_DISTANCE=0.15    # Emergency stop (meters)
WARNING_DISTANCE=0.25   # Slow down (meters)

# Detection cones
FRONT_CONE_DEG=35.0     # Forward obstacle detection angle
SIDE_CONE_DEG=45.0      # Side obstacle detection angle

# Recovery behavior
STUCK_THRESHOLD=4       # Cycles before stuck detection
TURNAROUND_ANGLE_DEG=185.0  # Turnaround rotation
```

### How It Works

```
1. Capture camera frame
2. Send to VILA model → "path is clear ahead"
3. Check LiDAR for obstacles
4. If clear: move forward
5. If obstacle: turn to find clear path
6. If stuck (4+ cycles): execute turnaround
7. Repeat
```

## 2. GPU-Accelerated Obstacle Avoidance

Uses YOLO object detection and ONNX inference for real-time collision avoidance.

### Start Obstacle Avoidance

```bash
cd ~/jetbot-ai
./launch/start_obstacle_nav_gpu2.sh
```

### Features

- **ONNX Runtime**: Fast inference (~15 FPS on Jetson)
- **Tight Space Navigation**: Optimized for indoor environments
- **Multi-Tier Detection**: MobileNet + Optional Cosmos reasoning

### Configuration

The script uses pre-trained obstacle detection models:
- `~/jetbot-ai/models/onnx/obstacle_avoidance_model.onnx`
- Trained on JetBot camera perspective

### Training Custom Models

If you want to train for your environment:

```bash
# Collect training data
cd ~/jetbot-ai/scripts/utils
python3 collect_collision_data.py

# Train model
python3 train_obstacle_model.py \
  --data_dir ./training_data \
  --epochs 50 \
  --batch_size 32

# Export to ONNX
python3 export_obstacle_model.py \
  --model obstacle_model.pth \
  --output models/onnx/obstacle_avoidance_model.onnx
```

## 3. Gesture Following

GPU-accelerated pose detection for human-following behavior.

### Start Gesture Mode

```bash
cd ~/jetbot-ai
./launch/start_gesture_complete.sh
```

### Gestures

- **Wave**: Robot approaches and follows
- **Stop gesture**: Robot stops
- **Point**: Robot moves in pointed direction

### How It Works

```
1. YOLO11-pose detects human keypoints
2. Calculates person position and gesture
3. Publishes /cmd_vel for following behavior
4. Safety layer prevents collision
```

### Tuning

Edit `launch/start_gesture_complete.sh`:

```bash
# Following distance
TARGET_DISTANCE=1.0     # meters

# Speed limits
MAX_LINEAR=0.3         # m/s
MAX_ANGULAR=0.6        # rad/s
```

## 4. SLAM Navigation

Build maps and navigate to waypoints using SLAM Toolbox and Nav2.

### Start SLAM Mapping

```bash
cd ~/jetbot-ai/ros2_ws
source install/setup.bash

# Start SLAM
ros2 launch jetbot_vlm_nav jetbot_slam_toolbox.launch.py
```

### Save Map

```bash
# In another terminal
ros2 run nav2_map_server map_saver_cli -f my_map
```

Creates:
- `my_map.pgm` - Occupancy grid image
- `my_map.yaml` - Map metadata

### Navigate with Map

```bash
# Load map and start Nav2
ros2 launch jetbot_vlm_nav jetbot_nav_with_map.launch.py \
  map:=my_map.yaml
```

Use RViz2 to send navigation goals:
1. Open RViz2
2. Click "2D Nav Goal" button
3. Click destination on map

### SLAM Configuration

Edit `ros2_ws/src/jetbot_hardware/config/slam_params.yaml`:

```yaml
slam_toolbox:
  ros__parameters:
    # Resolution
    resolution: 0.05  # 5cm grid cells

    # Solver
    solver_plugin: solver_plugins::CeresSolver

    # Loop closing
    do_loop_closing: true
    loop_search_maximum_distance: 3.0
```

## 5. Wall/Line Following

Simple line-following for indoor corridors.

### Start Wall Following

```bash
cd ~/jetbot-ai/ros2_ws
source install/setup.bash

ros2 launch jetbot_wall_line_nav wall_line_nav.launch.py
```

### Configuration

Edit `ros2_ws/src/jetbot_wall_line_nav/config/nav_params.yaml`:

```yaml
wall_line_navigator:
  ros__parameters:
    target_distance: 0.3  # Distance from wall (meters)
    linear_speed: 0.2     # Forward speed
    angular_gain: 0.5     # Turn correction strength
```

## Advanced Features

### Adaptive Transformer Controller

Uses IBM Granite TTM-R2 time-series model for predictive velocity smoothing.

Enable in VILA navigation:

```bash
export ADAPTIVE_TRANSFORMER_ENABLED=true
./launch/start_vila_with_lidar_enhanced.sh
```

Configuration:

```bash
TRANSFORMER_CONTEXT_LENGTH=64      # History window
TRANSFORMER_PREDICTION_HORIZON=10  # Lookahead steps
TRANSFORMER_UPDATE_RATE=5.0        # Hz
```

### Wall Following (with LiDAR)

Follows walls at consistent distance using LiDAR feedback.

```bash
export WALL_FOLLOW_ENABLED=true
export WALL_FOLLOW_TARGET_DISTANCE=0.35
./launch/start_vila_with_lidar_enhanced.sh
```

### Scene Narration

Robot narrates what it sees (requires TTS):

```bash
export TTS=1
export NARRATION_ENABLED=true
export NARRATION_INTERVAL=60.0  # seconds
./launch/start_vila_with_lidar_enhanced.sh
```

Narration languages:
- English: `NARRATION_LANGUAGE=en`
- French: `NARRATION_LANGUAGE=fr`

## Performance Tuning

### Speed vs Safety Trade-offs

**Conservative (safe, slow)**:
```bash
LIN_SPEED=0.15
WARNING_DISTANCE=0.40
WARN_SPEED_SCALE=0.70
```

**Balanced (recommended)**:
```bash
LIN_SPEED=0.20
WARNING_DISTANCE=0.25
WARN_SPEED_SCALE=0.90
```

**Aggressive (fast, risky)**:
```bash
LIN_SPEED=0.30
WARNING_DISTANCE=0.20
WARN_SPEED_SCALE=0.95
```

### Inference Speed

- **ONNX models**: ~15 FPS (recommended)
- **PyTorch models**: ~5 FPS
- **TensorRT**: ~30 FPS (requires conversion)

To improve speed:
1. Use ONNX runtime (already default)
2. Reduce camera resolution (640x480 → 320x240)
3. Lower inference frequency (skip frames)

## Monitoring Navigation

### Visualize in RViz2

```bash
# In separate terminal
ros2 run rviz2 rviz2
```

Add displays:
- **LaserScan** → /scan (LiDAR data)
- **Image** → /camera/image_raw
- **RobotModel** → Robot visualization
- **Map** → /map (if using SLAM)
- **Path** → /plan (if using Nav2)

### Monitor Topics

```bash
# Camera feed
ros2 topic hz /camera/image_raw

# LiDAR scan
ros2 topic echo /scan

# Motor commands
ros2 topic echo /cmd_vel

# Obstacle detections
ros2 topic echo /obstacle_distance
```

## Troubleshooting Navigation

### Robot spins in circles

- Increase `STUCK_THRESHOLD` to prevent false positives
- Check LiDAR is mounted level
- Verify camera field of view

### Robot doesn't avoid obstacles

- Check LiDAR connection: `ros2 topic hz /scan`
- Verify SAFETY_DISTANCE and WARNING_DISTANCE
- Test LiDAR: `ros2 run rplidar_ros rplidar_composition`

### Robot moves too slowly

- Increase `LIN_SPEED` and `ANG_SPEED`
- Reduce `WARNING_DISTANCE` for less conservative behavior
- Check `WARN_SPEED_SCALE` (closer to 1.0 = faster)

### Navigation gets stuck

- Enable turnaround recovery: `STUCK_THRESHOLD=4`
- Increase `TURNAROUND_ANGLE_DEG` for full 180° turns
- Check for low clearance areas (robot might be too large)

### SLAM map quality poor

- Reduce driving speed during mapping
- Increase `resolution` in slam_params.yaml (smaller = better)
- Drive in slow, deliberate loops for loop closure
- Ensure good lighting for visual features

## Safety Considerations

1. **Always test in open space first**
2. **Start with low speeds** (0.1-0.15 m/s)
3. **Keep emergency stop accessible** (keyboard, joystick)
4. **Monitor first runs** - be ready to intervene
5. **Check LiDAR safety layer** - critical for obstacle avoidance
6. **Avoid stairs and ledges** - no cliff detection by default

## Emergency Stop

To stop immediately:

```bash
# Publish zero velocity
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" --once

# Or kill all nodes
pkill -f ros2
```

## Next Steps

- Tune parameters for your environment
- Train custom obstacle models
- Build maps of your space
- Combine modes (e.g., SLAM + vision reasoning)

---

For conversation integration with navigation, see [conversation.md](conversation.md).
