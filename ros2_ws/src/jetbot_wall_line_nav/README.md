# JetBot Wall & Line Navigation

Lightweight wall detection and line following package for indoor navigation using OpenCV.

## Features

- **Line Following**: Detects and follows colored lines using HSV color space
- **Wall Detection**: Uses edge detection to identify walls and avoid collisions
- **Hybrid Navigation**: Combines line following with wall avoidance
- **Lightweight**: Pure OpenCV implementation - no heavy deep learning models
- **Configurable**: Easy parameter tuning via YAML configuration

## Requirements

- ROS 2 Humble
- Python 3.10
- OpenCV (cv2)
- cv_bridge

## Installation

```bash
cd ~/jetbot_vlm_ws
colcon build --packages-select jetbot_wall_line_nav
source install/setup.bash
```

## Usage

### Basic Launch

```bash
ros2 launch jetbot_wall_line_nav wall_line_nav.launch.py
```

### With Custom Camera Topic

```bash
ros2 launch jetbot_wall_line_nav wall_line_nav.launch.py camera_topic:=/your/camera/topic
```

### Run Node Directly

```bash
ros2 run jetbot_wall_line_nav wall_line_navigator
```

## Configuration

Edit `config/nav_params.yaml` to adjust:

- **Line color detection**: Modify HSV color ranges for your line color
- **Speed limits**: Adjust max_linear_speed and max_angular_speed
- **Wall sensitivity**: Change min_wall_distance (pixels)
- **Control gains**: Tune line_follow_kp for better tracking

### Common Line Colors

**Yellow Line:**
```yaml
line_color_lower_h: 20
line_color_upper_h: 30
```

**White Line:**
```yaml
line_color_lower_h: 0
line_color_upper_h: 180
line_color_lower_s: 0
line_color_upper_s: 30
line_color_lower_v: 200
line_color_upper_v: 255
```

**Red Line:**
```yaml
line_color_lower_h: 0
line_color_upper_h: 10
```

## Topics

### Subscribed
- `/camera/image_raw` (sensor_msgs/Image): Camera feed

### Published
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands
- `/wall_line_nav/debug_image` (sensor_msgs/Image): Visualization of detection

## How It Works

### Line Following
1. Converts image to HSV color space
2. Applies color mask to detect line
3. Finds contours and calculates centroid
4. Computes error from image center
5. Generates proportional steering command

### Wall Detection
1. Converts image to grayscale
2. Applies Gaussian blur
3. Performs Canny edge detection
4. Analyzes edge density in left/right margins
5. Triggers avoidance if walls too close

### Navigation Logic
- **Normal**: Follows line using proportional control
- **Wall Left**: Turns right to avoid
- **Wall Right**: Turns left to avoid
- **Both Walls**: Stops/reverses

## Future Enhancements (Stereo Camera)

When you add a stereo camera:
- Replace edge-based wall detection with depth mapping
- Add 3D obstacle avoidance
- Improve distance estimation
- Enable 3D path planning

## Troubleshooting

**Line not detected:**
- Check HSV color ranges match your line
- Verify camera topic is correct
- Check lighting conditions
- Adjust `min_line_area` parameter

**Wall detection too sensitive:**
- Increase `min_wall_distance`
- Adjust `wall_edge_threshold`

**Robot moves too fast/slow:**
- Adjust `max_linear_speed` and `max_angular_speed`
- Modify `line_follow_kp` gain

## Debug Visualization

View the debug image to see what the system detects:

```bash
ros2 run rqt_image_view rqt_image_view /wall_line_nav/debug_image
```

The debug image shows:
- Green line: Detected line center
- Blue line: Image center (target)
- Red rectangles: Wall warnings
- Status text: Detection status

## License

MIT
