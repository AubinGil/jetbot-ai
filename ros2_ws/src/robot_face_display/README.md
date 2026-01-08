# Robot Face Display - ROS 2 Package

Animated robot eyes with personality, controllable via ROS 2 topics!

## Features

- **6 Expressions**: default, happy, angry, tired, surprised, scanning
- **ROS 2 Integration**: Control via topics and services
- **Smooth Animations**: 60 FPS with auto-blinking, idle mode, and special effects
- **Manual Controls**: Keyboard shortcuts for testing

## Topics

### Subscriptions

- `/robot_face/expression` (`std_msgs/String`)
  - Set the robot's facial expression
  - Valid values: `default`, `happy`, `angry`, `tired`, `surprised`, `scanning`

- `/robot_face/gaze` (`std_msgs/String`)
  - Control where the eyes look
  - Valid values: `center`, `n`, `ne`, `e`, `se`, `s`, `sw`, `w`, `nw`

## Building

```bash
cd ~/jetbot_vlm_ws
colcon build --packages-select robot_face_display
source install/setup.bash
```

## Running

### Using launch file:
```bash
ros2 launch robot_face_display robot_face.launch.py
```

### Running directly:
```bash
ros2 run robot_face_display robot_face_node.py
```

## Testing with Command Line

### Change expression:
```bash
ros2 topic pub /robot_face/expression std_msgs/String "data: 'happy'" --once
ros2 topic pub /robot_face/expression std_msgs/String "data: 'angry'" --once
ros2 topic pub /robot_face/expression std_msgs/String "data: 'surprised'" --once
```

### Change gaze:
```bash
ros2 topic pub /robot_face/gaze std_msgs/String "data: 'e'" --once
ros2 topic pub /robot_face/gaze std_msgs/String "data: 'n'" --once
ros2 topic pub /robot_face/gaze std_msgs/String "data: 'center'" --once
```

## Keyboard Controls

When the display window is focused:

- `SPACE` - Cycle through expressions
- `Arrow keys` - Move eye gaze
- `I` - Toggle idle mode (auto eye movement)
- `C` - Toggle curiosity mode
- `S` - Toggle sweat drops
- `L` - Trigger laugh animation
- `B` - Blink
- `F` - Trigger confused animation
- `ESC` - Exit

## Integration Examples

### With TTS (Text-to-Speech):
```python
# When robot starts speaking, show happy expression
publisher.publish(String(data='happy'))
```

### With Vision/Detection:
```python
# When person detected, look in their direction and show surprised
gaze_pub.publish(String(data='e'))  # Look east
expr_pub.publish(String(data='surprised'))
```

### With Navigation:
```python
# When scanning environment
expr_pub.publish(String(data='scanning'))
```

## Dependencies

- ROS 2 Humble
- Python 3
- pygame
- rclpy
- std_msgs

## Author

Generated with Claude Code for Jetson robots
