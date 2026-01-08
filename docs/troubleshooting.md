# Troubleshooting Guide

Common issues and solutions for JetBot AI.

## Hardware Issues

### Camera Not Detected

**Symptoms:**
- `/dev/video0` not found
- "Failed to open camera" errors
- No image on `/camera/image_raw` topic

**Solutions:**

1. **Check USB connection:**
```bash
lsusb | grep -i camera
# Should show your camera device
```

2. **List video devices:**
```bash
v4l2-ctl --list-devices
ls -l /dev/video*
```

3. **Try different USB port** - some ports may have issues

4. **Test camera directly:**
```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video0 \
  -p image_size:=[640,480]
```

5. **Check permissions:**
```bash
sudo usermod -a -G video $USER
sudo reboot
```

6. **If wrong device number:**
   - Update `CAMERA_DEV` in launch scripts
   - Or create udev rule for consistent naming

### LiDAR Not Responding

**Symptoms:**
- `/dev/ttyUSB0` not found
- LiDAR not spinning
- No data on `/scan` topic

**Solutions:**

1. **Check USB connection:**
```bash
ls -l /dev/ttyUSB*
# Should show /dev/ttyUSB0
```

2. **Check permissions:**
```bash
sudo usermod -a -G dialout $USER
sudo reboot
```

3. **Manual permission (temporary):**
```bash
sudo chmod 666 /dev/ttyUSB0
```

4. **Test LiDAR:**
```bash
ros2 run rplidar_ros rplidar_composition --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p serial_baudrate:=115200
```

5. **Check power:**
   - Ensure LiDAR is powered (motor spinning)
   - Some models need 5V external power

6. **Try different baud rate:**
   - Most RPLidar use 115200
   - Some use 256000

### Motor Controller Not Working

**Symptoms:**
- Robot doesn't move
- `/dev/ttyACM0` not found
- "Failed to open serial port" errors

**Solutions:**

1. **Check serial device:**
```bash
ls -l /dev/ttyACM*
# Should show /dev/ttyACM0 or /dev/ttyACM1
```

2. **Check permissions:**
```bash
sudo usermod -a -G dialout $USER
sudo reboot
```

3. **Test with minicom:**
```bash
sudo minicom -D /dev/ttyACM0 -b 115200
```

4. **Verify wiring:**
   - TX → RX, RX → TX
   - GND connected
   - Power to motor controller

5. **Check motors directly:**
```bash
# Publish test velocity
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}" --once
```

6. **Wrong device path:**
   - Update `MOTOR_DEV` in scripts
   - Check if it's `/dev/ttyACM1` instead

### Audio Issues

**Symptoms:**
- No sound output
- Microphone not recording
- "Audio device not found" errors

**Solutions:**

1. **List audio devices:**
```bash
aplay -l   # List playback devices
arecord -l # List capture devices
```

2. **Test speaker:**
```bash
aplay /usr/share/sounds/alsa/Front_Center.wav
```

3. **Test microphone:**
```bash
arecord -f S16_LE -r 16000 -d 5 test.wav
aplay test.wav
```

4. **Check `.asoundrc` configuration:**
```bash
cat ~/.asoundrc
# Should have correct card numbers
```

5. **Fix card numbers:**
   - Update card numbers in `.asoundrc`
   - Match with `aplay -l` output

6. **Volume settings:**
```bash
alsamixer
# F4 for capture, F5 for playback
# Adjust levels
```

7. **Restart audio:**
```bash
pulseaudio --kill
pulseaudio --start
```

## Software Issues

### ROS2 Not Found

**Symptoms:**
- `ros2: command not found`
- Package not found errors

**Solutions:**

1. **Source ROS2:**
```bash
source /opt/ros/humble/setup.bash
source ~/jetbot-ai/ros2_ws/install/setup.bash
```

2. **Add to `.bashrc`:**
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/jetbot-ai/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

3. **Verify installation:**
```bash
which ros2
ros2 pkg list
```

### Build Errors (colcon)

**Symptoms:**
- `colcon build` fails
- Package not found
- Dependency errors

**Solutions:**

1. **Clean build:**
```bash
cd ~/jetbot-ai/ros2_ws
rm -rf build install log
colcon build --symlink-install
```

2. **Install missing dependencies:**
```bash
rosdep install --from-paths src --ignore-src -r -y
```

3. **Check Python dependencies:**
```bash
pip3 install -r requirements.txt
```

4. **Build specific package:**
```bash
colcon build --packages-select jetbot_conversation
```

5. **Verbose output:**
```bash
colcon build --event-handlers console_direct+
```

### Ollama Connection Issues

**Symptoms:**
- "Connection refused" to Ollama
- LLM not responding
- `curl http://localhost:11434` fails

**Solutions:**

1. **Check if Ollama is running:**
```bash
pgrep ollama
curl http://localhost:11434/api/tags
```

2. **Start Ollama:**
```bash
ollama serve &
```

3. **Check port:**
```bash
netstat -tuln | grep 11434
```

4. **Restart Ollama:**
```bash
pkill ollama
ollama serve &
```

5. **Check models:**
```bash
ollama list
# Should show granite4, granite3.2-vision, etc.
```

6. **Pull missing models:**
```bash
ollama pull granite4
ollama pull granite3.2-vision
```

### Remote Server Connection Issues

**Symptoms:**
- Can't reach Cosmos server
- Timeout connecting to remote GPU
- Network connection errors

**Solutions:**

1. **Test connectivity:**
```bash
ping 192.168.2.16
curl http://192.168.2.16:8000/v1/models
```

2. **Check server is running:**
   - On remote PC, verify vLLM/server is active
   - Check firewall settings

3. **Verify network:**
```bash
ip addr show
# Check IP is in same subnet (192.168.2.x)
```

4. **Use local fallback:**
```bash
# Comment out remote server in config
# System will use local Ollama
```

5. **Check environment variables:**
```bash
echo $COSMOS_HOST
# Should be http://192.168.2.16:8000
```

## Performance Issues

### Slow Inference

**Symptoms:**
- Laggy camera feed
- Delayed responses
- Low FPS

**Solutions:**

1. **Check GPU usage:**
```bash
tegrastats  # On Jetson
```

2. **Reduce camera resolution:**
```bash
# In launch script
-p image_size:=[320,240]  # Instead of [640,480]
```

3. **Use faster models:**
   - Whisper: `tiny.en` instead of `base.en`
   - LLM: granite4 instead of llama3.1:8b
   - YOLO: yolo11n instead of yolo11m

4. **Enable ONNX runtime:**
   - Already default for obstacle avoidance
   - Faster than PyTorch

5. **Reduce inference frequency:**
   - Skip frames for vision processing
   - Lower LLM query rate

6. **Close other applications:**
```bash
pkill -f firefox
pkill -f chrome
```

### Out of Memory Errors

**Symptoms:**
- "Out of memory" crashes
- Killed processes
- Swap usage at 100%

**Solutions:**

1. **Check memory:**
```bash
free -h
```

2. **Increase swap:**
```bash
sudo swapoff /swapfile
sudo fallocate -l 4G /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

3. **Use smaller models:**
   - Qwen2.5-VL: Use Q4 quantization
   - LLM: granite4 (4B) instead of llama3.1:8b
   - Whisper: tiny instead of small

4. **Close unnecessary processes:**
```bash
pkill ollama  # If not using
pkill docker   # If not needed
```

5. **Monitor memory:**
```bash
watch -n 1 free -h
```

### High CPU/GPU Temperature

**Symptoms:**
- Thermal throttling
- Reduced performance
- Jetson getting very hot

**Solutions:**

1. **Check temperature:**
```bash
tegrastats
# Look for CPU/GPU temps
```

2. **Add cooling:**
   - Install heatsink
   - Add fan (5V fan on GPIO)
   - Ensure airflow

3. **Reduce workload:**
   - Lower camera FPS
   - Use lighter models
   - Reduce simultaneous processes

4. **Power mode:**
```bash
sudo nvpmodel -m 0  # Max performance
sudo nvpmodel -m 1  # 15W mode (cooler)
```

## Navigation Issues

### Robot Spins in Circles

**Solutions:**

1. **Check LiDAR mounting:**
   - Must be level
   - Centered over rotation axis

2. **Increase stuck threshold:**
```bash
export STUCK_THRESHOLD=6
```

3. **Verify obstacle distances:**
```bash
ros2 topic echo /scan
# Check for consistent readings
```

4. **Disable periodic scanning:**
```bash
export SCAN_INTERVAL_CYCLES=0
```

### Robot Doesn't Avoid Obstacles

**Solutions:**

1. **Check LiDAR data:**
```bash
ros2 topic hz /scan
# Should be ~5-10 Hz
```

2. **Verify safety distances:**
```bash
# In launch script
SAFETY_DISTANCE=0.15
WARNING_DISTANCE=0.25
```

3. **Test LiDAR manually:**
```bash
ros2 run rplidar_ros rplidar_composition
ros2 topic echo /scan
```

4. **Check obstacle detection:**
```bash
ros2 topic echo /obstacle_distance
```

### Gesture Following Not Working

**Solutions:**

1. **Check pose detection:**
```bash
ros2 topic echo /pose_detections
```

2. **Verify GPU server:**
   - If using remote: check connection
   - If local: verify YOLO model loaded

3. **Test camera:**
```bash
ros2 topic hz /camera/image_raw
```

4. **Check lighting:**
   - Need good lighting for pose detection
   - Avoid backlighting

5. **Increase detection confidence:**
   - Lower threshold in pose detection node

### SLAM Map Poor Quality

**Solutions:**

1. **Drive slower during mapping:**
   - Use manual teleoperation
   - Smooth, deliberate movements

2. **Increase SLAM resolution:**
```yaml
# In slam_params.yaml
resolution: 0.03  # Smaller = better (but slower)
```

3. **Enable loop closing:**
```yaml
do_loop_closing: true
```

4. **Better lighting:**
   - SLAM needs visual features
   - Avoid featureless white walls

5. **Drive in loops:**
   - Helps with loop closure detection
   - Improves consistency

## Conversation Issues

### Wake Word Not Detected

**Solutions:**

1. **Test microphone:**
```bash
arecord -f S16_LE -r 16000 -d 5 test.wav
aplay test.wav
# Should hear your recording clearly
```

2. **Adjust microphone gain:**
```bash
alsamixer
# Press F4, increase capture volume
```

3. **Speak clearly:**
   - Moderate pace
   - Clear pronunciation
   - Face microphone

4. **Change wake word:**
```yaml
# In conversation.yaml
wake_word: 'robot'  # Try different word
```

5. **Check VAD settings:**
```yaml
vad_aggressiveness: 2  # Try 1 if too sensitive
```

### Poor Speech Recognition

**Solutions:**

1. **Use better Whisper model:**
```yaml
whisper_model: 'base.en'  # Or 'small.en'
```

2. **Reduce background noise:**
   - Quieter environment
   - Directional microphone
   - Higher VAD aggressiveness

3. **Speak clearly:**
   - Enunciate words
   - Moderate speed
   - Simple commands

4. **Check language:**
```yaml
# Ensure correct language model
whisper_model: 'tiny.en'  # For English
# whisper_model: 'tiny'   # For multilingual
```

### Robot Doesn't Respond

**Solutions:**

1. **Check conversation node:**
```bash
ros2 node list | grep conversation
```

2. **Monitor topics:**
```bash
ros2 topic echo /audio/transcript
ros2 topic echo /conversation/response
```

3. **Check LLM:**
```bash
curl http://localhost:11434/api/tags
ollama run granite4 "Hello"
```

4. **Verify TTS:**
```bash
ros2 topic pub /tts/speak std_msgs/msg/String "{data: 'test'}"
```

5. **Check logs:**
```bash
ros2 node list
ros2 topic list
```

### No Audio Output

**Solutions:**

1. **Test speaker:**
```bash
aplay /usr/share/sounds/alsa/Front_Center.wav
```

2. **Check TTS node:**
```bash
ros2 node list | grep tts
```

3. **Verify TTS config:**
```yaml
enable_tts: true  # Must be true
```

4. **Test TTS directly:**
```bash
ros2 topic pub /tts/speak std_msgs/msg/String "{data: 'Hello world'}"
```

5. **Check Piper model:**
```bash
ls -l ~/en_US-lessac-medium.onnx
```

## System-Wide Issues

### Permission Denied Errors

**Solutions:**

```bash
# Add to all required groups
sudo usermod -a -G dialout,video,audio $USER

# Reboot to apply
sudo reboot
```

### "Command not found" Errors

**Solutions:**

```bash
# Source ROS2
source /opt/ros/humble/setup.bash
source ~/jetbot-ai/ros2_ws/install/setup.bash

# Add to .bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/jetbot-ai/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Processes Don't Stop

**Solutions:**

```bash
# Kill all ROS2 nodes
pkill -f ros2

# Kill specific process
pkill -f vila_robot_bridge

# Force kill
pkill -9 -f ros2
```

### Docker Issues

**Solutions:**

1. **Start Docker:**
```bash
sudo systemctl start docker
```

2. **Check NVIDIA runtime:**
```bash
docker run --rm --runtime=nvidia nvidia/cuda:12.0-base nvidia-smi
```

3. **Permissions:**
```bash
sudo usermod -a -G docker $USER
newgrp docker
```

## Getting More Help

### Enable Debug Logging

In Python nodes:
```python
self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
```

### Check System Logs

```bash
# ROS2 logs
ros2 wtf

# System logs
journalctl -xe

# Kernel logs
dmesg | tail
```

### Monitor All Topics

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /camera/image_raw
```

### Report Issues

If you can't solve the problem:

1. **Collect logs:**
```bash
ros2 wtf > ros2_logs.txt
dmesg > system_logs.txt
```

2. **Create issue:**
   - Go to: https://github.com/AubinGil/jetbot-ai/issues
   - Include: error message, logs, steps to reproduce
   - Hardware details, OS version

## Quick Reference

### Emergency Stop

```bash
# Stop all movement
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" --once

# Kill all nodes
pkill -f ros2
```

### Restart Everything

```bash
# Stop all
pkill -f ros2
pkill ollama
docker stop $(docker ps -q)

# Start fresh
cd ~/jetbot-ai
./launch/start_conversation_complete.sh
```

### Check System Health

```bash
# Temperature
tegrastats

# Memory
free -h

# Storage
df -h

# Processes
htop
```

---

For more details, see:
- [SETUP.md](../SETUP.md) for installation issues
- [HARDWARE.md](../HARDWARE.md) for hardware problems
- [conversation.md](conversation.md) for voice issues
- [navigation.md](navigation.md) for navigation problems
