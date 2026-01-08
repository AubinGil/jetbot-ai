# Hardware Setup Guide

Complete hardware requirements and setup instructions for the JetBot AI platform.

## Bill of Materials

### Core Components

| Component | Model/Spec | Purpose | Device Path |
|-----------|------------|---------|-------------|
| Main Computer | NVIDIA Jetson Orin Nano 8GB | AI processing, ROS2 | - |
| Operating System | Ubuntu 22.04 (Jammy) | Base OS | - |
| JetPack | 6.x (R36.4.4) | NVIDIA SDK | - |

### Sensors

| Component | Model | Connection | Device Path |
|-----------|-------|------------|-------------|
| LiDAR | RPLidar A1/A2/A3 | USB Serial | `/dev/ttyUSB0` |
| Camera | j5 WebCam JVCU100 or compatible | USB | `/dev/video0` |
| Microphone | USB PnP Audio Device | USB | Card 3 (ALSA) |
| Speaker | UACDemoV1.0 or compatible | USB/3.5mm | Card 1 (ALSA) |

### Actuators

| Component | Spec | Connection | Device Path |
|-----------|------|------------|-------------|
| Motor Controller | Waveshare servo driver | USB Serial (115200 baud) | `/dev/ttyACM0` or `/dev/ttyACM1` |
| Motors | DC motors (compatible with controller) | Via motor controller | - |

### Optional Remote Hardware

| Component | Purpose | Network Address |
|-----------|---------|-----------------|
| RTX 5090 PC | Remote Cosmos inference | `192.168.2.16:8000` |
| Remote Ollama Server | Backup LLM inference | `192.168.2.29:11434` |

## Wiring Diagram

```
┌─────────────────────────────────────┐
│   NVIDIA Jetson Orin Nano           │
│                                     │
│  USB Ports:                         │
│    ├─ Port 1: Camera (/dev/video0) │
│    ├─ Port 2: RPLidar (/dev/ttyUSB0)│
│    ├─ Port 3: Motor Controller     │
│    │           (/dev/ttyACM0)       │
│    ├─ Port 4: Microphone            │
│    └─ Port 5: Speaker (or 3.5mm)   │
│                                     │
│  Network: Ethernet/WiFi             │
│    └─ 192.168.2.x (local network)  │
└─────────────────────────────────────┘
         │
         │ (Serial via Motor Controller)
         ↓
┌─────────────────────────────────────┐
│   Motor Controller                  │
│   (Waveshare or compatible)         │
│                                     │
│   Outputs:                          │
│    ├─ Motor Left                    │
│    └─ Motor Right                   │
└─────────────────────────────────────┘
```

## Device Setup

### 1. Jetson Orin Nano

**Minimum Requirements:**
- 8GB RAM
- 64GB+ storage (microSD or NVMe SSD recommended)
- JetPack 6.x installed
- Swap file (2GB+) for model loading

**Power:**
- 5V 4A power supply (barrel jack)
- Or powered via carrier board

### 2. RPLidar Setup

**Supported Models:**
- RPLidar A1 (12m range, 8000 samples/sec)
- RPLidar A2 (12m range, 8000 samples/sec)
- RPLidar A3 (25m range, 16000 samples/sec)

**Connection:**
- USB adapter (CP2102 or similar)
- Baud rate: 115200
- Device path: `/dev/ttyUSB0` (automatic)

**Verification:**
```bash
# Check device exists
ls -l /dev/ttyUSB0

# Test LiDAR
./launch/test_lidar.sh
```

**Mounting:**
- Mount horizontally on top of robot
- Ensure 360° clear view
- Center over robot's rotation axis

### 3. Camera Setup

**Specifications:**
- Resolution: 640x480 minimum (higher supported)
- Frame rate: 15-30 FPS
- Interface: USB UVC compatible
- Tested: j5 WebCam JVCU100

**Connection:**
- USB 2.0/3.0 port
- Device path: `/dev/video0`

**Verification:**
```bash
# List video devices
v4l2-ctl --list-devices

# Test camera
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video0 \
  -p image_size:=[640,480]
```

**Mounting:**
- Forward-facing on robot front
- Height: Eye-level perspective (10-20cm above ground)
- Angle: Slight downward tilt (10-15°) for ground view

### 4. Audio Setup

**Microphone Requirements:**
- USB or built-in
- Sample rate: 16kHz minimum
- Mono or stereo

**Speaker Requirements:**
- USB or 3.5mm audio jack
- Any quality (speech clarity sufficient)

**ALSA Configuration:**

Check audio devices:
```bash
aplay -l
arecord -l
```

Configure default devices in `~/.asoundrc`:
```
pcm.!default {
    type asym
    playback.pcm "plughw:1,0"  # Speaker (Card 1)
    capture.pcm "plughw:3,0"   # Microphone (Card 3)
}
```

**Verification:**
```bash
# Test microphone
arecord -D plughw:3,0 -f S16_LE -r 16000 test.wav

# Test speaker
aplay -D plughw:1,0 test.wav
```

### 5. Motor Controller Setup

**Supported Controllers:**
- Waveshare servo driver board
- Any UART-compatible motor controller (modify driver as needed)

**Connection:**
- USB-to-serial adapter or direct UART
- Baud rate: 115200
- Device path: `/dev/ttyACM0` or `/dev/ttyACM1`

**Wiring:**
- TX → Controller RX
- RX → Controller TX
- GND → Controller GND
- Motor terminals → Left/Right motors

**Verification:**
```bash
# Check serial device
ls -l /dev/ttyACM*

# Test motors (publishes /cmd_vel)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

**Safety:**
- Add emergency stop button (recommended)
- Test in open area first
- Start with low speeds (0.1-0.2 m/s)

## Network Setup

### Local Network (Jetson)

**IP Configuration:**
- Automatic DHCP or static IP in 192.168.2.x range
- Ensure connectivity to remote servers (if used)

**Test Connectivity:**
```bash
# Test remote Cosmos server
curl http://192.168.2.16:8000/v1/models

# Test remote Ollama server
curl http://192.168.2.29:11434/api/tags
```

### Optional: Remote GPU Server

**Requirements:**
- Windows PC or Linux with RTX GPU (3090/4090/5090)
- vLLM or OpenAI-compatible API server
- Port 8000 exposed on local network

**Setup Example (vLLM on Windows/WSL):**
```bash
vllm serve nvidia/Cosmos-Reason1-7B \
  --host 0.0.0.0 \
  --port 8000 \
  --max-model-len 4096
```

## Physical Assembly

### Chassis Requirements

**Dimensions:**
- Base: ~15-20cm diameter or square
- Height: 20-30cm (including sensors)
- Weight capacity: 2kg+ (Jetson + sensors)

**Recommended Layout:**
```
     [Camera]
        ↑
    ┌───────┐
    │ LiDAR │ (top, centered)
    └───────┘
    ┌───────┐
    │Jetson │ (middle layer)
    │ Orin  │
    └───────┘
    ┌───────┐
    │Battery│ (bottom)
    │Motors │
    └───────┘
```

### Cable Management

- Use zip ties or cable channels
- Keep USB cables short (< 50cm)
- Secure connections (vibration-proof)
- Label cables for troubleshooting

## Power System

### Option 1: Battery Powered (Portable)

**Battery:**
- 5V power bank (20000mAh+)
- Or 12V LiPo with 5V regulator (4A+)

**Runtime Estimate:**
- Jetson: ~10W
- Motors: ~5-10W (average)
- Sensors: ~2W
- Total: ~20W → 2-3 hours on 20000mAh

### Option 2: Tethered (Development)

- 5V 4A wall adapter
- Limits mobility but unlimited runtime

## Device Permissions

**Grant access to devices:**
```bash
# Add user to dialout group (for serial devices)
sudo usermod -a -G dialout $USER

# Add user to video group (for camera)
sudo usermod -a -G video $USER

# Add user to audio group
sudo usermod -a -G audio $USER

# Reboot to apply
sudo reboot
```

## Troubleshooting Hardware

### Camera not found
```bash
# Check USB connection
lsusb | grep -i camera

# Check v4l2 devices
v4l2-ctl --list-devices

# Try different USB port
```

### LiDAR not responding
```bash
# Check USB serial device
ls -l /dev/ttyUSB*

# Check permissions
sudo chmod 666 /dev/ttyUSB0

# Check baud rate in driver config
```

### Motor controller not responding
```bash
# Check serial connection
ls -l /dev/ttyACM*

# Test with minicom
sudo minicom -D /dev/ttyACM0 -b 115200

# Verify power to motor controller
```

### Audio issues
```bash
# List all audio devices
aplay -l
arecord -l

# Test specific device
aplay -D hw:1,0 /usr/share/sounds/alsa/Front_Center.wav

# Reconfigure ALSA if needed
```

## Next Steps

Once hardware is assembled and verified:
1. Proceed to [SETUP.md](SETUP.md) for software installation
2. Test individual components before integration
3. Calibrate sensors if needed (LiDAR, camera)
4. Tune motor speeds in configuration files

---

**Note**: This guide assumes standard components. Adjust device paths and configurations for your specific hardware.
