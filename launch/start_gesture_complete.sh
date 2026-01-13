#!/bin/bash
#
# Complete Gesture System with GPU and Motors
#

IMAGE="ultralytics/ultralytics:latest-jetson-jetpack6"

# Use ROS_DOMAIN_ID from environment if set (default: keep current domain)
if [ -n "${ROS_DOMAIN_ID}" ]; then
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"
  echo "Using ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
fi

echo "================================================"
echo "  Complete Jetbot Gesture System"
echo "================================================"
echo ""

GREEN='\033[0;32m'
NC='\033[0m'

CONTAINER_NAME="yolo_gpu_server"
CAMERA_TOPIC=${CAMERA_TOPIC:-/camera/image_raw}
CAMERA_PROCESS_REGEX='jetbot_camera_publisher\|isaac_ros_camera\|argus_camera_node\|v4l2_camera_node\|camera_node'

declare -A PROCESS_PIDS=()
GPU_CONTAINER_STARTED=false
CLEANUP_DONE=false

start_background_process() {
    local name="$1"
    shift
    "$@" &
    local pid=$!
    PROCESS_PIDS["$name"]=$pid
    echo "   ↳ ${name} started (PID ${pid})"
}

stop_tracked_processes() {
    for name in "${!PROCESS_PIDS[@]}"; do
        local pid="${PROCESS_PIDS[$name]}"
        if kill -0 "$pid" 2>/dev/null; then
            echo "   Stopping ${name} (PID ${pid})"
            kill "$pid" 2>/dev/null || true
            wait "$pid" 2>/dev/null || true
        fi
    done
}

cleanup() {
    if [ "${CLEANUP_DONE}" = true ]; then
        exit 0
    fi
    CLEANUP_DONE=true

    echo ""
    echo "Shutting down..."

    if [ "${GPU_CONTAINER_STARTED}" = true ]; then
        docker stop "$CONTAINER_NAME" 2>/dev/null || true
    fi

    stop_tracked_processes

    sleep 1
    exit 0
}

trap cleanup INT TERM

# Source both workspaces
echo -e "${GREEN}[1/4]${NC} Sourcing ROS2 workspaces..."
source /opt/ros/humble/setup.bash
source $HOME/jetbot_vlm_ws/install/setup.bash
source $HOME/jetbot_gesture_ws/install/setup.bash

# Start GPU server
echo -e "${GREEN}[2/4]${NC} Starting GPU inference server..."
if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "   ℹ️  GPU server container already running - reusing ${CONTAINER_NAME}"
else
    docker rm -f "$CONTAINER_NAME" 2>/dev/null || true
    if docker run -d --rm \
        --runtime=nvidia \
        --network host \
        -v "$HOME:/workspace" \
        --name "$CONTAINER_NAME" \
        $IMAGE \
        python3 /workspace/run_yolo_gpu_server.py; then
        GPU_CONTAINER_STARTED=true
    else
        echo "  Failed to launch GPU server container"
        cleanup
    fi
fi

if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo -n "  Waiting for GPU server"
    for i in {1..15}; do
        if docker logs "$CONTAINER_NAME" 2>&1 | grep -q "Waiting for connection"; then
            echo ""
            break
        fi
        echo -n "."
        sleep 1
    done
    echo -e "  ${GREEN}✓${NC} GPU server ready"
else
    echo "  GPU server container failed to start; check 'docker logs ${CONTAINER_NAME}'"
    cleanup
fi

# Start nodes
echo -e "${GREEN}[3/4]${NC} Starting ROS2 nodes..."

# Hardware - only start motor_controller if not already running
if pgrep -f "motor_controller" > /dev/null 2>&1; then
  echo "   ℹ️  Motor controller already running (started by conversation system)"
else
  echo "   Starting motor controller..."
  start_background_process "motor_controller" ros2 run jetbot_hardware motor_controller
  sleep 1
fi

# Camera and pose detection - skip if camera already running (from conversation system)
echo "   Camera topic:        ${CAMERA_TOPIC}"
if pgrep -f "${CAMERA_PROCESS_REGEX}" > /dev/null 2>&1; then
  echo "   ℹ️  Camera node already running (started by conversation system)"
else
  echo "   Starting camera node..."
  start_background_process "camera_node" ros2 run jetbot_gesture camera_node
  sleep 1
fi

start_background_process "pose_detector_bridge" \
  python3 "$HOME/jetbot_gesture_ws/src/jetbot_gesture/jetbot_gesture/pose_detector_gpu_bridge.py" \
  --ros-args -p camera_topic:="${CAMERA_TOPIC}"
sleep 2

# Gesture system
start_background_process "gesture_recognition_node" ros2 run jetbot_gesture gesture_recognition_node
start_background_process "person_tracker_node" \
  ros2 run jetbot_gesture person_tracker_node --ros-args -p tracking_timeout:=3.0 -p image_height:=480

# Follow distance/speed tuning
# - Increase STOP_DISTANCE to stop further away
# - Decrease MAX_LINEAR to approach slower
# - Decrease KP_LINEAR to reduce acceleration toward target distance
STOP_DISTANCE_VAL=${STOP_DISTANCE:-1.2}
MIN_DISTANCE_VAL=${MIN_DISTANCE:-0.7}
# Softer defaults to protect hardware
MAX_LINEAR_VAL=${MAX_LINEAR:-0.10}
KP_LINEAR_VAL=${KP_LINEAR:-0.15}
MAX_ANGULAR_VAL=${MAX_ANGULAR:-0.35}
KP_ANGULAR_VAL=${KP_ANGULAR:-0.45}  # Reduced from 0.60 to reduce oscillation
# Angular smoothing (tuned to eliminate jitter/twitching)
ANGULAR_DEADBAND_VAL=${ANGULAR_DEADBAND:-0.15}  # Increased from 0.08 to filter small corrections
ANGULAR_FILTER_ALPHA_VAL=${ANGULAR_FILTER_ALPHA:-0.5}  # Reduced from 0.8 for faster response, less lag
ANGULAR_SLEW_RATE_VAL=${ANGULAR_SLEW_RATE:-0.20}  # Increased from 0.15 for smoother transitions
# Linear smoothing
LINEAR_FILTER_ALPHA_VAL=${LINEAR_FILTER_ALPHA:-0.8}
LINEAR_SLEW_RATE_VAL=${LINEAR_SLEW_RATE:-0.02}
# Search behavior when person lost
SEARCH_ENABLED_VAL=${SEARCH_ENABLED:-1}
SEARCH_ANGULAR_VAL=${SEARCH_ANGULAR_SPEED:-0.30}
SEARCH_STEP_SEC_VAL=${SEARCH_STEP_SEC:-1.0}
SEARCH_TOTAL_SEC_VAL=${SEARCH_TOTAL_SEC:-12.0}
SEARCH_DELAY_SEC_VAL=${SEARCH_DELAY_SEC:-1.0}
echo "   Follow stop_distance: ${STOP_DISTANCE_VAL} m (set STOP_DISTANCE to override)"
echo "   Follow min_distance:  ${MIN_DISTANCE_VAL} m (set MIN_DISTANCE to override)"
echo "   Max linear speed:     ${MAX_LINEAR_VAL} m/s (set MAX_LINEAR to override)"
echo "   Linear gain (kp):     ${KP_LINEAR_VAL} (set KP_LINEAR to override)"
echo "   Max angular speed:    ${MAX_ANGULAR_VAL} rad/s (set MAX_ANGULAR to override)"
echo "   Angular gain (kp):    ${KP_ANGULAR_VAL} (set KP_ANGULAR to override)"
echo "   Angular deadband:     ${ANGULAR_DEADBAND_VAL} (ANGULAR_DEADBAND)"
echo "   Angular filter:       ${ANGULAR_FILTER_ALPHA_VAL} (ANGULAR_FILTER_ALPHA)"
echo "   Angular slew/tick:    ${ANGULAR_SLEW_RATE_VAL} rad/s (ANGULAR_SLEW_RATE)"
echo "   Linear filter:        ${LINEAR_FILTER_ALPHA_VAL} (LINEAR_FILTER_ALPHA)"
echo "   Linear slew/tick:     ${LINEAR_SLEW_RATE_VAL} m/s (LINEAR_SLEW_RATE)"
echo "   Search enabled:       ${SEARCH_ENABLED_VAL} (set SEARCH_ENABLED=0 to disable)"
echo "   Search turn speed:    ${SEARCH_ANGULAR_VAL} rad/s (SEARCH_ANGULAR_SPEED)"
echo "   Search step time:     ${SEARCH_STEP_SEC_VAL} s (SEARCH_STEP_SEC)"
echo "   Search total time:    ${SEARCH_TOTAL_SEC_VAL} s (SEARCH_TOTAL_SEC)"
echo "   Search start delay:   ${SEARCH_DELAY_SEC_VAL} s (SEARCH_DELAY_SEC)"
# Optionally auto-enable following (FOLLOW_START_ENABLED=1)
START_ENABLED_BOOL=false
if [ "${FOLLOW_START_ENABLED:-0}" != "0" ]; then
  START_ENABLED_BOOL=true
fi
echo "   Start enabled:        ${START_ENABLED_BOOL} (set FOLLOW_START_ENABLED=1 to auto-follow)"

start_background_process "navigation_controller_node" \
  ros2 run jetbot_gesture navigation_controller_node --ros-args \
    -p stop_distance:=${STOP_DISTANCE_VAL} \
    -p min_distance:=${MIN_DISTANCE_VAL} \
    -p max_linear_speed:=${MAX_LINEAR_VAL} \
    -p kp_linear:=${KP_LINEAR_VAL} \
    -p max_angular_speed:=${MAX_ANGULAR_VAL} \
    -p kp_angular:=${KP_ANGULAR_VAL} \
    -p angular_deadband:=${ANGULAR_DEADBAND_VAL} \
    -p angular_filter_alpha:=${ANGULAR_FILTER_ALPHA_VAL} \
    -p angular_slew_rate:=${ANGULAR_SLEW_RATE_VAL} \
    -p linear_filter_alpha:=${LINEAR_FILTER_ALPHA_VAL} \
    -p linear_slew_rate:=${LINEAR_SLEW_RATE_VAL} \
    -p start_enabled:=${START_ENABLED_BOOL} \
    -p search_enabled:=$([ "$SEARCH_ENABLED_VAL" != "0" ] && echo true || echo false) \
    -p search_angular_speed:=${SEARCH_ANGULAR_VAL} \
    -p search_step_sec:=${SEARCH_STEP_SEC_VAL} \
    -p search_total_sec:=${SEARCH_TOTAL_SEC_VAL} \
    -p search_delay_sec:=${SEARCH_DELAY_SEC_VAL}

# TTS interaction node: needed for wave gesture detection and TTS feedback
# When started via voice command (TTS=0 + FOLLOW_START_ENABLED=1), skip it entirely
# since following is auto-enabled and conversation system handles all TTS
if [ "${TTS:-0}" != "0" ]; then
  TTS_ENGINE_VAL=${TTS_ENGINE:-piper}
  PIPER_MODEL_VAL=${PIPER_MODEL:-$HOME/en_US-lessac-medium.onnx}
  GREETING_MSG_VAL=${TTS_GREETING_MESSAGE:-'Beep bop, I see you.'}
  APPROACH_MSG_VAL=${TTS_APPROACH_MESSAGE:-'Coming! master'}
  echo "🔊 Starting TTS (engine=${TTS_ENGINE_VAL})"
  APLAY_DEVICE_VAL=${APLAY_DEVICE:-default}

  # Cloud TTS (Magpie) parameters (read from env if set)
  MAGPIE_FUNCTION_ID_VAL=${MAGPIE_FUNCTION_ID:-}
  MAGPIE_TOKEN_VAL=${MAGPIE_TOKEN:-}
  MAGPIE_LANGUAGE_VAL=${MAGPIE_LANGUAGE_CODE:-en-US}
  MAGPIE_VOICE_VAL=${MAGPIE_VOICE:-Magpie-Multilingual.EN-US.Aria}
  MAGPIE_TALK_SCRIPT_VAL=${MAGPIE_TALK_SCRIPT:-$HOME/python-clients/scripts/tts/talk.py}
  MAGPIE_VENV_VAL=${MAGPIE_VENV_PATH:-}
  MAGPIE_PYTHON_VAL=${MAGPIE_PYTHON:-}

  # Auto-detect local magpie venv if not provided
  if [ -z "${MAGPIE_PYTHON_VAL}" ] && [ -z "${MAGPIE_VENV_VAL}" ] && [ -x "$HOME/magpie/bin/python" ]; then
    MAGPIE_PYTHON_VAL="$HOME/magpie/bin/python"
  fi

  # Build ros2 param list
  ROS_PARAMS=(
    -p tts_engine:="${TTS_ENGINE_VAL}"
    -p piper_model:="${PIPER_MODEL_VAL}"
    -p greeting_message:="${GREETING_MSG_VAL}"
    -p approach_message:="${APPROACH_MSG_VAL}"
    -p alsa_device:="${APLAY_DEVICE_VAL}"
  )

  if [ "${TTS_ENGINE_VAL}" = "magpie" ]; then
    ROS_PARAMS+=(
      -p magpie_server:="grpc.nvcf.nvidia.com:443"
      -p magpie_use_ssl:=true
      -p magpie_language_code:="${MAGPIE_LANGUAGE_VAL}"
      -p magpie_voice:="${MAGPIE_VOICE_VAL}"
    )
    # Only pass credentials/paths if non-empty to avoid ROS2 parse errors
    if [ -n "${MAGPIE_FUNCTION_ID_VAL}" ]; then
      ROS_PARAMS+=( -p magpie_function_id:="${MAGPIE_FUNCTION_ID_VAL}" )
    fi
    if [ -n "${MAGPIE_TOKEN_VAL}" ]; then
      ROS_PARAMS+=( -p magpie_token:="${MAGPIE_TOKEN_VAL}" )
    fi
    if [ -n "${MAGPIE_TALK_SCRIPT_VAL}" ]; then
      ROS_PARAMS+=( -p magpie_talk_script:="${MAGPIE_TALK_SCRIPT_VAL}" )
    fi
    if [ -n "${MAGPIE_VENV_VAL}" ]; then
      ROS_PARAMS+=( -p magpie_venv_path:="${MAGPIE_VENV_VAL}" )
    fi
    if [ -n "${MAGPIE_PYTHON_VAL}" ]; then
      ROS_PARAMS+=( -p magpie_python:="${MAGPIE_PYTHON_VAL}" )
    fi
  fi

  start_background_process "tts_interaction_node" \
    ros2 run jetbot_gesture tts_interaction_node --ros-args "${ROS_PARAMS[@]}"
elif [ "${FOLLOW_START_ENABLED:-0}" == "0" ]; then
  # TTS disabled but following not auto-enabled: still need interaction node for wave gesture
  echo "🤫 TTS disabled - interaction node will run silently for wave gesture detection"
  start_background_process "tts_interaction_node" \
    ros2 run jetbot_gesture tts_interaction_node --ros-args -p tts_engine:=none
else
  # TTS=0 and FOLLOW_START_ENABLED=1: skip interaction node entirely
  # (following auto-enabled, conversation system handles TTS)
  echo "🔇 TTS disabled + auto-follow enabled - skipping interaction node"
fi

sleep 2

echo ""
echo -e "${GREEN}[4/4]${NC} System ready!"
echo ""
echo "================================================"
echo ""
echo "✓ GPU pose detection: 15 FPS"
echo "✓ Motor controller: running"
echo "✓ Gesture recognition: active"
echo ""
echo "Wave at the camera to activate!"
echo "Press Ctrl+C to stop"
echo "================================================"
echo ""

while true; do
    if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo "GPU server stopped!"
        cleanup
    fi
    sleep 2
done
