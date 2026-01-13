#!/bin/bash
# VILA Auto-Navigation with RPLidar Safety Layer - ENHANCED VERSION
# Vision: Cosmos (nvidia/cosmos-reason1-7b) on laptop RTX 5090
# Safety: RPLidar prevents collisions
# Intelligence: Smart dead-end detection and turnaround
# Entertainment: Periodic scene narration with TTS
#
# Usage:
#   TTS=1 ./start_vila_with_lidar_enhanced.sh                        (enables voice narration via ROS tts_node)
#   TTS=1 NARRATION_LANGUAGE=fr ./start_vila_with_lidar_enhanced.sh  (French scene narration with Pascal voice)
#   ./start_vila_with_lidar_enhanced.sh                              (navigation only)

echo "=================================================="
echo "VILA Enhanced Auto-Navigation with Smart Recovery"
echo "=================================================="
echo ""

# Clean up any existing VILA container (running or stopped)
if docker ps -a --format '{{.Names}}' | grep -q '^vila_vision$'; then
    echo "Removing existing VILA container..."
    docker rm -f vila_vision >/dev/null 2>&1
    sleep 2
fi

# Check if ROS nodes are running
if pgrep -f "ros2" > /dev/null; then
    echo "⚠️  ROS nodes already running, stopping them..."
    pkill -9 -f ros2
    sleep 2
fi

# Source ROS2 and set ROS domain
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=37
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Server URL (override with --server-url/--vila-url or env)
# Priority: CLI flag > VILA_URL > GROOT_SERVER_URL > default localhost
VILA_URL="${VILA_URL:-${GROOT_SERVER_URL:-http://localhost:5000}}"
# Cosmos endpoint on laptop RTX 5090 (primary if set)
COSMOS_HOST=${COSMOS_HOST:-http://192.168.2.16:8000}
COSMOS_MODEL=${COSMOS_MODEL:-nvidia/Cosmos-Reason2-8B}
OLLAMA_REMOTE_HOST=${OLLAMA_REMOTE_HOST:-http://192.168.2.29:11434}
OLLAMA_REMOTE_MODEL=${OLLAMA_REMOTE_MODEL:-minicpm-v:8b}
OLLAMA_LOCAL_HOST=${OLLAMA_LOCAL_HOST:-http://127.0.0.1:11434}
OLLAMA_LOCAL_MODEL=${OLLAMA_LOCAL_MODEL:-minicpm-v:8b}
# Timeout for waiting for local or external VILA /health endpoint (seconds, 0 = wait forever)
VILA_WAIT_TIMEOUT=${VILA_WAIT_TIMEOUT:-60}
VILA_EXTERNAL_WAIT_TIMEOUT=${VILA_EXTERNAL_WAIT_TIMEOUT:-10}
# Helper to poll a URL until it becomes responsive (optional timeout in seconds).
wait_for_service() {
  local url="$1"
  local timeout="${2:-0}"
  local name="${3:-service}"

  if ! [[ "$timeout" =~ ^[0-9]+$ ]]; then
    timeout=0
  fi

  local start_time
  start_time=$(date +%s)

  while true; do
    if curl -fsS "$url" >/dev/null 2>&1; then
      return 0
    fi
    if [ "$timeout" -gt 0 ]; then
      local now
      now=$(date +%s)
      if (( now - start_time >= timeout )); then
        return 1
      fi
    fi
    sleep 2
  done
}
# Simple flag parsing for server URL override
if [ $# -gt 0 ]; then
  case "$1" in
    --server-url|--vila-url)
      if [ -n "${2:-}" ]; then
        VILA_URL="$2"
        shift 2
      else
        echo "Missing value for $1" >&2
        exit 1
      fi ;;
    --help|-h)
      echo "Usage: $0 [--server-url URL]" ; exit 0 ;;
  esac
fi

echo "Using vision server URL: ${VILA_URL}"

START_CONTAINER=1
case "$VILA_URL" in
  http://localhost:*|http://localhost|http://127.0.0.1:*|http://127.0.0.1)
    START_CONTAINER=1 ;;
  *)
    START_CONTAINER=0 ;;
esac

LOG_PID=""
if [ "$START_CONTAINER" -eq 1 ]; then
  echo "Step 1: Starting VILA Vision Server in container (URL: ${VILA_URL})..."
  echo "   Primary backend: Cosmos @ ${COSMOS_HOST} (${COSMOS_MODEL})"
  docker run -d --rm --runtime nvidia --name vila_vision \
    --network=host \
    -v /home/gildas01:/workspace \
    -v vila_models:/data/models \
    -e COSMOS_HOST="${COSMOS_HOST}" \
    -e COSMOS_MODEL="${COSMOS_MODEL}" \
    -e OLLAMA_REMOTE_HOST="${OLLAMA_REMOTE_HOST}" \
    -e OLLAMA_REMOTE_MODEL="${OLLAMA_REMOTE_MODEL}" \
    -e OLLAMA_LOCAL_HOST="${OLLAMA_LOCAL_HOST}" \
    -e OLLAMA_LOCAL_MODEL="${OLLAMA_LOCAL_MODEL}" \
    dustynv/nano_llm:r36.4.0 \
    python3 /workspace/vila_container_node.py

  echo "Waiting for VILA to load — streaming container logs (Ctrl+C to abort):"
  docker logs -f vila_vision &
  LOG_PID=$!

  if wait_for_service "${VILA_URL}/health" "${VILA_WAIT_TIMEOUT}" "VILA"; then
    echo "✓ VILA is ready!"
  else
    echo "⚠️  VLM server at ${VILA_URL} did not come online within ${VILA_WAIT_TIMEOUT}s."
    [ ! -z "$LOG_PID" ] && kill $LOG_PID >/dev/null 2>&1 || true
    docker stop vila_vision >/dev/null 2>&1
    exit 1
  fi
else
  echo "Step 1: Using external VLM server at ${VILA_URL} (skipping container)."
  echo "   Checking if server is reachable (may need time to warm up)..."

  # For external servers, just verify we can reach it (any HTTP response = reachable)
  # The bridge will handle retries and automatic fallback to Ollama
  HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" --max-time 5 "${VILA_URL}/generate" 2>/dev/null || echo "000")

  if [ "$HTTP_CODE" != "000" ]; then
    echo "✓ External VLM server is reachable at ${VILA_URL} (HTTP $HTTP_CODE)"
    echo "   Server will initialize fully on first request"
    echo "   Ollama fallback is enabled if server fails"
  else
    echo "⚠️  External VLM server not reachable at ${VILA_URL}"
    echo "   Will use Ollama fallback for all vision requests"
  fi
fi

[ ! -z "$LOG_PID" ] && kill $LOG_PID >/dev/null 2>&1 || true

echo ""
echo "Step 2: Starting hardware nodes..."

# Camera
echo "🎥 Starting camera..."
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video0 \
  -p image_size:=[640,480] \
  -r image_raw:=/camera/image_raw &
CAMERA_PID=$!
sleep 2

# Motors (JetBot hardware or fallback)
if [ -f "$HOME/jetbot_vlm_ws/install/setup.bash" ]; then
    echo "🚗 Starting JetBot hardware (motor controller) via jetbot_hardware.launch.py..."
    source "$HOME/jetbot_vlm_ws/install/setup.bash"
    if [ ! -e "/dev/ttyACM0" ]; then
        echo "⚠️  Motor controller port /dev/ttyACM0 not found — check USB connection/power."
    fi
    ros2 launch jetbot_hardware jetbot_hardware.launch.py enable_lidar:=false enable_camera:=false &
    MOTOR_PID=$!
    sleep 2
elif [ -f "$HOME/workspaces/isaac_ros-dev/src/jetbot_ros/scripts/jetbot_motors.py" ]; then
    echo "🚗 Starting motors (legacy jetbot_motors.py)..."
    python3 $HOME/workspaces/isaac_ros-dev/src/jetbot_ros/scripts/jetbot_motors.py &
    MOTOR_PID=$!
    sleep 1
else
    echo "⚠️  No motor driver found (jetbot_hardware or jetbot_motors.py missing)."
    echo "   /cmd_vel will be published but nothing will drive the wheels."
fi

# RPLidar (if available)
if [ -e "/dev/ttyUSB0" ]; then
    echo "📡 Starting RPLidar..."
    ros2 run rplidar_ros rplidar_composition --ros-args \
      -p serial_port:=/dev/ttyUSB0 \
      -p serial_baudrate:=115200 \
      -p frame_id:=laser_frame \
      -p angle_compensate:=true \
      -p scan_mode:=Standard &
    LIDAR_PID=$!
    sleep 2
    echo "✓ RPLidar running - Safety layer active!"
else
    echo "⚠️  RPLidar not found at /dev/ttyUSB0"
    echo "   Running VISION-ONLY mode (no lidar safety)"
    LIDAR_PID=""
fi

echo ""
echo "Step 3: Starting VILA-ROS2 Bridge with Enhanced Navigation..."

# Check if TTS node is already running (from auto-started conversation system)
if [ "${TTS:-0}" != "0" ]; then
  if pgrep -f "tts_node" > /dev/null; then
    echo "🔊 TTS node already running (reusing existing node)"
    echo "   Narration will use the existing /tts/speak topic"
    TTS_PID=""
  else
    echo "🔊 Starting TTS node (jetbot_conversation/tts_node)..."
    # Select TTS config based on narration language
    if [ "${NARRATION_LANGUAGE}" = "fr" ]; then
      TTS_PARAMS_FILE=${TTS_PARAMS_FILE:-$HOME/jetbot_vlm_ws/src/jetbot_conversation/config/conversation_french.yaml}
      echo "   Using French TTS (Pascal voice)"
    else
      TTS_PARAMS_FILE=${TTS_PARAMS_FILE:-$HOME/jetbot_vlm_ws/src/jetbot_conversation/config/conversation.yaml}
      echo "   Using English TTS (Aria voice)"
    fi
    if [ -f "$HOME/jetbot_vlm_ws/install/setup.bash" ]; then
      source "$HOME/jetbot_vlm_ws/install/setup.bash"
    fi
    ros2 run jetbot_conversation tts_node --ros-args --params-file "$TTS_PARAMS_FILE" &
    TTS_PID=$!
    sleep 1
  fi
fi

# Enhanced navigation parameters
# - Tighter safety cone (30° instead of 60°) for better forward detection
# - Smart stuck detection after 4 cycles
# - Angle-based turnaround for ~180° turn (computed from angular speed)
# - Increased angular speed for sharper turns
# - Periodic 360° scanning for spatial awareness (every 10 cycles = ~30 seconds)
# - Tight space detection and faster escape (corridor width < 0.6m)
# - Scene narration for entertainment (every 15 seconds, with TTS)
# Allow overriding nav/tight-space behavior via env vars
# Ensure floats are passed as DOUBLE (with decimal) to ROS2
floatify() {
  local v="$1"
  if [[ "$v" =~ ^-?[0-9]+$ ]]; then
    echo "${v}.0"
  else
    echo "$v"
  fi
}


LIN_SPEED=$(floatify "${LIN_SPEED:-0.20}")
# Active turn speed for navigating tight spaces
ANG_SPEED=$(floatify "${ANG_SPEED:-0.45}")
# Maintain momentum when near obstacles (confident navigation)
WARN_SPEED_SCALE=$(floatify "${WARN_SPEED_SCALE:-0.90}")
# Gentle backup speed to avoid violent jerks (50% of forward speed)
BACKUP_SPEED_SCALE=$(floatify "${BACKUP_SPEED_SCALE:-0.50}")
# Focused front cone for better tight space navigation
FRONT_CONE_DEG=$(floatify "${FRONT_CONE_DEG:-35.0}")
SIDE_CONE_DEG=$(floatify "${SIDE_CONE_DEG:-45.0}")
# Quick stuck detection for faster recovery
STUCK_THRESHOLD=${STUCK_THRESHOLD:-4}
# Larger turnaround angle for reliable dead-end recovery (~185°)
TURNAROUND_ANGLE_DEG=$(floatify "${TURNAROUND_ANGLE_DEG:-185.0}")
# Disable periodic 360° scans (they don't influence direction yet)
SCAN_INTERVAL_CYCLES=${SCAN_INTERVAL_CYCLES:-0}
SCAN_DURATION=$(floatify "${SCAN_DURATION:-4.0}")
CORRIDOR_WIDTH=$(floatify "${CORRIDOR_WIDTH:-1.2}")
# Living room mode: stay confident even when close to furniture
TIGHT_SPACE_BOOST=$(floatify "${TIGHT_SPACE_BOOST:-0.95}")
# Smooth movement tweaks to ramp velocities for calmer driving
SMOOTHING_ENABLED=${SMOOTHING_ENABLED:-true}
SMOOTH_LINEAR_DELTA=$(floatify "${SMOOTH_LINEAR_DELTA:-0.02}")
SMOOTH_ANGULAR_DELTA=$(floatify "${SMOOTH_ANGULAR_DELTA:-0.08}")
SMOOTH_TIMER_PERIOD=$(floatify "${SMOOTH_TIMER_PERIOD:-0.10}")
# Narration: Set NARRATION_ENABLED=false to disable completely
NARRATION_ENABLED=${NARRATION_ENABLED:-true}
# Keep less chatty
NARRATION_INTERVAL=$(floatify "${NARRATION_INTERVAL:-60.0}")
# Casual tone
NARRATION_STYLE=${NARRATION_STYLE:-casual}
# Language: en (English) or fr (French)
NARRATION_LANGUAGE=${NARRATION_LANGUAGE:-en}

# Adaptive Transformer parameters (IBM Granite TTM-R2)
# Set ADAPTIVE_TRANSFORMER_ENABLED=true to enable predictive smoothing
ADAPTIVE_TRANSFORMER_ENABLED=${ADAPTIVE_TRANSFORMER_ENABLED:-false}
TRANSFORMER_CONTEXT_LENGTH=${TRANSFORMER_CONTEXT_LENGTH:-64}
TRANSFORMER_PREDICTION_HORIZON=${TRANSFORMER_PREDICTION_HORIZON:-10}
TRANSFORMER_UPDATE_RATE=$(floatify "${TRANSFORMER_UPDATE_RATE:-5.0}")
TRANSFORMER_MODEL=${TRANSFORMER_MODEL:-ibm-granite/granite-timeseries-ttm-r2}
# Mode: 'smoothing_params' (recommended) or 'cmd_vel_filtered'
TRANSFORMER_PUBLISH_MODE=${TRANSFORMER_PUBLISH_MODE:-smoothing_params}
WALL_FOLLOW_ENABLED=${WALL_FOLLOW_ENABLED:-${ADAPTIVE_TRANSFORMER_ENABLED}}
WALL_FOLLOW_TARGET_DISTANCE=$(floatify "${WALL_FOLLOW_TARGET_DISTANCE:-0.35}")
WALL_FOLLOW_ACTIVATION_DISTANCE=$(floatify "${WALL_FOLLOW_ACTIVATION_DISTANCE:-0.75}")
WALL_FOLLOW_EXIT_DISTANCE=$(floatify "${WALL_FOLLOW_EXIT_DISTANCE:-1.15}")
WALL_FOLLOW_GAIN=$(floatify "${WALL_FOLLOW_GAIN:-0.45}")
WALL_FOLLOW_MAX_ADJUST=$(floatify "${WALL_FOLLOW_MAX_ADJUST:-0.30}")

echo "   Movement smoothing: ${SMOOTHING_ENABLED} (Δlinear=${SMOOTH_LINEAR_DELTA} m/s, Δangular=${SMOOTH_ANGULAR_DELTA} rad/s, period=${SMOOTH_TIMER_PERIOD}s)"
if [ "${ADAPTIVE_TRANSFORMER_ENABLED}" = "true" ]; then
  echo "   Adaptive transformer: ENABLED (model=${TRANSFORMER_MODEL}, context=${TRANSFORMER_CONTEXT_LENGTH}, mode=${TRANSFORMER_PUBLISH_MODE})"
fi
echo "   Wall following: ${WALL_FOLLOW_ENABLED} (target=${WALL_FOLLOW_TARGET_DISTANCE}m, activation=${WALL_FOLLOW_ACTIVATION_DISTANCE}m → exit=${WALL_FOLLOW_EXIT_DISTANCE}m, gain=${WALL_FOLLOW_GAIN}, max_adj=${WALL_FOLLOW_MAX_ADJUST})"

# Optionally start adaptive transformer controller
if [ "${ADAPTIVE_TRANSFORMER_ENABLED}" = "true" ]; then
  echo "🤖 Starting Adaptive Transformer Controller..."

  # Check if granite-tsfm is installed
  if ! python3 -c "import tsfm_public" 2>/dev/null; then
    echo "⚠️  granite-tsfm not found. Installing (this may take a few minutes)..."
    pip3 install granite-tsfm --user

    if [ $? -ne 0 ]; then
      echo "❌ Failed to install granite-tsfm. Disabling transformer."
      ADAPTIVE_TRANSFORMER_ENABLED=false
    else
      echo "✓ granite-tsfm installed successfully"
    fi
  fi

  if [ "${ADAPTIVE_TRANSFORMER_ENABLED}" = "true" ]; then
    python3 /home/gildas01/adaptive_transformer_controller.py \
      --ros-args \
      -p transformer_enabled:=true \
      -p context_length:=${TRANSFORMER_CONTEXT_LENGTH} \
      -p prediction_horizon:=${TRANSFORMER_PREDICTION_HORIZON} \
      -p model_path:=${TRANSFORMER_MODEL} \
      -p publish_mode:=${TRANSFORMER_PUBLISH_MODE} \
      -p update_rate:=${TRANSFORMER_UPDATE_RATE} \
      -p use_cpu:=true &
    TRANSFORMER_PID=$!
    sleep 3
    echo "✓ Adaptive Transformer Controller started (PID: $TRANSFORMER_PID)"
  fi
fi

  python3 /home/gildas01/vila_robot_bridge_with_lidar_enhanced.py \
    --ros-args \
    -p vila_url:=${VILA_URL} \
  -p ollama_url:=${OLLAMA_LOCAL_HOST} \
  -p ollama_model:=${OLLAMA_LOCAL_MODEL} \
  -p safety_distance:=0.15 \
  -p warning_distance:=0.25 \
  -p front_cone_deg:=${FRONT_CONE_DEG} \
  -p side_cone_deg:=${SIDE_CONE_DEG} \
  -p min_valid_range:=0.15 \
  -p linear_speed:=${LIN_SPEED} \
  -p angular_speed:=${ANG_SPEED} \
  -p warning_speed_scale:=${WARN_SPEED_SCALE} \
  -p backup_speed_scale:=${BACKUP_SPEED_SCALE} \
  -p stuck_threshold:=${STUCK_THRESHOLD} \
  -p turnaround_angle_deg:=${TURNAROUND_ANGLE_DEG} \
  -p scan_interval_cycles:=${SCAN_INTERVAL_CYCLES} \
  -p scan_duration:=${SCAN_DURATION} \
  -p corridor_width_threshold:=${CORRIDOR_WIDTH} \
  -p tight_space_boost:=${TIGHT_SPACE_BOOST} \
  -p wall_follow_enabled:=${WALL_FOLLOW_ENABLED} \
  -p wall_follow_target_distance:=${WALL_FOLLOW_TARGET_DISTANCE} \
  -p wall_follow_activation_distance:=${WALL_FOLLOW_ACTIVATION_DISTANCE} \
  -p wall_follow_exit_distance:=${WALL_FOLLOW_EXIT_DISTANCE} \
  -p wall_follow_gain:=${WALL_FOLLOW_GAIN} \
  -p wall_follow_max_adjust:=${WALL_FOLLOW_MAX_ADJUST} \
  -p smoothing_enabled:=${SMOOTHING_ENABLED} \
  -p smooth_linear_delta:=${SMOOTH_LINEAR_DELTA} \
  -p smooth_angular_delta:=${SMOOTH_ANGULAR_DELTA} \
  -p smooth_timer_period:=${SMOOTH_TIMER_PERIOD} \
  -p adaptive_smoothing:=${ADAPTIVE_SMOOTHING:-false} \
  -p narration_enabled:=${NARRATION_ENABLED} \
  -p narration_interval:=${NARRATION_INTERVAL} \
  -p narration_style:=${NARRATION_STYLE} \
  -p narration_language:=${NARRATION_LANGUAGE}

# Cleanup on exit
echo ""
echo "Shutting down..."
docker stop vila_vision 2>/dev/null
[ ! -z "$CAMERA_PID" ] && kill $CAMERA_PID 2>/dev/null
[ ! -z "$MOTOR_PID" ] && kill $MOTOR_PID 2>/dev/null
[ ! -z "$LIDAR_PID" ] && kill $LIDAR_PID 2>/dev/null
[ ! -z "$LOG_PID" ] && kill $LOG_PID 2>/dev/null
[ ! -z "$TTS_PID" ] && kill $TTS_PID 2>/dev/null
[ ! -z "$TRANSFORMER_PID" ] && kill $TRANSFORMER_PID 2>/dev/null
pkill -9 -f ros2
echo "✓ Stopped"
