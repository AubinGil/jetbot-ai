#!/bin/bash
# Complete JetBot Conversation Pipeline with Camera and TTS (single-instance with clean shutdown)
# Use strict flags but avoid -u because ROS setup scripts reference unset vars intentionally
set -Ee -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JETBOT_WS="${JETBOT_WS:-/home/gildas01/jetbot_vlm_ws}"
CONVERSATION_NODE_SRC="${JETBOT_WS}/src/jetbot_conversation/jetbot_conversation/conversation_node.py"
ADD_TOOLS_SCRIPT="${SCRIPT_DIR}/add_tools_to_conversation.py"

check_openai_key() {
  if [[ -z "${OPENAI_API_KEY:-}" ]]; then
    echo "⚠️  OPENAI_API_KEY not set. Remote Cosmos/OpenAI endpoint will be skipped; responses rely on local models."
    echo "    Export OPENAI_API_KEY before running for best accuracy."
  else
    echo "🔑 Detected OPENAI_API_KEY – enabling remote Cosmos/OpenAI reasoning."
  fi
}

ensure_conversation_tools() {
  if [[ ! -f "${CONVERSATION_NODE_SRC}" ]]; then
    echo "⚠️  Conversation node source not found at ${CONVERSATION_NODE_SRC}; cannot verify tool support."
    return
  fi

  if grep -q "from jetbot_conversation.tools import ToolRegistry" "${CONVERSATION_NODE_SRC}"; then
    echo "🌐 Conversation tools already enabled (web search, weather, calculator)."
    return
  fi

  if [[ ! -f "${ADD_TOOLS_SCRIPT}" ]]; then
    echo "⚠️  Missing ${ADD_TOOLS_SCRIPT}; cannot auto-enable web tools."
    return
  fi

  echo "🛠️  Enabling web/tool capabilities for the conversation node..."
  if python3 "${ADD_TOOLS_SCRIPT}"; then
    echo "🔁 Rebuilding jetbot_conversation so tool changes take effect..."
    if ! (cd "${JETBOT_WS}" && colcon build --packages-select jetbot_conversation >/dev/null); then
      echo "⚠️  Failed to rebuild jetbot_conversation; tool support may be incomplete."
    fi
  else
    echo "⚠️  Could not patch conversation node with tool support."
  fi
}

echo "🤖 Starting Complete JetBot Conversation System..."

# Configuration
# ----------------------------------------------------------------
# Network & Remote Services
OLLAMA_REMOTE_HOST=${OLLAMA_REMOTE_HOST:-http://192.168.2.29:11434}
NGROK_HOST=${NGROK_HOST:-}  # Optional: e.g., https://your-ngrok-url.ngrok-free.app
COSMOS_HOST=${COSMOS_HOST:-http://192.168.2.16:8000}
COSMOS_MODEL=${COSMOS_MODEL:-nvidia/cosmos-reason1-7b}
# Models
MODEL_CHAT=${MODEL_CHAT:-llama3.1:latest}
MODEL_VISION=${MODEL_VISION:-granite4:latest}
# Hardware Devices
CAMERA_DEV=${CAMERA_DEV:-/dev/video0}
MOTOR_DEV=${MOTOR_DEV:-/dev/ttyACM0}

# Language selection (default: English)
# Set CONVERSATION_LANGUAGE=fr for French
CONVERSATION_LANGUAGE=${CONVERSATION_LANGUAGE:-en}
# Voice-triggered mode controller (set MODE_CONTROL_ENABLED=0 to disable)
MODE_CONTROL_ENABLED=${MODE_CONTROL_ENABLED:-1}

# Parse optional args
TELEOP_MODE=""
DRY_RUN=0

usage() {
  echo "Usage: $0 [options]"
  echo "Options:"
  echo "  --teleop <mode>    Start teleop ('joy' or 'keyboard')"
  echo "  --dry-run          Check environment and exit without starting"
  echo "  --help             Show this help message"
  exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --teleop)
      TELEOP_MODE="$2"; shift 2 ;;
    --dry-run)
      DRY_RUN=1; shift ;;
    --help|-h)
      usage ;;
    *)
      echo "Unknown argument: $1"; usage ;;
  esac
done

# Acquire single-instance lock
exec 9>/tmp/start_conversation_complete.lock
if ! flock -n 9; then
  echo "Another conversation session is already running. Exiting."
  exit 1
fi

check_openai_key
ensure_conversation_tools

# Ensure libs are discoverable for audio backends (quietly, no sudo prompt)
if command -v ldconfig >/dev/null 2>&1; then
  ldconfig 2>/dev/null || true
fi

# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source ~/jetbot_vlm_ws/install/setup.bash

# Select config file based on language
if [ "$CONVERSATION_LANGUAGE" = "fr" ]; then
  CONFIG_FILE="/home/gildas01/jetbot_vlm_ws/src/jetbot_conversation/config/conversation_french.yaml"
  echo "🇫🇷 Using French configuration (wake word: 'bonjour')"
else
  CONFIG_FILE="/home/gildas01/jetbot_vlm_ws/src/jetbot_conversation/config/conversation.yaml"
  echo "🇺🇸 Using English configuration (wake word: 'hello')"
fi

CAMERA_PID=""
TELEOP_PID=""
HARDWARE_PID=""
VOICE_CONTROLLER_PID=""
OLLAMA_STARTED=0

cleanup() {
  echo "\n🧹 Cleaning up..."

  # Stop voice controller if enabled
  if [[ -n "${VOICE_CONTROLLER_PID}" ]] && kill -0 "${VOICE_CONTROLLER_PID}" 2>/dev/null; then
    kill "${VOICE_CONTROLLER_PID}" 2>/dev/null || true
    wait "${VOICE_CONTROLLER_PID}" 2>/dev/null || true
  fi
  # Make sure any running gesture/autopilot modes stop
  pkill -f "start_gesture_complete.sh" 2>/dev/null || true
  pkill -f "onnx_obstacle_avoidance_tight_spaces.py" 2>/dev/null || true

  # Stop background camera if we started it
  if [[ -n "${CAMERA_PID}" ]] && kill -0 "${CAMERA_PID}" 2>/dev/null; then
    kill "${CAMERA_PID}" 2>/dev/null || true
    wait "${CAMERA_PID}" 2>/dev/null || true
  fi
  # Stop hardware launch (motor controller) if running
  if [[ -n "${HARDWARE_PID}" ]] && kill -0 "${HARDWARE_PID}" 2>/dev/null; then
    kill "${HARDWARE_PID}" 2>/dev/null || true
    wait "${HARDWARE_PID}" 2>/dev/null || true
  fi
  # If we started ollama here, stop it
  if [[ "${OLLAMA_STARTED}" -eq 1 ]]; then
    pkill -x ollama 2>/dev/null || true
  fi
  # Stop teleop if running
  if [[ -n "${TELEOP_PID}" ]] && kill -0 "${TELEOP_PID}" 2>/dev/null; then
    kill "${TELEOP_PID}" 2>/dev/null || true
    wait "${TELEOP_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# Make sure Ollama is running (start if missing, and remember we did)
if ! pgrep -x "ollama" > /dev/null; then
  echo "⚠️  Starting Ollama..."
  ollama serve &
  OLLAMA_STARTED=1
  sleep 3
fi

# Start camera in background, tied to this script lifecycle
if [ -e "$CAMERA_DEV" ]; then
  echo "📷 Starting camera on $CAMERA_DEV..."
  if [ "$DRY_RUN" -eq 0 ]; then
    ros2 run v4l2_camera v4l2_camera_node \
      --ros-args \
      -p video_device:="$CAMERA_DEV" \
      -p image_size:="[640,480]" \
      -p camera_frame_id:="camera_link" \
      -p output_encoding:="rgb8" \
      -r /image_raw:=/camera/image_raw \
      -r /camera_info:=/camera/camera_info &
    CAMERA_PID=$!
    sleep 2
  fi
else
  echo "⚠️  Camera device $CAMERA_DEV not found! Vision features may fail."
fi

# Ensure motor controller is running (via hardware launch)
if [ -e "$MOTOR_DEV" ]; then
  echo "🛞 Starting motor controller (found $MOTOR_DEV)..."
  if [ "$DRY_RUN" -eq 0 ]; then
    ros2 launch jetbot_hardware jetbot_hardware.launch.py \
      enable_camera:=false enable_lidar:=false enable_pose_gesture:=false &
    HARDWARE_PID=$!
    sleep 2
  fi
else
  echo "⚠️  Motor controller $MOTOR_DEV not found! Robot will not move."
fi

if [ "$DRY_RUN" -eq 1 ]; then
  echo "✅ Dry run complete. Environment checked."
  exit 0
fi

if [ "${MODE_CONTROL_ENABLED}" -ne 0 ]; then
  echo "🎛️  Starting voice mode controller..."
  python3 ~/voice_mode_controller.py &
  VOICE_CONTROLLER_PID=$!
  sleep 2

  echo ""
  echo "🎮 Voice Mode Control enabled!"
  echo "   - 'follow me' → Start gesture-based following"
  echo "   - 'autopilot' → Start obstacle avoidance mode"
  echo "   - 'stop'      → Halt the active mode"
  echo ""
fi

echo ""
echo "🚀 Launching conversation pipeline..."
echo ""
if [ "$CONVERSATION_LANGUAGE" = "fr" ]; then
  echo "Dites: 'bonjour' (mot de réveil) puis votre commande"
  echo "Exemples:"
  echo "  - bonjour, que vois-tu?"
  echo "  - bonjour, avance"
  echo "  - bonjour, tourne à gauche"
else
  echo "Say: 'hello' (wake word) then your command"
  echo "Examples:"
  echo "  - hello, what do you see?"
  echo "  - hello, move forward"
  echo "  - hello, turn left"
fi
echo ""

# Optionally start teleop alongside conversation
if [[ -n "$TELEOP_MODE" ]]; then
  case "$TELEOP_MODE" in
    joy)
      echo "🎮 Starting teleop (joy) in background..."
      ros2 launch jetbot_conversation teleop_joy.launch.py &
      TELEOP_PID=$!
      ;;
    keyboard)
      echo "⌨️  Starting teleop (keyboard) in background..."
      ros2 launch jetbot_conversation teleop_keyboard.launch.py &
      TELEOP_PID=$!
      ;;
    *)
      echo "⚠️  Unknown teleop mode: $TELEOP_MODE (use 'joy' or 'keyboard')" ;;
  esac
fi

# Quick health check: Remote Ollama / Ngrok availability and models
PRIMARY_OLLAMA_URL="$OLLAMA_REMOTE_HOST"

# If Ngrok is set, prioritize checking that
if [ -n "$NGROK_HOST" ]; then
  echo "🔎 Checking Ngrok host at ${NGROK_HOST}..."
  if curl -sS --max-time 3 "${NGROK_HOST}/health" >/dev/null 2>&1; then
    echo "   ✓ Ngrok host reachable"
    PRIMARY_OLLAMA_URL="$NGROK_HOST"
  else
    echo "   ⚠️  Ngrok host unreachable, falling back to ${OLLAMA_REMOTE_HOST}"
  fi
fi

if command -v curl >/dev/null 2>&1; then
  echo "🔎 Checking Primary Ollama at ${PRIMARY_OLLAMA_URL}..."
  TAGS_JSON=$(curl -sS --max-time 2 "${PRIMARY_OLLAMA_URL}/api/tags" || true)
  if [[ -n "$TAGS_JSON" ]]; then
    # Extract model names in a simple way without jq
    MODELS=$(echo "$TAGS_JSON" | grep -o '"name"\s*:\s*"[^"]*"' | sed 's/.*:"\(.*\)"/\1/' | tr '\n' ' ')
    echo "   Available models: $MODELS"
    for M in "$MODEL_CHAT" "$MODEL_VISION"; do
      if echo "$MODELS" | grep -q "$M"; then
        echo "   ✓ $M"
      else
        echo "   ⚠️  Missing model: $M (run: ollama pull $M on the remote host)"
      fi
    done
  else
    echo "   ⚠️  Could not reach Ollama at ${PRIMARY_OLLAMA_URL} (is it running?)"
  fi
  
  # Check local Ollama for fallback
  echo "🔎 Checking local Ollama..."
  LOCAL_TAGS=$(curl -sS --max-time 2 "http://localhost:11434/api/tags" || true)
  if [[ -n "$LOCAL_TAGS" ]]; then
    if echo "$LOCAL_TAGS" | grep -q "\"name\"\s*:\s*\"$MODEL_VISION\""; then
      echo "   ✓ Local $MODEL_VISION available"
    else
      echo "   ⚠️  Missing local $MODEL_VISION (run: ollama pull $MODEL_VISION)"
    fi
  fi
fi

# Launch conversation (blocks until Ctrl+C)
ros2 launch jetbot_conversation conversation.launch.py \
  config_file:="${CONFIG_FILE}"
