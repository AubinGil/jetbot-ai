#!/bin/bash
#
# Legacy ONNXRuntime Obstacle Avoidance (MobileNet classifier)
# This script launches the original ONNX model instead of the TensorRT YOLO server
# so you can compare behavior between the two pipelines.
#
# Environment overrides:
#   ONNX_MODEL_PATH=/path/to/model.onnx
#   FORWARD_SPEED=0.12
#   TURN_SPEED=0.30
#   DECISION_INTERVAL=1.0
#   CONFIDENCE_THRESHOLD=0.7
#   ENABLE_ADAPTIVE_TRANSFORMER=true
#

set -euo pipefail

ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-37}
export ROS_DOMAIN_ID

ONNX_MODEL_PATH=${ONNX_MODEL_PATH:-$HOME/obstacle_models/mobilenet_collision_model.onnx}
ENABLE_ADAPTIVE_TRANSFORMER=${ENABLE_ADAPTIVE_TRANSFORMER:-false}

if [ ! -f "$ONNX_MODEL_PATH" ]; then
    echo "ERROR: ONNX model not found at $ONNX_MODEL_PATH"
    echo "Set ONNX_MODEL_PATH to the Mobilenet classifier you want to use."
    exit 1
fi

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

CAMERA_PID=""
MOTOR_PID=""
TRANSFORMER_PID=""

cleanup() {
    local exit_code=$?
    trap - EXIT INT TERM
    echo ""
    echo "Shutting down..."

    if [[ -n "$MOTOR_PID" ]] && kill -0 "$MOTOR_PID" 2>/dev/null; then
        kill "$MOTOR_PID" 2>/dev/null || true
        wait "$MOTOR_PID" 2>/dev/null || true
    fi
    if [[ -n "$CAMERA_PID" ]] && kill -0 "$CAMERA_PID" 2>/dev/null; then
        kill "$CAMERA_PID" 2>/dev/null || true
        wait "$CAMERA_PID" 2>/dev/null || true
    fi
    if [[ -n "$TRANSFORMER_PID" ]] && kill -0 "$TRANSFORMER_PID" 2>/dev/null; then
        kill "$TRANSFORMER_PID" 2>/dev/null || true
        wait "$TRANSFORMER_PID" 2>/dev/null || true
    fi

    pkill -f "motor_controller" 2>/dev/null || true
    pkill -f "camera_node" 2>/dev/null || true
    pkill -f "onnx_obstacle_avoidance.py" 2>/dev/null || true
    pkill -f "onnx_obstacle_avoidance_gpu.py" 2>/dev/null || true
    pkill -f "adaptive_transformer_controller" 2>/dev/null || true

    sleep 1
    exit "$exit_code"
}

trap cleanup EXIT INT TERM

echo -e "${GREEN}[1/4]${NC} Sourcing ROS2 workspace..."
set +u
source /opt/ros/humble/setup.bash
if [ -d "$HOME/jetbot_gesture_ws/install" ]; then
    source "$HOME/jetbot_gesture_ws/install/setup.bash"
fi
set -u

echo -e "${GREEN}[2/4]${NC} Starting camera..."
CAMERA_CHECK=$(timeout 1 ros2 topic info /camera/image_raw 2>&1 || true)
if echo "$CAMERA_CHECK" | grep -q "Publisher count: [1-9]"; then
    echo -e "  ${GREEN}✓${NC} Camera already running"
else
    echo "  Starting camera node..."
    ros2 run jetbot_gesture camera_node &
    CAMERA_PID=$!
    sleep 2
    CAMERA_VERIFY=$(timeout 1 ros2 topic info /camera/image_raw 2>&1 || true)
    if echo "$CAMERA_VERIFY" | grep -q "Publisher count: [1-9]"; then
        echo -e "  ${GREEN}✓${NC} Camera started"
    else
        echo -e "  ${YELLOW}⚠${NC} Camera topic not detected. Check hardware."
    fi
fi

if [ "$ENABLE_ADAPTIVE_TRANSFORMER" = "true" ]; then
    echo -e "${GREEN}[3/4]${NC} Starting Adaptive Transformer Controller..."
    if python3 -c "import tsfm_public" 2>/dev/null; then
        if pgrep -f "adaptive_transformer_controller" > /dev/null; then
            echo -e "  ${GREEN}✓${NC} Adaptive Transformer already running"
        else
            python3 ~/adaptive_transformer_controller.py --ros-args \
                -p transformer_enabled:=true \
                -p context_length:=64 \
                -p prediction_horizon:=10 \
                -p update_rate:=5.0 \
                -p publish_mode:=cmd_vel_filtered \
                -p use_cpu:=true &
            TRANSFORMER_PID=$!
            sleep 2
            echo -e "  ${GREEN}✓${NC} Adaptive Transformer started"
        fi
    else
        echo -e "  ${YELLOW}⚠${NC} granite-tsfm not installed. Continuing without transformer..."
        ENABLE_ADAPTIVE_TRANSFORMER=false
    fi
else
    echo -e "${BLUE}[3/4]${NC} Adaptive Transformer disabled (set ENABLE_ADAPTIVE_TRANSFORMER=true to enable)"
fi

echo -e "${GREEN}[3.5/4]${NC} Starting motor controller..."
if pgrep -f "motor_controller" > /dev/null; then
    echo -e "  ${YELLOW}⚠${NC} Motor controller already running - may need manual restart"
else
    if [ "$ENABLE_ADAPTIVE_TRANSFORMER" = "true" ]; then
        ros2 run jetbot_hardware motor_controller --ros-args \
            -r cmd_vel:=cmd_vel_filtered &
    else
        ros2 run jetbot_hardware motor_controller &
    fi
    MOTOR_PID=$!
    sleep 1
    echo -e "  ${GREEN}✓${NC} Motor controller ready"
fi

# FASTER MODE - Same exploration logic as safe mode, just faster speeds
FORWARD_SPEED=${FORWARD_SPEED:-0.15}
TURN_SPEED=${TURN_SPEED:-0.50}
DECISION_INTERVAL=${DECISION_INTERVAL:-0.7}
CONFIDENCE_THRESHOLD=${CONFIDENCE_THRESHOLD:-0.75}
STALE_IMAGE_TIMEOUT=${STALE_IMAGE_TIMEOUT:-0.75}
SMOOTHING_WINDOW=${SMOOTHING_WINDOW:-5}
CAUTIOUS_SPEED_SCALE=${CAUTIOUS_SPEED_SCALE:-0.35}

# Committed rotation - same exploration strategy as safe mode
SPIN_RECOVERY_THRESHOLD=${SPIN_RECOVERY_THRESHOLD:-999}  # Never reverse - commit to one direction
SPIN_RECOVERY_MULTIPLIER=${SPIN_RECOVERY_MULTIPLIER:-1.0}  # Consistent speed
SPIN_RECOVERY_PAUSE_CYCLES=${SPIN_RECOVERY_PAUSE_CYCLES:-0}  # No pausing
MAX_RECOVERY_SPIN_CYCLES=${MAX_RECOVERY_SPIN_CYCLES:-8}  # Allow full 360° scan
BACKOFF_CYCLES=${BACKOFF_CYCLES:-3}  # Back up with turning
BACKOFF_SPEED=${BACKOFF_SPEED:-0.18}  # Fast repositioning

echo ""
echo "================================================"
echo "  FASTER MODE - Obstacle Avoidance (ONNXRuntime)"
echo "================================================"
echo "Model: $ONNX_MODEL_PATH"
echo "Adaptive Transformer: $ENABLE_ADAPTIVE_TRANSFORMER"
echo ""
echo "SPEEDS (faster than safe mode):"
echo "  Forward:    $FORWARD_SPEED m/s"
echo "  Turn:       $TURN_SPEED rad/s"
echo "  Backoff:    $BACKOFF_SPEED m/s"
echo ""
echo "SETTINGS:"
echo "  Confidence min:  $CONFIDENCE_THRESHOLD"
echo "  Decision rate:   ${DECISION_INTERVAL}s"
echo "  Smoothing:       $SMOOTHING_WINDOW frames"
echo ""
echo "EXPLORATION (randomized + committed rotation):"
echo "  Strategy:        Never reverses - commits to random direction"
echo "  Max spins:       $MAX_RECOVERY_SPIN_CYCLES cycles (360° scan)"
echo "  Backoff cycles:  $BACKOFF_CYCLES (with turning)"
echo "================================================"
echo "Press Ctrl+C to stop"
echo ""

echo -e "${GREEN}[4/4]${NC} Starting ONNX obstacle avoidance node..."
python3 ~/onnx_obstacle_avoidance.py --ros-args \
    -p model_path:=${ONNX_MODEL_PATH} \
    -p forward_speed:=${FORWARD_SPEED} \
    -p turn_speed:=${TURN_SPEED} \
    -p decision_interval:=${DECISION_INTERVAL} \
    -p confidence_threshold:=${CONFIDENCE_THRESHOLD} \
    -p stale_image_timeout:=${STALE_IMAGE_TIMEOUT} \
    -p smoothing_window:=${SMOOTHING_WINDOW} \
    -p cautious_speed_scale:=${CAUTIOUS_SPEED_SCALE} \
    -p spin_recovery_threshold:=${SPIN_RECOVERY_THRESHOLD} \
    -p spin_recovery_multiplier:=${SPIN_RECOVERY_MULTIPLIER} \
    -p spin_recovery_pause_cycles:=${SPIN_RECOVERY_PAUSE_CYCLES} \
    -p max_recovery_spin_cycles:=${MAX_RECOVERY_SPIN_CYCLES} \
    -p backoff_cycles:=${BACKOFF_CYCLES} \
    -p backoff_speed:=${BACKOFF_SPEED}
