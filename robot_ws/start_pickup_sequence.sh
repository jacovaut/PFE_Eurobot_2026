#!/usr/bin/env bash
# start_pickup_sequence.sh
# -------------------------
# Builds local_camera and manip_action_node, then launches:
#   - local_camera_system  (dock server, solver, robot_state_publisher)
#   - manip_actions        (pick / dispense / therm action servers)
#   - pickup_orchestrator  (chains dock → pick on dock success)
#
# Usage:
#   ./start_pickup_sequence.sh [--skip-build] [--lidar]
#
# Options:
#   --skip-build   Skip colcon build (use if already built)
#   --lidar        Also launch the YDLidar driver + pointcloud node

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"   # robot_ws root

SKIP_BUILD=false
ENABLE_LIDAR=false

for arg in "$@"; do
  case $arg in
    --skip-build) SKIP_BUILD=true ;;
    --lidar)      ENABLE_LIDAR=true ;;
  esac
done

# ---------- Build ----------
if [ "$SKIP_BUILD" = false ]; then
  echo ""
  echo "========================================="
  echo "  Building local_camera + manip_action_node"
  echo "========================================="
  cd "$WS_DIR"
  colcon build \
    --symlink-install \
    --packages-select local_camera manip_action_node custom_msgs
fi

# ---------- Source ----------
echo ""
echo "========================================="
echo "  Sourcing workspace"
echo "========================================="
# shellcheck disable=SC1091
source "$WS_DIR/install/setup.bash"

# ---------- Launch nodes ----------
echo ""
echo "========================================="
echo "  Launching camera system + manip servers"
echo "========================================="

# Launch local_camera_system in the background (ros_node, dock_action_server, etc.)
ros2 launch local_camera local_camera_system.launch.py \
  enable_lidar:="$ENABLE_LIDAR" &
CAMERA_PID=$!

# Launch manip action servers in the background
ros2 launch manip_action_node manip_actions.launch.py &
MANIP_PID=$!

# Give nodes a moment to initialise before the orchestrator starts waiting
sleep 3

# ---------- Run orchestrator (foreground — blocks until pick completes) ----------
echo ""
echo "========================================="
echo "  Starting pickup orchestrator"
echo "========================================="
ros2 run local_camera pickup_orchestrator \
  --ros-args \
  -p dock_timeout_sec:=30.0 \
  -p pick_timeout_sec:=0.0

ORCH_EXIT=$?

# ---------- Cleanup ----------
echo ""
echo "========================================="
echo "  Sequence finished (exit $ORCH_EXIT) — shutting down background nodes"
echo "========================================="
kill "$CAMERA_PID" "$MANIP_PID" 2>/dev/null || true
wait "$CAMERA_PID" "$MANIP_PID" 2>/dev/null || true

exit $ORCH_EXIT
