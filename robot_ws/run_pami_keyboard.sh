#!/bin/bash
set -e

ROS_SETUP="/opt/ros/jazzy/setup.bash"
ROBOT_WS_SETUP="$(dirname "$0")/install/local_setup.bash"

if [ ! -f "$ROS_SETUP" ]; then
    echo "[ERROR] ROS setup file not found: $ROS_SETUP" >&2
    exit 1
fi

if [ ! -f "$ROBOT_WS_SETUP" ]; then
    echo "[ERROR] robot_ws setup file not found: $ROBOT_WS_SETUP" >&2
    echo "[INFO] Build robot_ws first with: colcon build --symlink-install" >&2
    exit 1
fi

source "$ROS_SETUP"
source "$ROBOT_WS_SETUP"

exec ros2 run manip_action_node pami_keyboard "$@"
