#!/bin/bash
set -e

ROS_SETUP="/opt/ros/jazzy/setup.bash"
MICROROS_WS_SETUP="/home/com2001/PFE_Eurobot_2026/ws/install/local_setup.bash"
AGENT_PORT="${1:-8895}"

if [ ! -f "$ROS_SETUP" ]; then
    echo "[ERROR] ROS setup file not found: $ROS_SETUP" >&2
    exit 1
fi

if [ ! -f "$MICROROS_WS_SETUP" ]; then
    echo "[ERROR] micro-ROS workspace setup file not found: $MICROROS_WS_SETUP" >&2
    echo "[INFO] Build the micro-ROS workspace first." >&2
    exit 1
fi

source "$ROS_SETUP"
source "$MICROROS_WS_SETUP"

echo "[INFO] Starting micro-ROS agent on UDP port $AGENT_PORT"
exec ros2 run micro_ros_agent micro_ros_agent udp4 -p "$AGENT_PORT" -m dds -v6
