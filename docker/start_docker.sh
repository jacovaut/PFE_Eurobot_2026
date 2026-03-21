#!/bin/bash
set -e

# Variables
IMAGE_NAME="ros-img"
CONTAINER_NAME="ros-container"
USER="piros"
WORKSPACE_HOST="$(pwd)/.."  # Local project dir
WORKSPACE_CONTAINER="/home/$USER/PFE_Eurobot_2026"

docker container rm -f "$CONTAINER_NAME" || true

# Build Docker image
docker build -t "$IMAGE_NAME" .

# Run container with X11 support
docker run -itd --rm \
  --privileged \
  --group-add dialout \
  --network host \
  --name "$CONTAINER_NAME" \
  -e ROS_DOMAIN_ID=0 \
  -e TERM=xterm-256color \
  -v /run/user/1000:/run/user/1000 \
  -v "$WORKSPACE_HOST":"$WORKSPACE_CONTAINER" \
  --workdir "$WORKSPACE_CONTAINER" \
  --user "$USER" \
  "$IMAGE_NAME" \
  bash -c "\
    echo 'export ROS_DOMAIN_ID=0' >> /home/$USER/.bashrc && \
    echo 'source /opt/ros/jazzy/setup.bash' >> /home/$USER/.bashrc && \
    echo 'source $WORKSPACE_CONTAINER/ws/install/setup.bash' >> /home/$USER/.bashrc && \
    sudo apt update && \
    exec bash
  "

docker ps