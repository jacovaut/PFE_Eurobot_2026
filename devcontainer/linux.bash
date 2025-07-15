#!/bin/bash
set -e

# Variables
IMAGE_NAME="$2-img"
CONTAINER_NAME="$2-container"
USER="pfe"
WORKSPACE_HOST="$(pwd)/.."  # Local project dir
WORKSPACE_CONTAINER="/home/$USER/PFE_EUROBOT_2026"

# For X11 GUI forwarding
XSOCK="/tmp/.X11-unix"
XAUTH="/tmp/.docker.xauth"

# Create Xauthority file if needed
if [ ! -f "$XAUTH" ]; then
    touch "$XAUTH"
    xauth_list=$(xauth nlist "$DISPLAY")
    if [ -n "$xauth_list" ]; then
        echo "$xauth_list" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge -
    fi
fi

# Allow local connections (may be unsafe on shared systems)
xhost +local:docker

docker container rm -f "$CONTAINER_NAME" || true

# Build Docker image
docker build -t "$IMAGE_NAME" -f "$1" .

# Run container with X11 support
docker run -itd --rm \
  --name "$CONTAINER_NAME" \
  --net=host \
  --ipc=host \
  --pid=host \
  -e DISPLAY="$DISPLAY" \
  -e XDG_RUNTIME_DIR=/run/user/1000 \
  -e XAUTHORITY="$XAUTH" \
  -e ROS_DOMAIN_ID=42 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e TERM=xterm-256color \
  -v /run/user/1000:/run/user/1000 \
  -v "$WORKSPACE_HOST":"$WORKSPACE_CONTAINER" \
  -v "$XSOCK":"$XSOCK" \
  -v "$XAUTH":"$XAUTH" \
  --workdir "$WORKSPACE_CONTAINER" \
  --user "$USER" \
  "$IMAGE_NAME" \
  bash -c "\
    echo 'source /opt/ros/jazzy/setup.bash' >> /home/$USER/.bashrc && \
    echo 'source $WORKSPACE_CONTAINER/ws/install/setup.bash' >> /home/$USER/.bashrc && \
    sudo apt update && \
    exec bash
  "

docker ps

# Revoke X11 permissions
xhost -local:docker