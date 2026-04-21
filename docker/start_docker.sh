#!/bin/bash

timeout=30
elapsed=0

until docker info >/dev/null 2>&1; do
  sleep 1
  elapsed=$((elapsed+1))
  if [ $elapsed -ge $timeout ]; then
    echo "Docker not ready after $timeout seconds"
    exit 1
  fi
done

set -e

IMAGE_NAME="ros-img"
CONTAINER_NAME="ros-container"
USER="piros"
WORKSPACE_HOST="$(pwd)/.."
WORKSPACE_CONTAINER="/home/$USER/PFE_Eurobot_2026"

# Remove old container if exists
docker rm -f "$CONTAINER_NAME" || true

# (Optional) build only if needed
# docker build -t "$IMAGE_NAME" .

# Run container
docker run -d \
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
  tail -f /dev/null
