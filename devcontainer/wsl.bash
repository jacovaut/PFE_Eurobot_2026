#!/bin/bash

# Variables
IMAGE_NAME="$2-img"
CONTAINER_NAME="$2-container"
USER="pfe"
WORKSPACE_HOST="$(pwd)/.."  # Local project dir
WORKSPACE_CONTAINER="/home/$USER/PFE_EUROBOT_2026"

docker container rm -f "$CONTAINER_NAME" || true

# Build Docker image
docker build -t "$IMAGE_NAME" -f "$1" .

# Step 2: Run the container with GPU and X11/WSLg support
echo "Running container..."
sudo docker run -it --rm\
    --gpus all \
    --net=host \
    --ipc=host \
    --pid=host \
    --device=/dev/dxg \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /mnt/wslg:/mnt/wslg \
    -v /usr/lib/wsl:/usr/lib/wsl \
    -v "$WORKSPACE_HOST":"$WORKSPACE_CONTAINER" \
    -e DISPLAY=$DISPLAY \
    -e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
    -e TERM=$TERM \
    --workdir "$WORKSPACE_CONTAINER" \
    --user "$USER" \
    --name "$CONTAINER_NAME" \
    "$IMAGE_NAME" \
    bash -c " \
        echo 'source /opt/ros/humble/setup.bash' >> /home/pfe/.bashrc && \
        echo 'source /home/pfe/PFE_EUROBOT_2026/ws/install/setup.bash' >> /home/pfe/.bashrc && \
        sudo apt update && \
        exec bash"
