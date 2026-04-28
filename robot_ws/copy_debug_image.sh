#!/bin/bash
# Continuously copy the debug image from the container to the host every 2 seconds

CONTAINER=ros-container
SRC_PATH=/tmp/merged_node_debug.jpg
DEST_PATH=/home/pi/merged_node_debug.jpg

while true; do
    docker cp $CONTAINER:$SRC_PATH $DEST_PATH 2>/dev/null
    sleep 2
done