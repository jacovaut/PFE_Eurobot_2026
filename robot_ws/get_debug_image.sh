#!/bin/bash
# Usage: ./get_debug_image.sh

PI_USER=pi
PI_HOST=192.168.1.121
PI_PATH=/home/pi/merged_node_debug.jpg
LOCAL_PATH=./merged_node_debug.jpg

echo "Image copied to ${LOCAL_PATH}"
while true; do
	scp -q ${PI_USER}@${PI_HOST}:${PI_PATH} ${LOCAL_PATH}
	echo "Image copied to ${LOCAL_PATH} at $(date)"
	sleep 2
done