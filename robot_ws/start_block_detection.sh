#!/bin/bash
set -e

# --- Minimal, robust camera pipeline and block detection launcher ---

# Cleanup function to kill background processes
cleanup() {
    echo "[INFO] Cleaning up camera pipeline..."
    pkill -9 -f rpicam-vid || true
    pkill -9 -f ffmpeg || true
    pkill -9 -f libcamera || true
    if [ -n "$PIPELINE_PID" ]; then
        kill $PIPELINE_PID 2>/dev/null || true
    fi
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

echo "[INFO] Killing old camera pipeline processes..."
pkill -9 -f rpicam-vid || true
pkill -9 -f ffmpeg || true
pkill -9 -f libcamera || true
sleep 1

echo "[INFO] Reloading v4l2loopback..."
sudo modprobe -r v4l2loopback || true
sudo modprobe v4l2loopback video_nr=10 card_label=VirtualCam exclusive_caps=1

# Start the camera pipeline in the background
(
  rpicam-vid -t 0 -n --codec mjpeg --width 1280 --height 720 --framerate 30 -o - | \
  ffmpeg -loglevel error -i - -f v4l2 -pix_fmt yuv422 /dev/video10
) &
PIPELINE_PID=$!

echo "[INFO] Waiting for /dev/video10 to be ready..."
for i in {1..10}; do
    if [ -e /dev/video10 ]; then
        break
    fi
    sleep 1
done
sleep 2

echo "[INFO] Starting block detection..."
python3 block_publisher.py

