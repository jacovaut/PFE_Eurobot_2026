#!/bin/bash
set -e

cleanup() {
    echo "[INFO] Cleaning up..."
    pkill -f "rpicam-vid.*8888" || true
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

echo "[INFO] Killing old camera streams..."
pkill -f "rpicam-vid.*8888" || true
sleep 1

echo "[INFO] Starting LOW-LATENCY MJPEG TCP stream..."

rpicam-vid -t 0 -n \
  --listen \
  --codec mjpeg \
  --width 4656 \
  --height 3496 \
  --framerate 30 \
  --flush \
  -o tcp://0.0.0.0:8888 &

CAMERA_PID=$!

sleep 2

echo "[INFO] Starting block detection..."
python3 block_publisher.py

kill $CAMERA_PID 2>/dev/null || true