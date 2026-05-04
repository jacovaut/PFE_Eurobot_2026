#!/bin/bash
set -e

cleanup() {
    echo "[INFO] Cleaning up..."
    pkill -f "rpicam-vid" || true
    pkill -f "mjpeg_broadcast.py" || true
    pkill -f "block_publisher.py" || true
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

echo "[INFO] Killing old camera streams..."
pkill -f "rpicam-vid" || true
pkill -f "block_publisher.py" || true
pkill -f "mjpeg_broadcast.py" || true
# force-free ports in case of lingering sockets
fuser -k 8887/tcp 8888/tcp 8080/tcp 2>/dev/null || true
sleep 1

echo "[INFO] Starting LOW-LATENCY MJPEG TCP stream (internal port 8887)..."

rpicam-vid -t 0 -n \
  --listen \
  --codec mjpeg \
  --width 2328 \
  --height 1748 \
  --framerate 30 \
  --flush \
  -o tcp://0.0.0.0:8888 &

CAMERA_PID=$!

sleep 1

echo "[INFO] Starting MJPEG broadcast proxy (8887 -> raw:8888, HTTP:8080)..."
# python3 "$(dirname "$0")/mjpeg_broadcast.py"
python3 "$(dirname "$0")/block_publisher.py"

kill $CAMERA_PID 2>/dev/null || true