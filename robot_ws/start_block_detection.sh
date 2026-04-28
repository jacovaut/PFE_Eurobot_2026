
# --- SYSTEM PYTHON/OPENCV: No venv required ---
echo "[INFO] Using system Python: $(which python3)"
echo "[INFO] Python version: $(python3 --version)"
# --- END SYSTEM PYTHON/OPENCV ---

#!/bin/bash

# Trap SIGINT/SIGTERM and clean up pipeline
cleanup() {
    echo "[INFO] Caught exit signal, killing pipeline..."
    pkill -9 -f rpicam-vid
    pkill -9 -f ffmpeg
    pkill -9 -f libcamera
    if [ -n "$PIPELINE_PID" ]; then
        kill $PIPELINE_PID 2>/dev/null
    fi
    exit 0
}
trap cleanup SIGINT SIGTERM

# Kill any old camera pipeline processes
echo "[INFO] Killing old rpicam-vid, ffmpeg, and libcamera processes..."
pkill -9 -f rpicam-vid
pkill -9 -f ffmpeg
pkill -9 -f libcamera
sleep 1

# Always reload v4l2loopback with exclusive_caps=1 (previous working state)
sudo modprobe -r v4l2loopback
sudo modprobe v4l2loopback video_nr=10 card_label=VirtualCam exclusive_caps=1


# Start the camera pipeline in the background (entire pipeline as a group)
( rpicam-vid -t 0 -n --codec mjpeg --width 1280 --height 720 --framerate 30 -o - | \
    ffmpeg -i - -f v4l2 -pix_fmt yuv420p /dev/video10
# Debug: Print first few frames' shape in block_publisher.py
PIPELINE_PID=$!



# Wait for /dev/video10 to be ready (max 10 seconds)
for i in {1..10}; do
    if [ -e /dev/video10 ]; then
        break
    fi
    sleep 1
done

# Extra delay to ensure device is ready
sleep 2



# No venv activation needed; using system Python

# Start block detection (main script)
python3 block_publisher.py
