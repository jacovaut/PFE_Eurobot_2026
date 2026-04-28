# Ensure venv exists and is activated
if [ ! -d venv ]; then
    echo "[INFO] Creating Python virtual environment..."
    python3 -m venv venv
    echo "[INFO] Installing dependencies (opencv-contrib-python, numpy, psutil)..."
    ./venv/bin/pip install --upgrade pip
    ./venv/bin/pip install opencv-contrib-python numpy psutil
    echo "[INFO] Virtual environment ready. Please re-run this script."
    exit 0
fi

# Activate venv
source venv/bin/activate

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

# Always reload v4l2loopback with exclusive_caps=0 for OpenCV compatibility
sudo modprobe -r v4l2loopback
sudo modprobe v4l2loopback video_nr=10 card_label=VirtualCam exclusive_caps=0


# Start the camera pipeline in the background (entire pipeline as a group)
( rpicam-vid -t 0 -n --codec mjpeg --width 1280 --height 720 --framerate 30 -o - | \
    ffmpeg -i - -f v4l2 -pix_fmt yuv420p /dev/video10 ) &
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


# Activate venv if needed
if [ -f venv/bin/activate ]; then
    source venv/bin/activate
fi

# Start block detection (main script)
python3 block_publisher.py
