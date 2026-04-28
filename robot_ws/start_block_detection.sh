#!/bin/bash
# Start the camera pipeline in the background
python3 block_publisher.py --start-pipeline &
PIPELINE_PID=$!

# Wait for /dev/video10 to be ready (max 10 seconds)
for i in {1..10}; do
    if [ -e /dev/video10 ]; then
        break
    fi
    sleep 1
done

# Activate venv if needed
if [ -f venv/bin/activate ]; then
    source venv/bin/activate
fi

# Start block detection (main script)
python3 block_publisher.py

# Optional: Clean up pipeline on exit
kill $PIPELINE_PID
