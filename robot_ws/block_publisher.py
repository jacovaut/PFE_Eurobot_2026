import sys
import os
import subprocess

venv_dir = os.path.join(os.path.dirname(__file__), 'venv')

# 1. Auto-create venv if missing
if not os.path.exists(venv_dir):
    print(f"[AUTO] Creating virtual environment in {venv_dir}...")
    subprocess.run([sys.executable, '-m', 'venv', venv_dir], check=True)
    print(f"[AUTO] Installing dependencies (opencv-contrib-python, numpy)...")
    subprocess.run([os.path.join(venv_dir, 'bin', 'pip'), 'install', 'opencv-contrib-python', 'numpy'], check=True)
    print(f"[AUTO] Please re-run: source venv/bin/activate && python block_publisher.py")
    sys.exit(0)

# 2. Auto-activate venv if not already active
if sys.prefix != os.path.abspath(venv_dir):
    activate_this = os.path.join(venv_dir, 'bin', 'activate_this.py')
    if os.path.exists(activate_this):
        exec(open(activate_this).read(), {'__file__': activate_this})
    else:
        # Fallback: print instructions
        print(f"[AUTO] Please run: source venv/bin/activate && python block_publisher.py")
        sys.exit(0)

# 3. Start rpicam-vid pipeline if not running
import psutil
pipeline_running = any('rpicam-vid' in p.name() or 'ffmpeg' in p.name() for p in psutil.process_iter())
if not pipeline_running:
    print('[AUTO] Starting rpicam-vid + ffmpeg pipeline...')
    subprocess.Popen(['sudo', 'modprobe', 'v4l2loopback', 'video_nr=10', 'card_label=VirtualCam', 'exclusive_caps=1'])
    cmd = (
        'rpicam-vid -t 0 -n --codec mjpeg --width 1280 --height 720 --framerate 30 -o - | '
        'ffmpeg -i - -f v4l2 -pix_fmt yuv420p /dev/video10'
    )
    subprocess.Popen(cmd, shell=True)
    import time; time.sleep(2)  # Give pipeline time to start

# Utility: start rpicam-vid pipeline if requested (must be before cv2 import)
if '--start-pipeline' in sys.argv:
    subprocess.run(['sudo', 'modprobe', 'v4l2loopback', 'video_nr=10', 'card_label=VirtualCam', 'exclusive_caps=1'])
    cmd = (
        'rpicam-vid -t 0 -n --codec mjpeg --width 1280 --height 720 --framerate 30 -o - | '
        'ffmpeg -i - -f v4l2 -pix_fmt yuv420p /dev/video10'
    )
    print('[INFO] Starting rpicam-vid + ffmpeg pipeline...')
    print('[INFO] Press Ctrl+C to stop.')
    subprocess.run(cmd, shell=True)
    sys.exit(0)

import cv2
import numpy as np
import socket
import json
import time
import os
import sys
import subprocess

# UDP setup
UDP_IP = "127.0.0.1"  # Use "127.0.0.1" for host-networked Docker, or container IP if bridged
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ArUco setup
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()

cap = cv2.VideoCapture('/dev/video10', cv2.CAP_V4L2)  # Use device path and force V4L2 backend
# Debug: Print first few frames' shape
for i in range(5):
    ret, frame = cap.read()
    print(f"[DEBUG] Frame {i}: ret={ret}, shape={None if frame is None else frame.shape}")

while True:
    ret, frame = cap.read()
    if not ret:
        print("No frame from camera")
        time.sleep(0.1)
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    blocks = []
    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            c = corners[i][0]
            cx = float(np.mean(c[:, 0]))
            cy = float(np.mean(c[:, 1]))
            blocks.append({'id': int(marker_id), 'cx': cx, 'cy': cy})
    print("Blocks:", blocks)  # Debug print

    msg = json.dumps(blocks).encode()
    sock.sendto(msg, (UDP_IP, UDP_PORT))
    time.sleep(0.1)  # 10Hz, adjust as needed

cap.release()