import sys
import os
import subprocess

# Utility: create venv if requested (must be before cv2 import)
if '--make-venv' in sys.argv:
    venv_dir = 'venv'
    if not os.path.exists(venv_dir):
        print(f"[INFO] Creating virtual environment in {venv_dir}...")
        subprocess.run([sys.executable, '-m', 'venv', venv_dir], check=True)
        print(f"[INFO] Run: source {venv_dir}/bin/activate && pip install opencv-contrib-python numpy")
    else:
        print(f"[INFO] Virtual environment already exists: {venv_dir}")
    sys.exit(0)

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
cap = cv2.VideoCapture(10)  # Use your camera device

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