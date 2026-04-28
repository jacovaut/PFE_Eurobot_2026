#!/usr/bin/env python3
import cv2
import numpy as np
import socket
import json
import time

# UDP setup
UDP_IP = "127.0.0.1"  # Use "127.0.0.1" for host-networked Docker, or container IP if bridged
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ArUco setup
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()
cap = cv2.VideoCapture(0)  # Use your camera device

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

    msg = json.dumps(blocks).encode()
    sock.sendto(msg, (UDP_IP, UDP_PORT))
    time.sleep(0.1)  # 10Hz, adjust as needed

cap.release()