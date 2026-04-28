
import cv2
import numpy as np
import socket
import json
import time



# Always use device index 10 with V4L2 backend for v4l2loopback
print('[DEBUG] Opening OpenCV camera at index 10 (V4L2 backend)')
cap = cv2.VideoCapture(10, cv2.CAP_V4L2)
for i in range(3):
    ret, frame = cap.read()
    print(f"[DEBUG] Index 10: ret={ret}, shape={None if frame is None else frame.shape}")
if not ret or frame is None:
    print('[ERROR] Could not open camera at index 10.')
    exit(1)

# UDP setup
UDP_IP = "127.0.0.1"  # Use "127.0.0.1" for host-networked Docker, or container IP if bridged
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ArUco setup
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()



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