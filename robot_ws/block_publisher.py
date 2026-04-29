

# Minimal, robust block detection publisher for Raspberry Pi camera (OpenCV + UDP)
import cv2
import numpy as np
import socket
import json
import time
import sys

CAMERA_DEVICE = '/dev/video10'  # v4l2loopback device
UDP_IP = "127.0.0.1"
UDP_PORT = 5005


def open_camera(device):
    print(f"[INFO] Opening camera at {device} (V4L2 backend)")
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    for i in range(3):
        ret, frame = cap.read()
        print(f"[DEBUG] Camera open test {i}: ret={ret}, shape={None if frame is None else frame.shape}")
    if not ret or frame is None:
        print(f"[ERROR] Could not open camera at {device}.")
        sys.exit(1)
    return cap

def main():
    cap = open_camera(CAMERA_DEVICE)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                print("[WARN] No frame from camera")
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
            print("[INFO] Blocks:", blocks)

            msg = json.dumps(blocks).encode()
            sock.sendto(msg, (UDP_IP, UDP_PORT))
            time.sleep(0.1)  # 10Hz
    except KeyboardInterrupt:
        print("[INFO] Interrupted, exiting...")
    finally:
        cap.release()

if __name__ == "__main__":
    main()