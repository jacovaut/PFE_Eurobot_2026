#!/usr/bin/env python3
import cv2
import numpy as np
import socket
import json
import time
import sys

STREAM_URL = "tcp://127.0.0.1:8888"
UDP_IP = "127.0.0.1"
UDP_PORT = 5005


def open_camera():
    print(f"[INFO] Opening TCP stream at {STREAM_URL}")
    cap = cv2.VideoCapture(STREAM_URL)

    for i in range(10):
        ret, frame = cap.read()
        print(f"[DEBUG] Camera test {i}: ret={ret}, shape={None if frame is None else frame.shape}")
        if ret and frame is not None:
            return cap
        time.sleep(0.2)

    print(f"[ERROR] Could not open TCP camera stream at {STREAM_URL}")
    sys.exit(1)


def main():
    cap = open_camera()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    try:
        parameters = cv2.aruco.DetectorParameters_create()
    except AttributeError:
        parameters = cv2.aruco.DetectorParameters()

    try:
        while True:
            ret, frame = cap.read()

            if not ret or frame is None:
                print("[WARN] No frame from TCP stream")
                time.sleep(0.1)
                continue

            cv2.imwrite("/tmp/arducam_raw.jpg", frame)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray,
                dictionary,
                parameters=parameters
            )

            blocks = []

            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    c = corners[i][0]
                    cx = float(np.mean(c[:, 0]))
                    cy = float(np.mean(c[:, 1]))

                    blocks.append({
                        "id": int(marker_id),
                        "cx": cx,
                        "cy": cy
                    })

            print("[INFO] Blocks:", blocks)

            msg = json.dumps(blocks).encode()
            sock.sendto(msg, (UDP_IP, UDP_PORT))

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("[INFO] Interrupted, exiting...")

    finally:
        cap.release()
        sock.close()


if __name__ == "__main__":
    main()