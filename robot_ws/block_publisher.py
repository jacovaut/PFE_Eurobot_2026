#!/usr/bin/env python3
import cv2
import numpy as np
import socket
import json
import time

STREAM_HOST = "127.0.0.1"
STREAM_PORT = 8888

UDP_IP = "127.0.0.1"
UDP_PORT = 5005


# ==========================================================
# LOW-LATENCY MJPEG TCP CLIENT (NO BUFFER LAG)
# ==========================================================
class MjpegTcpClient:
    def __init__(self, host, port):
        print(f"[INFO] Connecting to MJPEG stream at {host}:{port}")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))
        self.sock.settimeout(1.0)
        self.buffer = b""

    def read(self):
        try:
            while True:
                chunk = self.sock.recv(65536)
                if not chunk:
                    return False, None

                self.buffer += chunk

                start = self.buffer.find(b"\xff\xd8")
                end = self.buffer.find(b"\xff\xd9")

                if start != -1 and end != -1 and end > start:
                    jpg = self.buffer[start:end + 2]

                    # 🔥 CRITICAL: drop old frames → no latency
                    self.buffer = self.buffer[end + 2:]

                    arr = np.frombuffer(jpg, dtype=np.uint8)
                    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

                    if frame is not None:
                        return True, frame

                # prevent memory buildup
                if len(self.buffer) > 2_000_000:
                    self.buffer = self.buffer[-500_000:]

        except socket.timeout:
            return False, None

    def release(self):
        self.sock.close()


# ==========================================================
# MAIN
# ==========================================================
def main():
    cap = MjpegTcpClient(STREAM_HOST, STREAM_PORT)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    try:
        parameters = cv2.aruco.DetectorParameters_create()
    except:
        parameters = cv2.aruco.DetectorParameters()

    print("[INFO] Block detection started")

    try:
        while True:
            ret, frame = cap.read()

            if not ret or frame is None:
                print("[WARN] No frame")
                continue

            # Save debug image
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

            # 🔥 FAST LOOP (no lag)
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("[INFO] Stopping...")

    finally:
        cap.release()
        sock.close()


if __name__ == "__main__":
    main()