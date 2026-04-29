import cv2
import os

print("[INFO] Scanning /dev/video* devices with OpenCV...")

for idx in range(0, 20):
    dev_path = f"/dev/video{idx}"
    if not os.path.exists(dev_path):
        continue
    print(f"[TEST] Trying index {idx} ({dev_path})...")
    cap = cv2.VideoCapture(idx)
    if not cap.isOpened():
        print(f"[FAIL] Could not open index {idx} ({dev_path})")
        continue
    ret, frame = cap.read()
    if ret and frame is not None:
        print(f"[OK] Index {idx} ({dev_path}): Frame shape: {frame.shape}")
    else:
        print(f"[FAIL] Index {idx} ({dev_path}): Opened but could not read frame")
    cap.release()

print("[INFO] Scan complete.")
