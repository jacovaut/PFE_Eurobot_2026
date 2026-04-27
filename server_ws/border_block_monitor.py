#!/usr/bin/env python3
"""
Monitor detection rate of border block ArUco markers (IDs 36 and 47).
Captures frames continuously and shows per-ID detection statistics.
Press 'q' to quit, 'r' to reset counters.
"""
import cv2
import numpy as np
import time
import argparse
from collections import deque

TARGET_IDS = [36, 47]
WINDOW_SIZE = 30  # rolling window in frames

def make_params(pad_rescue=False):
    p = cv2.aruco.DetectorParameters_create()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    if pad_rescue:
        p.minDistanceToBorder = 0
        p.adaptiveThreshWinSizeMin = 3
        p.adaptiveThreshWinSizeMax = 181
        p.adaptiveThreshWinSizeStep = 30
        p.minMarkerPerimeterRate = 0.003
        p.errorCorrectionRate = 0.6
    return p

def detect(gray, dictionary, params):
    _, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    if ids is None:
        return set()
    return set(int(i) for i in ids.flatten())

def apply_clahe(gray, clip_limit=3.0):
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(8, 8))
    return clahe.apply(gray)

def pad_frame(gray, pad_ratio=0.06):
    h, w = gray.shape
    pad = int(min(h, w) * pad_ratio)
    return cv2.copyMakeBorder(gray, pad, pad, pad, pad, cv2.BORDER_REPLICATE), pad

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', default='/dev/video0')
    parser.add_argument('--ids', default='36,47', help='Comma-separated target IDs')
    parser.add_argument('--window', type=int, default=WINDOW_SIZE, help='Rolling window size (frames)')
    parser.add_argument('--show-preview', action='store_true', help='Show scaled camera preview')
    args = parser.parse_args()

    target_ids = [int(x) for x in args.ids.split(',')]
    window_size = args.window

    cap = cv2.VideoCapture(args.device)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
    cap.set(cv2.CAP_PROP_FPS, 15)

    if not cap.isOpened():
        print(f"Cannot open {args.device}")
        return

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    normal_params = make_params(pad_rescue=False)
    rescue_params = make_params(pad_rescue=True)

    # Per-ID rolling history: True = detected in that frame, False = not
    history = {tid: deque(maxlen=window_size) for tid in target_ids}
    frame_count = 0
    start_time = time.time()

    print(f"\nMonitoring IDs: {target_ids}  |  Rolling window: {window_size} frames")
    print(f"Device: {args.device}  |  Press Ctrl+C to quit\n")
    print(f"{'Frame':>7}  {'FPS':>5}  " +
          "  ".join(f"ID {tid:>2} normal  rescue  combined  rate/{window_size}" for tid in target_ids))
    print("-" * (20 + 42 * len(target_ids)))

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Normal pass
            found_normal = detect(gray, dictionary, normal_params)

            # Rescue pass: CLAHE + padded frame
            gray_clahe = apply_clahe(gray, clip_limit=3.0)
            gray_padded, pad = pad_frame(gray_clahe, pad_ratio=0.06)
            found_rescue = detect(gray_padded, dictionary, rescue_params)

            found_combined = found_normal | found_rescue

            frame_count += 1
            elapsed = time.time() - start_time
            fps = frame_count / elapsed if elapsed > 0 else 0

            # Update history
            for tid in target_ids:
                history[tid].append(tid in found_combined)

            # Print stats
            stats = []
            for tid in target_ids:
                n = "✓" if tid in found_normal else "✗"
                r = "✓" if tid in found_rescue else "✗"
                c = "✓" if tid in found_combined else "✗"
                wins = sum(history[tid])
                total = len(history[tid])
                rate = f"{wins}/{total}"
                stats.append(f"  ID {tid:>2}:  {n}       {r}      {c}       {rate:>6}")

            print(f"{frame_count:>7}  {fps:>5.1f}" + "".join(stats))

            if args.show_preview:
                preview = cv2.resize(frame, (1280, 720))
                cv2.aruco.drawDetectedMarkers(preview,
                    [c for i, c in zip([], [])], np.array([]))  # placeholder
                cv2.imshow("Border Block Monitor", preview)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('r'):
                    for tid in target_ids:
                        history[tid].clear()
                    frame_count = 0
                    start_time = time.time()
                    print("\n--- Counters reset ---\n")

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if args.show_preview:
            cv2.destroyAllWindows()

        # Final summary
        print(f"\n{'='*60}")
        print("FINAL SUMMARY")
        print(f"Total frames captured: {frame_count}")
        for tid in target_ids:
            if history[tid]:
                wins = sum(history[tid])
                total = len(history[tid])
                pct = 100.0 * wins / total
                print(f"  ID {tid}: {wins}/{total} frames detected ({pct:.1f}%) [last {window_size}]")
        print(f"{'='*60}")

if __name__ == '__main__':
    main()
