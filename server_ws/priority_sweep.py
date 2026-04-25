#!/usr/bin/env python3
"""
Priority-weighted camera parameter grid search.

Scoring (per trial):
  score = (robot_rate * 10_000) + (table_rate * 1_000) + (block_rate * 100)

  where each *_rate = fraction of frames that group's marker(s) are detected.

  Robot IDs  : 1, 6
  Table IDs  : 20, 21, 22, 23
  Block IDs  : 36, 47

Usage:
    python3 priority_sweep.py
"""

import subprocess
import sys
import time

import cv2
import numpy as np

# ── Config ───────────────────────────────────────────────────────────────────
CAMERA_DEVICE = "/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera_01.00.00-video-index0"
WIDTH, HEIGHT, FPS = 3840, 2160, 15
FOURCC = "MJPG"

ARUCO_DICT = cv2.aruco.DICT_4X4_50

ROBOT_IDS  = {1, 6}
TABLE_IDS  = {20, 21, 22, 23}
BLOCK_IDS  = {36, 47}

ROBOT_W  = 10_000
TABLE_W  =  1_000
BLOCK_W  =    100

WARMUP_FRAMES   = 6
MEASURE_FRAMES  = 20   # per trial (more = more reliable)
RESCUE_CLAHE    = True  # add CLAHE pass when marker missed

# Grid values to try
GAIN_VALUES  = [0, 30, 60, 90, 110, 140, 170, 210, 255]
GAMMA_VALUES = [64, 90, 110, 130, 150, 175, 200, 220]
BC_VALUES    = [0, 10, 20, 27, 36, 50, 70, 80]

# ── Helpers ──────────────────────────────────────────────────────────────────

def v4l2_set(ctrl, value):
    subprocess.run(
        ["v4l2-ctl", "-d", CAMERA_DEVICE, "--set-ctrl", f"{ctrl}={value}"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )


def make_detector():
    params = cv2.aruco.DetectorParameters_create()
    params.perspectiveRemovePixelPerCell = 10
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 101
    params.adaptiveThreshWinSizeStep = 10
    params.adaptiveThreshConstant = 7.0
    params.minMarkerPerimeterRate = 0.005
    params.maxMarkerPerimeterRate = 4.0
    params.errorCorrectionRate = 0.45
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    return dictionary, params


def detect_ids(gray, dictionary, params):
    _, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    if ids is None:
        return set()
    return set(ids.flatten().tolist())


def detect_with_rescue(frame, dictionary, params):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found = detect_ids(gray, dictionary, params)
    if RESCUE_CLAHE:
        missing = (ROBOT_IDS | TABLE_IDS | BLOCK_IDS) - found
        if missing:
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(16, 16))
            eq = clahe.apply(gray)
            found |= detect_ids(eq, dictionary, params)
    return found


def measure_trial(cap, dictionary, params):
    for _ in range(WARMUP_FRAMES):
        cap.read()

    robot_hits  = {i: 0 for i in ROBOT_IDS}
    table_hits  = {i: 0 for i in TABLE_IDS}
    block_hits  = {i: 0 for i in BLOCK_IDS}
    good_frames = 0

    for _ in range(MEASURE_FRAMES):
        ok, frame = cap.read()
        if not ok or frame is None:
            continue
        good_frames += 1
        found = detect_with_rescue(frame, dictionary, params)
        for i in ROBOT_IDS:
            if i in found:
                robot_hits[i] += 1
        for i in TABLE_IDS:
            if i in found:
                table_hits[i] += 1
        for i in BLOCK_IDS:
            if i in found:
                block_hits[i] += 1

    if good_frames == 0:
        return 0.0, {}, {}, {}

    robot_rates = {i: robot_hits[i] / good_frames for i in ROBOT_IDS}
    table_rates = {i: table_hits[i] / good_frames for i in TABLE_IDS}
    block_rates = {i: block_hits[i] / good_frames for i in BLOCK_IDS}

    avg_robot = sum(robot_rates.values()) / len(robot_rates)
    avg_table = sum(table_rates.values()) / len(table_rates)
    avg_block = sum(block_rates.values()) / len(block_rates)

    score = avg_robot * ROBOT_W + avg_table * TABLE_W + avg_block * BLOCK_W
    return score, robot_rates, table_rates, block_rates


def open_camera():
    cap = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"ERROR: Cannot open {CAMERA_DEVICE}")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*FOURCC))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    for _ in range(5):
        cap.read()
    return cap


def fmt_rates(d):
    return " ".join(f"{k}:{v*100:.0f}%" for k, v in sorted(d.items()))


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    print("╔══════════════════════════════════════════════════════╗")
    print("║  Priority-Weighted Camera Optimisation Sweep         ║")
    print("║  Robots (1,6) > Table (20-23) > Blocks (36,47)      ║")
    print("╚══════════════════════════════════════════════════════╝")
    print(f"Camera: {CAMERA_DEVICE}")

    # Ensure manual exposure, autofocus off
    v4l2_set("auto_exposure", 1)
    v4l2_set("focus_automatic_continuous", 0)
    time.sleep(0.2)

    cap = open_camera()
    dictionary, params = make_detector()

    # Baseline
    print("\n--- Baseline (current settings) ---")
    base_score, r_rates, t_rates, b_rates = measure_trial(cap, dictionary, params)
    print(f"  Score={base_score:.0f}")
    print(f"  Robots : {fmt_rates(r_rates)}")
    print(f"  Table  : {fmt_rates(t_rates)}")
    print(f"  Blocks : {fmt_rates(b_rates)}")

    total = len(GAIN_VALUES) * len(GAMMA_VALUES) * len(BC_VALUES)
    print(f"\nGrid search: {len(GAIN_VALUES)} gain × {len(GAMMA_VALUES)} gamma × {len(BC_VALUES)} bc = {total} trials")
    print(f"Estimated time: ~{total * (MEASURE_FRAMES + WARMUP_FRAMES) / 15 / 60:.1f} min\n")

    best_score = -1.0
    best_params = None
    best_rates  = None
    results = []

    trial = 0
    for gain in GAIN_VALUES:
        for gamma in GAMMA_VALUES:
            for bc in BC_VALUES:
                trial += 1
                v4l2_set("gain", gain)
                v4l2_set("gamma", gamma)
                v4l2_set("backlight_compensation", bc)
                time.sleep(0.12)

                score, r, t, b = measure_trial(cap, dictionary, params)
                results.append((score, gain, gamma, bc, r, t, b))

                flag = ""
                if score > best_score:
                    best_score = score
                    best_params = (gain, gamma, bc)
                    best_rates = (r, t, b)
                    flag = " ← BEST"

                robot_pct  = sum(r.values()) / len(r) * 100
                table_pct  = sum(t.values()) / len(t) * 100
                block_pct  = sum(b.values()) / len(b) * 100
                print(f"[{trial:3d}/{total}] gain={gain:3d} gamma={gamma:3d} bc={bc:3d} "
                      f"| score={score:8.0f} "
                      f"| R={robot_pct:5.1f}% T={table_pct:5.1f}% B={block_pct:5.1f}%{flag}")

    cap.release()

    # ── Results ──────────────────────────────────────────────────────────────
    print("\n╔══════════════════════════════════════════════════════╗")
    print("║                    RESULTS                           ║")
    print("╚══════════════════════════════════════════════════════╝")

    results.sort(reverse=True, key=lambda x: x[0])

    print("\nTop 15 configurations:")
    print(f"  {'Rank':>4}  {'Score':>8}  {'gain':>5}  {'gamma':>6}  {'bc':>4}  {'Robots':>14}  {'Table':>22}  {'Blocks':>12}")
    for i, (score, gain, gamma, bc, r, t, b) in enumerate(results[:15], 1):
        print(f"  {i:4d}  {score:8.0f}  {gain:5d}  {gamma:6d}  {bc:4d}  "
              f"{fmt_rates(r):>14}  {fmt_rates(t):>22}  {fmt_rates(b):>12}")

    g, gm, bc = best_params
    r, t, b = best_rates
    print(f"\n★  Best: gain={g}  gamma={gm}  backlight_compensation={bc}  score={best_score:.0f}")
    print(f"   Robots : {fmt_rates(r)}")
    print(f"   Table  : {fmt_rates(t)}")
    print(f"   Blocks : {fmt_rates(b)}")

    print(f"\nApply best settings:")
    print(f"  v4l2-ctl -d {CAMERA_DEVICE} --set-ctrl=gain={g},gamma={gm},backlight_compensation={bc}")
    print(f"\n  Or update camera_global_map.yaml:")
    print(f"    camera_gain: {g}")
    print(f"    camera_gamma: {gm}")
    print(f"    camera_backlight_compensation: {bc}")

    # Apply best
    v4l2_set("gain", g)
    v4l2_set("gamma", gm)
    v4l2_set("backlight_compensation", bc)
    print("\nBest settings have been applied to the camera.")


if __name__ == "__main__":
    main()
