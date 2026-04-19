#!/usr/bin/env python3
"""
Test ArUco detection resilience to lighting changes.
Simulates lighting variation by sweeping exposure_time_absolute,
and compares different gain/gamma/preprocessing strategies.
"""

import cv2
import numpy as np
import subprocess
import sys
import time

CAMERA = "/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera_01.00.00-video-index0"
WIDTH, HEIGHT, FPS = 3840, 2160, 15
TABLE_IDS = {20, 21, 22, 23}
ROBOT_ID = 1
ALL_IDS = TABLE_IDS | {ROBOT_ID}
N_FRAMES = 10
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# Exposure values to test (simulates dim → bright lighting)
# Current=200, lower=darker, higher=brighter
EXPOSURES = [50, 80, 100, 130, 156, 200, 280, 400, 500]


def v4l2_set(controls: dict):
    ctrl_str = ",".join(f"{k}={v}" for k, v in controls.items())
    subprocess.run(["v4l2-ctl", "-d", CAMERA, "--set-ctrl", ctrl_str],
                   capture_output=True, check=True)


def v4l2_get(control: str) -> str:
    r = subprocess.run(["v4l2-ctl", "-d", CAMERA, "--get-ctrl", control],
                       capture_output=True, text=True)
    return r.stdout.strip()


def base_params():
    p = cv2.aruco.DetectorParameters_create()
    p.perspectiveRemovePixelPerCell = 10
    p.adaptiveThreshWinSizeMin = 3
    p.adaptiveThreshWinSizeMax = 101
    p.adaptiveThreshWinSizeStep = 10
    p.adaptiveThreshConstant = 7.0
    p.minMarkerPerimeterRate = 0.005
    p.maxMarkerPerimeterRate = 4.0
    p.errorCorrectionRate = 0.45
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return p


def rescue_params():
    p = base_params()
    p.adaptiveThreshWinSizeMax = 181
    p.adaptiveThreshWinSizeStep = 30
    p.minMarkerPerimeterRate = 0.015
    p.errorCorrectionRate = 0.6
    return p


def detect(gray, params=None):
    if params is None:
        params = base_params()
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, DICT, parameters=params)
    return set(ids.flatten().tolist()) if ids is not None else set()


def detect_with_rescue(gray):
    """Mimics the C++ node: base pass + CLAHE rescue for missing table markers."""
    bp = base_params()
    found = detect(gray, bp)
    missing = TABLE_IDS - found
    if missing:
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(16, 16))
        enhanced = clahe.apply(gray)
        rp = rescue_params()
        rescue_found = detect(enhanced, rp)
        found |= (rescue_found & TABLE_IDS)
    return found


def clahe_preprocess(gray, clip=3.0, tile=16):
    c = cv2.createCLAHE(clipLimit=clip, tileGridSize=(tile, tile))
    return c.apply(gray)


class Strategy:
    def __init__(self, name, hw_controls, preprocess_fn, detect_fn):
        self.name = name
        self.hw_controls = hw_controls  # V4L2 controls (excluding exposure)
        self.preprocess_fn = preprocess_fn  # gray -> gray
        self.detect_fn = detect_fn  # gray -> set of IDs


def make_strategies():
    """Define candidate strategies to compare."""
    strategies = []

    # 1. Current: high gain+gamma, raw detection + rescue
    strategies.append(Strategy(
        "gain=227 gamma=300 (current, raw+rescue)",
        {"gain": 227, "gamma": 300, "brightness": 0, "contrast": 0,
         "saturation": 70, "sharpness": 1},
        lambda g: g,
        detect_with_rescue
    ))

    # 2. Moderate gain, high gamma, raw + rescue
    strategies.append(Strategy(
        "gain=170 gamma=300 (raw+rescue)",
        {"gain": 170, "gamma": 300, "brightness": 0, "contrast": 0,
         "saturation": 70, "sharpness": 1},
        lambda g: g,
        detect_with_rescue
    ))

    # 3. Default gain+gamma, CLAHE preprocessing on ALL detection
    strategies.append(Strategy(
        "gain=110 gamma=110 + CLAHE(3,16) all frames",
        {"gain": 110, "gamma": 110, "brightness": 0, "contrast": 0,
         "saturation": 70, "sharpness": 1},
        lambda g: clahe_preprocess(g, 3.0, 16),
        detect_with_rescue
    ))

    # 4. Default gain+gamma, CLAHE(4,8) preprocessing
    strategies.append(Strategy(
        "gain=110 gamma=110 + CLAHE(4,8) all frames",
        {"gain": 110, "gamma": 110, "brightness": 0, "contrast": 0,
         "saturation": 70, "sharpness": 1},
        lambda g: clahe_preprocess(g, 4.0, 8),
        detect_with_rescue
    ))

    # 5. Moderate gain, default gamma, CLAHE preprocessing
    strategies.append(Strategy(
        "gain=170 gamma=120 + CLAHE(3,16) all frames",
        {"gain": 170, "gamma": 120, "brightness": 0, "contrast": 0,
         "saturation": 70, "sharpness": 1},
        lambda g: clahe_preprocess(g, 3.0, 16),
        detect_with_rescue
    ))

    # 6. Moderate gain, moderate gamma, CLAHE
    strategies.append(Strategy(
        "gain=170 gamma=200 + CLAHE(3,16) all frames",
        {"gain": 170, "gamma": 200, "brightness": 0, "contrast": 0,
         "saturation": 70, "sharpness": 1},
        lambda g: clahe_preprocess(g, 3.0, 16),
        detect_with_rescue
    ))

    # 7. High gain, low gamma, CLAHE
    strategies.append(Strategy(
        "gain=200 gamma=120 + CLAHE(3,16) all frames",
        {"gain": 200, "gamma": 120, "brightness": 0, "contrast": 0,
         "saturation": 70, "sharpness": 1},
        lambda g: clahe_preprocess(g, 3.0, 16),
        detect_with_rescue
    ))

    # 8. Original settings with CLAHE on all frames
    strategies.append(Strategy(
        "gain=40 gamma=120 + CLAHE(3,16) (old settings+CLAHE)",
        {"gain": 40, "gamma": 120, "brightness": 0, "contrast": 0,
         "saturation": 70, "sharpness": 1},
        lambda g: clahe_preprocess(g, 3.0, 16),
        detect_with_rescue
    ))

    # 9. gain=140 gamma=200 no CLAHE
    strategies.append(Strategy(
        "gain=140 gamma=200 (raw+rescue, moderate)",
        {"gain": 140, "gamma": 200, "brightness": 0, "contrast": 0,
         "saturation": 70, "sharpness": 1},
        lambda g: g,
        detect_with_rescue
    ))

    # 10. gain=200 gamma=200 + CLAHE
    strategies.append(Strategy(
        "gain=200 gamma=200 + CLAHE(2,16) all frames",
        {"gain": 200, "gamma": 200, "brightness": 0, "contrast": 0,
         "saturation": 70, "sharpness": 1},
        lambda g: clahe_preprocess(g, 2.0, 16),
        detect_with_rescue
    ))

    return strategies


def open_camera():
    cap = cv2.VideoCapture(CAMERA, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        print("ERROR: cannot open camera")
        sys.exit(1)
    return cap


def flush_frames(cap, n=8):
    for _ in range(n):
        cap.read()


def measure_at_exposure(cap, exposure, strategy, n_frames=N_FRAMES):
    v4l2_set({"exposure_time_absolute": exposure})
    time.sleep(0.15)
    flush_frames(cap, 5)

    counts = {mid: 0 for mid in ALL_IDS}
    for _ in range(n_frames):
        ret, frame = cap.read()
        if not ret:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        processed = strategy.preprocess_fn(gray)
        found = strategy.detect_fn(processed)
        for fid in found:
            if fid in counts:
                counts[fid] += 1

    rates = {mid: counts[mid] / n_frames for mid in ALL_IDS}
    return rates


def main():
    cap = open_camera()

    strategies = make_strategies()

    print("=" * 140)
    print("  LIGHTING RESILIENCE SWEEP")
    print("  Simulating lighting changes via exposure_time_absolute")
    print(f"  Exposures: {EXPOSURES}")
    print(f"  Strategies: {len(strategies)}")
    print(f"  Frames per measurement: {N_FRAMES}")
    print("=" * 140)

    # Store results: strategy_idx -> exposure -> rates
    all_results = {}

    for si, strategy in enumerate(strategies):
        print(f"\n{'─'*140}")
        print(f"  Strategy {si+1}/{len(strategies)}: {strategy.name}")
        print(f"{'─'*140}")

        # Apply hardware controls
        v4l2_set(strategy.hw_controls)
        time.sleep(0.3)
        flush_frames(cap, 8)

        results = {}
        for exp in EXPOSURES:
            rates = measure_at_exposure(cap, exp, strategy)
            table_rate = np.mean([rates[mid] for mid in TABLE_IDS])
            robot_rate = rates[ROBOT_ID]
            all_ok = all(rates[mid] >= 0.8 for mid in ALL_IDS)
            flag = " ✓" if all_ok else ""
            print(f"    exp={exp:4d}  "
                  f"20={rates[20]:4.0%}  21={rates[21]:4.0%}  "
                  f"22={rates[22]:4.0%}  23={rates[23]:4.0%}  "
                  f"robot={robot_rate:4.0%}  tableAvg={table_rate:4.0%}{flag}")
            results[exp] = rates

        all_results[si] = results

    # ── Summary ──
    print("\n" + "=" * 140)
    print("  SUMMARY: Number of exposures where ALL 5 markers detected ≥80% of frames")
    print("=" * 140)

    summary = []
    for si, strategy in enumerate(strategies):
        results = all_results[si]
        ok_count = 0
        for exp in EXPOSURES:
            rates = results[exp]
            if all(rates[mid] >= 0.8 for mid in ALL_IDS):
                ok_count += 1
        pct = ok_count / len(EXPOSURES)
        summary.append((ok_count, pct, si, strategy.name))
        print(f"  {ok_count}/{len(EXPOSURES)} ({pct:4.0%})  {strategy.name}")

    summary.sort(key=lambda x: -x[0])
    print(f"\n  MOST RESILIENT: {summary[0][3]}  ({summary[0][0]}/{len(EXPOSURES)} exposures ok)")

    # Also show which exposures fail per strategy
    print("\n" + "=" * 140)
    print("  DETAIL: Which exposures fail for each strategy (any marker <80%)")
    print("=" * 140)
    for si, strategy in enumerate(strategies):
        results = all_results[si]
        fails = []
        for exp in EXPOSURES:
            rates = results[exp]
            bad = [f"id{mid}={rates[mid]:.0%}" for mid in ALL_IDS if rates[mid] < 0.8]
            if bad:
                fails.append(f"exp={exp}({','.join(bad)})")
        if fails:
            print(f"  {strategy.name}")
            print(f"    FAILS: {'; '.join(fails)}")
        else:
            print(f"  {strategy.name}  → NO FAILURES")

    # Restore good settings
    v4l2_set({"gain": 227, "gamma": 300, "exposure_time_absolute": 200,
              "brightness": 0, "contrast": 0, "saturation": 70, "sharpness": 1})

    cap.release()
    print("\nDone. Camera restored to gain=227 gamma=300 exp=200.")


if __name__ == "__main__":
    main()
