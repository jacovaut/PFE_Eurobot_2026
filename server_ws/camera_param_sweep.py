#!/usr/bin/env python3
"""
Camera parameter sweep for ArUco marker detection optimisation.

Opens the overhead USB camera according to the same settings used by
global_localization_node, then sweeps V4L2 controls one-at-a-time and
measures how many ArUco markers are detected per frame.

Usage:
    # Make sure the ROS node is NOT running (camera can only have one user).
    python3 camera_param_sweep.py

    # Optional: only sweep a subset of controls
    python3 camera_param_sweep.py --controls brightness contrast exposure_time_absolute
"""

import argparse
import subprocess
import sys
import time
from collections import OrderedDict
from dataclasses import dataclass, field

import cv2
import numpy as np

# ── Camera / detection constants (must match global_localization_node) ───────

CAMERA_DEVICE = "/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera_01.00.00-video-index0"
WIDTH = 3840
HEIGHT = 2160
FPS = 15
FOURCC = "MJPG"

# ArUco detector params (same as camera_global_map.yaml)
ARUCO_DICT = cv2.aruco.DICT_4X4_50
DETECTOR_PARAMS = {
    "perspectiveRemovePixelPerCell": 10,
    "adaptiveThreshWinSizeMin": 3,
    "adaptiveThreshWinSizeMax": 101,
    "adaptiveThreshWinSizeStep": 10,
    "adaptiveThreshConstant": 7.0,
    "minMarkerPerimeterRate": 0.005,
    "maxMarkerPerimeterRate": 4.0,
    "errorCorrectionRate": 0.45,
    "cornerRefinementMethod": cv2.aruco.CORNER_REFINE_SUBPIX,
}

# Expected marker IDs on the table (the ones we care about detecting)
TABLE_MARKER_IDS = {20, 21, 22, 23}
ROBOT_MARKER_ID = 1
BLOCK_MARKER_IDS = {36, 47}
ALL_EXPECTED_IDS = TABLE_MARKER_IDS | {ROBOT_MARKER_ID} | BLOCK_MARKER_IDS

# ── Sweep configuration ─────────────────────────────────────────────────────

WARMUP_FRAMES = 5      # frames to discard after changing a setting
MEASURE_FRAMES = 10    # frames to average for each measurement
STEPS_PER_CONTROL = 12 # how many values to try for each control

@dataclass
class V4L2Control:
    name: str
    min_val: int
    max_val: int
    default_val: int
    current_val: int
    step: int = 1

# Controls we want to sweep (ranges from v4l2-ctl output)
ALL_CONTROLS = OrderedDict([
    ("brightness",            V4L2Control("brightness",            -64,    64,    0,   64)),
    ("contrast",              V4L2Control("contrast",                0,    95,    0,   40)),
    ("saturation",            V4L2Control("saturation",              0,   255,   70,   70)),
    ("gamma",                 V4L2Control("gamma",                  64,   300,  110,  120)),
    ("gain",                  V4L2Control("gain",                    0,   255,  110,   40)),
    ("sharpness",             V4L2Control("sharpness",               0,     7,    0,    3)),
    ("backlight_compensation",V4L2Control("backlight_compensation",  0,   100,   80,    0)),
    ("exposure_time_absolute",V4L2Control("exposure_time_absolute",  1, 5000,  156,  200)),
    ("focus_absolute",        V4L2Control("focus_absolute",          0,  1023,    0,  510)),
])

# ── Helpers ──────────────────────────────────────────────────────────────────

def v4l2_set(ctrl_name: str, value: int):
    """Set a V4L2 control via v4l2-ctl."""
    subprocess.run(
        ["v4l2-ctl", "-d", CAMERA_DEVICE, "--set-ctrl", f"{ctrl_name}={value}"],
        check=True,
        capture_output=True,
    )


def v4l2_get(ctrl_name: str) -> int:
    """Read a V4L2 control's current value."""
    result = subprocess.run(
        ["v4l2-ctl", "-d", CAMERA_DEVICE, "--get-ctrl", ctrl_name],
        check=True,
        capture_output=True,
        text=True,
    )
    # Output like "brightness: 64"
    return int(result.stdout.strip().split(":")[-1].strip())


def make_detector():
    params = cv2.aruco.DetectorParameters_create()
    for k, v in DETECTOR_PARAMS.items():
        setattr(params, k, v)
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    return dictionary, params


def detect_markers(frame, dictionary, params):
    """Run ArUco detection and return (total_count, table_count, ids_set)."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    if ids is None:
        return 0, 0, set()
    id_set = set(ids.flatten().tolist())
    table_count = len(id_set & TABLE_MARKER_IDS)
    return len(ids), table_count, id_set


def open_camera():
    cap = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"ERROR: Cannot open camera {CAMERA_DEVICE}")
        sys.exit(1)
    cc = cv2.VideoWriter_fourcc(*FOURCC)
    cap.set(cv2.CAP_PROP_FOURCC, cc)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    # Read a few warmup frames
    for _ in range(3):
        cap.read()
    return cap


def measure(cap, dictionary, params, warmup=WARMUP_FRAMES, n_frames=MEASURE_FRAMES):
    """Capture n_frames and return average stats."""
    # Flush stale buffered frames
    for _ in range(warmup):
        cap.read()

    total_counts = []
    table_counts = []
    all_ids_seen = set()

    for _ in range(n_frames):
        ret, frame = cap.read()
        if not ret or frame is None:
            continue
        total, table, ids = detect_markers(frame, dictionary, params)
        total_counts.append(total)
        table_counts.append(table)
        all_ids_seen |= ids

    if not total_counts:
        return 0.0, 0.0, set()

    return (
        sum(total_counts) / len(total_counts),
        sum(table_counts) / len(table_counts),
        all_ids_seen,
    )


def generate_sweep_values(ctrl: V4L2Control, n_steps: int):
    """Return a list of values to test for this control, always including the
    current value and the default value."""
    values = set()
    values.add(ctrl.current_val)
    values.add(ctrl.default_val)

    # Spread evenly across the range
    for i in range(n_steps):
        v = ctrl.min_val + (ctrl.max_val - ctrl.min_val) * i / max(n_steps - 1, 1)
        v = int(round(v / ctrl.step) * ctrl.step)
        v = max(ctrl.min_val, min(ctrl.max_val, v))
        values.add(v)

    return sorted(values)


# ── Main ─────────────────────────────────────────────────────────────────────

@dataclass
class SweepResult:
    control: str
    best_value: int
    best_total: float
    best_table: float
    baseline_total: float
    baseline_table: float
    detail: list = field(default_factory=list)   # list of (value, avg_total, avg_table, ids_seen)


def sweep_control(cap, dictionary, params, ctrl: V4L2Control, n_steps=STEPS_PER_CONTROL, n_frames=MEASURE_FRAMES) -> SweepResult:
    """Sweep one control across its range and return results."""
    values = generate_sweep_values(ctrl, n_steps)
    print(f"\n{'='*70}")
    print(f"  Sweeping: {ctrl.name}")
    print(f"  Range: [{ctrl.min_val} .. {ctrl.max_val}]  default={ctrl.default_val}  current={ctrl.current_val}")
    print(f"  Testing {len(values)} values: {values}")
    print(f"{'='*70}")

    detail = []
    baseline_total = 0.0
    baseline_table = 0.0

    for val in values:
        try:
            v4l2_set(ctrl.name, val)
        except subprocess.CalledProcessError:
            print(f"  [{ctrl.name}={val}] SKIP (v4l2 error)")
            continue

        time.sleep(0.15)  # v4l2 settle time

        avg_total, avg_table, ids_seen = measure(cap, dictionary, params, n_frames=n_frames)
        ids_str = ",".join(str(i) for i in sorted(ids_seen)) if ids_seen else "none"

        marker = ""
        if val == ctrl.current_val:
            marker = " <-- CURRENT"
            baseline_total = avg_total
            baseline_table = avg_table
        elif val == ctrl.default_val:
            marker = " <-- DEFAULT"

        print(f"  {ctrl.name}={val:>6d}  →  total={avg_total:.1f}  table={avg_table:.1f}  ids=[{ids_str}]{marker}")
        detail.append((val, avg_total, avg_table, ids_seen))

    # Restore current value
    try:
        v4l2_set(ctrl.name, ctrl.current_val)
    except subprocess.CalledProcessError:
        pass

    if not detail:
        return SweepResult(ctrl.name, ctrl.current_val, 0, 0, 0, 0)

    # Best = highest table marker count, ties broken by total count
    best = max(detail, key=lambda d: (d[2], d[1]))
    return SweepResult(
        control=ctrl.name,
        best_value=best[0],
        best_total=best[1],
        best_table=best[2],
        baseline_total=baseline_total,
        baseline_table=baseline_table,
        detail=detail,
    )


def main():
    parser = argparse.ArgumentParser(description="Sweep camera V4L2 controls for ArUco detection")
    parser.add_argument(
        "--controls", nargs="*", default=None,
        help="Only sweep these controls (default: all)")
    parser.add_argument(
        "--steps", type=int, default=STEPS_PER_CONTROL,
        help=f"Number of values to try per control (default {STEPS_PER_CONTROL})")
    parser.add_argument(
        "--frames", type=int, default=MEASURE_FRAMES,
        help=f"Frames to average per measurement (default {MEASURE_FRAMES})")
    args = parser.parse_args()

    steps_per_control = args.steps
    measure_frames = args.frames

    controls_to_sweep = list(ALL_CONTROLS.keys())
    if args.controls:
        controls_to_sweep = [c for c in args.controls if c in ALL_CONTROLS]
        if not controls_to_sweep:
            print("ERROR: None of the specified controls are known.")
            print(f"  Available: {list(ALL_CONTROLS.keys())}")
            sys.exit(1)

    print("╔══════════════════════════════════════════════════════════╗")
    print("║        Camera Parameter Sweep for ArUco Detection       ║")
    print("╚══════════════════════════════════════════════════════════╝")
    print(f"Camera: {CAMERA_DEVICE}")
    print(f"Resolution: {WIDTH}x{HEIGHT} @ {FPS} FPS")
    print(f"Controls to sweep: {controls_to_sweep}")
    print(f"Steps per control: {steps_per_control}, Frames per measurement: {measure_frames}")

    # Read actual current values from hardware
    print("\nReading current camera settings from hardware...")
    for name, ctrl in ALL_CONTROLS.items():
        try:
            ctrl.current_val = v4l2_get(name)
            print(f"  {name}: {ctrl.current_val} (default: {ctrl.default_val})")
        except Exception as e:
            print(f"  {name}: read failed ({e})")

    print("\nOpening camera...")
    cap = open_camera()
    dictionary, params = make_detector()

    # Baseline measurement with current settings
    print("\n--- Baseline measurement (current settings) ---")
    base_total, base_table, base_ids = measure(cap, dictionary, params, warmup=10, n_frames=measure_frames * 2)
    base_ids_str = ",".join(str(i) for i in sorted(base_ids)) if base_ids else "none"
    print(f"  Total markers: {base_total:.1f}")
    print(f"  Table markers: {base_table:.1f}")
    print(f"  IDs seen: [{base_ids_str}]")

    # Sweep each control
    results: list[SweepResult] = []
    for ctrl_name in controls_to_sweep:
        ctrl = ALL_CONTROLS[ctrl_name]
        r = sweep_control(cap, dictionary, params, ctrl, steps_per_control, measure_frames)
        results.append(r)

    cap.release()

    # ── Summary report ───────────────────────────────────────────────────
    print("\n")
    print("╔══════════════════════════════════════════════════════════════════════╗")
    print("║                          SWEEP RESULTS SUMMARY                      ║")
    print("╚══════════════════════════════════════════════════════════════════════╝")
    print(f"\n  Baseline (current settings): total={base_total:.1f}  table={base_table:.1f}  ids=[{base_ids_str}]")
    print()

    # Sort by improvement in table marker count
    results.sort(key=lambda r: r.best_table - r.baseline_table, reverse=True)

    print(f"  {'Control':<28s} {'Current':>8s} {'Best':>8s} {'Table Δ':>8s} {'Total Δ':>8s} {'Impact':>10s}")
    print(f"  {'─'*28} {'─'*8} {'─'*8} {'─'*8} {'─'*8} {'─'*10}")

    for r in results:
        table_delta = r.best_table - r.baseline_table
        total_delta = r.best_total - r.baseline_total
        current_str = f"{ALL_CONTROLS[r.control].current_val}"
        best_str = f"{r.best_value}"

        if table_delta > 0.5:
            impact = "★★★ HIGH"
        elif table_delta > 0.1:
            impact = "★★  MED"
        elif total_delta > 0.5:
            impact = "★   LOW"
        elif table_delta < -0.1:
            impact = "⚠  WORSE"
        else:
            impact = "─  NONE"

        print(f"  {r.control:<28s} {current_str:>8s} {best_str:>8s} {table_delta:>+8.1f} {total_delta:>+8.1f} {impact:>10s}")

    print()
    print("  Recommended changes (controls where a different value helps):")
    print()
    any_improvement = False
    for r in results:
        if r.best_value != ALL_CONTROLS[r.control].current_val and r.best_table > r.baseline_table + 0.05:
            any_improvement = True
            print(f"    v4l2-ctl -d {CAMERA_DEVICE} --set-ctrl {r.control}={r.best_value}")
            print(f"      (table markers: {r.baseline_table:.1f} → {r.best_table:.1f})")

    if not any_improvement:
        print("    No individual control change clearly improves detection.")
        print("    Current settings may already be near-optimal.")

    print()
    print("  To apply all recommendations at once, copy the v4l2-ctl commands above.")
    print("  You can also add them to your camera_global_map.yaml or a startup script.")
    print()


if __name__ == "__main__":
    main()
