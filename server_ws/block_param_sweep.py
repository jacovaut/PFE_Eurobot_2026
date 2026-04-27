#!/usr/bin/env python3
"""
Sweep hardware and rescue-pass parameters for border block detection.

This script mirrors the block detection path used by global_localization_node:
  - main ArUco pass on raw grayscale
  - block border rescue pass on padded + CLAHE grayscale

It can:
  - sweep one parameter at a time
  - run a 2D grid sweep over two parameters
  - count only detections near the frame borders

Examples:
    python3 block_param_sweep.py --controls exposure_time_absolute gain
    python3 block_param_sweep.py --mode grid --controls gain exposure_time_absolute
    python3 block_param_sweep.py --controls block_rescue_pad_ratio block_rescue_clahe_clip_limit --border-only
"""

import argparse
import subprocess
import sys
import time
from dataclasses import dataclass
from math import hypot

import cv2


CAMERA_DEVICE = "/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera_01.00.00-video-index0"
WIDTH = 3840
HEIGHT = 2160
FPS = 15
FOURCC = "MJPG"

ARUCO_DICT = cv2.aruco.DICT_4X4_50
BLOCK_MARKER_IDS = {36, 47}

WARMUP_FRAMES = 6
MEASURE_FRAMES = 15


@dataclass
class SweepControl:
    name: str
    min_val: float
    max_val: float
    default_val: float
    current_val: float
    step: float = 1.0
    kind: str = "hardware"
    value_type: str = "int"


HARDWARE_CONTROLS = {
    "contrast": SweepControl("contrast", 0, 95, 0, 0, 1, "hardware", "int"),
    "sharpness": SweepControl("sharpness", 0, 7, 0, 0, 1, "hardware", "int"),
    "exposure_time_absolute": SweepControl("exposure_time_absolute", 1, 5000, 156, 200, 1, "hardware", "int"),
    "gain": SweepControl("gain", 0, 255, 110, 210, 1, "hardware", "int"),
    "gamma": SweepControl("gamma", 64, 300, 110, 230, 1, "hardware", "int"),
    "focus_absolute": SweepControl("focus_absolute", 0, 1023, 0, 0, 1, "hardware", "int"),
}

SOFTWARE_CONTROLS = {
    "block_rescue_pad_ratio": SweepControl("block_rescue_pad_ratio", 0.02, 0.14, 0.06, 0.06, 0.01, "software", "float"),
    "block_rescue_clahe_clip_limit": SweepControl("block_rescue_clahe_clip_limit", 1.0, 6.0, 3.0, 3.0, 0.5, "software", "float"),
    "block_rescue_adaptive_thresh_win_size_max": SweepControl("block_rescue_adaptive_thresh_win_size_max", 61, 241, 181, 181, 20, "software", "int"),
    "block_rescue_adaptive_thresh_win_size_step": SweepControl("block_rescue_adaptive_thresh_win_size_step", 10, 60, 30, 30, 5, "software", "int"),
    "block_rescue_min_marker_perimeter_rate": SweepControl("block_rescue_min_marker_perimeter_rate", 0.001, 0.01, 0.003, 0.003, 0.001, "software", "float"),
    "block_rescue_error_correction_rate": SweepControl("block_rescue_error_correction_rate", 0.2, 0.8, 0.6, 0.6, 0.05, "software", "float"),
}

ALL_CONTROLS = {**HARDWARE_CONTROLS, **SOFTWARE_CONTROLS}

DEFAULT_RESCUE_CONFIG = {
    "block_rescue_pad_ratio": 0.02,
    "block_rescue_clahe_clip_limit": 2.0,
    "block_rescue_adaptive_thresh_win_size_max": 200,
    "block_rescue_adaptive_thresh_win_size_step": 30,
    "block_rescue_min_marker_perimeter_rate": 0.003,
    "block_rescue_error_correction_rate": 0.8,
    "block_rescue_min_distance_to_border": 0,
}


def normalize_value(control: SweepControl, value: float):
    scaled = round(value / control.step) * control.step if control.step else value
    clamped = max(control.min_val, min(control.max_val, scaled))
    if control.value_type == "int":
        return int(round(clamped))
    decimals = 0
    if control.step < 1:
        step_str = f"{control.step:.6f}".rstrip("0")
        if "." in step_str:
            decimals = len(step_str.split(".")[1])
    return round(clamped, decimals)


def format_value(control: SweepControl, value) -> str:
    if control.value_type == "int":
        return str(int(round(value)))
    return f"{value:.3f}".rstrip("0").rstrip(".")


def generate_values(control: SweepControl, steps: int):
    values = {
        normalize_value(control, control.default_val),
        normalize_value(control, control.current_val),
    }
    for i in range(max(steps, 2)):
        raw = control.min_val + (control.max_val - control.min_val) * i / max(steps - 1, 1)
        values.add(normalize_value(control, raw))
    return sorted(values)


def v4l2_set_many(values: dict[str, int]) -> None:
    if not values:
        return
    ctrl_string = ",".join(f"{name}={value}" for name, value in values.items())
    subprocess.run(
        ["v4l2-ctl", "-d", CAMERA_DEVICE, "--set-ctrl", ctrl_string],
        check=True,
        capture_output=True,
        text=True,
    )


def v4l2_get(ctrl_name: str) -> int:
    result = subprocess.run(
        ["v4l2-ctl", "-d", CAMERA_DEVICE, "--get-ctrl", ctrl_name],
        check=True,
        capture_output=True,
        text=True,
    )
    return int(result.stdout.strip().split(":")[-1].strip())


def make_base_params():
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
    return params


def make_block_rescue_params(rescue_config):
    params = make_base_params()
    params.adaptiveThreshWinSizeMax = int(rescue_config["block_rescue_adaptive_thresh_win_size_max"])
    params.adaptiveThreshWinSizeStep = int(rescue_config["block_rescue_adaptive_thresh_win_size_step"])
    params.minMarkerPerimeterRate = float(rescue_config["block_rescue_min_marker_perimeter_rate"])
    params.errorCorrectionRate = float(rescue_config["block_rescue_error_correction_rate"])
    params.minDistanceToBorder = int(rescue_config["block_rescue_min_distance_to_border"])
    return params


def open_camera():
    cap = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"ERROR: cannot open camera {CAMERA_DEVICE}")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*FOURCC))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    for _ in range(4):
        cap.read()
    return cap


def detect_markers(gray, dictionary, params):
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    if ids is None or corners is None:
        return []

    detections = []
    for marker_id, marker_corners in zip(ids.flatten(), corners):
        pts = marker_corners[0]
        center_x = float(sum(pt[0] for pt in pts) / 4.0)
        center_y = float(sum(pt[1] for pt in pts) / 4.0)
        detections.append({
            "id": int(marker_id),
            "center": (center_x, center_y),
        })
    return detections


def merge_detections(primary, rescue, distance_px: float = 40.0):
    merged = list(primary)
    for candidate in rescue:
        duplicate = False
        for existing in merged:
            if existing["id"] != candidate["id"]:
                continue
            dx = existing["center"][0] - candidate["center"][0]
            dy = existing["center"][1] - candidate["center"][1]
            if hypot(dx, dy) <= distance_px:
                duplicate = True
                break
        if not duplicate:
            merged.append(candidate)
    return merged


def filter_border_detections(detections, width: int, height: int, border_margin_px: int):
    filtered = []
    for detection in detections:
        x, y = detection["center"]
        if x <= border_margin_px or x >= width - border_margin_px:
            filtered.append(detection)
            continue
        if y <= border_margin_px or y >= height - border_margin_px:
            filtered.append(detection)
    return filtered


def detect_blocks(frame, dictionary, base_params, rescue_config, border_only: bool, border_margin_px: int):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    found = detect_markers(gray, dictionary, base_params)
    block_found = [d for d in found if d["id"] in BLOCK_MARKER_IDS]

    # Always run the rescue pass — it can recover additional instances of IDs
    # that are partially cut off at the border, even if one instance was found.
    if True:
        pad_ratio = float(rescue_config["block_rescue_pad_ratio"])
        pad_px = max(8, int(round(min(gray.shape[0], gray.shape[1]) * pad_ratio)))
        padded = cv2.copyMakeBorder(
            gray,
            pad_px,
            pad_px,
            pad_px,
            pad_px,
            cv2.BORDER_REPLICATE,
        )

        clahe = cv2.createCLAHE(
            clipLimit=float(rescue_config["block_rescue_clahe_clip_limit"]),
            tileGridSize=(16, 16),
        )
        enhanced = clahe.apply(padded)
        rescue_params = make_block_rescue_params(rescue_config)
        rescue_found = detect_markers(enhanced, dictionary, rescue_params)

        adjusted_rescue = []
        for detection in rescue_found:
            if detection["id"] not in BLOCK_MARKER_IDS:
                continue
            adjusted_rescue.append({
                "id": detection["id"],
                "center": (
                    detection["center"][0] - pad_px,
                    detection["center"][1] - pad_px,
                ),
            })

        block_found = merge_detections(block_found, adjusted_rescue)

    if border_only:
        return filter_border_detections(block_found, gray.shape[1], gray.shape[0], border_margin_px)
    return block_found


def measure(cap, dictionary, base_params, rescue_config, frames: int, border_only: bool, border_margin_px: int):
    for _ in range(WARMUP_FRAMES):
        cap.read()

    total_block_counts = []
    hits = {block_id: 0 for block_id in BLOCK_MARKER_IDS}
    total_instances = {block_id: 0 for block_id in BLOCK_MARKER_IDS}

    for _ in range(frames):
        ok, frame = cap.read()
        if not ok or frame is None:
            continue

        found = detect_blocks(frame, dictionary, base_params, rescue_config, border_only, border_margin_px)
        total_block_counts.append(len(found))
        for block_id in BLOCK_MARKER_IDS:
            instance_count = sum(1 for detection in found if detection["id"] == block_id)
            total_instances[block_id] += instance_count
            if instance_count > 0:
                hits[block_id] += 1

    if not total_block_counts:
        zeros = {block_id: 0.0 for block_id in BLOCK_MARKER_IDS}
        return 0.0, zeros, zeros

    usable = len(total_block_counts)
    rates = {block_id: hits[block_id] / usable for block_id in BLOCK_MARKER_IDS}
    avg_instances = {block_id: total_instances[block_id] / usable for block_id in BLOCK_MARKER_IDS}
    avg_count = sum(total_block_counts) / usable
    return avg_count, rates, avg_instances


def apply_overrides(overrides: dict[str, float]):
    hardware = {}
    rescue_config = dict(DEFAULT_RESCUE_CONFIG)

    for name, value in overrides.items():
        control = ALL_CONTROLS[name]
        normalized = normalize_value(control, value)
        if control.kind == "hardware":
            hardware[name] = int(normalized)
        else:
            rescue_config[name] = normalized

    if hardware:
        v4l2_set_many(hardware)
    return rescue_config


def score_measurement(avg_count, avg_instances):
    return avg_count + 0.1 * avg_instances[36] + 0.1 * avg_instances[47]


def sweep_one_control(cap, dictionary, base_params, control: SweepControl, steps: int, frames: int, border_only: bool, border_margin_px: int):
    values = generate_values(control, steps)
    print(f"\n{'=' * 100}")
    print(f"Sweeping {control.name}: range=[{format_value(control, control.min_val)}, {format_value(control, control.max_val)}] current={format_value(control, control.current_val)} default={format_value(control, control.default_val)}")
    print(f"Testing values: {[format_value(control, value) for value in values]}")
    print(f"{'=' * 100}")

    best = None
    baseline = None

    for value in values:
        try:
            rescue_config = apply_overrides({control.name: value})
        except subprocess.CalledProcessError as exc:
            stderr = exc.stderr.strip() if exc.stderr else "set failed"
            print(f"  {control.name}={format_value(control, value):>6s} -> SKIP ({stderr})")
            continue

        time.sleep(0.2)
        avg_count, rates, avg_instances = measure(cap, dictionary, base_params, rescue_config, frames, border_only, border_margin_px)
        line = (
            f"  {control.name}={format_value(control, value):>6s} -> blocks/frame={avg_count:.2f}  "
            f"36 hit={rates[36]:.0%} avg={avg_instances[36]:.2f}  "
            f"47 hit={rates[47]:.0%} avg={avg_instances[47]:.2f}"
        )

        if value == normalize_value(control, control.current_val):
            baseline = (value, avg_count, rates, avg_instances)
            line += "  <-- CURRENT"
        elif value == normalize_value(control, control.default_val):
            line += "  <-- DEFAULT"

        print(line)

        candidate = (score_measurement(avg_count, avg_instances), avg_count, value, rates, avg_instances)
        if best is None or candidate > best:
            best = candidate

    return baseline, best


def sweep_grid(cap, dictionary, base_params, controls: list[SweepControl], steps: int, frames: int, border_only: bool, border_margin_px: int):
    first_values = generate_values(controls[0], steps)
    second_values = generate_values(controls[1], steps)

    print(f"\n{'=' * 100}")
    print(f"Grid sweep: {controls[0].name} x {controls[1].name}")
    print(f"{controls[0].name}: {[format_value(controls[0], value) for value in first_values]}")
    print(f"{controls[1].name}: {[format_value(controls[1], value) for value in second_values]}")
    print(f"{'=' * 100}")

    best = None
    total_trials = len(first_values) * len(second_values)
    trial = 0
    for first in first_values:
        for second in second_values:
            trial += 1
            overrides = {
                controls[0].name: first,
                controls[1].name: second,
            }
            try:
                rescue_config = apply_overrides(overrides)
            except subprocess.CalledProcessError as exc:
                stderr = exc.stderr.strip() if exc.stderr else "set failed"
                print(f"[{trial:>3d}/{total_trials}] SKIP ({stderr})")
                continue

            time.sleep(0.2)
            avg_count, rates, avg_instances = measure(cap, dictionary, base_params, rescue_config, frames, border_only, border_margin_px)
            print(
                f"[{trial:>3d}/{total_trials}] "
                f"{controls[0].name}={format_value(controls[0], first):>6s} "
                f"{controls[1].name}={format_value(controls[1], second):>6s} -> "
                f"blocks/frame={avg_count:.2f} 36={avg_instances[36]:.2f}/{rates[36]:.0%} 47={avg_instances[47]:.2f}/{rates[47]:.0%}"
            )

            candidate = (score_measurement(avg_count, avg_instances), avg_count, first, second, rates, avg_instances)
            if best is None or candidate > best:
                best = candidate

    return best


def restore_current_hardware_values():
    current_values = {
        name: int(HARDWARE_CONTROLS[name].current_val)
        for name in HARDWARE_CONTROLS
    }
    v4l2_set_many(current_values)


def main():
    parser = argparse.ArgumentParser(description="Sweep hardware and rescue parameters for border block detection")
    parser.add_argument("--controls", nargs="*", default=["gain", "exposure_time_absolute"], help="Controls to sweep")
    parser.add_argument("--mode", choices=["single", "grid"], default="single", help="Sweep one control at a time or a 2D grid")
    parser.add_argument("--steps", type=int, default=6, help="Values to try per control")
    parser.add_argument("--frames", type=int, default=MEASURE_FRAMES, help="Frames per measurement")
    parser.add_argument("--border-only", action="store_true", help="Only count block detections near the frame border")
    parser.add_argument("--border-margin-ratio", type=float, default=0.18, help="Border width as a fraction of min(frame width, frame height)")
    args = parser.parse_args()

    selected = [ALL_CONTROLS[name] for name in args.controls if name in ALL_CONTROLS]
    if not selected:
        print(f"ERROR: no supported controls selected. Available: {sorted(ALL_CONTROLS)}")
        sys.exit(1)
    if args.mode == "grid" and len(selected) != 2:
        print("ERROR: grid mode requires exactly 2 controls")
        sys.exit(1)

    for name, control in HARDWARE_CONTROLS.items():
        control.current_val = v4l2_get(name)
    for name, control in SOFTWARE_CONTROLS.items():
        control.current_val = DEFAULT_RESCUE_CONFIG[name]

    border_margin_px = int(round(min(WIDTH, HEIGHT) * args.border_margin_ratio))

    print("=" * 100)
    print("BLOCK DETECTION PARAMETER SWEEP")
    print(f"Camera: {CAMERA_DEVICE}")
    print(f"Resolution: {WIDTH}x{HEIGHT} @ {FPS} FPS")
    print(f"Mode: {args.mode}")
    print(f"Controls: {[control.name for control in selected]}")
    print(f"Frames per measurement: {args.frames}")
    print(f"Target block IDs: {sorted(BLOCK_MARKER_IDS)}")
    print(f"Border only: {args.border_only} (margin {border_margin_px}px)")
    print("Detection path: raw grayscale + block rescue pass")
    print("=" * 100)

    cap = open_camera()
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    base_params = make_base_params()

    try:
        if args.mode == "grid":
            best = sweep_grid(cap, dictionary, base_params, selected, args.steps, args.frames, args.border_only, border_margin_px)
            print(f"\n{'=' * 100}")
            print("GRID SUMMARY")
            print(f"{'=' * 100}")
            if best is None:
                print("  No valid grid measurements")
            else:
                _, best_avg, first, second, _, avg_instances = best
                print(
                    f"  best {selected[0].name}={format_value(selected[0], first)} "
                    f"{selected[1].name}={format_value(selected[1], second)}  "
                    f"blocks/frame={best_avg:.2f} 36={avg_instances[36]:.2f} 47={avg_instances[47]:.2f}"
                )
        else:
            results = []
            for control in selected:
                baseline, best = sweep_one_control(cap, dictionary, base_params, control, args.steps, args.frames, args.border_only, border_margin_px)
                results.append((control, baseline, best))

            print(f"\n{'=' * 100}")
            print("SUMMARY")
            print(f"{'=' * 100}")
            for control, baseline, best in results:
                if best is None:
                    print(f"  {control.name}: no valid measurements")
                    continue

                _, best_avg, best_value, best_rates, best_instances = best
                if baseline is None:
                    print(
                        f"  {control.name}: best={format_value(control, best_value)} "
                        f"blocks/frame={best_avg:.2f} 36={best_instances[36]:.2f} 47={best_instances[47]:.2f}"
                    )
                    continue

                _, base_avg, base_rates, base_instances = baseline
                print(
                    f"  {control.name}: current={format_value(control, control.current_val)} -> {format_value(control, best_value)}  "
                    f"blocks/frame {base_avg:.2f} -> {best_avg:.2f}  "
                    f"36 {base_instances[36]:.2f}/{base_rates[36]:.0%} -> {best_instances[36]:.2f}/{best_rates[36]:.0%}  "
                    f"47 {base_instances[47]:.2f}/{base_rates[47]:.0%} -> {best_instances[47]:.2f}/{best_rates[47]:.0%}"
                )
    finally:
        cap.release()
        restore_current_hardware_values()


if __name__ == "__main__":
    main()