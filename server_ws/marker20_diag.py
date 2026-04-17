#!/usr/bin/env python3
"""
Diagnostic script to figure out why table marker ID 20 is hard to detect
and find the best preprocessing/param combo to make it reliable.
"""

import cv2
import numpy as np
import sys

CAMERA = "/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera_01.00.00-video-index0"
WIDTH, HEIGHT, FPS = 3840, 2160, 15
TABLE_IDS = {20, 21, 22, 23}
ROBOT_ID = 1
N_FRAMES = 15  # average over this many frames


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
    # flush
    for _ in range(5):
        cap.read()
    return cap


def base_detector_params():
    """Same params as camera_global_map.yaml"""
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


def rescue_detector_params():
    """Rescue pass params from YAML"""
    p = base_detector_params()
    p.adaptiveThreshWinSizeMax = 181
    p.adaptiveThreshWinSizeStep = 30
    p.minMarkerPerimeterRate = 0.015
    p.errorCorrectionRate = 0.6
    return p


DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def detect(gray, params=None):
    if params is None:
        params = base_detector_params()
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, DICT, parameters=params)
    found = set()
    if ids is not None:
        found = set(ids.flatten().tolist())
    return found, ids, corners, rejected


def measure(cap, preprocess_fn, params, label, n_frames=N_FRAMES):
    """Run detection n_frames times and report per-marker hit rates."""
    counts = {}
    for _ in range(n_frames):
        ret, frame = cap.read()
        if not ret:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        processed = preprocess_fn(gray)
        found, _, _, _ = detect(processed, params)
        for fid in found:
            counts[fid] = counts.get(fid, 0) + 1

    # per-marker hit rate
    rates = {mid: counts.get(mid, 0) / n_frames for mid in sorted(TABLE_IDS | {ROBOT_ID})}
    m20 = rates.get(20, 0.0)
    table_avg = np.mean([rates.get(mid, 0.0) for mid in TABLE_IDS])
    print(f"  {label:<55s}  20={m20:.0%}  21={rates[21]:.0%}  22={rates[22]:.0%}  23={rates[23]:.0%}  robot={rates[1]:.0%}  TableAvg={table_avg:.0%}")
    return rates


def identity(gray):
    return gray


def clahe(clip, tile):
    def fn(gray):
        c = cv2.createCLAHE(clipLimit=clip, tileGridSize=(tile, tile))
        return c.apply(gray)
    return fn


def global_histeq(gray):
    return cv2.equalizeHist(gray)


def gaussian_blur(ksize):
    def fn(gray):
        return cv2.GaussianBlur(gray, (ksize, ksize), 0)
    return fn


def blur_then_clahe(ksize, clip, tile):
    def fn(gray):
        blurred = cv2.GaussianBlur(gray, (ksize, ksize), 0)
        c = cv2.createCLAHE(clipLimit=clip, tileGridSize=(tile, tile))
        return c.apply(blurred)
    return fn


def sharpen(gray):
    kernel = np.array([[-1, -1, -1],
                       [-1,  9, -1],
                       [-1, -1, -1]], dtype=np.float32)
    return cv2.filter2D(gray, -1, kernel)


def unsharp_mask(sigma, amount):
    def fn(gray):
        blurred = cv2.GaussianBlur(gray, (0, 0), sigma)
        return cv2.addWeighted(gray, 1.0 + amount, blurred, -amount, 0)
    return fn


def morph_open(ksize):
    def fn(gray):
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (ksize, ksize))
        return cv2.morphologyEx(gray, cv2.MORPH_OPEN, kernel)
    return fn


def morph_close(ksize):
    def fn(gray):
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (ksize, ksize))
        return cv2.morphologyEx(gray, cv2.MORPH_CLOSE, kernel)
    return fn


def bilateral(d, sigmaColor, sigmaSpace):
    def fn(gray):
        return cv2.bilateralFilter(gray, d, sigmaColor, sigmaSpace)
    return fn


def clahe_then_sharpen(clip, tile):
    def fn(gray):
        c = cv2.createCLAHE(clipLimit=clip, tileGridSize=(tile, tile))
        enhanced = c.apply(gray)
        kernel = np.array([[-1, -1, -1],
                           [-1,  9, -1],
                           [-1, -1, -1]], dtype=np.float32)
        return cv2.filter2D(enhanced, -1, kernel)
    return fn


def gamma_correct(gamma):
    def fn(gray):
        inv = 1.0 / gamma
        table = np.array([((i / 255.0) ** inv) * 255 for i in range(256)], dtype=np.uint8)
        return cv2.LUT(gray, table)
    return fn


def multi_clahe(gray):
    """Apply CLAHE at multiple tile sizes and average."""
    results = []
    for tile in [8, 16, 32]:
        c = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(tile, tile))
        results.append(c.apply(gray).astype(np.float32))
    return np.mean(results, axis=0).astype(np.uint8)


def main():
    cap = open_camera()

    # ── Part 1: Analyze a single frame around each table marker ──
    print("=" * 120)
    print("  PART 1: Single-frame brightness analysis around each marker region")
    print("=" * 120)
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, ids_arr, corners_arr, rejected = detect(gray)

    print(f"  Detected: {sorted(found)}")
    print(f"  Rejected candidate count: {len(rejected)}")

    # Quadrant analysis (marker positions are roughly in quadrants)
    h, w = gray.shape
    quadrants = {
        "top-left (ID22 0.6,0.6)":     gray[0:h//2, 0:w//2],
        "top-right (ID23 2.4,0.6)":    gray[0:h//2, w//2:w],
        "bottom-left (ID20 0.6,1.4)":  gray[h//2:h, 0:w//2],
        "bottom-right (ID21 2.4,1.4)": gray[h//2:h, w//2:w],
    }
    for name, roi in quadrants.items():
        print(f"  {name:40s}  mean={roi.mean():.1f}  std={roi.std():.1f}  "
              f"min={roi.min()}  max={roi.max()}")

    # Check if marker 20 is among rejected candidates
    # Rejected markers have corners but failed bit extraction
    print(f"\n  Looking at rejected candidates near marker 20 expected region...")
    # ID 20 is at (0.6, 1.4) - bottom-left area of image
    m20_rejected_count = 0
    for rej in rejected:
        center = np.mean(rej[0], axis=0)
        # bottom-left quadrant
        if center[0] < w // 2 and center[1] > h // 2:
            m20_rejected_count += 1
    print(f"  Rejected candidates in bottom-left quadrant: {m20_rejected_count}")

    # ── Part 2: Systematic preprocessing sweep ──
    print("\n" + "=" * 120)
    print("  PART 2: Preprocessing sweep (base detector params)")
    print("=" * 120)

    tests_base = [
        (identity,                              "raw (no preprocessing)"),
        (global_histeq,                         "global histogram equalization"),
        (clahe(2.0, 8),                         "CLAHE clip=2.0 tile=8"),
        (clahe(2.0, 16),                        "CLAHE clip=2.0 tile=16"),
        (clahe(3.0, 8),                         "CLAHE clip=3.0 tile=8"),
        (clahe(3.0, 16),                        "CLAHE clip=3.0 tile=16"),
        (clahe(3.0, 32),                        "CLAHE clip=3.0 tile=32"),
        (clahe(4.0, 8),                         "CLAHE clip=4.0 tile=8"),
        (clahe(4.0, 16),                        "CLAHE clip=4.0 tile=16"),
        (clahe(5.0, 8),                         "CLAHE clip=5.0 tile=8"),
        (clahe(5.0, 16),                        "CLAHE clip=5.0 tile=16"),
        (clahe(6.0, 8),                         "CLAHE clip=6.0 tile=8"),
        (clahe(8.0, 8),                         "CLAHE clip=8.0 tile=8"),
        (clahe(10.0, 8),                        "CLAHE clip=10.0 tile=8"),
        (gaussian_blur(3),                      "Gaussian blur k=3"),
        (gaussian_blur(5),                      "Gaussian blur k=5"),
        (sharpen,                               "Sharpening kernel"),
        (unsharp_mask(1.0, 0.5),                "Unsharp mask sigma=1 amt=0.5"),
        (unsharp_mask(2.0, 1.0),                "Unsharp mask sigma=2 amt=1.0"),
        (unsharp_mask(3.0, 1.5),                "Unsharp mask sigma=3 amt=1.5"),
        (morph_open(3),                         "Morph open k=3"),
        (morph_close(3),                        "Morph close k=3"),
        (bilateral(5, 50, 50),                  "Bilateral d=5 sC=50 sS=50"),
        (bilateral(9, 75, 75),                  "Bilateral d=9 sC=75 sS=75"),
        (clahe_then_sharpen(3.0, 16),           "CLAHE(3,16) + sharpen"),
        (clahe_then_sharpen(4.0, 8),            "CLAHE(4,8) + sharpen"),
        (blur_then_clahe(3, 3.0, 16),           "Blur(3) + CLAHE(3,16)"),
        (blur_then_clahe(5, 3.0, 16),           "Blur(5) + CLAHE(3,16)"),
        (blur_then_clahe(3, 5.0, 8),            "Blur(3) + CLAHE(5,8)"),
        (gamma_correct(0.5),                    "Gamma correction γ=0.5 (brighten)"),
        (gamma_correct(0.7),                    "Gamma correction γ=0.7"),
        (gamma_correct(1.5),                    "Gamma correction γ=1.5 (darken)"),
        (gamma_correct(2.0),                    "Gamma correction γ=2.0 (darken)"),
        (multi_clahe,                           "Multi-tile CLAHE average"),
    ]

    for fn, label in tests_base:
        measure(cap, fn, base_detector_params(), label)

    # ── Part 3: Detector param sweep with best preprocessors ──
    print("\n" + "=" * 120)
    print("  PART 3: Detector param variants (with CLAHE preprocessing)")
    print("=" * 120)

    param_variants = []

    # Vary error correction rate (higher = more tolerant of bit errors)
    for ecr in [0.3, 0.45, 0.6, 0.8, 1.0]:
        p = base_detector_params()
        p.errorCorrectionRate = ecr
        param_variants.append((clahe(3.0, 16), p, f"CLAHE(3,16) + errorCorr={ecr}"))

    # Vary adaptive threshold constant
    for tc in [3.0, 5.0, 7.0, 10.0, 15.0, 20.0]:
        p = base_detector_params()
        p.adaptiveThreshConstant = tc
        param_variants.append((clahe(3.0, 16), p, f"CLAHE(3,16) + threshConst={tc}"))

    # Vary adaptive threshold window max
    for wmax in [61, 101, 141, 181, 221, 301]:
        p = base_detector_params()
        p.adaptiveThreshWinSizeMax = wmax
        param_variants.append((clahe(3.0, 16), p, f"CLAHE(3,16) + winMax={wmax}"))

    # Vary adaptive threshold step
    for step in [5, 10, 20, 30, 40]:
        p = base_detector_params()
        p.adaptiveThreshWinSizeStep = step
        param_variants.append((clahe(3.0, 16), p, f"CLAHE(3,16) + winStep={step}"))

    # Vary perspectiveRemovePixelPerCell
    for ppc in [4, 6, 8, 10, 14, 18]:
        p = base_detector_params()
        p.perspectiveRemovePixelPerCell = ppc
        param_variants.append((clahe(3.0, 16), p, f"CLAHE(3,16) + pixPerCell={ppc}"))

    # Vary minMarkerPerimeterRate
    for mpr in [0.002, 0.005, 0.01, 0.015, 0.02, 0.03]:
        p = base_detector_params()
        p.minMarkerPerimeterRate = mpr
        param_variants.append((clahe(3.0, 16), p, f"CLAHE(3,16) + minPerimRate={mpr}"))

    # Combined: best rescue-like with varying CLAHE
    for clip in [2.0, 3.0, 4.0, 5.0]:
        p = rescue_detector_params()
        param_variants.append((clahe(clip, 16), p, f"CLAHE({clip},16) + rescue params"))

    for fn, params, label in param_variants:
        measure(cap, fn, params, label)

    # ── Part 4: Combined best candidates ──
    print("\n" + "=" * 120)
    print("  PART 4: Combined best strategies")
    print("=" * 120)

    # Two-pass approach: first raw, then CLAHE rescue for what's missing
    def two_pass_detect(preproc2_fn, params2):
        """Factory for a two-pass measurement."""
        def run(cap_inner, n=N_FRAMES):
            base_p = base_detector_params()
            counts = {}
            for _ in range(n):
                ret, frame = cap_inner.read()
                if not ret:
                    continue
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                found1, ids1, corners1, _ = detect(gray, base_p)
                # rescue pass
                processed = preproc2_fn(gray)
                found2, _, _, _ = detect(processed, params2)
                # merge: use first pass, add missing from second
                all_found = found1 | found2
                for fid in all_found:
                    counts[fid] = counts.get(fid, 0) + 1
            rates = {mid: counts.get(mid, 0) / n for mid in sorted(TABLE_IDS | {ROBOT_ID})}
            return rates
        return run

    two_pass_configs = [
        (clahe(3.0, 16), rescue_detector_params(), "Raw + rescue(CLAHE(3,16), rescue params)"),
        (clahe(4.0, 8),  rescue_detector_params(), "Raw + rescue(CLAHE(4,8), rescue params)"),
        (clahe(5.0, 8),  rescue_detector_params(), "Raw + rescue(CLAHE(5,8), rescue params)"),
        (clahe(3.0, 16), base_detector_params(),   "Raw + rescue(CLAHE(3,16), base params)"),
    ]

    # Also try triple pass
    def triple_pass(preproc2_fn, params2, preproc3_fn, params3):
        def run(cap_inner, n=N_FRAMES):
            base_p = base_detector_params()
            counts = {}
            for _ in range(n):
                ret, frame = cap_inner.read()
                if not ret:
                    continue
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                found1, _, _, _ = detect(gray, base_p)
                found2, _, _, _ = detect(preproc2_fn(gray), params2)
                found3, _, _, _ = detect(preproc3_fn(gray), params3)
                all_found = found1 | found2 | found3
                for fid in all_found:
                    counts[fid] = counts.get(fid, 0) + 1
            rates = {mid: counts.get(mid, 0) / n for mid in sorted(TABLE_IDS | {ROBOT_ID})}
            return rates
        return run

    # Two-pass measurements
    for preproc_fn, params, label in two_pass_configs:
        runner = two_pass_detect(preproc_fn, params)
        rates = runner(cap)
        m20 = rates.get(20, 0.0)
        table_avg = np.mean([rates.get(mid, 0.0) for mid in TABLE_IDS])
        print(f"  {label:<55s}  20={m20:.0%}  21={rates[21]:.0%}  22={rates[22]:.0%}  23={rates[23]:.0%}  robot={rates[1]:.0%}  TableAvg={table_avg:.0%}")

    # Triple pass: raw + CLAHE rescue + global histeq
    runner3 = triple_pass(
        clahe(3.0, 16), rescue_detector_params(),
        global_histeq,  base_detector_params()
    )
    rates = runner3(cap)
    m20 = rates.get(20, 0.0)
    table_avg = np.mean([rates.get(mid, 0.0) for mid in TABLE_IDS])
    print(f"  {'Raw + CLAHE rescue + globalHistEq':<55s}  20={m20:.0%}  21={rates[21]:.0%}  22={rates[22]:.0%}  23={rates[23]:.0%}  robot={rates[1]:.0%}  TableAvg={table_avg:.0%}")

    # Triple pass: raw + CLAHE + gamma
    runner3b = triple_pass(
        clahe(3.0, 16), rescue_detector_params(),
        gamma_correct(0.5), base_detector_params()
    )
    rates = runner3b(cap)
    m20 = rates.get(20, 0.0)
    table_avg = np.mean([rates.get(mid, 0.0) for mid in TABLE_IDS])
    print(f"  {'Raw + CLAHE rescue + gamma=0.5':<55s}  20={m20:.0%}  21={rates[21]:.0%}  22={rates[22]:.0%}  23={rates[23]:.0%}  robot={rates[1]:.0%}  TableAvg={table_avg:.0%}")

    cap.release()
    print("\nDone.")


if __name__ == "__main__":
    main()
