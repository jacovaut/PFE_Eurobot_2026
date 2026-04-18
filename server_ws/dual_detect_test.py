#!/usr/bin/env python3
"""Test dual-detection: raw grayscale + CLAHE grayscale merged,
compared against single-pass approaches, across multiple exposures."""

import subprocess, time, cv2
import numpy as np

DEV = '/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera_01.00.00-video-index0'
TABLE_IDS = {20, 21, 22, 23}
ROBOT_ID = 1
BLOCK_IDS = {36, 47}
ALL_IDS = TABLE_IDS | {ROBOT_ID} | BLOCK_IDS
FRAMES = 12
EXPOSURES = [50, 80, 100, 130, 156, 200, 280, 400, 500]

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

def make_params():
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

def make_rescue_params():
    p = cv2.aruco.DetectorParameters_create()
    p.perspectiveRemovePixelPerCell = 10
    p.adaptiveThreshWinSizeMin = 3
    p.adaptiveThreshWinSizeMax = 181
    p.adaptiveThreshWinSizeStep = 30
    p.adaptiveThreshConstant = 7.0
    p.minMarkerPerimeterRate = 0.015
    p.maxMarkerPerimeterRate = 4.0
    p.errorCorrectionRate = 0.6
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return p

def set_v4l2(gain, gamma, exp):
    subprocess.run([
        'v4l2-ctl', '-d', DEV, '--set-ctrl',
        f'gain={gain},gamma={gamma},exposure_time_absolute={exp},'
        'brightness=0,contrast=0,saturation=70,sharpness=1'
    ], capture_output=True)
    time.sleep(0.3)

def detect_raw(gray, params):
    """Detect on raw grayscale."""
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
    return set(ids.flatten()) if ids is not None else set()

def detect_clahe(gray, clip, tile, params):
    """Detect on CLAHE-enhanced grayscale."""
    clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(tile, tile))
    enhanced = clahe.apply(gray)
    corners, ids, _ = cv2.aruco.detectMarkers(enhanced, aruco_dict, parameters=params)
    return set(ids.flatten()) if ids is not None else set()

def test_strategy(cap, name, gain, gamma, detect_fn, frames=FRAMES):
    """Test across all exposures. Returns dict of results."""
    results = {}
    for exp in EXPOSURES:
        set_v4l2(gain, gamma, exp)
        for _ in range(4):
            cap.read()
        time.sleep(0.15)

        counts = {mid: 0 for mid in ALL_IDS}
        for _ in range(frames):
            ret, frame = cap.read()
            if not ret:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detected = detect_fn(gray)
            for mid in detected:
                if mid in counts:
                    counts[mid] += 1

        pcts = {mid: counts[mid] * 100 // frames for mid in ALL_IDS}
        robot_ok = pcts[ROBOT_ID] >= 80
        table_ok = all(pcts[tid] >= 80 for tid in TABLE_IDS)
        all_ok = robot_ok and table_ok
        results[exp] = (pcts, all_ok)

    # Print
    ok_count = sum(1 for _, aok in results.values() if aok)
    print(f'\n  {name}  [{ok_count}/9 exposures ok]')
    for exp in EXPOSURES:
        pcts, aok = results[exp]
        mark = '✓' if aok else ' '
        r = pcts[ROBOT_ID]
        t = ' '.join(f'{tid}={pcts[tid]:3d}%' for tid in sorted(TABLE_IDS))
        print(f'    exp={exp:3d}  robot={r:3d}%  {t}  {mark}')

    return ok_count

# Open camera
cap = cv2.VideoCapture(DEV, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
cap.set(cv2.CAP_PROP_FPS, 15)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

params = make_params()
rescue_params = make_rescue_params()

print('='*110)
print('  DUAL-DETECTION APPROACH: Testing strategies across 9 exposure levels')
print('='*110)

# Strategy A: Previous settings (raw + existing rescue)
def strat_raw_rescue_227_300(gray):
    found = detect_raw(gray, params)
    missing_table = TABLE_IDS - found
    if missing_table:
        rescue_found = detect_clahe(gray, 3.0, 16, rescue_params)
        found |= (rescue_found & TABLE_IDS)
    return found

test_strategy(cap, "A: g=227 γ=300 raw+rescue (previous)", 227, 300, strat_raw_rescue_227_300)

# Strategy B: Current (CLAHE all frames, hurts robot)
def strat_clahe_all_170_200(gray):
    return detect_clahe(gray, 3.0, 16, params)

test_strategy(cap, "B: g=170 γ=200 CLAHE(3,16) all (current)", 170, 200, strat_clahe_all_170_200)

# Strategy C: DUAL DETECT - raw main + CLAHE for table markers
def make_dual_fn(clahe_clip, clahe_tile):
    def fn(gray):
        # Main: raw grayscale (robot + blocks + some tables)
        found = detect_raw(gray, params)
        # Second pass: CLAHE-enhanced, add any table markers found
        missing_table = TABLE_IDS - found
        if missing_table:
            clahe_found = detect_clahe(gray, clahe_clip, clahe_tile, params)
            found |= (clahe_found & TABLE_IDS)
        return found
    return fn

test_strategy(cap, "C1: g=170 γ=200 DUAL(raw + CLAHE(3,16) table)", 170, 200,
              make_dual_fn(3.0, 16))

test_strategy(cap, "C2: g=200 γ=200 DUAL(raw + CLAHE(3,16) table)", 200, 200,
              make_dual_fn(3.0, 16))

test_strategy(cap, "C3: g=200 γ=250 DUAL(raw + CLAHE(3,16) table)", 200, 250,
              make_dual_fn(3.0, 16))

test_strategy(cap, "C4: g=227 γ=300 DUAL(raw + CLAHE(3,16) table)", 227, 300,
              make_dual_fn(3.0, 16))

# Strategy D: DUAL with rescue params on the CLAHE pass
def make_dual_rescue_fn(clahe_clip, clahe_tile):
    def fn(gray):
        found = detect_raw(gray, params)
        missing_table = TABLE_IDS - found
        if missing_table:
            # Use relaxed rescue params on CLAHE-enhanced image
            clahe_found = detect_clahe(gray, clahe_clip, clahe_tile, rescue_params)
            found |= (clahe_found & TABLE_IDS)
        return found
    return fn

test_strategy(cap, "D1: g=170 γ=200 DUAL(raw + CLAHE(3,16)+rescue)", 170, 200,
              make_dual_rescue_fn(3.0, 16))

test_strategy(cap, "D2: g=200 γ=200 DUAL(raw + CLAHE(3,16)+rescue)", 200, 200,
              make_dual_rescue_fn(3.0, 16))

test_strategy(cap, "D3: g=200 γ=250 DUAL(raw + CLAHE(3,16)+rescue)", 200, 250,
              make_dual_rescue_fn(3.0, 16))

# Strategy E: DUAL with MAIN params on CLAHE pass (not rescue params)
def make_dual_main_fn(clahe_clip, clahe_tile):
    """Like C but uses main detector params (not rescue) on CLAHE image for ALL markers."""
    def fn(gray):
        found_raw = detect_raw(gray, params)
        found_clahe = detect_clahe(gray, clahe_clip, clahe_tile, params)
        # Take robot/blocks from raw, table markers from best of both
        found = set()
        # Robot + blocks: prefer raw (CLAHE hurts these)
        for mid in (found_raw | found_clahe):
            if mid == ROBOT_ID or mid in BLOCK_IDS:
                if mid in found_raw:
                    found.add(mid)
            elif mid in TABLE_IDS:
                found.add(mid)
            else:
                if mid in found_raw:
                    found.add(mid)
        return found
    return fn

test_strategy(cap, "E1: g=170 γ=200 SMART(raw→robot, CLAHE→table)", 170, 200,
              make_dual_main_fn(3.0, 16))

test_strategy(cap, "E2: g=200 γ=200 SMART(raw→robot, CLAHE→table)", 200, 200,
              make_dual_main_fn(3.0, 16))

test_strategy(cap, "E3: g=200 γ=250 SMART(raw→robot, CLAHE→table)", 200, 250,
              make_dual_main_fn(3.0, 16))

# Restore
set_v4l2(170, 200, 200)
cap.release()
print('\n' + '='*110)
print('  Done. Camera restored.')
