#!/usr/bin/env python3
"""Diagnose robot marker (ID 1) detection under various CLAHE settings.
Tests the current settings plus alternatives to find the sweet spot for ALL markers."""

import subprocess, time, cv2
import numpy as np

DEV = '/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera_01.00.00-video-index0'
TABLE_IDS = {20, 21, 22, 23}
ROBOT_ID = 1
BLOCK_IDS = {36, 47}
FRAMES = 15

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

def set_v4l2(gain, gamma):
    subprocess.run([
        'v4l2-ctl', '-d', DEV, '--set-ctrl',
        f'gain={gain},gamma={gamma},exposure_time_absolute=200,'
        'brightness=0,contrast=0,saturation=70,sharpness=1'
    ], capture_output=True)
    time.sleep(0.3)

def test_strategy(cap, name, gain, gamma, clahe_clip, clahe_tile, frames=FRAMES):
    set_v4l2(gain, gamma)
    # flush
    for _ in range(5):
        cap.read()
    time.sleep(0.2)

    params = make_params()
    counts = {}

    for _ in range(frames):
        ret, frame = cap.read()
        if not ret:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if clahe_clip > 0:
            clahe = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=(clahe_tile, clahe_tile))
            gray = clahe.apply(gray)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
        if ids is not None:
            for mid in ids.flatten():
                counts[mid] = counts.get(mid, 0) + 1

    # Print results
    robot_pct = counts.get(ROBOT_ID, 0) * 100 // frames
    table_pcts = {tid: counts.get(tid, 0) * 100 // frames for tid in sorted(TABLE_IDS)}
    block_pcts = {bid: counts.get(bid, 0) * 100 // frames for bid in sorted(BLOCK_IDS)}

    table_str = ' '.join(f'{tid}={table_pcts[tid]:3d}%' for tid in sorted(TABLE_IDS))
    block_str = ' '.join(f'{bid}={block_pcts[bid]:3d}%' for bid in sorted(BLOCK_IDS))
    all_ok = robot_pct >= 80 and all(v >= 80 for v in table_pcts.values())
    mark = '✓' if all_ok else ' '

    print(f'  {name:55s}  robot={robot_pct:3d}%  {table_str}  blocks: {block_str}  {mark}')
    return robot_pct, table_pcts, all_ok

# Open camera
cap = cv2.VideoCapture(DEV, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
cap.set(cv2.CAP_PROP_FPS, 15)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

print('='*130)
print('  ROBOT MARKER DIAGNOSTIC: Finding settings that work for ALL marker types')
print('='*130)

strategies = [
    # (name, gain, gamma, clahe_clip, clahe_tile)
    # Current settings
    ("gain=170 gamma=200 CLAHE(3.0, 16)  [CURRENT]",       170, 200, 3.0, 16),
    # Previous settings (known good for robot)
    ("gain=227 gamma=300 NO CLAHE  [PREVIOUS]",             227, 300, 0, 0),

    # Lower CLAHE intensity
    ("gain=170 gamma=200 CLAHE(2.0, 16)",                   170, 200, 2.0, 16),
    ("gain=170 gamma=200 CLAHE(1.5, 16)",                   170, 200, 1.5, 16),
    ("gain=170 gamma=200 CLAHE(3.0, 8)",                    170, 200, 3.0, 8),
    ("gain=170 gamma=200 CLAHE(2.0, 8)",                    170, 200, 2.0, 8),

    # Slightly higher gain to help robot marker
    ("gain=200 gamma=200 CLAHE(3.0, 16)",                   200, 200, 3.0, 16),
    ("gain=200 gamma=200 CLAHE(2.0, 16)",                   200, 200, 2.0, 16),
    ("gain=200 gamma=200 CLAHE(2.5, 16)",                   200, 200, 2.5, 16),

    # More gamma to help contrast for small markers
    ("gain=170 gamma=250 CLAHE(3.0, 16)",                   170, 250, 3.0, 16),
    ("gain=170 gamma=250 CLAHE(2.0, 16)",                   170, 250, 2.0, 16),
    ("gain=200 gamma=250 CLAHE(3.0, 16)",                   200, 250, 3.0, 16),
    ("gain=200 gamma=250 CLAHE(2.0, 16)",                   200, 250, 2.0, 16),

    # Compromise: moderate everything  
    ("gain=190 gamma=200 CLAHE(2.5, 16)",                   190, 200, 2.5, 16),
    ("gain=190 gamma=220 CLAHE(2.5, 16)",                   190, 220, 2.5, 16),
    ("gain=180 gamma=200 CLAHE(3.0, 16)",                   180, 200, 3.0, 16),
    ("gain=180 gamma=220 CLAHE(3.0, 16)",                   180, 220, 3.0, 16),
    ("gain=180 gamma=250 CLAHE(3.0, 16)",                   180, 250, 3.0, 16),
]

for name, gain, gamma, clip, tile in strategies:
    test_strategy(cap, name, gain, gamma, clip, tile)

# Restore
set_v4l2(170, 200)
cap.release()
print('='*130)
print('  Done. Camera restored to gain=170 gamma=200.')
