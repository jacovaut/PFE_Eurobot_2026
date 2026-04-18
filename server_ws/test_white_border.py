#!/usr/bin/env python3
"""
Quick test: does adding a synthetic white border around ArUco markers improve detection?

Strategy:
  Pass 1 — normal detection on the raw frame.
  Pass 2 — for each marker found in pass 1, paint a white border (N px) around
            its bounding box, then re-run detection to see if *additional* markers
            appear (the border of one marker can help neighbours).
  Pass 3 — apply a global morphological "close" (dilate then erode) on the
            thresholded image to grow white regions, then detect.

Press 'q' to quit.
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import sys

# --- CONFIG ---
DEVICE = "/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera_01.00.00-video-index0"
WIDTH, HEIGHT, FPS = 3840, 2160, 15
BORDER_PX = 15          # white border thickness to paint (in pixels)
MORPH_KERNEL_SIZE = 7   # for the morphological approach
# ---------------

def make_detector():
    params = aruco.DetectorParameters_create()
    params.perspectiveRemovePixelPerCell = 10
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 101
    params.adaptiveThreshWinSizeStep = 10
    params.adaptiveThreshConstant = 7
    params.minMarkerPerimeterRate = 0.005
    params.maxMarkerPerimeterRate = 4.0
    params.errorCorrectionRate = 0.45
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    return params

def detect(gray, dictionary, params):
    corners, ids, rejected = aruco.detectMarkers(gray, dictionary, parameters=params)
    return corners, ids, rejected

def add_white_margin_around_rejected(gray, rejected, margin_px,
                                     min_perim=200, max_perim=800,
                                     min_aspect=0.6):
    """
    Paint white margin around REJECTED candidates that look like plausible
    table markers based on:
      - perimeter within [min_perim, max_perim] pixels
      - aspect ratio (short/long side) >= min_aspect (i.e. roughly square)
    """
    out = gray.copy()
    for r in rejected:
        pts = r[0].astype(int)

        # --- Filter: perimeter ---
        perim = cv2.arcLength(pts.reshape(-1, 1, 2), closed=True)
        if perim < min_perim or perim > max_perim:
            continue

        # --- Filter: squareness ---
        x_min, y_min = pts.min(axis=0)
        x_max, y_max = pts.max(axis=0)
        w = max(x_max - x_min, 1)
        h = max(y_max - y_min, 1)
        aspect = min(w, h) / max(w, h)
        if aspect < min_aspect:
            continue

        # outer rectangle (expanded by margin)
        ox1 = max(0, x_min - margin_px)
        oy1 = max(0, y_min - margin_px)
        ox2 = min(gray.shape[1] - 1, x_max + margin_px)
        oy2 = min(gray.shape[0] - 1, y_max + margin_px)

        # fill the margin ring white (outer rect minus inner rect)
        cv2.rectangle(out, (ox1, oy1), (ox2, oy2), 255, -1)
        # restore original pixels inside the marker area
        out[y_min:y_max, x_min:x_max] = gray[y_min:y_max, x_min:x_max]

    return out

def morph_enhance(gray, ksize):
    """CLAHE + relaxed detector params (mimics global_localization rescue pass)."""
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(16, 16))
    return clahe.apply(gray)

def make_rescue_detector():
    """Relaxed params matching global_localization_node rescue pass."""
    params = aruco.DetectorParameters_create()
    params.perspectiveRemovePixelPerCell = 10
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 181
    params.adaptiveThreshWinSizeStep = 30
    params.adaptiveThreshConstant = 7
    params.minMarkerPerimeterRate = 0.015
    params.maxMarkerPerimeterRate = 4.0
    params.errorCorrectionRate = 0.6
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    return params

def draw_results(img, corners, ids, label):
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if len(img.shape) == 2 else img.copy()
    if ids is not None:
        aruco.drawDetectedMarkers(vis, corners, ids)
    n = 0 if ids is None else len(ids)
    id_list = [] if ids is None else sorted(ids.flatten().tolist())
    cv2.putText(vis, f"{label}: {n} ids={id_list}", (30, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 4)
    return vis

def main():
    cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"Cannot open {DEVICE}")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    params = make_detector()

    cv2.namedWindow("comparison", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("comparison", 1800, 600)

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # --- Pass 1: normal ---
        c1, id1, rejected1 = detect(gray, dictionary, params)

        # --- Pass 2: white margin around REJECTED candidates, then re-detect ---
        bordered = add_white_margin_around_rejected(gray, rejected1, BORDER_PX)
        c2, id2, _ = detect(bordered, dictionary, params)

        # --- Pass 3: CLAHE + white margin around rejected, then re-detect ---
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(16, 16))
        enhanced = clahe.apply(gray)
        _, _, rejected_clahe = detect(enhanced, dictionary, params)
        clahe_bordered = add_white_margin_around_rejected(enhanced, rejected_clahe, BORDER_PX)
        c3, id3, _ = detect(clahe_bordered, dictionary, params)

        # Draw
        v1 = draw_results(gray, c1, id1, "raw")
        v2 = draw_results(bordered, c2, id2, "white margin")
        v3 = draw_results(clahe_bordered, c3, id3, "CLAHE+margin")

        # Resize for display
        h = 700
        def rs(img):
            scale = h / img.shape[0]
            return cv2.resize(img, (int(img.shape[1] * scale), h))

        combined = np.hstack([rs(v1), rs(v2), rs(v3)])
        cv2.imshow("comparison", combined)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
