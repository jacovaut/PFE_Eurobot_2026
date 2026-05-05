#!/usr/bin/env python3
"""
Onboard camera calibration tool — headless / SSH-friendly.

Phase 1 — Manual capture:
  Connects to the Pi camera's MJPEG TCP stream (rpicam-vid -> port 8888).
  Serves a live MJPEG preview on http://<pi-ip>:8090/ so you can watch
  from a browser while in SSH.
  Commands typed in the terminal:
    c   — capture current frame (checks for chessboard first)
    f   — force-save current frame (no chessboard check)
    cal — run calibration on captured frames so far
    q   — quit (and run calibration if frames were captured)

Phase 2 — Calibration:
  Runs OpenCV chessboard calibration on captured images and saves a YAML
  file readable by OpenCV FileStorage.

Usage:
  1. Start the camera stream:
       rpicam-vid -t 0 -n --listen --codec mjpeg \\
           --width 2328 --height 1748 --framerate 30 --flush \\
           -o tcp://0.0.0.0:8888 &
  2. Run this script:
       python3 calibrate_onboard_cam.py
  3. Open http://<pi-ip>:8090/ in a browser to see the live feed.
  4. Type commands in the terminal to capture / calibrate.

Adjust the constants below to match your printed chessboard.
"""

import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np

# ── Chessboard parameters ─────────────────────────────────────────────────────
# Number of *inner* corners (cols, rows)
CHESSBOARD_SIZE = (7, 5)   # Adjust to match your printed chessboard

# Physical size of one square in metres (measure your printed board!)
SQUARE_SIZE_M = 0.025   # 25 mm

# ── Stream ────────────────────────────────────────────────────────────────────
STREAM_HOST = "127.0.0.1"
STREAM_PORT = 8888

# ── Preview HTTP server ───────────────────────────────────────────────────────
PREVIEW_HOST = "0.0.0.0"
PREVIEW_PORT = 8090          # open http://<pi-ip>:8090/ in your browser

# ── Output ────────────────────────────────────────────────────────────────────
script_dir = os.path.dirname(os.path.abspath(__file__))
IMAGES_DIR  = os.path.join(script_dir, "calib_captures")
OUTPUT_YAML = os.path.join(script_dir, "onboard_cam_calibration.yml")

# ── Subpix refinement criteria ────────────────────────────────────────────────
SUBPIX_CRITERIA = (
    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
    30,
    0.001,
)


# =============================================================================
# Shared latest JPEG bytes for the preview server
# =============================================================================
_preview_lock  = threading.Lock()
_preview_frame: bytes | None = None


def _set_preview_raw(jpg_bytes: bytes) -> None:
    """Store raw JPEG bytes directly — no re-encode, maximum speed."""
    with _preview_lock:
        global _preview_frame
        _preview_frame = jpg_bytes


# =============================================================================
# HTTP MJPEG preview server
# =============================================================================
_BOUNDARY = b"calibboundary"


class _PreviewHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # suppress per-request noise

    def do_GET(self):
        self.send_response(200)
        self.send_header(
            "Content-Type",
            f"multipart/x-mixed-replace; boundary={_BOUNDARY.decode()}"
        )
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "close")
        self.end_headers()
        try:
            while True:
                with _preview_lock:
                    frame = _preview_frame
                if frame is None:
                    time.sleep(0.05)
                    continue
                header = (
                    f"--{_BOUNDARY.decode()}\r\n"
                    f"Content-Type: image/jpeg\r\n"
                    f"Content-Length: {len(frame)}\r\n\r\n"
                ).encode()
                self.wfile.write(header + frame + b"\r\n")
                self.wfile.flush()
                time.sleep(0.05)   # ~20 fps
        except (BrokenPipeError, ConnectionResetError):
            pass


def _start_preview_server() -> None:
    srv = HTTPServer((PREVIEW_HOST, PREVIEW_PORT), _PreviewHandler)
    print(f"[PREVIEW] Live feed at http://0.0.0.0:{PREVIEW_PORT}/  "
          f"(open http://<pi-ip>:{PREVIEW_PORT}/ in your browser)")
    threading.Thread(target=srv.serve_forever, daemon=True).start()


# =============================================================================
# TCP MJPEG client (same as block_publisher.py)
# =============================================================================
class MjpegTcpClient:
    def __init__(self, host: str, port: int):
        print(f"[INFO] Connecting to {host}:{port} ...", end=" ", flush=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        while True:
            try:
                self.sock.connect((host, port))
                break
            except (ConnectionRefusedError, OSError):
                print("waiting...", end=" ", flush=True)
                time.sleep(1.0)
        print("connected.")
        self.sock.settimeout(2.0)
        self.buffer = b""

    def read(self):
        """Returns (ok, frame, raw_jpg_bytes)."""
        try:
            while True:
                chunk = self.sock.recv(65536)
                if not chunk:
                    return False, None, None
                self.buffer += chunk
                start = self.buffer.find(b"\xff\xd8")
                end   = self.buffer.find(b"\xff\xd9")
                if start != -1 and end != -1 and end > start:
                    jpg  = self.buffer[start : end + 2]
                    self.buffer = self.buffer[end + 2 :]
                    arr  = np.frombuffer(jpg, dtype=np.uint8)
                    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    if frame is not None:
                        return True, frame, jpg
                if len(self.buffer) > 2_000_000:
                    self.buffer = self.buffer[-500_000:]
        except socket.timeout:
            return False, None, None

    def release(self):
        self.sock.close()


# =============================================================================
# Frame grabber thread — keeps _preview_frame fresh and exposes latest frame
# =============================================================================
class FrameGrabber:
    """Continuously reads frames from the TCP stream in a background thread."""

    def __init__(self, client: MjpegTcpClient):
        self._client = client
        self._lock   = threading.Lock()
        self._frame: np.ndarray | None = None
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            ret, frame, raw_jpg = self._client.read()
            if not ret or frame is None:
                continue
            with self._lock:
                self._frame = frame
            _set_preview_raw(raw_jpg)

    def get(self) -> np.ndarray | None:
        with self._lock:
            return self._frame


# =============================================================================
# Phase 1: terminal-driven capture
# =============================================================================
def capture_phase() -> list[np.ndarray]:
    os.makedirs(IMAGES_DIR, exist_ok=True)

    _start_preview_server()

    cap     = MjpegTcpClient(STREAM_HOST, STREAM_PORT)
    grabber = FrameGrabber(cap)

    saved_frames: list[np.ndarray] = []
    img_idx = 0

    print("\n─── Capture phase ──────────────────────────────────────────────")
    print("  Commands:")
    print("    c   — capture current frame")
    print("    cal — run calibration now")
    print("    q   — quit (calibrates if frames were captured)")
    print("────────────────────────────────────────────────────────────────\n")

    while True:
        try:
            cmd = input("> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if cmd == "q":
            break

        if cmd == "cal":
            if saved_frames:
                cap.release()
                return saved_frames
            else:
                print("[WARN] No frames captured yet.")
            continue

        if cmd == "c":
            frame = grabber.get()
            if frame is None:
                print("[WARN] No frame available yet — is the stream running?")
                continue

            fname = os.path.join(IMAGES_DIR, f"calib_{img_idx:03d}.png")
            cv2.imwrite(fname, frame)
            saved_frames.append(frame)
            img_idx += 1
            print(f"  [SAVED {len(saved_frames):3d}] {fname}")
            continue

        if cmd:
            print("  Unknown command. Use: c / cal / q")

    cap.release()
    print(f"[INFO] Capture complete — {len(saved_frames)} frames saved to {IMAGES_DIR}")
    return saved_frames


# =============================================================================
# Phase 2: calibration
# =============================================================================
def calibrate_phase(frames: list[np.ndarray]) -> None:
    if len(frames) < 5:
        print(f"[ERROR] Need at least 5 frames, got {len(frames)}. Aborting.")
        return

    objp = np.zeros(
        (CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32
    )
    objp[:, :2] = (
        np.mgrid[0 : CHESSBOARD_SIZE[0], 0 : CHESSBOARD_SIZE[1]]
        .T.reshape(-1, 2)
    )
    objp *= SQUARE_SIZE_M

    obj_points: list[np.ndarray] = []
    img_points: list[np.ndarray] = []
    img_size   = None

    print("\n[INFO] Detecting chessboard corners in captured frames...")
    good = 0
    for i, frame in enumerate(frames):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if img_size is None:
            img_size = (gray.shape[1], gray.shape[0])   # (width, height)

        found, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)
        if not found:
            print(f"  frame {i:03d} : SKIP (chessboard not found)")
            continue

        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1), SUBPIX_CRITERIA
        )
        obj_points.append(objp)
        img_points.append(corners2)
        good += 1
        print(f"  frame {i:03d} : OK")

    print(f"\n[INFO] {good}/{len(frames)} frames usable for calibration")

    if good < 5:
        print("[ERROR] Not enough usable frames. Aborting.")
        return

    print("[INFO] Running calibration (may take a moment)...")
    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, img_size, None, None
    )

    print(f"\n  RMS reprojection error : {rms:.4f} px")
    print(f"  Camera matrix :\n{camera_matrix}")
    print(f"  Distortion coefficients :\n{dist_coeffs.ravel()}")

    if rms > 1.0:
        print(
            "\n  [WARN] RMS > 1.0 px — consider recapturing with more "
            "diverse board positions/angles."
        )

    # Compute per-image reprojection errors for diagnostics
    errors = []
    for objp_i, imgp_i, rvec, tvec in zip(
        obj_points, img_points, rvecs, tvecs
    ):
        proj, _ = cv2.projectPoints(
            objp_i, rvec, tvec, camera_matrix, dist_coeffs
        )
        err = cv2.norm(imgp_i, proj, cv2.NORM_L2) / len(proj)
        errors.append(err)

    print(
        f"\n  Per-image error — min: {min(errors):.4f}  "
        f"max: {max(errors):.4f}  "
        f"mean: {np.mean(errors):.4f} px"
    )

    # ── Save ────────────────────────────────────────────────────────────────
    fs = cv2.FileStorage(OUTPUT_YAML, cv2.FILE_STORAGE_WRITE)
    fs.write("image_width",              img_size[0])
    fs.write("image_height",             img_size[1])
    fs.write("chessboard_cols",          CHESSBOARD_SIZE[0])
    fs.write("chessboard_rows",          CHESSBOARD_SIZE[1])
    fs.write("square_size_m",            SQUARE_SIZE_M)
    fs.write("rms_reprojection_error",   rms)
    fs.write("camera_matrix",            camera_matrix)
    fs.write("distortion_coefficients",  dist_coeffs)
    fs.release()

    print(f"\n[OK] Calibration saved to: {OUTPUT_YAML}")


# =============================================================================
# Entry point
# =============================================================================
def main():
    frames = capture_phase()
    calibrate_phase(frames)


if __name__ == "__main__":
    main()
