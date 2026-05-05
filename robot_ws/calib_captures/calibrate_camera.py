import cv2
import numpy as np
import glob
import os

# ── ChArUco board parameters ─────────────────────────────────────────────────
# Must match the printed board exactly.
SQUARES_X   = 7        # number of squares horizontally (columns)
SQUARES_Y   = 5        # number of squares vertically (rows)
SQUARE_SIZE = 0.040    # physical size of one square in metres — MEASURE YOUR BOARD
MARKER_SIZE = 0.030    # physical size of the ArUco marker inside each square

ARUCO_DICT  = cv2.aruco.DICT_4X4_50

OUTPUT_NAME = "onboard_charuco_calibration.yml"

# ── Paths ─────────────────────────────────────────────────────────────────────
script_dir  = os.path.dirname(os.path.abspath(__file__))
image_path  = os.path.join(script_dir, "*.png")
images      = sorted(glob.glob(image_path))

output_dir  = os.path.join(script_dir, "..", "calibration_files")
os.makedirs(output_dir, exist_ok=True)

if not images:
    print("[ERROR] No PNG images found in", script_dir)
    raise SystemExit(1)

print(f"[INFO] Found {len(images)} images")

# ── Build ChArUco board ───────────────────────────────────────────────────────
dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)

try:
    board = cv2.aruco.CharucoBoard_create(
        SQUARES_X, SQUARES_Y, SQUARE_SIZE, MARKER_SIZE, dictionary
    )
    detector_params = cv2.aruco.DetectorParameters_create()
except AttributeError:
    # OpenCV 4.7+ API
    board = cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y), SQUARE_SIZE, MARKER_SIZE, dictionary
    )
    detector_params = cv2.aruco.DetectorParameters()

# ── Detect corners in every image ────────────────────────────────────────────
all_charuco_corners = []
all_charuco_ids     = []
img_size            = None

for fname in images:
    img  = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if img_size is None:
        img_size = (gray.shape[1], gray.shape[0])

    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
        gray, dictionary, parameters=detector_params
    )

    if marker_ids is None or len(marker_ids) < 4:
        print(f"  {os.path.basename(fname)} : SKIP (only {len(marker_ids) if marker_ids is not None else 0} markers)")
        continue

    try:
        ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            marker_corners, marker_ids, gray, board
        )
    except cv2.error:
        print(f"  {os.path.basename(fname)} : SKIP (interpolation failed)")
        continue

    if ret < 4:
        print(f"  {os.path.basename(fname)} : SKIP (only {ret} ChArUco corners)")
        continue

    all_charuco_corners.append(charuco_corners)
    all_charuco_ids.append(charuco_ids)
    print(f"  {os.path.basename(fname)} : OK  ({ret} corners, {len(marker_ids)} markers)")

print(f"\n[INFO] {len(all_charuco_corners)}/{len(images)} frames usable")

if len(all_charuco_corners) < 5:
    print("[ERROR] Need at least 5 usable frames. Recapture with the board more visible.")
    raise SystemExit(1)

# ── Calibrate ─────────────────────────────────────────────────────────────────
print("[INFO] Running ChArUco calibration...")
try:
    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
        all_charuco_corners, all_charuco_ids, board, img_size, None, None
    )
except cv2.error as e:
    print(f"[ERROR] Calibration failed: {e}")
    raise SystemExit(1)

print(f"\n  RMS reprojection error : {rms:.4f} px")
print(f"  Camera matrix :\n{camera_matrix}")
print(f"  Distortion coefficients :\n{dist_coeffs.ravel()}")

if rms > 1.0:
    print("\n  [WARN] RMS > 1.0 px — consider recapturing with more diverse board poses.")

# ── Save ──────────────────────────────────────────────────────────────────────
output_file = os.path.join(output_dir, OUTPUT_NAME)
fs = cv2.FileStorage(output_file, cv2.FILE_STORAGE_WRITE)
fs.write("image_width",             img_size[0])
fs.write("image_height",            img_size[1])
fs.write("squares_x",               SQUARES_X)
fs.write("squares_y",               SQUARES_Y)
fs.write("square_size_m",           SQUARE_SIZE)
fs.write("marker_size_m",           MARKER_SIZE)
fs.write("rms_reprojection_error",  rms)
fs.write("camera_matrix",           camera_matrix)
fs.write("distortion_coefficients", dist_coeffs)
fs.release()

print(f"\n[OK] Calibration saved to: {output_file}")
