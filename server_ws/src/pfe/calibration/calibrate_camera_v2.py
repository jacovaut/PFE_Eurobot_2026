import cv2
import numpy as np
import glob
import os
import sys

# ---------------------------------------------------------------------------
# Chessboard parameters — adjust to match your printed board
# ---------------------------------------------------------------------------
chessboard_size = (6, 8)   # number of INNER corners (cols, rows)
square_size     = 0.07     # meters — measure the physical square side length

# ---------------------------------------------------------------------------
# Output filename — change to something specific for the camera you are calibrating
# e.g. "3840_2160_GlobalCam2.yml"
# ---------------------------------------------------------------------------
output_name = "3840_2160_GlobalCam_new.yml"

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
script_dir = os.path.dirname(os.path.abspath(__file__))
image_path = os.path.join(script_dir, "calibration_images", "*.png")
images = sorted(glob.glob(image_path))

if not images:
    print(f"ERROR: No .png images found in {os.path.join(script_dir, 'calibration_images')}")
    sys.exit(1)

output_dir = os.path.join(script_dir, "..", "camera_calibration")
os.makedirs(output_dir, exist_ok=True)

# ---------------------------------------------------------------------------
# Collect object / image point pairs
# ---------------------------------------------------------------------------
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []
imgpoints = []
img_shape  = None

subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

for fname in images:
    img  = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_shape = gray.shape[::-1]  # (width, height)

    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), subpix_criteria)
        objpoints.append(objp)
        imgpoints.append(corners2)

        cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
        cv2.imshow("corners", img)
        cv2.waitKey(200)
    else:
        print(f"  [SKIP] corners not found: {os.path.basename(fname)}")

cv2.destroyAllWindows()

if len(objpoints) < 15:
    print(f"\nWARNING: Only {len(objpoints)} usable images. "
          "Use at least 20-30 for a reliable calibration.")

print(f"\nUsing {len(objpoints)} / {len(images)} images for calibration.")

# ---------------------------------------------------------------------------
# Calibrate with the rational (12-coefficient) distortion model.
#
# CALIB_RATIONAL_MODEL adds k4, k5, k6 to the standard k1,k2,p1,p2,k3 model
# (5 coefficients) and enables the rational denominator part,
# giving 12 coefficients total — same model as 3840_2160_ELM12MP.yml.
#
# Do NOT use CALIB_FIX_PRINCIPAL_POINT; let cx/cy float freely.
# ---------------------------------------------------------------------------
flags = cv2.CALIB_RATIONAL_MODEL

rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, img_shape,
    None, None,
    flags=flags
)

# Per-image reprojection error
total_err = 0.0
for i in range(len(objpoints)):
    proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i],
                                 camera_matrix, dist_coeffs)
    err = cv2.norm(imgpoints[i], proj, cv2.NORM_L2) / len(proj)
    total_err += err
avg_reprojection_error = total_err / len(objpoints)

print(f"\nRMS reprojection error : {rms:.4f} px  (target < 1.0)")
print(f"Avg per-point error    : {avg_reprojection_error:.4f} px")
print(f"\nCamera Matrix:\n{camera_matrix}")
print(f"\nDistortion ({dist_coeffs.shape[1]} coefficients):\n{dist_coeffs}")

if rms > 1.5:
    print("\nWARNING: RMS > 1.5 px — calibration may be poor. "
          "Try: more images (30+), cover all corners of the frame, "
          "tilt the board at various angles, ensure the board is flat.")

# ---------------------------------------------------------------------------
# Save in the same format as 3840_2160_ELM12MP.yml
# ---------------------------------------------------------------------------
output_file = os.path.join(output_dir, output_name)

fs = cv2.FileStorage(output_file, cv2.FILE_STORAGE_WRITE)
fs.write("image_width",  img_shape[0])
fs.write("image_height", img_shape[1])
fs.write("camera_matrix", camera_matrix)
fs.write("distortion_coefficients", dist_coeffs)
fs.write("rms", rms)
fs.write("avg_reprojection_error_px", avg_reprojection_error)
fs.release()

print(f"\nCalibration saved to: {output_file}")
