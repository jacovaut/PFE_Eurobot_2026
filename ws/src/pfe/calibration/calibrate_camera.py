import cv2
import numpy as np
import glob
import os

# chessboard parameters
chessboard_size = (6,8)   # inner corners
square_size = 0.07        # meters (70mm square example)

# paths
script_dir = os.path.dirname(os.path.abspath(__file__))
image_path = os.path.join(script_dir, "calibration_images", "*.png")
images = glob.glob(image_path)

# create output folder if missing
output_dir = os.path.join(script_dir, "..", "camera_calibration")
os.makedirs(output_dir, exist_ok=True)

objpoints = []
imgpoints = []

objp = np.zeros((chessboard_size[0]*chessboard_size[1],3), np.float32)
objp[:,:2] = np.mgrid[0:chessboard_size[0],0:chessboard_size[1]].T.reshape(-1,2)
objp *= square_size

for fname in images:

    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(
            gray,
            corners,
            (11,11),
            (-1,-1),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,30,0.001)
        )

        imgpoints.append(corners2)

        cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
        cv2.imshow('corners', img)
        cv2.waitKey(200)

cv2.destroyAllWindows()

# run calibration
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints,
    imgpoints,
    gray.shape[::-1],
    None,
    None
)

print("\nCamera Matrix:\n", camera_matrix)
print("\nDistortion:\n", dist_coeffs)

# save calibration
output_file = os.path.join(output_dir, "onboard_cam_1080p.yml")

fs = cv2.FileStorage(output_file, cv2.FILE_STORAGE_WRITE)
fs.write("camera_matrix", camera_matrix)
fs.write("distortion_coefficients", dist_coeffs)
fs.release()

print("\nCalibration saved to:", output_file)