import cv2
import numpy as np
import glob

# chessboard parameters
chessboard_size = (9,6)   # inner corners
square_size = 0.024       # meters (24mm square example)

images = glob.glob("calibration_images/*.png")

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
fs = cv2.FileStorage("camera_calibration/onboard_cam_1080p.yml", cv2.FILE_STORAGE_WRITE)
fs.write("camera_matrix", camera_matrix)
fs.write("distortion_coefficients", dist_coeffs)
fs.release()

print("\nCalibration saved to camera_calibration/onboard_cam_1080p.yml")