import argparse
import cv2
import numpy as np
from pathlib import Path
import os

def compute_reprojection_error(objpoints, imgpoints, rvecs, tvecs, K, dist):
    total_err = 0.0
    total_points = 0
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
        proj = proj.reshape(-1, 2)
        pts = imgpoints[i].reshape(-1, 2)
        err = cv2.norm(pts, proj, cv2.NORM_L2)
        total_err += err * err
        total_points += len(objpoints[i])
    return np.sqrt(total_err / max(total_points, 1))

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_images = os.path.join(script_dir, "calibration_images")
    default_out = os.path.join(script_dir, "..", "camera_calibration", "calibration_mec.yml")

    ap = argparse.ArgumentParser()
    ap.add_argument("--images", type=str, default=default_images, help="Path to folder with images (default: calibration_images/)")
    ap.add_argument("--dict", type=str, default="DICT_4X4_250")
    ap.add_argument("--squares_x", type=int, default=9)
    ap.add_argument("--squares_y", type=int, default=7)
    ap.add_argument("--square_length_m", type=float, default=0.065)
    ap.add_argument("--marker_length_m", type=float, default=0.050)
    ap.add_argument("--min_corners", type=int, default=12)
    ap.add_argument("--out", type=str, default=default_out)
    args = ap.parse_args()

    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dict))

    # OpenCV >= 4.7 uses new API; older versions use legacy functions
    new_api = hasattr(cv2.aruco, 'ArucoDetector')

    if new_api:
        board = cv2.aruco.CharucoBoard(
            (args.squares_x, args.squares_y),
            args.square_length_m,
            args.marker_length_m,
            dictionary
        )
        detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    else:
        board = cv2.aruco.CharucoBoard_create(
            args.squares_x,
            args.squares_y,
            args.square_length_m,
            args.marker_length_m,
            dictionary
        )
        aruco_params = cv2.aruco.DetectorParameters_create()

    # Get all images
    image_paths = sorted(Path(args.images).glob("*.jpg")) + \
                  sorted(Path(args.images).glob("*.png")) + \
                  sorted(Path(args.images).glob("*.JPG"))
    
    if not image_paths:
        print(f"No images found in {args.images}")
        return

    all_objpoints = []
    all_imgpoints = []
    image_size = None

    print(f"Found {len(image_paths)} images")
    print("Controls: k=keep, s=skip, q=quit")

    for idx, img_path in enumerate(image_paths):
        frame = cv2.imread(str(img_path))
        if frame is None:
            print(f"Failed to load: {img_path}")
            continue

        if image_size is None:
            image_size = (frame.shape[1], frame.shape[0])

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if new_api:
            corners, ids, _ = detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=aruco_params)

        vis = frame.copy()
        detected_corners = 0

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)
            retval, cc, ci = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
            
            if retval and retval >= args.min_corners:
                detected_corners = retval
                cv2.aruco.drawDetectedCornersCharuco(vis, cc, ci)

        cv2.putText(vis, f"Image {idx+1}/{len(image_paths)}: {img_path.name}", 
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(vis, f"Corners: {detected_corners}", 
                    (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if detected_corners >= args.min_corners else (0, 0, 255), 2)

        cv2.imshow("Calibration", vis)
        
        while True:
            key = cv2.waitKey(0) & 0xFF
            
            if key == ord('k'):
                if detected_corners >= args.min_corners:
                    if new_api:
                        obj_pts, img_pts = board.matchImagePoints(cc, ci)
                    else:
                        obj_pts = board.chessboardCorners[ci.flatten()]
                        img_pts = cc
                    all_objpoints.append(obj_pts.astype(np.float32))
                    all_imgpoints.append(img_pts.astype(np.float32))
                    print(f"✓ Kept: {len(all_objpoints)} total")
                else:
                    print(f"✗ Not enough corners ({detected_corners} < {args.min_corners})")
                break
            elif key == ord('s'):
                print("Skipped")
                break
            elif key == ord('q'):
                cv2.destroyAllWindows()
                if len(all_objpoints) < 8:
                    print("Not enough images kept. Need at least 8.")
                    return
                goto_calibration = True
                break
        
        if 'goto_calibration' in locals():
            break

    cv2.destroyAllWindows()

    if len(all_objpoints) < 8:
        print(f"Only {len(all_objpoints)} images kept. Need at least 8.")
        return

    print(f"\nCalibrating with {len(all_objpoints)} images...")
    # ADD CALIBRATION FLAGS for better distortion modeling
    flags = (
        cv2.CALIB_RATIONAL_MODEL |      # Adds k4, k5, k6 (handles strong radial distortion)
        cv2.CALIB_THIN_PRISM_MODEL      # Adds s1-s4 (handles asymmetric distortion)
    )
    
    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        all_objpoints, all_imgpoints, image_size, None, None,
        flags=flags  # <-- ADD THIS
    )

    avg_err = compute_reprojection_error(all_objpoints, all_imgpoints, rvecs, tvecs, K, dist)

    print(f"RMS: {ret:.3f}")
    print(f"Avg reprojection error: {avg_err:.3f} px")
    print("Camera matrix:\n", K)
    print("Distortion:\n", dist.ravel())

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    fs = cv2.FileStorage(args.out, cv2.FILE_STORAGE_WRITE)
    fs.write("image_width", image_size[0])
    fs.write("image_height", image_size[1])
    fs.write("camera_matrix", K)
    fs.write("distortion_coefficients", dist)
    fs.write("rms", float(ret))
    fs.write("avg_reprojection_error_px", float(avg_err))
    fs.release()

    print(f"Saved: {args.out}")

if __name__ == "__main__":
    main()