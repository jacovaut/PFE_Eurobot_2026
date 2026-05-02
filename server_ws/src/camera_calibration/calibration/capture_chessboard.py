import cv2
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
output_dir = os.path.join(script_dir, "calibration_images_blue")
os.makedirs(output_dir, exist_ok=True)

CAMERA_DEVICE = "/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera_01.00.00-video-index0"
camera = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)

camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
camera.set(cv2.CAP_PROP_FPS, 15)
camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

img_id = 0

print("Press SPACE to capture chessboard image")
print("Press Q to quit")

while True:
    ret, frame = camera.read()
    if not ret:
        break

    cv2.imshow("Calibration Capture", frame)

    key = cv2.waitKey(1)

    if key == ord(' '):
        filename = f"{output_dir}/img_{img_id:03d}.png"
        cv2.imwrite(filename, frame)
        print("Saved:", filename)
        img_id += 1

    if key == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()