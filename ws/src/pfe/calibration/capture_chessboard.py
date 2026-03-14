import cv2
import os

output_dir = "calibration_images"
os.makedirs(output_dir, exist_ok=True)

camera = cv2.VideoCapture(0)

camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

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