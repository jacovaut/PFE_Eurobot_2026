#!/usr/bin/env python3

import argparse
import sys

import cv2
import numpy as np


def open_camera(source: str, width: int, height: int, fps: int):
    cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open camera: {source}")

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def normalize_frame(frame: np.ndarray) -> np.ndarray:
    if frame is None:
        return np.zeros((720, 1280, 3), dtype=np.uint8)
    if len(frame.shape) == 2:
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    return frame


def annotate(frame: np.ndarray, label: str, extra_text: str | None = None) -> np.ndarray:
    output = frame.copy()
    h, w = output.shape[:2]
    banner_height = 48 if extra_text is None else 78
    cv2.rectangle(output, (0, 0), (w, banner_height), (0, 0, 0), -1)
    cv2.putText(
        output,
        f"{label}  {w}x{h}",
        (16, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (0, 255, 0),
        2,
    )
    if extra_text is not None:
        cv2.putText(
            output,
            extra_text,
            (16, 64),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )
    return output


def center_crop(frame: np.ndarray, crop_fraction: float) -> np.ndarray:
    h, w = frame.shape[:2]
    crop_w = max(1, int(w * crop_fraction))
    crop_h = max(1, int(h * crop_fraction))
    x0 = max(0, (w - crop_w) // 2)
    y0 = max(0, (h - crop_h) // 2)
    return frame[y0:y0 + crop_h, x0:x0 + crop_w]


def focus_score(frame: np.ndarray) -> float:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def build_focus_view(frame: np.ndarray, label: str, crop_fraction: float, crop_scale: int) -> np.ndarray:
    full_view = annotate(frame, label)

    crop = center_crop(frame, crop_fraction)
    score = focus_score(crop)
    crop_view = cv2.resize(
        crop,
        (crop.shape[1] * crop_scale, crop.shape[0] * crop_scale),
        interpolation=cv2.INTER_NEAREST,
    )
    crop_view = annotate(crop_view, f"{label} zoom", f"focus score: {score:.1f}")
    return full_view, crop_view


def main() -> int:
    parser = argparse.ArgumentParser(description="Side-by-side preview for two cameras.")
    parser.add_argument(
        "--left",
        default="/dev/video0",
        help="Left camera source (device path or by-id path). Default: /dev/video0",
    )
    parser.add_argument(
        "--right",
        default="/dev/video4",
        help="Right camera source (device path or by-id path). Default: /dev/video4",
    )
    parser.add_argument(
        "--left-label",
        default="Left camera",
        help="Display label for the left camera.",
    )
    parser.add_argument(
        "--right-label",
        default="Right camera",
        help="Display label for the right camera.",
    )
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument(
        "--crop-fraction",
        type=float,
        default=0.25,
        help="Fraction of the center area to crop for the zoomed focus view. Default: 0.25",
    )
    parser.add_argument(
        "--crop-scale",
        type=int,
        default=3,
        help="Integer scale factor for the zoomed crop. Default: 3",
    )
    parser.add_argument(
        "--window-name",
        default="camera_compare",
        help="OpenCV window name.",
    )
    args = parser.parse_args()

    try:
        left_cap = open_camera(args.left, args.width, args.height, args.fps)
        right_cap = open_camera(args.right, args.width, args.height, args.fps)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    cv2.namedWindow(args.window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(args.window_name, 1600, 700)

    print("Preview open. Press q or Esc to quit.")

    try:
        while True:
            left_ok, left_frame = left_cap.read()
            right_ok, right_frame = right_cap.read()

            if not left_ok and not right_ok:
                break

            left_view = normalize_frame(left_frame if left_ok else None)
            right_view = normalize_frame(right_frame if right_ok else None)

            left_full, left_crop = build_focus_view(
                left_view, args.left_label, args.crop_fraction, args.crop_scale)
            right_full, right_crop = build_focus_view(
                right_view, args.right_label, args.crop_fraction, args.crop_scale)

            if left_full.shape[:2] != right_full.shape[:2]:
                right_full = cv2.resize(right_full, (left_full.shape[1], left_full.shape[0]))
            if left_crop.shape[:2] != right_crop.shape[:2]:
                right_crop = cv2.resize(right_crop, (left_crop.shape[1], left_crop.shape[0]))

            combined_top = np.hstack([left_full, right_full])
            combined_bottom = np.hstack([left_crop, right_crop])
            if combined_bottom.shape[1] != combined_top.shape[1]:
                combined_bottom = cv2.resize(
                    combined_bottom,
                    (combined_top.shape[1], combined_bottom.shape[0]),
                    interpolation=cv2.INTER_NEAREST,
                )
            combined = np.vstack([combined_top, combined_bottom])
            cv2.imshow(args.window_name, combined)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
    finally:
        left_cap.release()
        right_cap.release()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())