"""Camera preview and optional planar mapping for a blue pick target."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def detect_blue(frame, minimum_area=350):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (95, 80, 45), (135, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask
    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    moments = cv2.moments(contour)
    if area < minimum_area or moments["m00"] == 0:
        return None, mask
    return (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"], area), mask


def load_mapping(path, frame_size):
    config = json.loads(Path(path).read_text())
    if tuple(config["image_size"]) != frame_size:
        raise ValueError("Camera resolution differs from calibration image_size")
    pixels = np.asarray(config["pixels_uv"], dtype=np.float32)
    robot = np.asarray(config["robot_xy_mm"], dtype=np.float32)
    if pixels.shape != (4, 2) or robot.shape != (4, 2):
        raise ValueError("Calibration requires four pixel and four robot points")
    matrix = cv2.getPerspectiveTransform(pixels, robot)
    if not np.isfinite(matrix).all() or abs(np.linalg.det(matrix)) < 1e-12:
        raise ValueError("Degenerate calibration points")
    return matrix, config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--calibration", type=Path)
    parser.add_argument("--minimum-area", type=float, default=350)
    args = parser.parse_args()
    camera = cv2.VideoCapture(args.camera)
    if not camera.isOpened():
        raise RuntimeError(f"Cannot open camera {args.camera}")
    matrix = config = None
    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("Camera stopped producing frames")
            if args.calibration and matrix is None:
                matrix, config = load_mapping(args.calibration, (frame.shape[1], frame.shape[0]))
            found, mask = detect_blue(frame, args.minimum_area)
            if found:
                u, v, area = found
                label = f"pixel ({u:.0f}, {v:.0f}) area {area:.0f}"
                if matrix is not None:
                    xy = cv2.perspectiveTransform(np.array([[[u, v]]], np.float32), matrix)[0, 0]
                    x, y = map(float, xy)
                    bounds = config["workspace_xy_mm"]
                    valid = bounds["x"][0] <= x <= bounds["x"][1] and bounds["y"][0] <= y <= bounds["y"][1]
                    label += f" | robot ({x:.1f}, {y:.1f}) mm | {'IN' if valid else 'OUT OF'} bounds"
                cv2.circle(frame, (round(u), round(v)), 8, (0, 255, 0), 2)
                cv2.putText(frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 2)
            cv2.imshow("Blue target (q to quit)", frame)
            cv2.imshow("Blue mask", mask)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
