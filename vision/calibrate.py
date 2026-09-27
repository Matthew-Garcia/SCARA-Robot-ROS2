"""Click four table points and save a planar camera-to-robot mapping."""

import argparse
import json
from pathlib import Path

import cv2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("calibration.json"))
    args = parser.parse_args()
    camera = cv2.VideoCapture(args.camera)
    if not camera.isOpened():
        raise RuntimeError("Camera unavailable")
    try:
        ok, frame = camera.read()
        if not ok:
            raise RuntimeError("Cannot capture a calibration image")
    finally:
        camera.release()
    points = []

    def on_click(event, x, y, flags, userdata):
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
            points.append([x, y])

    window = "Click 4 measured table points; r reset; q finish"
    cv2.namedWindow(window)
    cv2.setMouseCallback(window, on_click)
    while True:
        preview = frame.copy()
        for index, (x, y) in enumerate(points, 1):
            cv2.circle(preview, (x, y), 6, (0, 255, 0), -1)
            cv2.putText(preview, str(index), (x + 8, y - 8), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 255, 0), 2)
        cv2.imshow(window, preview)
        key = cv2.waitKey(30) & 0xFF
        if key == ord("r"):
            points.clear()
        elif key == ord("q") or len(points) == 4:
            break
    cv2.destroyAllWindows()
    if len(points) != 4:
        print("Cancelled; no calibration saved")
        return
    print("Use actual SCARA base coordinates in mm for the marked table points.")
    robot = []
    for index in range(4):
        while True:
            try:
                x, y = map(float, input(f"Point {index + 1} pixel {points[index]} -> robot x y (mm): ").replace(",", " ").split())
                robot.append([x, y])
                break
            except ValueError:
                print("Enter two numbers, for example: 200 -50")
    xs, ys = zip(*robot)
    print("Set conservative workspace bounds independently; these defaults span only the four entered points.")
    config = {"image_size": [frame.shape[1], frame.shape[0]], "pixels_uv": points,
              "robot_xy_mm": robot, "workspace_xy_mm": {"x": [min(xs), max(xs)], "y": [min(ys), max(ys)]}}
    args.output.write_text(json.dumps(config, indent=2) + "\n")
    print(f"Saved {args.output}")


if __name__ == "__main__":
    main()
