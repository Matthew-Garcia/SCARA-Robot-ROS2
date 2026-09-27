"""Standalone overhead blue-object preview. No serial connection or robot motion."""

import argparse
import json
import math
from pathlib import Path

from target_stability import TargetStability
from candidate_filter import candidate_fits


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--calibration', type=Path,
                        default=Path(__file__).with_name('calibration.json'))
    parser.add_argument('--pixels-only', action='store_true')
    args = parser.parse_args()
    try:
        import cv2
        import numpy as np
        from blue_target import load_mapping
    except ImportError as error:
        parser.exit(1, f'Missing dependency: {error}\nRun: python -m pip install opencv-python numpy\n')

    config = None
    if not args.pixels_only:
        config = json.loads(args.calibration.read_text())
    camera = cv2.VideoCapture(args.camera)
    if not camera.isOpened():
        camera.release()
        parser.exit(1, f'Cannot open camera {args.camera}; try --camera 1.\n')
    title = 'Blue cylinder preview - NO ROBOT MOTION'
    controls = 'Blue mask and tuning'
    tracker = TargetStability()
    matrix = polygon = None
    previous_settings = None
    try:
        if config is not None:
            width, height = config['image_size']
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cv2.namedWindow(title)
        cv2.namedWindow(controls)
        for name, value, maximum in [('Hue min', 95, 179), ('Hue max', 135, 179),
                                      ('Saturation min', 80, 255), ('Brightness min', 45, 255),
                                      ('Area min', 350, 10000), ('Area max', 7000, 50000),
                                      ('Roundness %', 45, 100)]:
            cv2.createTrackbar(name, controls, value, maximum, lambda _: None)
        while True:
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError('Camera stopped producing frames')
            if config is not None:
                size = (frame.shape[1], frame.shape[0])
                if tuple(config['image_size']) != size:
                    raise ValueError('Camera resolution does not match calibration; recalibrate or use --pixels-only')
                if matrix is None:
                    matrix, config = load_mapping(args.calibration, size)
                    polygon = cv2.convexHull(np.asarray(config['pixels_uv'], np.float32))
                    bounds = config['workspace_xy_mm']
                    for axis in ('x', 'y'):
                        values = bounds[axis]
                        if len(values) != 2 or not all(math.isfinite(v) for v in values) or values[0] > values[1]:
                            raise ValueError('Invalid calibration workspace bounds')
            settings = tuple(cv2.getTrackbarPos(name, controls) for name in
                             ('Hue min', 'Hue max', 'Saturation min', 'Brightness min', 'Area min', 'Area max', 'Roundness %'))
            if settings != previous_settings:
                tracker.update(None)
                previous_settings = settings
            lo, hi, saturation, brightness, minimum_area, maximum_area, roundness = settings
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, (lo, saturation, brightness), (hi, 255, 255))
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            candidates = []
            for contour in contours:
                area = cv2.contourArea(contour)
                perimeter = cv2.arcLength(contour, True)
                if perimeter <= 0 or not candidate_fits(
                        area, cv2.boundingRect(contour), (frame.shape[1], frame.shape[0]),
                        max(1, minimum_area), maximum_area):
                    continue
                circularity = 4 * math.pi * area / perimeter**2
                if circularity < roundness / 100:
                    continue
                moments = cv2.moments(contour)
                if moments['m00']:
                    candidates.append((area, contour, moments, circularity))
            lines = ['PREVIEW ONLY - Q or Esc to quit', 'No qualifying blue target (hidden or filtered)']
            color = (0, 180, 255)
            if polygon is not None:
                cv2.polylines(frame, [polygon.astype(np.int32)], True, (255, 180, 0), 2)
            if candidates:
                area, contour, moments, circularity = max(candidates, key=lambda c: c[0])
                u, v = moments['m10'] / moments['m00'], moments['m01'] / moments['m00']
                stable = tracker.update((u, v))
                lines[1] = f'Pixel: {u:.1f}, {v:.1f} | area {area:.0f} | roundness {circularity:.2f}'
                lines.append('Stable candidate' if stable else 'Waiting for stationary candidate...')
                if matrix is not None:
                    x, y = map(float, cv2.perspectiveTransform(np.array([[[u, v]]], np.float32), matrix)[0, 0])
                    inside = cv2.pointPolygonTest(polygon, (u, v), False) >= 0
                    in_bounds = all(math.isfinite(a) and bounds[k][0] <= a <= bounds[k][1]
                                    for k, a in [('x', x), ('y', y)])
                    lines.append(f'Robot estimate: X {x:.1f}  Y {y:.1f} mm')
                    lines.append('Within calibration region' if inside and in_bounds else 'OUTSIDE calibration region / bounds')
                    color = (0, 220, 0) if stable and inside and in_bounds else (0, 180, 255)
                else:
                    lines.append('Pixels only - no robot coordinate estimate')
                cv2.drawContours(frame, [contour], -1, color, 2)
                cv2.drawMarker(frame, (round(u), round(v)), color, cv2.MARKER_CROSS, 20, 2)
            else:
                tracker.update(None)
            # Place status outside the camera image so the target is never covered.
            panel = np.zeros((155, max(frame.shape[1], 720), 3), np.uint8)
            for index, line in enumerate(lines):
                cv2.putText(panel, line, (10, 24 + index * 27), cv2.FONT_HERSHEY_SIMPLEX, .55, color, 1, cv2.LINE_AA)
            padded = cv2.copyMakeBorder(frame, 0, 0, 0, panel.shape[1] - frame.shape[1], cv2.BORDER_CONSTANT)
            cv2.imshow(title, np.vstack([padded, panel]))
            cv2.imshow(controls, mask)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27) or cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
