"""Pixel -> robot XY calibration built from cylinder placements in the GUI joint frame.

Each point pairs the camera pixel of the cylinder top with the J1/J2 the robot
used to set it down. XY comes from forward kinematics of those joints, so the
mapping lands in the same frame the robot is commanded in, at cylinder-top height.
Needs numpy only (no OpenCV), so it can be unit-tested without a camera.
"""

import json
import math
from pathlib import Path

import numpy as np

from vision_kinematics import J2_DIRECTION, forward


def _normalizer(points):
    points = np.asarray(points, float)
    centre = points.mean(axis=0)
    spread = np.sqrt(((points - centre) ** 2).sum(axis=1)).mean()
    scale = math.sqrt(2) / spread if spread > 0 else 1.0
    return np.array([[scale, 0, -scale * centre[0]], [0, scale, -scale * centre[1]], [0, 0, 1]])


def fit_homography(pixels, xy):
    """Least-squares normalised DLT homography mapping pixels to XY (needs >= 4 points)."""
    pixels = np.asarray(pixels, float)
    xy = np.asarray(xy, float)
    if len(pixels) < 4 or pixels.shape != xy.shape or pixels.shape[1] != 2:
        raise ValueError('Need at least four matching pixel/XY points')
    tp, tx = _normalizer(pixels), _normalizer(xy)
    p = (tp @ np.c_[pixels, np.ones(len(pixels))].T).T
    q = (tx @ np.c_[xy, np.ones(len(xy))].T).T
    rows = []
    for (u, v, _), (x, y, _) in zip(p, q):
        rows.append([-u, -v, -1, 0, 0, 0, u * x, v * x, x])
        rows.append([0, 0, 0, -u, -v, -1, u * y, v * y, y])
    _, singular, vt = np.linalg.svd(np.asarray(rows))
    h = vt[-1].reshape(3, 3)
    matrix = np.linalg.inv(tx) @ h @ tp
    if abs(matrix[2, 2]) < 1e-12 or not np.isfinite(matrix).all():
        raise ValueError('Degenerate calibration (points may be collinear)')
    matrix = matrix / matrix[2, 2]
    if len(singular) >= 8 and singular[7] < 1e-9 * singular[0]:
        raise ValueError('Degenerate calibration (points may be collinear)')
    return matrix


def pixel_to_xy(matrix, u, v):
    x, y, w = np.asarray(matrix) @ np.array([u, v, 1.0])
    if abs(w) < 1e-12:
        raise ValueError('Pixel maps to infinity')
    return float(x / w), float(y / w)


def residuals(matrix, pixels, xy):
    return [math.dist(pixel_to_xy(matrix, *p), tuple(q)) for p, q in zip(pixels, xy)]


def leave_one_out(pixels, xy):
    """Error at each point when it is left out of the fit (honest accuracy estimate)."""
    if len(pixels) < 5:
        return []
    errors = []
    for skip in range(len(pixels)):
        keep_p = [p for i, p in enumerate(pixels) if i != skip]
        keep_q = [q for i, q in enumerate(xy) if i != skip]
        try:
            matrix = fit_homography(keep_p, keep_q)
            errors.append(math.dist(pixel_to_xy(matrix, *pixels[skip]), tuple(xy[skip])))
        except ValueError:
            errors.append(float('inf'))
    return errors


def convex_hull(points):
    pts = sorted(set(map(tuple, points)))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def inside_hull(hull, point, margin_px=0.0):
    """True if point lies inside a counter-clockwise convex hull, or within margin_px outside it.

    The margin matters near full arm extension, where calibration points sit on an arc and
    the straight hull edges cut slightly inside it.
    """
    if len(hull) < 3:
        return False
    for i in range(len(hull)):
        a, b = hull[i], hull[(i + 1) % len(hull)]
        length = math.dist(a, b)
        if length == 0:
            continue
        signed = ((b[0] - a[0]) * (point[1] - a[1]) - (b[1] - a[1]) * (point[0] - a[0])) / length
        if signed < -margin_px:
            return False
    return True


def build(points, image_size, drop_pose=None):
    """points: list of dicts with pixel [u, v], j1, j2. Returns the calibration dict to save."""
    pixels = [p['pixel'] for p in points]
    xy = [forward(p['j1'], p['j2']) for p in points]
    matrix = fit_homography(pixels, xy)
    fit = residuals(matrix, pixels, xy)
    loo = leave_one_out(pixels, xy)
    data = {
        'kind': 'joint_frame_placement_calibration',
        'frame': 'Processing GUI joint frame (zero = pose at controller power-on/reset)',
        'j2_direction': J2_DIRECTION,
        'image_size': list(image_size),
        'points': [{'pixel': [round(p['pixel'][0], 2), round(p['pixel'][1], 2)], 'j1': p['j1'], 'j2': p['j2'],
                    'xy_mm': [round(x, 2), round(y, 2)], 'fit_error_mm': round(e, 2)}
                   for p, (x, y), e in zip(points, xy, fit)],
        'homography': [[float(v) for v in row] for row in matrix],
        'rms_fit_error_mm': round(math.sqrt(sum(e * e for e in fit) / len(fit)), 2),
        'max_leave_one_out_error_mm': round(max(loo), 2) if loo else None,
        'pixel_hull': [list(p) for p in convex_hull([tuple(p) for p in pixels])],
    }
    if drop_pose is not None:
        data['drop_pose'] = drop_pose
    return data


def load(path, frame_size=None):
    data = json.loads(Path(path).read_text())
    if data.get('kind') != 'joint_frame_placement_calibration':
        raise ValueError(f'{path} is not a placement calibration; run calibrate_by_placing.py')
    if frame_size is not None and tuple(data['image_size']) != tuple(frame_size):
        raise ValueError('Camera resolution differs from the calibration; recalibrate')
    if data.get('j2_direction') != J2_DIRECTION:
        raise ValueError('Calibration was made with an older J2 direction; recalibrate '
                         '(your saved points in calibration_points.json can be reused)')
    matrix = np.asarray(data['homography'], float)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError('Invalid homography in calibration file')
    data['matrix'] = matrix
    data['hull'] = [tuple(p) for p in data['pixel_hull']]
    return data
