"""Blue-cylinder detection shared by the calibration and pick/place scripts.

Same pipeline and default thresholds as blue_cylinder_preview.py; tune there with
the sliders, then copy the values into vision_pick_place.json -> "detection".
"""

import math
from collections import deque

from candidate_filter import candidate_fits
from target_stability import TargetStability


def find_candidates(frame, settings, cv2, np):
    """All qualifying blue blobs as dicts, largest first."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (settings['hue_min'], settings['saturation_min'], settings['brightness_min']),
                       (settings['hue_max'], 255, 255))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    size = (frame.shape[1], frame.shape[0])
    found = []
    for contour in contours:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0 or not candidate_fits(area, cv2.boundingRect(contour), size,
                                                max(1, settings['area_min']), settings['area_max']):
            continue
        roundness = 4 * math.pi * area / perimeter ** 2
        if roundness < settings['roundness_min']:
            continue
        moments = cv2.moments(contour)
        if moments['m00']:
            found.append({'u': moments['m10'] / moments['m00'], 'v': moments['m01'] / moments['m00'],
                          'area': area, 'roundness': roundness, 'contour': contour})
    found.sort(key=lambda c: c['area'], reverse=True)
    return found, mask


class StableTarget:
    """Wraps TargetStability and returns the median pixel once the target is stationary."""

    def __init__(self, frames=8, tolerance_px=8):
        self.check = TargetStability(frames, tolerance_px)
        self.recent = deque(maxlen=frames)

    def update(self, point):
        stable = self.check.update(point)
        if point is None or len(self.check.points) == 1:
            self.recent.clear()
        if point is not None:
            self.recent.append(tuple(point))
        if not stable:
            return None
        us = sorted(p[0] for p in self.recent)
        vs = sorted(p[1] for p in self.recent)
        return us[len(us) // 2], vs[len(vs) // 2]
