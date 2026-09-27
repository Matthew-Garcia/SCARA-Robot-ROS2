"""Small, camera-independent stability check for a stationary target."""

import math
from collections import deque


class TargetStability:
    def __init__(self, frames=8, tolerance_px=8):
        if frames < 2 or not math.isfinite(tolerance_px) or tolerance_px <= 0:
            raise ValueError("Use at least two frames and a positive finite tolerance")
        self.points = deque(maxlen=frames)
        self.tolerance = tolerance_px

    def update(self, point):
        if point is None or not all(math.isfinite(v) for v in point):
            self.points.clear()
            return False
        # Compare against every sample, preventing slow drift from looking stable.
        if any(math.dist(point, old) > self.tolerance for old in self.points):
            self.points.clear()
        self.points.append(tuple(point))
        return len(self.points) == self.points.maxlen
