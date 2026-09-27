import unittest

from target_stability import TargetStability


class StabilityTests(unittest.TestCase):
    def test_requires_full_window(self):
        tracker = TargetStability(frames=3)
        self.assertFalse(tracker.update((100, 100)))
        self.assertFalse(tracker.update((101, 100)))
        self.assertTrue(tracker.update((100, 101)))

    def test_loss_invalid_and_jump_reset(self):
        for interruption in (None, (float('nan'), 0), (200, 200)):
            tracker = TargetStability(frames=3)
            for _ in range(3):
                tracker.update((100, 100))
            self.assertFalse(tracker.update(interruption))
            self.assertFalse(tracker.update((100, 100)))

    def test_slow_drift_is_not_stable(self):
        tracker = TargetStability(frames=8, tolerance_px=8)
        for x in range(30):
            self.assertFalse(tracker.update((x * 2, 0)))

    def test_invalid_settings(self):
        for frames, tolerance in ((1, 8), (8, 0), (8, float('nan'))):
            with self.assertRaises(ValueError):
                TargetStability(frames, tolerance)


if __name__ == '__main__':
    unittest.main()
