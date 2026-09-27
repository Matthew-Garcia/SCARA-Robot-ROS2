import unittest

from candidate_filter import candidate_fits


class CandidateFilterTests(unittest.TestCase):
    def test_observed_cylinder_accepted(self):
        self.assertTrue(candidate_fits(2380, (340, 270, 60, 70), (640, 480), 350, 7000))

    def test_observed_arm_areas_rejected(self):
        for area in (29960, 40412):
            self.assertFalse(candidate_fits(area, (100, 100, 300, 300), (640, 480), 350, 7000))

    def test_each_image_edge_rejected_even_for_small_area(self):
        for rect in ((0, 30, 50, 50), (30, 0, 50, 50),
                     (590, 30, 50, 50), (30, 430, 50, 50)):
            self.assertFalse(candidate_fits(2380, rect, (640, 480), 350, 7000))

    def test_invalid_area_and_inverted_limits_rejected(self):
        for area, minimum, maximum in ((float('nan'), 350, 7000), (200, 350, 7000),
                                        (2380, 3500, 2000), (2380, 350, 0)):
            self.assertFalse(candidate_fits(area, (100, 100, 60, 60), (640, 480), minimum, maximum))


if __name__ == '__main__':
    unittest.main()
