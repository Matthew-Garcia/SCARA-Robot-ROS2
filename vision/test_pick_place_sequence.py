import unittest

from preview_pick_place import build_sequence


class SequenceTests(unittest.TestCase):
    def setUp(self):
        self.config = {
            'travel_z_mm': 0,
            'confirmed_pickup': {
                'joint_angles_deg': dict(j1=-14, j2=2, j3=0),
                'z_mm': 11, 'serial_gripper_hold': 164,
                'user_confirmed_successful_grip': True},
            'placement': {
                'joint_angles_deg': dict(j1=-23, j2=4, j3=0),
                'z_mm': 10, 'serial_gripper_release': -83}}

    def test_horizontal_motion_only_while_raised(self):
        rows = build_sequence(self.config)
        for previous, current in zip(rows, rows[1:]):
            if previous[1:4] != current[1:4]:
                self.assertEqual((previous[4], current[4]), (0, 0))
                self.assertEqual((previous[5], current[5]), (164, 164))

    def test_grip_and_release_happen_at_stationary_poses(self):
        rows = build_sequence(self.config)
        transitions = [(a, b) for a, b in zip(rows, rows[1:]) if a[5] != b[5]]
        self.assertEqual(len(transitions), 2)
        for previous, current in transitions:
            self.assertEqual(previous[1:5], current[1:5])
        self.assertEqual(transitions[0][1][4:], (11, 164))
        self.assertEqual(transitions[1][1][4:], (10, -83))

    def test_finishes_raised_and_open(self):
        self.assertEqual(build_sequence(self.config)[-1][1:], (-23, 4, 0, 0, -83))

    def test_unconfirmed_pickup_rejected(self):
        self.config['confirmed_pickup']['user_confirmed_successful_grip'] = False
        with self.assertRaises(ValueError):
            build_sequence(self.config)


if __name__ == '__main__':
    unittest.main()
