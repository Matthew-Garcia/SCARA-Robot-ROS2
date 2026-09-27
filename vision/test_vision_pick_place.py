"""Camera- and robot-free tests for the camera-guided pick and place additions."""

import contextlib
import io
import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

import joint_calibration as jc
import vision_kinematics as vk
from run_vision_pick_place import (describe, load_setup, main, plan_from_pixel, recovery_rows, return_to_gui_zero,
                                   run_cycle)

HERE = Path(__file__).parent
SETTINGS = json.loads((HERE / 'vision_pick_place.json').read_text())
TAUGHT = json.loads((HERE / 'taught_positions.json').read_text())

# A made-up but realistic overhead camera: robot XY (mm) -> pixel.
TRUE_XY_TO_PIXEL = np.array([[0.02, 4.0, 720.0], [-8.0, 0.03, 2960.0], [0.00002, -0.00001, 1.0]])


def xy_to_pixel(x, y):
    u, v, w = TRUE_XY_TO_PIXEL @ np.array([x, y, 1.0])
    return u / w, v / w


def placements():
    joints = [(-10, -10), (-12, -25), (-11, -1), (-17, -1), (-20, -2), (-25, -25), (-30, -5), (-15, -20), (-28, -14),
              (-13, 3), (-22, 3)]
    return [{'pixel': list(xy_to_pixel(*vk.forward(a, b))), 'j1': a, 'j2': b} for a, b in joints]


class FirmwareSim:
    """Parses packets like SCARA_Robot.ino manual mode and tracks absolute step targets."""

    def __init__(self):
        self.steps = [0, 0, 0, 0]
        self.gripper = None
        self.sent = []

    def write(self, packet):
        self.sent.append(packet)
        content = packet.decode()
        data = []
        for _ in range(10):
            index = content.find(',')
            data.append(int(content[:index] if index >= 0 else content))
            content = content[index + 1:]
        assert data[0] == 0 and data[1] == 0, 'must stay in manual mode'
        self.steps = [int(v * k) for v, k in zip(data[2:6], vk.STEPS_PER_UNIT)]
        self.gripper = data[6]
        return len(packet)

    def flush(self):
        pass


class KinematicsTests(unittest.TestCase):
    def test_j2_direction_matches_measured_robot(self):
        # Real calibration 2026-09-27: these six placements only agree (~2 mm) with J2 reversed.
        pts = [((502.9, 322.9), -6, -1), ((255.7, 369.9), -16, 11), ((489.1, 204.8), -16, -21),
               ((213.6, 181.9), -22, -16), ((346.9, 257.8), -13, 0), ((161.3, 341.6), -21, 5)]
        data = jc.build([{'pixel': list(p), 'j1': a, 'j2': b} for p, a, b in pts], (640, 480))
        self.assertLess(data['rms_fit_error_mm'], 3.0)
        self.assertEqual(vk.J2_DIRECTION, -1)

    def test_ik_round_trip_taught_poses(self):
        for j1, j2 in [(-14, 2), (-23, 4), (-40, 60), (10, 100)]:
            x, y = vk.forward(j1, j2)
            self.assertEqual(vk.integer_joints(x, y, 1, 0.5)[:2], (j1, j2))
            self.assertEqual(vk.integer_joints(x, y, 0, 0.5, prefer=(j1, j2))[:2], (j1, j2))
        x, y = vk.forward(-16, -21)
        self.assertEqual(vk.integer_joints(x, y, -1, 0.5)[:2], (-16, -21))

    def test_unreachable_rejected(self):
        with self.assertRaises(ValueError):
            vk.integer_joints(400, 0, 1, 3)

    def test_rounding_error_bounded(self):
        worst = 0
        for x in range(300, 361, 3):
            for y in range(-160, -39, 3):
                if math.hypot(x, y) < vk.L1 + vk.L2 - 0.5:
                    worst = max(worst, vk.integer_joints(x, y, 1, 10)[2])
        self.assertLess(worst, 3.0)

    def test_move_time_profile(self):
        self.assertAlmostEqual(vk.move_seconds(2000, 4000, 2000), 2 * math.sqrt(1.0))
        self.assertAlmostEqual(vk.move_seconds(16000, 4000, 2000), 16000 / 4000 + 2)
        self.assertEqual(vk.move_seconds(0, 4000, 2000), 0)


class CalibrationTests(unittest.TestCase):
    def test_exact_fit_and_leave_one_out(self):
        data = jc.build(placements(), (640, 480))
        self.assertLess(data['rms_fit_error_mm'], 0.01)
        self.assertLess(data['max_leave_one_out_error_mm'], 0.05)

    def test_noisy_pixels_give_small_error(self):
        rng = np.random.default_rng(1)
        points = placements()
        for p in points:
            p['pixel'] = [c + rng.normal(0, 1.0) for c in p['pixel']]
        data = jc.build(points, (640, 480))
        self.assertLess(data['max_leave_one_out_error_mm'], 3.0)

    def test_collinear_rejected(self):
        with self.assertRaises(ValueError):
            jc.fit_homography([(0, 0), (1, 1), (2, 2), (3, 3)], [(0, 0), (1, 1), (2, 2), (3, 3)])

    def test_hull(self):
        hull = jc.convex_hull([(0, 0), (10, 0), (10, 10), (0, 10), (5, 5)])
        self.assertTrue(jc.inside_hull(hull, (5, 5)))
        self.assertFalse(jc.inside_hull(hull, (11, 5)))
        self.assertTrue(jc.inside_hull(hull, (11, 5), margin_px=2))


class RunnerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.cal_path = Path(self.tmp.name) / 'calibration_joint.json'
        self.cal_path.write_text(json.dumps(jc.build(placements(), (640, 480))))
        self.settings, self.cal, self.drop, self.source = load_setup(
            HERE / 'vision_pick_place.json', self.cal_path, HERE / 'taught_positions.json')

    def tearDown(self):
        self.tmp.cleanup()

    def plan_for_joints(self, j1, j2):
        return plan_from_pixel(*xy_to_pixel(*vk.forward(j1, j2)), self.settings, self.cal, self.drop)

    def test_taught_pickup_recovered_from_camera(self):
        plan = self.plan_for_joints(-14, 2)
        self.assertEqual(plan['joints'], (-14, 2))
        self.assertEqual(self.source, 'taught_positions.json')
        self.assertEqual(plan['rows'][6][1:5], (-23, 4, 0, 10))

    def test_cylinder_in_hole_is_ignored(self):
        with self.assertRaisesRegex(ValueError, 'drop hole'):
            self.plan_for_joints(-23, 4)

    def test_outside_calibrated_area_rejected(self):
        with self.assertRaisesRegex(ValueError, 'calibrated area'):
            self.plan_for_joints(-60, 40)

    def test_poor_calibration_refused(self):
        data = json.loads(self.cal_path.read_text())
        data['max_leave_one_out_error_mm'] = 12.0
        self.cal_path.write_text(json.dumps(data))
        with self.assertRaises(ValueError):
            load_setup(HERE / 'vision_pick_place.json', self.cal_path, HERE / 'taught_positions.json')

    def test_full_cycle_reaches_targets_and_returns_to_park(self):
        plan = self.plan_for_joints(-18, -12)
        robot = FirmwareSim()
        waits = []
        with contextlib.redirect_stdout(io.StringIO()):
            done, last = run_cycle(robot, plan, self.settings, auto=True,
                                   wait=lambda s: waits.append(s) or False)
        self.assertTrue(done)
        self.assertEqual(len(robot.sent), 10)
        # Grip at the cylinder: firmware steps equal the absolute pick pose (park is zero).
        grip_packet = robot.sent[3].decode().split(',')
        j1, j2 = map(int, grip_packet[2:4])
        self.assertEqual(grip_packet[4:7], ['0', '11', '164'])
        # Either elbow side is fine; the gripper must land on the cylinder.
        self.assertLess(math.dist(vk.forward(j1, j2), vk.forward(-18, -12)), 3.0)
        self.assertEqual(robot.steps, [0, 0, 0, 0])
        self.assertTrue(all(w >= 1.5 for w in waits))

    def test_nonzero_park_is_subtracted(self):
        settings = dict(self.settings, park_pose={'j1': -5, 'j2': 3, 'j3': 0, 'z': 0})
        plan = plan_from_pixel(*xy_to_pixel(*vk.forward(-14, 2)), settings, self.cal, self.drop)
        packet = vk.packet_for(plan['rows'][1], settings['park_pose'])
        self.assertEqual(packet, b'0,0,-9,-1,0,0,-83,500,500')

    def test_step_mode_stops_on_refusal(self):
        plan = self.plan_for_joints(-18, -12)
        robot = FirmwareSim()
        answers = iter(['MOVE', 'DONE', 'MOVE', 'no'])
        with contextlib.redirect_stdout(io.StringIO()):
            done, last = run_cycle(robot, plan, self.settings, auto=False,
                                   read=lambda _: next(answers), wait=lambda s: False)
        self.assertFalse(done)
        self.assertEqual(len(robot.sent), 2)
        self.assertEqual(last[0], 'Above cylinder')

    def test_auto_quit_key_stops_after_current_step(self):
        plan = self.plan_for_joints(-18, -12)
        robot = FirmwareSim()
        with contextlib.redirect_stdout(io.StringIO()):
            done, last = run_cycle(robot, plan, self.settings, auto=True, wait=lambda s: True)
        self.assertFalse(done)
        self.assertEqual(len(robot.sent), 1)

    def test_recovery_raises_before_parking(self):
        last = ('Lower into hole', -23, 4, 0, 10, 164)
        rows = recovery_rows(last, self.settings)
        self.assertEqual(rows[0][1:6], (-23, 4, 0, 0, 164))
        self.assertEqual(rows[1][1:6], (0, 0, 0, 0, 164))

    def test_test_pixel_cli(self):
        out = io.StringIO()
        u, v = xy_to_pixel(*vk.forward(-14, 2))
        with contextlib.redirect_stdout(out):
            main(['--calibration', str(self.cal_path), '--test-pixel', str(u), str(v)])
        self.assertIn('J1=-14 J2=2', out.getvalue())
        self.assertIn('Return to park', out.getvalue())

    def test_nonzero_park_offers_return_to_gui_zero(self):
        settings = dict(self.settings, park_pose={'j1': -40, 'j2': 20, 'j3': 0, 'z': 0})
        robot = FirmwareSim()
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertTrue(return_to_gui_zero(robot, settings, None, read=lambda _: 'HOME'))
            self.assertFalse(return_to_gui_zero(FirmwareSim(), self.settings, None, read=lambda _: 'HOME'))
        self.assertEqual(robot.sent, [b'0,0,40,-20,0,0,-83,500,500'])

    def test_describe_lists_all_steps(self):
        text = describe(self.plan_for_joints(-18, -12), self.settings)
        self.assertEqual(sum(line.lstrip().split()[0] != 'Step' and '500,500' in line for line in text.splitlines()), 10)


if __name__ == '__main__':
    unittest.main()
