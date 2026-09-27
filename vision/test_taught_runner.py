import contextlib
import io
import json
from pathlib import Path
import unittest

from run_taught_pick_place import main, packet_for, prepare, send_once, supervised_steps


class FakePort:
    def __init__(self, partial=False):
        self.sent = []
        self.partial = partial

    def write(self, packet):
        self.sent.append(packet)
        return len(packet) - int(self.partial)


class RunnerTests(unittest.TestCase):
    def setUp(self):
        self.settings = json.loads(Path(__file__).with_name('taught_positions.json').read_text())
        self.reference, self.rows = prepare(self.settings)

    def test_pickup_and_drop_rebased_after_reset(self):
        self.assertEqual(packet_for(self.rows[0], self.reference), b'0,0,0,0,0,0,-83,500,500')
        self.assertEqual(packet_for(self.rows[2], self.reference), b'0,0,0,0,0,11,164,500,500')
        self.assertEqual(packet_for(self.rows[4], self.reference), b'0,0,-9,2,0,0,164,500,500')

    def test_cancel_before_move_sends_nothing(self):
        port = FakePort()
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertFalse(supervised_steps(port, self.reference, self.rows, lambda _: 'quit', lambda _: None))
        self.assertEqual(port.sent, [])

    def test_failed_completion_never_sends_next_step(self):
        port = FakePort()
        answers = iter(['MOVE', 'not done'])
        delays = []
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertFalse(supervised_steps(port, self.reference, self.rows, lambda _: next(answers), delays.append))
        self.assertEqual(len(port.sent), 1)
        self.assertEqual(delays, [2.0])

    def test_one_cycle_no_repetition(self):
        port = FakePort()
        answers = iter(['MOVE', 'DONE'] * 8)
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertTrue(supervised_steps(port, self.reference, self.rows, lambda _: next(answers), lambda _: None))
        self.assertEqual(len(port.sent), 8)
        self.assertTrue(all(packet.startswith(b'0,0,') for packet in port.sent))

    def test_partial_write_is_not_retried(self):
        port = FakePort(partial=True)
        with self.assertRaises(RuntimeError):
            send_once(port, packet_for(self.rows[0], self.reference), lambda _: None)
        self.assertEqual(len(port.sent), 1)

    def test_invalid_joint_limit_rejected(self):
        self.settings['placement']['joint_angles_deg']['j1'] = 300
        with self.assertRaises(ValueError):
            prepare(self.settings)

    def test_default_is_preview(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            main([])
        self.assertIn('PREVIEW ONLY', output.getvalue())


if __name__ == '__main__':
    unittest.main()
