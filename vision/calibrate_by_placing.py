"""Calibrate the camera by letting the ROBOT place the cylinder at several spots.

Everything is typed in this Command Prompt window. The camera window only shows
what the camera sees (green outline = cylinder found). No serial port is opened.

For each spot:
  1. In Processing: set the cylinder down on blank paper, raise to Z=0.
  2. Write down J1 and J2 from the GUI.
  3. In Processing: swing the arm out of the camera view.
  4. Here: type J1 and J2 (example: -6 -1) and press Enter.

Other things you can type:  undo   delete N   save   quit
"""

import argparse
import json
from pathlib import Path
import time

from vision_kinematics import GUI_LIMITS, forward

HERE = Path(__file__).parent


def parse_ints(text, count, limits):
    """Returns (values, error_message)."""
    parts = text.replace(',', ' ').split()
    try:
        values = [int(v) for v in parts]
    except ValueError:
        return None, 'Type whole numbers like: -6 -1   (or undo / save / quit)'
    if len(values) != count:
        return None, f'Type {count} numbers separated by a space.'
    if not all(lo <= v <= hi for v, (lo, hi) in zip(values, limits)):
        return None, 'Outside the GUI slider limits; check the numbers.'
    return values, None


def report(points, joint_calibration):
    if len(points) < 4:
        print(f'  {len(points)} point(s) so far. Need at least 5 (6-8 spread over the paper is better).')
        return None
    data = joint_calibration.build(points, (0, 0))
    print('  #   pixel u,v        J1   J2    X mm    Y mm   fit err')
    for i, p in enumerate(data['points'], 1):
        print(f"  {i:<3} {p['pixel'][0]:6.1f},{p['pixel'][1]:6.1f}  {p['j1']:5} {p['j2']:4} "
              f"{p['xy_mm'][0]:7.1f} {p['xy_mm'][1]:7.1f}  {p['fit_error_mm']:5.1f}")
    print(f"  RMS fit error {data['rms_fit_error_mm']} mm", end='')
    loo = data['max_leave_one_out_error_mm']
    print(f"; worst leave-one-out error {loo} mm" if loo is not None else '; add a 5th point for an accuracy check')
    return data


class ConsoleLine:
    """Reads a typed line from the console without blocking the camera loop."""

    def __init__(self):
        self.buffer = ''
        try:
            import msvcrt  # Windows Command Prompt / PowerShell
            self.msvcrt = msvcrt
        except ImportError:
            self.msvcrt = None

    def prompt(self, text):
        self.buffer = ''
        print(text, end='', flush=True)

    def poll(self):
        """Returns a finished line, or None if the user is still typing."""
        if self.msvcrt is None:
            import select
            import sys
            if select.select([sys.stdin], [], [], 0)[0]:
                return sys.stdin.readline().strip()
            return None
        while self.msvcrt.kbhit():
            ch = self.msvcrt.getwch()
            if ch in ('\r', '\n'):
                print(flush=True)
                line, self.buffer = self.buffer, ''
                return line.strip()
            if ch == '\x03':
                raise KeyboardInterrupt
            if ch == '\b':
                if self.buffer:
                    self.buffer = self.buffer[:-1]
                    print('\b \b', end='', flush=True)
            elif ch in ('\x00', '\xe0'):
                self.msvcrt.getwch()  # arrow/function key: ignore
            elif ch.isprintable():
                self.buffer += ch
                print(ch, end='', flush=True)
        return None


def load_points(path):
    try:
        data = json.loads(path.read_text())
        return [{'pixel': [float(p['pixel'][0]), float(p['pixel'][1])], 'j1': int(p['j1']), 'j2': int(p['j2'])}
                for p in data['points']]
    except (OSError, ValueError, KeyError, TypeError, IndexError):
        return []


def store_points(path, points, size):
    path.write_text(json.dumps({'note': 'Work-in-progress calibration points (autosaved).',
                                'image_size': list(size) if size else None, 'points': points}, indent=2) + '\n')


def open_camera(cv2, index, width, height, seconds=20):
    camera = cv2.VideoCapture(index)
    if not camera.isOpened():
        camera.release()
        return None, f'Cannot open camera {index}. Close other programs using it, or try --camera 0'
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        ok, frame = camera.read()
        if ok and frame is not None:
            return camera, None
        time.sleep(0.1)
    camera.release()
    return None, (f'Camera {index} opened but sent no picture. Close any other window using the camera '
                  '(an old calibration or preview), then try again.')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--settings', type=Path, default=HERE / 'vision_pick_place.json')
    parser.add_argument('--output', type=Path, default=HERE / 'calibration_joint.json')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--points', type=Path, default=HERE / 'calibration_points.json',
                        help='autosave file; points are resumed from it')
    args = parser.parse_args()
    try:
        import cv2
        import numpy as np
        import joint_calibration
        from cylinder_detection import StableTarget, find_candidates
    except ImportError as error:
        parser.exit(1, f'Missing dependency: {error}\nRun: python -m pip install opencv-python numpy\n')

    settings = json.loads(args.settings.read_text())
    points = []
    saved = load_points(args.points) if args.points.exists() else []
    if saved:
        print(f'Found {len(saved)} saved point(s) from last time in {args.points.name}:')
        report(saved, joint_calibration) if len(saved) >= 4 else print(f'  {len(saved)} point(s).')
        print('Only reuse them if the camera has NOT moved and the Arduino has NOT been restarted since.')
        if input('Continue with these points? type yes, or press Enter to start fresh > ').strip().lower() == 'yes':
            points = saved
    print(f'Opening camera {args.camera}...')
    camera, problem = open_camera(cv2, args.camera, args.width, args.height)
    if problem:
        parser.exit(1, problem + '\n')
    print(__doc__)
    console = ConsoleLine()
    stability = StableTarget(settings['stability']['frames'], settings['stability']['tolerance_px'])
    title = 'Camera view (type everything in the Command Prompt)'
    size = None
    pending = None          # joints typed, waiting for a steady cylinder
    pending_until = 0.0
    mode = 'point'          # 'point', 'confirm_quit', 'drop'

    def ask_next():
        console.prompt(f'Point {len(points) + 1}: type J1 J2 (or undo / delete N / save / quit) > ')

    try:
        ok, frame = camera.read()
        size = (frame.shape[1], frame.shape[0])
        print(f'Camera {args.camera} running at {size[0]}x{size[1]}. '
              'Keep Processing open and do not restart the Arduino.\n')
        ask_next()
        while True:
            ok, frame = camera.read()
            if not ok:
                print('\nCamera stopped producing frames.')
                return
            if (frame.shape[1], frame.shape[0]) != size:
                print('\nCamera resolution changed; start again.')
                return
            candidates, _ = find_candidates(frame, settings['detection'], cv2, np)
            target = candidates[0] if candidates else None
            stable = stability.update((target['u'], target['v']) if target else None)

            for i, p in enumerate(points, 1):
                u, v = map(round, p['pixel'])
                cv2.circle(frame, (u, v), 5, (255, 0, 255), -1)
                cv2.putText(frame, str(i), (u + 7, v - 7), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 0, 255), 1)
            if target:
                color = (0, 220, 0) if stable else (0, 180, 255)
                cv2.drawContours(frame, [target['contour']], -1, color, 2)
                status = 'CYLINDER FOUND' if stable else 'settling...'
            else:
                color, status = (0, 0, 255), 'No cylinder visible'
            cv2.putText(frame, f'{status} | points: {len(points)}', (10, 22), cv2.FONT_HERSHEY_SIMPLEX, .55, color, 2)
            cv2.imshow(title, frame)
            cv2.waitKey(1)

            if pending is not None:
                if stable is not None:
                    points.append({'pixel': [stable[0], stable[1]], 'j1': pending[0], 'j2': pending[1]})
                    store_points(args.points, points, size)
                    x, y = forward(*pending)
                    print(f'  Saved point {len(points)}: J1={pending[0]} J2={pending[1]} -> X={x:.1f} Y={y:.1f} mm '
                          f'(camera pixel {stable[0]:.0f},{stable[1]:.0f})')
                    if len(candidates) > 1:
                        print('  Note: more than one blue object visible; the largest was used.')
                    report(points, joint_calibration)
                    print('  Next: move the cylinder to a NEW spot with the robot, swing the arm away, type its J1 J2.\n')
                    pending = None
                    ask_next()
                elif time.monotonic() > pending_until:
                    print('  The camera does not see a steady cylinder (arm still in view? cylinder on the paper?).')
                    print('  Point NOT saved. Type the numbers again when the camera window says CYLINDER FOUND.\n')
                    pending = None
                    ask_next()
                continue

            line = console.poll()
            if line is None:
                continue
            text = line.lower()
            if mode == 'confirm_quit':
                if text == 'yes':
                    print('Quit without saving.')
                    return
                mode = 'point'
                ask_next()
                continue
            if mode == 'drop':
                if not text:
                    drop_pose = None
                else:
                    values, problem = parse_ints(text, 4, GUI_LIMITS)
                    if problem:
                        print('  Type four whole numbers like: -23 4 0 10   (or just press Enter)')
                        console.prompt('Drop J1 J2 J3 Z (or Enter to skip) > ')
                        continue
                    drop_pose = dict(zip(('j1', 'j2', 'j3', 'z'), values))
                save(points, size, args.output, settings, joint_calibration, drop_pose)
                return
            if not text:
                ask_next()
                continue
            if text in ('q', 'quit', 'exit'):
                if not points:
                    print('Quit without saving.')
                    return
                mode = 'confirm_quit'
                console.prompt('  Quit WITHOUT saving? type yes > ')
                continue
            if text.split()[:1] in (['delete'], ['del']):
                parts = text.split()
                if len(parts) == 2 and parts[1].isdigit() and 1 <= int(parts[1]) <= len(points):
                    removed = points.pop(int(parts[1]) - 1)
                    store_points(args.points, points, size)
                    print(f"  Deleted point {parts[1]} (J1={removed['j1']} J2={removed['j2']}); "
                          f'the points after it are renumbered. {len(points)} left.')
                    report(points, joint_calibration)
                else:
                    print(f'  Type delete and a point number from 1 to {len(points)}, e.g. delete 5')
                ask_next()
                continue
            if text in ('u', 'undo'):
                if points:
                    points.pop()
                    store_points(args.points, points, size)
                    print(f'  Removed last point; {len(points)} left.')
                    report(points, joint_calibration)
                else:
                    print('  Nothing to undo.')
                ask_next()
                continue
            if text in ('s', 'save'):
                if len(points) < 5:
                    print(f'  Need at least 5 points before saving (you have {len(points)}).')
                    ask_next()
                    continue
                print('\nOptional: the drop pose (the rightmost round hole in the tray).')
                print('  To record it: in Processing, hold the cylinder and lower it into the hole where it')
                print('  should be released, then type J1 J2 J3 Z (example: -23 4 0 10).')
                print('  To keep the one already saved in taught_positions.json: just press Enter.')
                mode = 'drop'
                console.prompt('Drop J1 J2 J3 Z (or Enter to skip) > ')
                continue
            joints, problem = parse_ints(text, 2, GUI_LIMITS[:2])
            if problem:
                print('  ' + problem)
                ask_next()
                continue
            pending, pending_until = joints, time.monotonic() + 3.0
    finally:
        camera.release()
        cv2.destroyAllWindows()


def save(points, size, output, settings, joint_calibration, drop_pose):
    data = joint_calibration.build(points, size, drop_pose)
    if output.exists():
        backup = output.with_suffix('.previous.json')
        backup.write_text(output.read_text())
        print(f'Previous calibration kept as {backup.name}')
    output.write_text(json.dumps(data, indent=2) + '\n')
    print(f"\nSaved {output.name}: RMS {data['rms_fit_error_mm']} mm, "
          f"worst leave-one-out {data['max_leave_one_out_error_mm']} mm")
    limit = settings['max_calibration_error_mm']
    if data['max_leave_one_out_error_mm'] > limit:
        print(f'WARNING: above the {limit} mm limit; the runner will refuse it. Recheck the worst points '
              '(typo, arm in view, camera bumped) and recalibrate.')
    else:
        print('Good. Next: python run_vision_pick_place.py --camera 1   (preview, robot does not move)')


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        print('\nQuit without saving.')
