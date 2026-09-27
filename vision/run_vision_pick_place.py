"""Camera-guided pick and place: find the blue cylinder, pick it, drop it in the hole.
"""

import argparse
import json
import math
from pathlib import Path
import time

import vision_kinematics as vk

HERE = Path(__file__).parent


# ---------------------------------------------------------------- planning
def load_setup(settings_path, calibration_path, taught_path, frame_size=None, ignore_quality=False):
    import joint_calibration
    settings = json.loads(Path(settings_path).read_text())
    calibration = joint_calibration.load(calibration_path, frame_size)
    limit = settings['max_calibration_error_mm']
    loo = calibration.get('max_leave_one_out_error_mm')
    if not ignore_quality and (loo is None or loo > limit):
        raise ValueError(f'Calibration worst leave-one-out error {loo} mm exceeds {limit} mm; recalibrate')
    drop = calibration.get('drop_pose')
    drop_source = 'calibration_joint.json'
    if drop is None:
        taught = json.loads(Path(taught_path).read_text())['placement']
        drop = {**taught['joint_angles_deg'], 'z': taught['z_mm']}
        drop_source = 'taught_positions.json'
    return settings, calibration, drop, drop_source


def plan_from_pixel(u, v, settings, calibration, drop):
    """Returns (plan dict) or raises ValueError with the reason the target is rejected."""
    import joint_calibration
    if not joint_calibration.inside_hull(calibration['hull'], (u, v), settings.get('hull_margin_px', 0)):
        raise ValueError('Outside the calibrated area (magenta outline)')
    x, y = joint_calibration.pixel_to_xy(calibration['matrix'], u, v)
    drop_xy = vk.forward(drop['j1'], drop['j2'])
    if math.dist((x, y), drop_xy) < settings['drop_exclusion_mm']:
        raise ValueError('Too close to the drop hole (probably an already placed cylinder)')
    # Prefer the elbow side and joint values of the nearest calibration placement: those poses
    # were measured, so model errors (arm lengths, J2 zero) stay small near them.
    nearest = min(calibration['points'], key=lambda p: math.dist(p['pixel'], (u, v)))
    j1, j2, error = vk.integer_joints(x, y, settings['elbow_sign'], settings['max_rounding_error_mm'],
                                      prefer=(nearest['j1'], nearest['j2']))
    rows = vk.plan_cycle((j1, j2), drop, settings['park_pose'], settings['travel_z_mm'], settings['pick_z_mm'],
                         settings['pick_j3'], settings['gripper_open'], settings['gripper_hold'])
    return {'pixel': (u, v), 'xy': (x, y), 'joints': (j1, j2), 'rounding_mm': error, 'rows': rows}


def describe(plan, settings):
    park = settings['park_pose']
    x, y = plan['xy']
    lines = [f"Cylinder at X={x:.1f} Y={y:.1f} mm -> J1={plan['joints'][0]} J2={plan['joints'][1]} "
             f"(whole-degree error {plan['rounding_mm']:.1f} mm)",
             f"{'Step':<24}{'J1':>5}{'J2':>5}{'J3':>5}{'Z':>5}{'Grip':>6}   packet (relative to park)   wait"]
    previous = None
    for label, *values in plan['rows']:
        row = (label, *values)
        wait = vk.estimate_step_seconds(previous, row, park, settings['timing'])
        lines.append(f"{label:<24}" + ''.join(f'{v:>5}' for v in values[:4]) + f'{values[4]:>6}   '
                     f"{vk.packet_for(row, park).decode():<27} {wait:4.1f}s")
        previous = row
    return '\n'.join(lines)


# ---------------------------------------------------------------- camera
def wait_for_target(camera, settings, calibration, drop, cv2, np, prompt):
    """Live view until the user presses G on a valid stable target (returns plan) or Q (returns None)."""
    from cylinder_detection import StableTarget, find_candidates
    stability = StableTarget(settings['stability']['frames'], settings['stability']['tolerance_px'])
    hull = np.asarray(calibration['hull'], np.int32)
    title = 'Vision pick and place - G go, Q quit'
    while True:
        ok, frame = camera.read()
        if not ok:
            raise RuntimeError('Camera stopped producing frames')
        size = (frame.shape[1], frame.shape[0])
        if tuple(calibration['image_size']) != size:
            raise RuntimeError('Camera resolution differs from the calibration')
        candidates, _ = find_candidates(frame, settings['detection'], cv2, np)
        plan = reason = None
        chosen = None
        for candidate in candidates:  # largest valid blob wins
            try:
                plan = plan_from_pixel(candidate['u'], candidate['v'], settings, calibration, drop)
                chosen = candidate
                break
            except ValueError as error:
                reason = reason or str(error)
        stable = stability.update((chosen['u'], chosen['v']) if chosen else None)
        if stable is not None:
            try:
                plan = plan_from_pixel(*stable, settings, calibration, drop)
            except ValueError as error:
                plan, stable, reason = None, None, str(error)
        cv2.polylines(frame, [hull], True, (255, 0, 255), 1)
        for candidate in candidates:
            color = (0, 220, 0) if candidate is chosen and stable else (0, 180, 255) if candidate is chosen else (0, 0, 255)
            cv2.drawContours(frame, [candidate['contour']], -1, color, 2)
        if chosen and stable and plan:
            x, y = plan['xy']
            lines = [f"READY  X {x:.1f}  Y {y:.1f} mm  J1 {plan['joints'][0]}  J2 {plan['joints'][1]}", prompt]
            color = (0, 220, 0)
        elif chosen:
            lines, color = ['Waiting for a stationary target...'], (0, 180, 255)
        else:
            lines, color = [reason or 'No blue cylinder detected'], (0, 0, 255)
        for i, line in enumerate(lines):
            cv2.putText(frame, line, (10, 22 + 24 * i), cv2.FONT_HERSHEY_SIMPLEX, .55, color, 2)
        cv2.imshow(title, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            return None
        if key == ord('g') and chosen and stable and plan:
            return plan


def keep_window_alive(cv2, seconds):
    """Sleep while keeping the camera window responsive. Returns True if Q/Esc was pressed."""
    end = time.monotonic() + seconds
    quit_requested = False
    while time.monotonic() < end:
        if cv2 is not None:
            if (cv2.waitKey(50) & 0xFF) in (ord('q'), 27):
                quit_requested = True
        else:
            time.sleep(min(0.05, max(0.0, end - time.monotonic())))
    return quit_requested


# ---------------------------------------------------------------- motion
def ask(phrase, message, read=input):
    print(message)
    return read(f'Type {phrase} to continue, or anything else to stop: ').strip().upper() == phrase


def send(port, payload):
    if port.write(payload) != len(payload):
        raise RuntimeError('Partial packet written; stopping without retry')
    port.flush()


def run_cycle(port, plan, settings, auto, read=input, cv2=None, wait=None, progress=None):
    """Send one cycle. Returns (finished, last_row_sent). progress['last'] tracks the last row sent."""
    progress = {} if progress is None else progress
    wait = wait or (lambda seconds: keep_window_alive(cv2, seconds))
    park = settings['park_pose']
    previous = None
    for index, row in enumerate(plan['rows'], 1):
        seconds = vk.estimate_step_seconds(previous, row, park, settings['timing'])
        print(f"Step {index}/{len(plan['rows'])}: {row[0]}  J1/J2/J3/Z={row[1:5]} grip={row[5]}")
        if not auto and not ask('MOVE', '  Check the path is clear.', read):
            return False, previous
        send(port, vk.packet_for(row, park))
        previous = progress['last'] = row
        stop = wait(seconds)
        if auto:
            if stop and index < len(plan['rows']):
                print('  Q pressed: no further steps will be sent.')
                return False, previous
        elif not ask('DONE', '  Continue only once the robot has stopped and the step succeeded.', read):
            return False, previous
    return True, previous


def recovery_rows(last, settings):
    """Raise to travel height at the current XY, then return to park, keeping the gripper as-is."""
    park = settings['park_pose']
    travel = settings['travel_z_mm']
    return [('Raise (recovery)', last[1], last[2], last[3], travel, last[5]),
            ('Park (recovery)', park['j1'], park['j2'], park['j3'], travel, last[5])]


# ---------------------------------------------------------------- main
def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--port', default='COM3')
    parser.add_argument('--execute', action='store_true', help='open the serial port and move the robot')
    parser.add_argument('--auto', action='store_true', help='timed steps after one G per cycle (needs --execute)')
    parser.add_argument('--test-pixel', type=float, nargs=2, metavar=('U', 'V'),
                        help='print the plan for a pixel, no camera or robot')
    parser.add_argument('--settings', type=Path, default=HERE / 'vision_pick_place.json')
    parser.add_argument('--calibration', type=Path, default=HERE / 'calibration_joint.json')
    parser.add_argument('--taught', type=Path, default=HERE / 'taught_positions.json')
    parser.add_argument('--ignore-calibration-quality', action='store_true')
    args = parser.parse_args(argv)
    if args.auto and not args.execute:
        parser.error('--auto only applies with --execute')

    try:
        settings, calibration, drop, drop_source = load_setup(
            args.settings, args.calibration, args.taught, ignore_quality=args.ignore_calibration_quality)
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(1, f'Setup problem: {error}\nRun calibrate_by_placing.py first (see VISION_PICK_PLACE.md).\n')
    park = settings['park_pose']
    print(f"Calibration: RMS {calibration['rms_fit_error_mm']} mm, worst leave-one-out "
          f"{calibration['max_leave_one_out_error_mm']} mm")
    print(f"Drop pose from {drop_source}: J1={drop['j1']} J2={drop['j2']} J3={drop['j3']} Z={drop['z']}")
    print(f"Park pose (controller zero): J1={park['j1']} J2={park['j2']} J3={park['j3']} Z={park['z']}")

    if args.test_pixel:
        try:
            print(describe(plan_from_pixel(*args.test_pixel, settings, calibration, drop), settings))
        except ValueError as error:
            parser.exit(1, f'Target rejected: {error}\n')
        return 0

    try:
        import cv2
        import numpy as np
    except ImportError as error:
        parser.exit(1, f'Missing dependency: {error}\nRun: python -m pip install opencv-python numpy\n')
    camera = cv2.VideoCapture(args.camera)
    if not camera.isOpened():
        parser.exit(1, f'Cannot open camera {args.camera}; try --camera 1.\n')
    width, height = calibration['image_size']
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    try:
        if not args.execute:
            print('\nPREVIEW: no serial port. Press G on a green target to print its plan, Q to quit.')
            while True:
                plan = wait_for_target(camera, settings, calibration, drop, cv2, np, 'G = print plan (preview)')
                if plan is None:
                    return 0
                print('\n' + describe(plan, settings))
        return execute(args, camera, settings, calibration, drop, cv2, np)
    finally:
        camera.release()
        cv2.destroyAllWindows()


def execute(args, camera, settings, calibration, drop, cv2, np):
    try:
        import serial
    except ImportError:
        print('Install the serial dependency first: python -m pip install pyserial')
        return 1
    park = settings['park_pose']
    print('\nBEFORE CONNECTING:')
    print(f"1. Arm at the park pose (GUI J1={park['j1']} J2={park['j2']} J3={park['j3']} Z={park['z']}), gripper empty.")
    print('2. Close the Processing window so the serial port is free.')
    print('3. Physical power cutoff within reach. This script cannot stop a move in progress.')
    if not ask('PARKED', 'Confirm the arm is stationary at the park pose.'):
        return 0
    port = None
    progress = {'last': None}
    try:
        port = serial.Serial(args.port, 115200, timeout=0.2, write_timeout=2)
        if not ask('RESET', 'Press and RELEASE the Arduino RESET button (arm stays put), then type RESET.'):
            return 0
        time.sleep(3.0)  # bootloader + setup(); first loop makes the park pose zero
        if not ask('UNCHANGED', 'Confirm the arm and Z did not shift during reset.'):
            return 0
        cycle = 0
        while True:
            print('\nPlace the cylinder in the calibrated area and clear your hands. G in the camera window to pick, Q to quit.')
            plan = wait_for_target(camera, settings, calibration, drop, cv2, np,
                                   'G = PICK AND PLACE' + (' (AUTO)' if args.auto else ' (step by step)'))
            if plan is None:
                print('Stopped at park.')
                return_to_gui_zero(port, settings, cv2)
                return 0
            cycle += 1
            print(f'\nCycle {cycle}\n' + describe(plan, settings))
            done, last = run_cycle(port, plan, settings, args.auto, cv2=cv2, progress=progress)
            if not done:
                return recover(port, last, settings, cv2)
            progress['last'] = None
            print('Cycle complete; arm back at park. Remove the cylinder from the hole before the next cycle.')
    except (serial.SerialException, OSError, RuntimeError) as error:
        print(f'Stopped: {error}. No automatic retry.')
        return 1
    except KeyboardInterrupt:
        print('\nCtrl+C: no further commands. A move already sent may still finish.')
        if port is not None and port.is_open and progress['last'] is not None:
            return recover(port, progress['last'], settings, cv2)
        return 1
    finally:
        if port is not None and port.is_open:
            port.close()
        print('If the arm is not at park now, move it there before reopening Processing: Processing resets '
              'the controller and makes wherever the arm is the new zero.')


def return_to_gui_zero(port, settings, cv2, read=input):
    """With a non-zero park pose, offer to finish at GUI zero so Processing's frame stays valid."""
    park = settings['park_pose']
    if all(park[k] == 0 for k in ('j1', 'j2', 'j3', 'z')):
        return False
    row = ('GUI zero', 0, 0, 0, settings['travel_z_mm'], settings['gripper_open'])
    vk.check_rows([row])
    start = ('Park', park['j1'], park['j2'], park['j3'], park['z'], settings['gripper_open'])
    if not ask('HOME', 'Move the arm to GUI zero (J1=J2=J3=0) so reopening Processing keeps the same frame?', read):
        print('Arm left at park. Processing will treat the park pose as zero if reopened now.')
        return False
    send(port, vk.packet_for(row, park))
    keep_window_alive(cv2, vk.estimate_step_seconds(start, row, park, settings['timing']))
    print('Arm sent to GUI zero.')
    return True


def recover(port, last, settings, cv2):
    if last is None:
        print('Nothing was sent this cycle; arm is still at park.')
        return 0
    print(f'\nStopped mid-cycle at: {last[0]} (J1/J2/J3/Z={last[1:5]}, grip={last[5]}).')
    print('Recovery raises to travel height, then returns to park. The gripper stays as it is.')
    try:
        answer = input('Type PARK to send recovery moves, anything else to leave the arm where it is: ')
    except (KeyboardInterrupt, EOFError):
        answer = ''
    if answer.strip().upper() != 'PARK':
        print('Left in place. Jog it back to park manually before trusting any coordinates.')
        return 1
    previous = last
    for row in recovery_rows(last, settings):
        print(f'{row[0]}: J1/J2/J3/Z={row[1:5]}')
        send(port, vk.packet_for(row, settings['park_pose']))
        keep_window_alive(cv2, vk.estimate_step_seconds(previous, row, settings['park_pose'], settings['timing']))
        previous = row
    print('Recovery sent; confirm by eye that the arm is at park.')
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
