"""Preview by default; --execute enables a supervised, fixed-position trial.

No camera coordinates are used. No firmware or Processing changes are required.
The user establishes a new controller zero at the known raised pickup pose.
"""

import argparse
import json
from pathlib import Path
import time

from preview_pick_place import build_sequence


def prepare(settings):
    rows = build_sequence(settings)
    pickup = settings['confirmed_pickup']['joint_angles_deg']
    reference = (pickup['j1'], pickup['j2'], pickup['j3'], settings['travel_z_mm'])
    # Bounds are the existing GUI limits, not a collision or reachability check.
    for row in rows:
        for value, lower, upper in zip(row[1:5], (-90, -150, -162, -50), (266, 150, 162, 50)):
            if type(value) is not int or not lower <= value <= upper:
                raise ValueError('A taught position exceeds the existing GUI limits')
        if not -180 <= row[5] <= 230:
            raise ValueError('Gripper command exceeds the pasted GUI command range')
    if settings['travel_z_mm'] != 0:
        raise ValueError('This trial requires the confirmed travel height Z=0')
    return reference, rows


def packet_for(row, reference):
    # Firmware zero is now the stationary raised pickup, not the old GUI zero.
    relative = [target - origin for target, origin in zip(row[1:5], reference)]
    # The last two fields preserve the protocol. Manual firmware motion ignores
    # these speed/acceleration fields and uses its startup settings (4000/2000).
    fields = [0, 0, *relative, row[5], 500, 500]
    return ','.join(map(str, fields)).encode('ascii')


def send_once(port, payload, sleep=time.sleep):
    count = port.write(payload)
    if count != len(payload):
        raise RuntimeError('Partial packet: stop trial; do not retry or resume')
    # The Arduino parses using readString() timeout, not newline framing.
    # This is a packet-separation delay, NOT a movement-complete indication.
    sleep(2.0)


def ask(phrase, message, read=input):
    print(message)
    return read(f'Type {phrase} to continue, or anything else to end: ').strip() == phrase


def supervised_steps(port, reference, rows, read=input, sleep=time.sleep):
    for index, row in enumerate(rows, 1):
        print(f'\nStep {index}/{len(rows)}: {row[0]}')
        print(f'Taught J1/J2/J3/Z: {row[1:5]}; gripper command: {row[5]}')
        if not ask('MOVE', 'Check that this next move has a clear path.', read):
            return False
        send_once(port, packet_for(row, reference), sleep)
        if not ask('DONE', 'Watch the robot. Continue only after it has stopped and the step succeeded.', read):
            return False
    return True


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--settings', type=Path, default=Path(__file__).with_name('taught_positions.json'))
    parser.add_argument('--port', default='COM3')
    parser.add_argument('--execute', action='store_true', help='connect and enable individually confirmed movements')
    args = parser.parse_args(argv)
    try:
        reference, rows = prepare(json.loads(args.settings.read_text()))
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(1, f'Invalid taught settings: {error}\n')
    print('Fixed-position trial. Camera calibration is NOT used.')
    print(f'Required starting pose in the CURRENT taught GUI frame: J1/J2/J3/Z = {reference}')
    for index, row in enumerate(rows, 1):
        print(f'{index}. {row[0]:23} J1/J2/J3/Z={row[1:5]} gripper={row[5]}')
    if not args.execute:
        print('\nPREVIEW ONLY. No serial dependency loaded, no port opened, no robot commands sent.')
        print('Read RUN_TAUGHT_TRIAL.md before enabling --execute.')
        return

    try:
        import serial
    except ImportError:
        parser.exit(1, 'Install the serial dependency first: python -m pip install pyserial\n')
    print('\nPREPARATION using the still-running Processing session:')
    print('1. Put down any held cylinder, open the gripper, and raise Z to 0.')
    print(f'2. Jog the empty gripper to J1/J2/J3/Z = {reference}.')
    print('3. Confirm this is physically above the taught pickup, with clearance.')
    print('4. Keep the cylinder out from between the fingers during reset.')
    print('5. Close the running Processing control window to release the serial port.')
    print('If the controller already restarted since teaching, STOP: the old GUI readings may be wrong.')
    print('Opening the port and resetting can open/close the gripper. Keep it empty.')
    print('Movement uses firmware manual speeds (4000 steps/s, acceleration 2000), not GUI 500.')
    print('This script has NO emergency stop. Quitting prevents later commands, but cannot stop an active move.')
    if not ask('PARKED', 'Confirm the empty robot is stationary at the required physical pose.'):
        print('Cancelled before opening port.')
        return

    # Do not rely on OS-specific DTR behavior to establish the coordinate origin.
    # Explicit physical RESET after opening establishes a known zero every trial.
    try:
        with serial.Serial(args.port, 115200, timeout=0.2, write_timeout=2) as port:
            if not ask('RESET', 'Press and RELEASE the Arduino RESET button now, without moving the arm. Then type RESET.'):
                return
            time.sleep(3.0)  # setup delay and initial zero-position loop must finish
            if not ask('UNCHANGED', 'Confirm arm and Z have not shifted from the parked pose. Gripper should now be open. If anything shifted, end the trial.'):
                return
            print('Place the cylinder at the taught pickup, below the raised open fingers. Clear your hands.')
            print('After a reset, loss of power, or disconnect during this trial, END it; never resume.')
            complete = supervised_steps(port, reference, rows)
            print('Trial finished: all steps confirmed.' if complete else 'Trial ended; no further commands will be sent.')
    except (serial.SerialException, OSError, RuntimeError) as error:
        print(f'Trial ended: {error}. No automatic reconnect or retry.')
        return 1
    finally:
        print('The controller now uses the parked pickup as zero. Do not resume the old GUI coordinates without re-establishing its reference.')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (KeyboardInterrupt, EOFError):
        print('\nEnded. No later commands will be sent; an active robot move may still finish.')
