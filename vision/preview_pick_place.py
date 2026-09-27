"""Print a taught pick/place sequence. Never connects to a robot."""

import argparse
import json
from pathlib import Path


def build_sequence(settings):
    pickup = settings['confirmed_pickup']
    drop = settings['placement']
    raised = settings['travel_z_mm']
    closed = pickup['serial_gripper_hold']
    opened = drop['serial_gripper_release']
    if not pickup.get('user_confirmed_successful_grip'):
        raise ValueError('Pickup grip has not been confirmed')
    rows = []

    def step(label, pose, z, grip):
        angles = pose['joint_angles_deg']
        values = [angles['j1'], angles['j2'], angles['j3'], z, grip]
        if any(type(value) is not int for value in values):
            raise ValueError('Taught joint, Z and gripper values must be integers')
        rows.append((label, *values))

    # Preconditions: empty gripper, already above the taught pickup at travel Z.
    # No approach from an unknown current pose is synthesized here.
    step('Open above pickup', pickup, raised, opened)
    step('Lower to cylinder', pickup, pickup['z_mm'], opened)
    step('Grip cylinder', pickup, pickup['z_mm'], closed)
    step('Lift cylinder', pickup, raised, closed)
    step('Travel above hole', drop, raised, closed)
    step('Lower into hole', drop, drop['z_mm'], closed)
    step('Release cylinder', drop, drop['z_mm'], opened)
    step('Raise empty gripper', drop, raised, opened)
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--settings', type=Path,
                        default=Path(__file__).with_name('taught_positions.json'))
    args = parser.parse_args()
    rows = build_sequence(json.loads(args.settings.read_text()))
    print('PREVIEW ONLY -- no serial port is opened; no movement is commanded.\n')
    print('Assumed start: empty gripper above the TAUGHT pickup position at Z=0.')
    print('The cylinder must be at that same fixed position; camera correction is not applied.')
    print('This list does not verify the physical travel path.\n')
    print(f'{"Step":<27} {"J1":>5} {"J2":>5} {"J3":>5} {"Z":>5} {"Gripper*":>9}')
    for number, (label, *values) in enumerate(rows, 1):
        print(f'{str(number) + ". " + label:<27}' + ''.join(f'{value:>6}' for value in values))
    print('\n* Gripper values are controller commands, NOT Processing slider readings.')
    print('Motion is not enabled: startup reference and completion handling remain unresolved.')


if __name__ == '__main__':
    main()
