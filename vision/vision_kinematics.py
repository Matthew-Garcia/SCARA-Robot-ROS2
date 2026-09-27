"""Pure-Python SCARA kinematics and pick/place planning. No camera, no serial.

Angles are in the Processing GUI joint frame (degrees, integer on the wire),
Z in GUI millimetres (larger Z = lower gripper; travel height is Z=0).
Geometry and step constants come from the unchanged GUI and firmware.
"""

import math

L1 = 228.0     # mm, GUI / CAD
L2 = 136.5     # mm, GUI / CAD
# Firmware constants: J1, J2, J3 steps per degree, Z steps per mm.
STEPS_PER_UNIT = (44.444444, 35.555555, 10.0, 100.0)
# Existing GUI slider limits (not a collision check).
GUI_LIMITS = ((-90, 266), (-150, 150), (-162, 162), (-50, 50))
GRIPPER_RANGE = (-180, 230)
# Measured on the real robot (calibration, 2026-09-27): a positive GUI J2 turns the
# elbow the opposite way to the GUI's forward-kinematics display. The GUI's own
# inverse kinematics also negates theta2. So the physical elbow angle is -J2.
J2_DIRECTION = -1


def forward(j1, j2):
    """Physical tool X, Y in mm for GUI joint values (J2 direction corrected)."""
    a, b = math.radians(j1), math.radians(J2_DIRECTION * j2)
    return (L1 * math.cos(a) + L2 * math.cos(a + b),
            L1 * math.sin(a) + L2 * math.sin(a + b))


def _wrap_j1(angle):
    # Bring J1 into the GUI range (-90..266) when an equivalent angle exists.
    while angle < GUI_LIMITS[0][0]:
        angle += 360.0
    while angle > GUI_LIMITS[0][1]:
        angle -= 360.0
    return angle


def inverse(x, y, elbow_sign):
    """Continuous IK for one elbow branch, in GUI joint values.

    elbow_sign is the sign of the GUI J2 value (+1 or -1). Returns (j1, j2) or None if unreachable.
    """
    if elbow_sign not in (1, -1):
        raise ValueError('elbow_sign must be 1 or -1')
    c = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if not -1.0 <= c <= 1.0:
        return None
    b = J2_DIRECTION * elbow_sign * math.acos(c)          # physical elbow angle
    a = math.atan2(y, x) - math.atan2(L2 * math.sin(b), L1 + L2 * math.cos(b))
    return _wrap_j1(math.degrees(a)), J2_DIRECTION * math.degrees(b)


def integer_joints(x, y, elbow_sign, max_error_mm, prefer=(0, 0)):
    """Best whole-degree (J1, J2) for a target, since the firmware only accepts whole degrees.

    elbow_sign: +1 or -1 to force the sign of GUI J2, or 0 to allow both and take the
    solution closest to `prefer` (normally the park pose). Returns (j1, j2, error_mm).
    Raises ValueError if unreachable, outside the GUI limits, or if whole-degree
    rounding misses the target by more than max_error_mm.
    """
    signs = (1, -1) if elbow_sign == 0 else (elbow_sign,)
    branch_best = []     # most accurate whole-degree pose on each elbow side
    for sign in signs:
        solution = inverse(x, y, sign)
        if solution is None:
            continue
        a, b = solution
        best = None
        for ia in range(math.floor(a) - 1, math.ceil(a) + 2):
            for ib in range(math.floor(b) - 1, math.ceil(b) + 2):
                if sign * ib < 0:
                    continue
                if not (GUI_LIMITS[0][0] <= ia <= GUI_LIMITS[0][1] and GUI_LIMITS[1][0] <= ib <= GUI_LIMITS[1][1]):
                    continue
                fx, fy = forward(ia, ib)
                option = (math.hypot(fx - x, fy - y), ia, ib)
                if best is None or option < best:
                    best = option
        if best is not None:
            branch_best.append(best)
    if not branch_best:
        radius = math.hypot(x, y)
        if not abs(L1 - L2) <= radius <= L1 + L2:
            raise ValueError(f'Target ({x:.1f}, {y:.1f}) is out of reach: radius {radius:.1f} mm, '
                             f'arm reaches {abs(L1 - L2):.1f}-{L1 + L2:.1f} mm')
        raise ValueError('No whole-degree solution inside the GUI joint limits')
    good = [o for o in branch_best if o[0] <= max_error_mm]
    if not good:
        raise ValueError(f'Whole-degree rounding error {min(branch_best)[0]:.1f} mm exceeds {max_error_mm} mm')
    # Between the two elbow sides, take the one needing less joint travel from `prefer`.
    error, j1, j2 = min(good, key=lambda o: (abs(o[1] - prefer[0]) + abs(o[2] - prefer[1]), o[0]))
    return j1, j2, error


def plan_cycle(pick, drop, park, travel_z, pick_z, pick_j3, grip_open, grip_hold):
    """Absolute rows (label, j1, j2, j3, z, gripper) for one pick-and-place cycle.

    pick: (j1, j2). drop, park: dicts with j1, j2, j3, z.
    Starts and ends at the park pose so the controller zero stays valid.
    """
    pj1, pj2 = pick
    dj = (drop['j1'], drop['j2'], drop['j3'])
    kj = (park['j1'], park['j2'], park['j3'])
    rows = [
        ('Open gripper at park', *kj, travel_z, grip_open),
        ('Above cylinder', pj1, pj2, pick_j3, travel_z, grip_open),
        ('Lower to cylinder', pj1, pj2, pick_j3, pick_z, grip_open),
        ('Grip cylinder', pj1, pj2, pick_j3, pick_z, grip_hold),
        ('Lift cylinder', pj1, pj2, pick_j3, travel_z, grip_hold),
        ('Above hole', *dj, travel_z, grip_hold),
        ('Lower into hole', *dj, drop['z'], grip_hold),
        ('Release cylinder', *dj, drop['z'], grip_open),
        ('Raise empty gripper', *dj, travel_z, grip_open),
        ('Return to park', *kj, travel_z, grip_open),
    ]
    check_rows(rows)
    return rows


def check_rows(rows):
    for row in rows:
        values = row[1:6]
        if any(type(v) is not int for v in values):
            raise ValueError(f'{row[0]}: joint, Z and gripper values must be integers')
        for value, (lower, upper) in zip(values[:4], GUI_LIMITS):
            if not lower <= value <= upper:
                raise ValueError(f'{row[0]}: {values[:4]} exceeds the GUI limits')
        if not GRIPPER_RANGE[0] <= values[4] <= GRIPPER_RANGE[1]:
            raise ValueError(f'{row[0]}: gripper command {values[4]} out of range')


def packet_for(row, park):
    """Firmware packet, relative to the park pose (controller zero after reset)."""
    origin = (park['j1'], park['j2'], park['j3'], park['z'])
    relative = [target - zero for target, zero in zip(row[1:5], origin)]
    # Manual firmware mode ignores the last two (speed/accel) fields.
    return ','.join(map(str, [0, 0, *relative, row[5], 500, 500])).encode('ascii')


def _firmware_steps(row, park):
    origin = (park['j1'], park['j2'], park['j3'], park['z'])
    # Firmware: int steps = data * constant (truncation toward zero).
    return [int((v - o) * k) for v, o, k in zip(row[1:5], origin, STEPS_PER_UNIT)]


def move_seconds(steps, max_speed, accel):
    """Trapezoidal / triangular profile time for an AccelStepper move."""
    steps = abs(steps)
    if steps == 0:
        return 0.0
    ramp_steps = max_speed * max_speed / accel      # accelerate + decelerate
    if steps >= ramp_steps:
        return steps / max_speed + max_speed / accel
    return 2.0 * math.sqrt(steps / accel)


def estimate_step_seconds(previous, row, park, timing):
    """Conservative wait before the next packet may be sent.

    The firmware reads with Serial.readString() (1 s timeout), moves all axes in a
    blocking loop, then writes the servo and delays 200 ms. It sends no
    completion message, so this estimate is the only pacing available.
    """
    before = _firmware_steps(previous, park) if previous is not None else [0, 0, 0, 0]
    after = _firmware_steps(row, park)
    motion = max(move_seconds(b - a, timing['max_speed_steps_s'], timing['accel_steps_s2'])
                 for a, b in zip(before, after))
    gripper = timing['gripper_settle_s'] if previous is None or previous[5] != row[5] else 0.0
    total = timing['serial_parse_s'] + motion + timing['firmware_delay_s'] + gripper
    return total * timing['safety_factor'] + timing['margin_s']
