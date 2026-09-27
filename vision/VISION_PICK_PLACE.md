# Camera-guided pick and place

The camera finds the blue cylinder, the runner converts its position to whole-degree
J1/J2, and the robot picks it and drops it in the hole. Each cycle starts and ends at
the park pose. Firmware and Processing are unchanged, and everything here lives in `vision`.

| File | Purpose |
| --- | --- |
| `calibrate_by_placing.py` | New calibration: the robot places the cylinder, and you type the J1/J2 the GUI shows |
| `run_vision_pick_place.py` | Preview, then step-by-step, then timed pick and place |
| `vision_pick_place.json` | Park pose, heights, gripper values, detection thresholds, timing |
| `calibration_joint.json` | Written by the calibration script (the old `calibration.json` is left alone) |
| `vision_kinematics.py`, `joint_calibration.py`, `cylinder_detection.py` | Helpers |
| `test_vision_pick_place.py` | Tests with no camera or robot connected |

Install once: `python -m pip install opencv-python numpy pyserial`

## 0. Park pose: one repeatable start

The robot has no limit switches, so the zero is wherever the arm sits when the
controller powers on or resets. By default the park pose is GUI all-zeros, which is
that power-on pose. Mark the pose physically (a pencil line or a printed jig) and
always power on and reset with the arm there. Calibration, the taught drop pose and
the runner all depend on this frame.

The camera must be able to see the cylinder while the arm is parked. If the parked arm
covers the pick area, set `park_pose` in `vision_pick_place.json` to a pose that is
clear of the view. The runner then offers to send the arm back to GUI zero when you quit.

## 1. Tune detection (only if needed)

Run `python blue_cylinder_preview.py --pixels-only`, adjust the sliders until only the
cylinder shows white, and copy the values into `"detection"` in `vision_pick_place.json`.

## 2. Calibrate (about 10 minutes, and again whenever the camera moves)

Power on at the park pose, open Processing, then run:

```powershell
cd C:\Users\matt1\Documents\SCARA-Robot-ROS2\vision
python calibrate_by_placing.py
```

Repeat these steps for 6–8 spots spread over the whole pick area. Include the far edge
near full reach and the corners:

1. In Processing, grip the cylinder, move at Z=0 to a new spot, lower it to the table, release, and raise to Z=0.
2. Note the J1 and J2 the GUI shows.
3. Jog the arm out of the camera's view. Once the outline turns green, press **C** and type `J1 J2`.

Press **S** to save. The script prints the worst leave-one-out error, which is how
far off a point lands when it is left out of the fit. That number is the honest
accuracy estimate. The runner refuses a calibration worse than 5 mm
(`max_calibration_error_mm`). When saving, you can also record the hole's release
pose (`J1 J2 J3 Z`) in the same session. If you skip it, the placement from
`taught_positions.json` is used.

Close Processing and return the arm to park before running.

## 3. Preview (no robot motion)

```powershell
python run_vision_pick_place.py
```

A green outline means the target is stable, inside the calibrated area (magenta),
reachable, and not sitting in the hole. Press **G** to print the 10-step plan with the
exact packets. Place the cylinder at your old taught pickup: the plan should show
J1 ≈ -14, J2 ≈ 2.

## 4. First real runs: step by step

```powershell
python run_vision_pick_place.py --execute --port COM3
```

The prompts are PARKED, then RESET (press and release the Arduino button), then
UNCHANGED. After that, press **G** in the camera window and type MOVE, then DONE, for
each step. Check "Above cylinder" carefully: if the fingers are not centered over the
cylinder, stop and recalibrate.

## 5. Timed runs

```powershell
python run_vision_pick_place.py --execute --port COM3 --auto
```

You confirm once per cycle with **G**, and the steps are then paced by estimated move
time. **Q** in the camera window stops after the current step. After a stop, you can
type PARK to raise the arm and return it to park. Remove the cylinder from the hole
before starting the next cycle, because there is only one drop hole.

## Limits

- J2 direction: on the real robot a positive GUI J2 bends the elbow the opposite way to the GUI's X/Y
  display (the GUI's own IK negates it too). `vision_kinematics.J2_DIRECTION = -1` accounts for this;
  ignore the GUI's X/Y readout. Calibration points autosave to `calibration_points.json` and can be resumed.

- The firmware sends no "move finished" message. Timed mode waits for an estimate
  (a trapezoid profile at 4000 steps/s and 2000 steps/s², the 1 s `readString` timeout,
  a servo delay, and a 1.5× safety factor). If moves ever overlap, raise
  `timing.safety_factor`.
- The software cannot stop a move that is already in progress. Keep the power
  cutoff within reach.
- The firmware only accepts whole degrees. The runner picks the closest whole-degree
  pose and rejects any target that pose misses by more than 3 mm.
- J3 stays at `pick_j3` (0, as taught). Paths between poses are not collision-checked.
- Do not open Processing while the arm is away from its GUI zero. Processing resets
  the controller, and whatever pose the arm is in becomes the new zero.

Tests: `python -B -m unittest discover -s . -p "test_*.py"`
