# Supervised taught-position trial

This is one fixed-position cycle, not camera-guided or unattended operation.
It leaves Processing and firmware unchanged. It never loops or retries moves.
The source was tested with a simulated serial port; real motion is unverified.

## Start with the preview

```powershell
cd C:\Users\matt1\Documents\SCARA-Robot-ROS2\vision
python run_taught_pick_place.py
```

No serial port is opened in preview mode. The eight steps use your recorded
pickup (-14, 2, 0, Z=11), travel Z=0, and drop (-23, 4, 0, Z=10).
Gripper commands are 164 to hold and -83 to release, from your console packets.
These are not GUI slider values.

## Establish the starting reference

Do this only while the same Processing/controller session used for teaching is
still valid. If it has restarted, the recorded numbers alone cannot identify a
physical pose; re-establish/reteach the positions first.

1. Using Processing, put down any held cylinder and open the gripper.
2. Raise Z to 0, then jog to J1=-14, J2=2, J3=0. Confirm the empty gripper is
   physically above the original pickup location and clear of objects.
3. Keep the cylinder away from the fingers during reset, because firmware
   startup changes the gripper. Do not manually shift any arm axis.
4. Close the running Processing robot-control window to release COM3.

## Enable the supervised trial

Install once if needed:

```powershell
python -m pip install pyserial
```

Then run:

```powershell
python run_taught_pick_place.py --execute --port COM3
```

The prompts require PARKED before opening the port. After the port is open,
press and release the Arduino's physical RESET button, then type RESET. The
program waits for startup. Confirm UNCHANGED only if the arm and Z stayed at
the parked pose. If resetting shifts an axis, end the trial and investigate.
Place the cylinder beneath the raised open fingers and clear your hands.

The new controller zero is the parked pickup pose. The runner subtracts that
pose from every taught target: pickup XY needs no arm rotation; drop commands
J1=-9, J2=2, J3=0 relative to the new zero. Integer step rounding may differ by
a step from the original absolute commands. This translation does not repair
the camera calibration or validate the travel path.

For each step, type MOVE to send exactly one packet, then watch it finish.
Type DONE only once the robot has stopped and the step succeeded. Any other
response ends the sequence. The two-second delay is for packet separation;
it is not proof of completion. The firmware supplies no completion feedback.

The unchanged firmware's manual mode ignores the packet speed/acceleration
fields and uses its startup values (4000 steps/s and 2000 steps/s squared).
The script cannot provide an immediate software stop: quitting prevents future
commands but an active motion can continue. Keep the physical power cutoff
accessible. Do not use RUN PROGRAM mode during this trial.

If the controller resets or disconnects during a cycle, end it. There is no
automatic reconnect or resume. Afterward, old Processing coordinates no longer
apply until its reference has been re-established; do not reopen it and replay
old absolute positions blindly.

The explicit reset avoids assuming that opening a serial port preserves the
controller origin. pySerial documents that operating systems/drivers may
change DTR/RTS during open: https://pyserial.readthedocs.io/en/latest/pyserial_api.html

## Tests (no robot connection)

```powershell
python -B -m unittest discover -s . -p "test_*.py"
```
