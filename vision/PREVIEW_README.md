# Standalone blue-cylinder preview

All additions live in `vision`. Processing and Arduino firmware are unchanged.
This program never opens a serial port or sends robot commands.

From PowerShell:

```powershell
cd C:\Users\matt1\Documents\SCARA-Robot-ROS2\vision
python -m pip install opencv-python numpy
python blue_cylinder_preview.py
```

If dependencies are already installed, skip the install command. If `python`
is unavailable but the Windows Python launcher is installed, use `py` instead.
For another camera use `--camera 1`. For detection without calibration use
`--pixels-only`. Press Q or Escape to exit.

The preview reads the existing `calibration.json` without changing it, requests
its camera resolution, and rejects a different actual resolution. The cyan
outline marks the region covered by the calibration points. The mask window
has live blue hue, saturation, brightness, minimum area and roundness sliders.
Tune until the cylinder is white and the background is black in the mask.
Settings are temporary and reset when the program restarts.

The largest qualifying blue contour is selected. The Area max slider defaults to
7000 pixels: your cylinder measured about 2380-2442 pixels, whereas the blue arm
measured about 29960-40412. Contours touching any camera-image edge are also
rejected. These checks reduce arm false detections; smaller visible pieces of
the arm can still resemble the target. An obscured cylinder should produce no
target, rather than a remembered coordinate. Restart the preview to load updates.

This is a color/shape heuristic,
not proof that the object is a cylinder. Circularity can be reduced by shadows,
occlusion and perspective; lower the roundness slider if needed. Other round blue
objects can also be selected. Eight consistent frames within eight pixels mark
a stationary candidate; target loss, a jump, or tuning changes reset that check.

Robot coordinates are estimates from the saved mapping. A green marker means a
stable detection within the calibration region and configured X/Y bounds. It
does not mean the pose is reachable or collision-free. Compare estimates with
several measured positions before using them for picking. Moving the camera
invalidates the mapping. Mapping a cylinder top with table-plane calibration
can introduce height-related position error; validate at the actual target height.

Unattended camera-guided pick and place is not implemented. A separate supervised
fixed-position runner is now available; see `RUN_TAUGHT_TRIAL.md` for the explicit
startup-reference procedure. User-taught heights, joint poses and console gripper
commands are recorded in `taught_positions.json`. The existing Processing sketch prints the outgoing
comma-separated packet when the gripper changes. Capture one packet after
opening and one after closing using the normal controls. The seventh field is
the gripper value actually sent. No GUI or firmware changes are needed to read
that console output. Motion sequencing still needs completion handling; the
current firmware does not send a movement-complete acknowledgement.

Preview the taught sequence (no camera or serial dependencies, no robot motion):

```powershell
python preview_pick_place.py
```

This assumes an empty gripper already raised above the fixed taught pickup.
It uses the measured joint poses instead of the inaccurate camera mapping.
It does not execute or validate a collision-free path. The unchanged firmware
zeros its internal positions at startup and does not acknowledge completed
moves; a separate motion client needs an established startup reference and
completion handling before this preview can become an executable routine.

Camera-independent checks:

```powershell
python -B -m unittest discover -s . -p "test_*.py"
```
