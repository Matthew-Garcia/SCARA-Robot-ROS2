# Connecting the original Arduino later

The ROS workspace does not write to or replace `firmware/arduino/SCARA_Robot/SCARA_Robot.ino`. Gazebo and mock execution do not connect to a serial device.

The existing firmware expects a ten-field comma-separated GUI packet at 115200 baud. Fields 2–5 carry shoulder degrees, elbow degrees, wrist degrees and Z millimetres; field 6 controls the servo. The ROS joint order differs: shoulder, Z, elbow, wrist.

| Firmware constant | Existing value |
| --- | --- |
| Shoulder conversion | 44.444444 steps/degree |
| Elbow conversion | 35.555555 steps/degree |
| Wrist conversion | 10 steps/degree |
| Z conversion | 100 steps/mm |

These are source-code constants, not verified calibration results. They do not prove independent motor/joint correspondence in the belt-driven mechanism.

A reliable physical ros2_control integration requires a separately reviewed firmware addition with nonblocking step generation, bounded command parsing, sequence acknowledgments, periodic state reporting, a communication watchdog, verified motor signs and offsets, and a real homing procedure. Open-loop step counts must be labeled estimated positions; encoder feedback must not be implied. The existing `homing()` only assigns zeros, and the first GUI synchronization also resets software position. Its STOP behavior returns to a prior manual position; that is not an emergency stop.

Then implement a `hardware_interface::SystemInterface` using SI units and the actual protocol, validate one joint at a time, confirm travel limits and transmission coupling, and enable trajectory execution only after tracking and stop behavior have been tested. A physical stop should remain independent of ROS. No untested serial bridge is presented as working hardware control in this workspace.
