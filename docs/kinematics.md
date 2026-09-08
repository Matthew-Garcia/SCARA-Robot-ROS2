# SCARA frames and kinematics

The serial chain follows the source assembly:

`base_link → shoulder_link → arm1_link → arm2_link → tool_link → tcp_link`

| Joint | Parent → child | Axis | Units |
| --- | --- | --- | --- |
| `shoulder_joint` | base → rotating tower | +Z rotation | radians |
| `z_joint` | tower → vertical carriage and first arm | +Z translation | metres |
| `elbow_joint` | first arm → forearm | +Z rotation | radians |
| `wrist_joint` | forearm → original gripper | +Z rotation | radians |

Joint vectors use `[shoulder, slide, elbow, wrist]`. The arm lengths are 0.228 m and 0.1365 m, verified against the original CAD and Processing GUI. Zero slide position is the exported carriage height.

## Forward kinematics

For q = `[a, d, b, c]`:

```text
x = 0.228 cos(a) + 0.1365 cos(a + b)
y = 0.228 sin(a) + 0.1365 sin(a + b)
z = z0 + d
yaw = a + b + c
```

`z0 = 0.07255508189967187 m` is the defined TCP height at zero slide, measured in `base_link`. The TCP lies on the wrist axis at the lowest source-gripper height. Its local +Z points up; its X axis follows the forearm at zero wrist angle. This is a software reference, not a calibrated physical grasp frame.

## Inverse kinematics

```text
cos(b) = (x² + y² − L1² − L2²) / (2 L1 L2)
b = ± acos(cos(b))
a = atan2(y, x) − atan2(L2 sin(b), L1 + L2 cos(b))
d = z − z0
c = yaw − a − b
```

The implementation considers both elbow signs and bounded equivalents of the angles, then ranks solutions by distance to the seed. It checks joint limits, consistency limits and MoveIt's collision callbacks. Invalid positions and roll/pitch requests are rejected.

The unrestricted geometric radial interval is 0.0915–0.3645 m. Actual reachable targets are further limited by joint travel, collision geometry and orientation. Full extension is a singular configuration; the analytical solver can represent it, but that does not imply well-conditioned Cartesian velocity control there.

## Implementation

- `ros2_ws/src/scara_kinematics/include/scara_kinematics/analytic.hpp`: standalone analytical core.
- `ros2_ws/src/scara_kinematics/src/scara_plugin.cpp`: MoveIt plugin, with geometry read from the robot model.
- `ros2_ws/src/scara_description/config/cad_manifest.json`: CAD frames, source hash, part assignments and dimensions.
- MoveIt `/compute_fk` and `/compute_ik`: standard ROS service interfaces.

The `cad_reference` named state reproduces the original STEP assembly frame arrangement. The gripper's exported jaw opening remains fixed.
