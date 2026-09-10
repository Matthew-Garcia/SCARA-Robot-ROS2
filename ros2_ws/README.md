# Original SCARA robot — ROS 2 Humble

Ubuntu **22.04 LTS**, **ROS 2 Humble**, **Gazebo Classic 11**, **ros2_control**, **MoveIt 2**, and **RViz2**.

This workspace adds software around the repository's original SCARA assembly. All original CAD, firmware and image contents are preserved in the new folder layout; the original README is archived in `docs/archive/`. The robot meshes are tessellated from `cad/step/SCARA Robot 3D Model.STEP`, including its arms, base, rods, pulleys, electronics enclosure and original gripper. No substitute robot model is used.

![Original SCARA CAD preview](../docs/images/scara_cad_preview.png)

This is a render of the exported CAD geometry, not a screenshot of a running Gazebo session.

## Install and build

On Ubuntu 22.04 with ROS 2 Humble already installed:

```bash
sudo apt update
sudo apt install ros-humble-moveit ros-humble-ros2-control \
  ros-humble-ros2-controllers ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control ros-humble-xacro ros-humble-rviz2 \
  python3-colcon-common-extensions python3-rosdep python3-yaml
source /opt/ros/humble/setup.bash
cd ~/SCARA-Robot-ROS2/ros2_ws
# If your clone is elsewhere, use that path instead.
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install
source install/setup.bash
```

`rosdep` must have been initialized during ROS installation (`sudo rosdep init`, then `rosdep update`). There is no CAD conversion dependency for normal building: the generated meshes are included.

## Gazebo + MoveIt + RViz

```bash
ros2 launch scara_bringup sim.launch.py
```

This starts the bright Gazebo workspace, wood board, original robot, arm and gripper controllers, MoveIt, and RViz. Gazebo provides `/clock`; all participating ROS nodes use simulation time. The Gazebo plugin owns the controller manager.

In RViz, expand **MotionPlanning**, select planning group **arm**, choose **ready** or **cad_reference** as the goal, then **Plan** and **Execute**. The `home` state has straight arms; `cad_reference` reconstructs the source STEP assembly pose. The original gripper is rendered in its exported configuration.

The SCARA arm has four controlled axes: X/Y positioning, vertical motion and yaw. The gripper adds two synchronized sliding-finger joints. For interactive pose goals, translate the marker or rotate about Z; roll and pitch are unreachable and are rejected by the solver.

### Gripper and automatic pick/lift demo

In a second sourced terminal, open or close the white gripper:

```bash
ros2 run scara_bringup gripper_command.py open
ros2 run scara_bringup gripper_command.py close
```

Run a complete Gazebo-only pick, lift, carry, and release sequence:

```bash
ros2 run scara_bringup pick_lift_demo.py --object cube
# cube, sphere, cylinder, or hex
```

The grasp plugin creates a temporary fixed physics joint only when the fingers close near an approved pickup object and removes it when the fingers open. It does not teleport objects. Relaunch the world to reset all object positions.

For the red cube, the sequence also verifies that the released cube settles inside the square locating pocket rather than merely being dropped near the fixture.

### Separate OpenCV conveyor sorting world

Launch the continuous vision demo:

```bash
ros2 launch scara_bringup conveyor_demo.launch.py
```

This starts a separate bright Gazebo world with a wood board, conveyor, overhead RGB camera, three colored cubes, and red/green/blue receiving bins. `conveyor_vision.py` uses OpenCV HSV segmentation on `/conveyor/camera/image_raw`; `conveyor_sort_demo.py` advances a cube to the pickup point, requires a matching vision detection, picks it, places it in the matching bin, and repeats until `Ctrl+C`.

Useful launch options:

```bash
ros2 launch scara_bringup conveyor_demo.launch.py cycles:=3
ros2 launch scara_bringup conveyor_demo.launch.py autorun:=false
```

With `autorun:=false`, start a bounded run manually from a second sourced terminal:

```bash
ros2 run scara_bringup conveyor_sort_demo.py --cycles 3
```

The annotated OpenCV image is published on `/conveyor/vision/debug`, and detections are published on `/conveyor/vision/detection`.

### RViz + MoveIt without Gazebo

```bash
ros2 launch scara_bringup sim.launch.py mode:=mock
```

This uses `mock_components/GenericSystem` with the same trajectory controller and MoveIt configuration. It is a motion-planning demonstration with command-following mock states, not a physics simulation.

### Cartesian target through MoveIt

In a second sourced terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/SCARA-Robot-ROS2/ros2_ws/install/setup.bash
ros2 run scara_bringup move_to_pose.py --x 0.33 --y 0.02 --z 0.09 --yaw 0
# To plan without moving:
ros2 run scara_bringup move_to_pose.py --x 0.33 --y 0.02 --z 0.09 --yaw 0 --plan-only
```

The client obtains current joint states, requests collision-aware analytical IK, and submits the resulting goal to MoveIt. MoveIt plans, time-parameterizes and executes via `/arm_controller/follow_joint_trajectory`. Unreachable targets, rejected goals and execution failures are reported.

## Colored pickup set, placement tray, and three-piece Hanoi

The default scene includes a red cube, green sphere, yellow cylinder, purple diamond block, three movable Hanoi rings, a blue placement tray, and a raised wood work surface. Their poses are mirrored into MoveIt for collision checking. See the [scene guide](../docs/manipulation-scene.md) for dimensions and commands.

## Forward and inverse kinematics

The controlled joint order is:

| Joint | Motion | Software position range |
| --- | --- | --- |
| `shoulder_joint` | rotation of tower and arm | −90° to 266°; 2.4 rad/s |
| `z_joint` | carriage translation along tower | −0.05 to +0.05 m; 0.16 m/s |
| `elbow_joint` | forearm rotation | −150° to 150°; 3.0 rad/s |
| `wrist_joint` | gripper yaw | −162° to 162°; 4.0 rad/s |

These ranges come from the original Processing GUI; they are not independently measured mechanical limits. Zero slide position is the exported CAD carriage height. The software frame convention must be calibrated against physical motor zeros before hardware use.

CAD-measured lengths agree with `software/processing/GUI_for_SCARA_Robot/GUI_for_SCARA_Robot.pde`: L1 = **0.228 m**, L2 = **0.1365 m**. In `base_link`, with q = [shoulder, slide, elbow, wrist]:

```text
x   = L1 cos(q0) + L2 cos(q0 + q2)
y   = L1 sin(q0) + L2 sin(q0 + q2)
z   = z0 + q1
yaw = q0 + q2 + q3
z0  = 0.07255508189967187 m
```

The TCP is a defined reference point on the wrist axis at the lowest gripper CAD height. Its X axis follows the forearm at zero wrist angle; it is not a calibrated physical grasp frame. The geometry and frames are recorded in `scara_description/config/cad_manifest.json`.

The analytical MoveIt plugin solves both elbow branches, enumerates angle wraps within the bounded shoulder travel, ranks results by proximity to the seed, and respects consistency limits and collision callbacks. It rejects non-finite values, unreachable positions, joint-limit violations and tilted orientations. Standard MoveIt services `/compute_fk` and `/compute_ik` expose the model; the supplied client demonstrates IK use.

## What is modeled, and what still needs physical validation

- **109 STEP part instances** are assigned to five rigid bodies. The two loose laser accessory components beside the base are recorded as excluded; their original source files are retained.
- Visual geometry preserves the actual CAD surfaces. Collision geometry uses convex hulls of individual structural parts; small belts, bearings and fasteners do not have separate collision hulls. These hulls are conservative approximations, not exact contact surfaces.
- Inertias are positive box approximations with explicitly assumed link masses, suitable for initial position-control simulation. They are **not measured mass properties** and do not establish torque or payload performance.
- The original gripper CAD remains preserved. Two simple white collision fingers and a simulation-only attach/release plugin provide reliable Gazebo grasp demonstrations. Physical servo geometry, force, limits, and firmware still require measurement and validation.
- No hardware serial plugin is enabled. The existing Arduino firmware uses blocking motion, lacks feedback packets, and treats startup as zero without switch-seeking homing. It cannot honestly provide a feedback-based ros2_control hardware interface unchanged. See [hardware integration](../docs/hardware-integration.md).
- Adjacent bodies are excluded from MoveIt self-collision checking. Nonadjacent bodies remain checked. Gazebo self-contact is disabled to avoid contact forces between assembled components; MoveIt supplies geometric self-collision checking.
- Gazebo Classic is the legacy simulator paired here with Humble; use the documented Ubuntu 22.04 environment.

## Verification

See [validation record](../docs/validation.md) for the checks actually performed and the pending runtime checks.

Local checks:

```bash
python3 -m pytest -q tests
g++ -std=c++17 -I src/scara_kinematics/include tests/test_analytic.cpp -o /tmp/scara_kinematics_test
/tmp/scara_kinematics_test
```

The geometry tests verify source CAD SHA-256, complete rigid-body assignment, mesh paths, controller joint consistency, positive inertias, and exact reconstruction of original CAD frames. The C++ tests exercise 10,000 FK/IK round trips plus elbow branches, angular wraps, singular and unreachable targets.

With either running stack:

```bash
ros2 run scara_bringup check_stack.py
ros2 control list_controllers
ros2 action list
```

`check_stack.py` checks active controllers, FK/IK round-trip through MoveIt, rejection of tilt, collision-aware planning, execution, and final joint feedback. `.github/workflows/scara-humble.yml` builds on Ubuntu 22.04/Humble and runs both mock and Gazebo integration checks. Runtime validation status belongs to that workflow; successful standalone math tests alone do not establish Gazebo operation.

## CAD regeneration

From a separate Python environment with `cadquery==2.8.0`, `trimesh` and `networkx` installed:

```bash
python ../tools/export_cad.py
```

This exports meshes and the provenance manifest without writing to the original CAD. The source assembly instance count is checked; review mapping and joint frames if the source CAD ever changes. Preserve the existing URDF geometry unless its source frames have been deliberately re-derived.

## Package layout

| Package | Contents |
| --- | --- |
| `scara_description` | CAD-derived meshes, component collision hulls, URDF/Xacro, provenance |
| `scara_kinematics` | Standalone analytical FK/IK core and MoveIt plugin |
| `scara_moveit_config` | SRDF, OMPL planning, controller mapping, limits, RViz configuration |
| `scara_bringup` | Gazebo/mock launch, controllers, world, Cartesian client, integration check |

## References

- [Humble gazebo_ros2_control documentation](https://control.ros.org/humble/doc/gazebo_ros2_control/doc/index.html)
- [MoveIt Humble kinematics configuration](https://moveit.picknik.ai/humble/doc/examples/kinematics_configuration/kinematics_configuration_tutorial.html)
- [Original SCARA repository](https://github.com/Matthew-Garcia/SCARA-Robot)
