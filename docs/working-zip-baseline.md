# Working ZIP baseline

Restored from Matthew's `SCARA-Robot-ROS2-development-functional-gripper(1).zip`, reported working on his computer. Source files are included; machine-specific build, install, and log directories are excluded.

The original CAD finger visuals, joint definitions, pre-contact grasp thresholds, and standard pick/lift script come from that ZIP. The obsolete additional box gripper is removed. The grasp helper is a simulation pose coupling, not a hardware grasp controller.

The only robot visual change is blue upper/lower shoulder plates with light-gray rods. `color_shoulder.py` preserves the original STL triangles and emits a Collada mesh with separate materials during the build. Collision meshes and joint dynamics remain those from the ZIP.

The separate conveyor world uses the same `sim.launch.py`, robot description, controllers, and grasp helper. Its cubes are added to the helper's target list and its closing command is the ZIP's 0.0235 m setting. OpenCV remains a simulation color detector; conveyor runtime must be validated after rebuilding.

Build in a fresh workspace, or remove your old build/install/log outputs before rebuilding, to avoid loading stale installed robot descriptions.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch scara_bringup sim.launch.py
```

In another sourced terminal: `ros2 run scara_bringup pick_lift_demo.py --object cube`.

For the separate conveyor scene: `ros2 launch scara_bringup conveyor_demo.launch.py`.
