# Robot materials

Materials are assigned from the original STEP export manifest, not inferred from screenshots:

- Four NEMA 17 instances (indices 1, 27, 38, 48): black.
- Four Smooth Rod D10mm L400mm instances (12–15): light gray.
- Z-axis Bottom Plate (11), Z-axis Top Plate (20), Base cover (79), Top cover (80): blue.

`color_parts.py` splits the existing binary STL triangle records by these named CAD components. It does not change triangle coordinates, collision geometry, inertias, actuator joints, or the working ZIP's gripper.

Fixed visual-only child links give Gazebo explicit materials per color group. Gazebo reduces them into their physical parents. RViz uses the corresponding URDF RGBA colors. This replaces the previous shoulder Collada approach that displayed gray in Gazebo.

Rebuild `scara_description` after updating, source the newly built workspace, and restart Gazebo/RViz so the robot is spawned from the new description:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select scara_description
source install/setup.bash
ros2 launch scara_bringup sim.launch.py
```

The conveyor launch uses this same description. Exact visual appearance still needs confirmation in Gazebo on the user's machine.
