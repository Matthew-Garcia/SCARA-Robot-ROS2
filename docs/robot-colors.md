# Robot materials

Materials are assigned from the original STEP export manifest, not inferred from screenshots:

- Four NEMA 17 instances (indices 1, 27, 38, 48): black.
- Four Smooth Rod D10mm L400mm instances (12–15): light gray.
- J1 coupler (10) and J3 Coupler (49): blue.
- Both Gripper rail 6mm instances (63, 66): light gray.
- Servo Motor MG996R 3D Model (59): black; servo horn retains its existing color.
- Z-axis Bottom Plate (11), Z-axis Top Plate (20), Base cover (79), Top cover (80): blue.

`color_parts.py` splits the existing binary STL triangle records by these named CAD components. It does not change triangle coordinates, collision geometry, inertias, actuator joints, or the working ZIP's gripper.

RViz uses URDF RGBA colors. Gazebo Classic can lose material extensions when reducing the fixed visual links. The simulation launch now converts the same URDF with `gz sdf -p`, then assigns native SDF ambient/diffuse colors to each mesh before spawning that SDF. It checks that every colored mesh survived conversion; physics and controller plugins remain in the converted model.

Update the source, rebuild **both** packages, and close/restart the simulation:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select scara_description scara_bringup
source install/setup.bash
ros2 launch scara_bringup sim.launch.py
```

The conveyor launch uses this same description. Exact visual appearance still needs confirmation in Gazebo on the user's machine.
