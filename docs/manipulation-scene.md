# Gazebo manipulation scene

The default launch includes four colored pickup solids, a blue placement tray, and a **three-piece Tower of Hanoi**. Seven objects are dynamic rigid bodies; the wood work surface, tray, and peg stand are static. The original robot CAD is preserved.

| Object | Dimensions | Initial center in world, metres |
| --- | --- | --- |
| Red cube | 30 mm edges | (0.255, 0.105, 0.060) |
| Green sphere | 32 mm diameter | (0.310, 0.120, 0.061) |
| Yellow cylinder | 30 mm diameter × 34 mm | (0.205, 0.135, 0.062) |
| Purple diamond block | 28 × 28 × 32 mm, rotated 45° | (0.350, 0.055, 0.061) |
| Large Hanoi ring | 48 mm outer diameter, 10 mm hole, 8 mm thick | (0.200, −0.115, 0.057) |
| Medium Hanoi ring | 38 mm outer diameter, 10 mm hole, 8 mm thick | (0.200, −0.115, 0.065) |
| Small Hanoi ring | 28 mm outer diameter, 10 mm hole, 8 mm thick | (0.200, −0.115, 0.073) |

The work surface is 45 mm high. The Hanoi board top is 53 mm high. Pegs are 6 mm in diameter, reach 86 mm above the world floor, and are positioned at X = 0.200, 0.255 and 0.310 m, Y = −0.115 m. This keeps the three-ring stack and nominal lift height inside the modeled vertical range. Joint limits, collisions and the fixed CAD gripper geometry still constrain approach poses.

## Launch

```bash
ros2 launch scara_bringup sim.launch.py
```

Gazebo gives the movable objects gravity, mass, inertia and friction. Rings use true annular visual meshes and 24 collision segments each, keeping the center holes open around the pegs. The single `hanoi_stand` model contains the board and all three pegs.

`scene_objects.py` mirrors the models' world poses from `/gazebo/model_states` to MoveIt's `/planning_scene` at 1 Hz. Cube, sphere, surface, board, pegs and segmented ring collision shapes match between the two systems. Mock mode publishes the same initial scene without physics. This is a slow scene synchronization loop for demonstrations, not high-rate perception or contact feedback.

Operate the fingers and run the automatic demonstration:

```bash
ros2 run scara_bringup gripper_command.py open
ros2 run scara_bringup gripper_command.py close
ros2 run scara_bringup pick_lift_demo.py --object cube
```

The two fingers are independently represented but commanded together. In Gazebo, closing near an approved object attaches it with a temporary fixed physics joint; opening releases it. The demonstration supports `cube`, `sphere`, `cylinder`, and `hex`. An autonomous Hanoi solver is not included.

The placement fixture is in `cad/development/` as both OpenSCAD and STL. It measures 240 × 70 × 10 mm and has four shallow locating pockets.

To restore the initial arrangement, stop and relaunch the simulation. `check_stack.py` checks all ten scene models, both controllers, arm planning/execution, and gripper motion.

Configuration: `ros2_ws/src/scara_bringup/config/scene_objects.yaml`. Physical world: `ros2_ws/src/scara_bringup/worlds/scara.world`. Keep both synchronized; `test_scene.py` checks parity.
