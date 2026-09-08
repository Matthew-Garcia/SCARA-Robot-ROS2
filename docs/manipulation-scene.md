# Gazebo manipulation scene

The default launch includes a cube, a sphere, and a **three-piece Tower of Hanoi** with three pegs. Five objects are dynamic rigid bodies; the work surface and peg stand are static. The robot CAD is unchanged.

| Object | Dimensions | Initial center in world, metres |
| --- | --- | --- |
| Cube | 30 mm edges | (0.255, 0.105, 0.060) |
| Sphere | 32 mm diameter | (0.310, 0.120, 0.061) |
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

Plan an approach above the cube:

```bash
ros2 run scara_bringup move_to_pose.py --x 0.255 --y 0.105 --z 0.110 --yaw 0 --plan-only
```

Omit `--plan-only` to request execution if MoveIt finds a collision-free path. The target is above the cube; it is not a grasp command.

**The original gripper is still fixed at its exported opening.** This addition provides physical props and collision-aware approach planning. It does not implement jaw actuation, grasp attachment, pick-and-place completion, or an autonomous Hanoi solver. The scene is prepared for those next steps, without faking successful grasps by teleporting or attaching objects.

To restore the initial object arrangement, stop and relaunch the simulation. For the scene test, run `ros2 run scara_bringup check_stack.py`; it checks that all seven scene models appear in MoveIt before testing arm planning and execution.

Configuration: `ros2_ws/src/scara_bringup/config/scene_objects.yaml`. Physical world: `ros2_ws/src/scara_bringup/worlds/scara.world`. Keep both synchronized; `test_scene.py` checks parity.
