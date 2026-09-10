# Validation record

## Passed in the development environment

- Python tests: source CAD hash and 109-part membership; expanded mock/Gazebo URDF and mesh/controller consistency; reconstruction of the original STEP assembly frames to 1e-9 tolerance; square-pocket fixture geometry; and separate conveyor/camera/OpenCV assets.
- Standalone C++17 analytical solver: 10,000 seeded FK/IK round trips, both elbow branches, wrapped shoulder angles, full extension, unreachable inner/outer positions, slide limits and non-finite input.
- Scene checks: exactly three movable Hanoi rings plus four colored pickup solids; positive object inertias; open ring holes; matching Gazebo/MoveIt collision shapes and starting poses.
- Python syntax compilation for the launch and ROS client/check scripts.
- Visual inspection of the CAD-derived preview.
- All 77 original files preserve their byte contents after relocation from source commit `0050e850bd5414da97d7ea8488d65917e560edf8`.

## Pending

The development container runs Ubuntu 24.04 and has no ROS installation or Docker runtime. The MoveIt plugin has therefore **not been compiled against installed Humble libraries**, and the complete Gazebo/MoveIt/RViz stack has **not been run** here. The ROS API signatures were checked against the Humble source headers, but that does not replace compilation.

The included Ubuntu 22.04/Humble workflow installs dependencies, builds the workspace, and exercises controller activation, MoveIt FK/IK, rejection of unreachable tilt, collision-aware planning, trajectory execution and final joint-state feedback in mock and Gazebo modes. It has not run yet. Publication of this new repository and its first GitHub Actions run remain pending.

No real-hardware, gripper-actuation, payload, torque or physical accuracy claim is made. Hardware integration requirements are in [hardware-integration.md](hardware-integration.md).
