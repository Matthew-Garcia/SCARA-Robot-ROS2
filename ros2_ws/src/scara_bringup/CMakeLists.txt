cmake_minimum_required(VERSION 3.16)
project(scara_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config worlds models DESTINATION share/${PROJECT_NAME})

# These Python tools are launched with launch_ros/ros2 run.  GitHub's contents
# API stores new text files without the executable bit; with colcon
# --symlink-install that permission is preserved by the symlink target.
# Mark every installed Python tool executable during CMake configure so both
# normal installs and symlink installs work reliably.
set(SCARA_PY_SCRIPTS
  scripts/move_to_pose.py
  scripts/check_stack.py
  scripts/scene_objects.py
  scripts/gripper_command.py
  scripts/pick_lift_demo.py
  scripts/simulation_grasp.py
)

foreach(_script IN LISTS SCARA_PY_SCRIPTS)
  file(CHMOD "${CMAKE_CURRENT_SOURCE_DIR}/${_script}"
    PERMISSIONS
      OWNER_READ OWNER_WRITE OWNER_EXECUTE
      GROUP_READ GROUP_EXECUTE
      WORLD_READ WORLD_EXECUTE
  )
endforeach()

install(PROGRAMS ${SCARA_PY_SCRIPTS} DESTINATION lib/${PROJECT_NAME})

ament_package()
