cmake_minimum_required(VERSION 3.16)
project(scara_bringup)
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
add_library(scara_grasp_world_plugin SHARED src/scara_grasp_world_plugin.cpp)
target_include_directories(scara_grasp_world_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(scara_grasp_world_plugin ${GAZEBO_LIBRARIES})
target_compile_features(scara_grasp_world_plugin PRIVATE cxx_std_17)
install(TARGETS scara_grasp_world_plugin LIBRARY DESTINATION lib)
install(DIRECTORY launch config worlds models DESTINATION share/${PROJECT_NAME})

set(SCARA_PY_SCRIPTS
  move_to_pose.py
  check_stack.py
  scene_objects.py
  gripper_command.py
  pick_lift_demo.py
  conveyor_vision.py
  conveyor_sort_demo.py
)
set(SCARA_PY_BUILD_SCRIPTS "")
foreach(_script IN LISTS SCARA_PY_SCRIPTS)
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/${_script}"
    "${CMAKE_CURRENT_BINARY_DIR}/${_script}"
    COPYONLY
  )
  file(CHMOD "${CMAKE_CURRENT_BINARY_DIR}/${_script}"
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
  list(APPEND SCARA_PY_BUILD_SCRIPTS "${CMAKE_CURRENT_BINARY_DIR}/${_script}")
endforeach()
install(PROGRAMS ${SCARA_PY_BUILD_SCRIPTS} DESTINATION lib/${PROJECT_NAME})
ament_package()
