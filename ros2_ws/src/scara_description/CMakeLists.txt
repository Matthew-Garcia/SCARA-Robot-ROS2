cmake_minimum_required(VERSION 3.16)
project(scara_description)
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
execute_process(COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/color_shoulder.py
  ${CMAKE_CURRENT_SOURCE_DIR}/meshes/shoulder_link.stl ${CMAKE_CURRENT_BINARY_DIR}/shoulder_colored.dae
  COMMAND_ERROR_IS_FATAL ANY)
install(DIRECTORY urdf meshes config DESTINATION share/${PROJECT_NAME})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/shoulder_colored.dae DESTINATION share/${PROJECT_NAME}/meshes)
ament_package()
