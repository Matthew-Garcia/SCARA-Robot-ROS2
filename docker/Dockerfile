FROM ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
    ros-humble-xacro ros-humble-rviz2 python3-colcon-common-extensions \
    python3-pytest python3-numpy python3-yaml xvfb xauth \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /scara_ws
COPY ros2_ws/src src
RUN source /opt/ros/humble/setup.bash && colcon build --event-handlers console_direct+
CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash && source /scara_ws/install/setup.bash && ros2 launch scara_bringup sim.launch.py"]
