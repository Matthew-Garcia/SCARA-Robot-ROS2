"""Separate OpenCV conveyor sorting demonstration."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = Path(get_package_share_directory('scara_bringup'))
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup/'launch/sim.launch.py')),
        launch_arguments={
            'mode': 'gazebo',
            'world': 'conveyor_sorting.world',
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )
    vision = Node(
        package='scara_bringup', executable='conveyor_vision.py',
        parameters=[{'use_sim_time': True}], output='screen')
    sorter = Node(
        package='scara_bringup', executable='conveyor_sort_demo.py',
        arguments=['--cycles', LaunchConfiguration('cycles')],
        parameters=[{'use_sim_time': True}], output='screen',
        condition=IfCondition(LaunchConfiguration('autorun')))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('autorun', default_value='true'),
        DeclareLaunchArgument('cycles', default_value='0'),
        stack, vision, sorter,
    ])
