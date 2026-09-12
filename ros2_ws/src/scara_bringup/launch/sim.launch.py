"""Humble: Gazebo Classic or mock ros2_control, MoveIt 2 and RViz2."""
from pathlib import Path
import os
import runpy
import tempfile
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, EmitEvent, LogInfo, SetEnvironmentVariable
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    mode=LaunchConfiguration('mode').perform(context)
    if mode not in ('gazebo','mock'):raise ValueError('mode must be gazebo or mock')
    sim=mode=='gazebo'
    gui=LaunchConfiguration('gui').perform(context)
    rviz=LaunchConfiguration('rviz').perform(context).lower()=='true'
    desc=Path(get_package_share_directory('scara_description'))
    bringup=Path(get_package_share_directory('scara_bringup'))
    moveit=Path(get_package_share_directory('scara_moveit_config'))
    world_path=Path(LaunchConfiguration('world').perform(context))
    if not world_path.is_absolute():world_path=bringup/'worlds'/world_path
    controllers=str(bringup/'config/controllers.yaml')
    urdf=xacro.process_file(str(desc/'urdf/scara.urdf.xacro'),mappings={
        'mode':mode,'description_share':str(desc),'controllers_file':controllers}).toxml()
    def config(name):
        with (moveit/'config'/name).open() as stream:return yaml.safe_load(stream)
    robot={'robot_description':urdf}
    params=[robot,{'robot_description_semantic':(moveit/'config/scara.srdf').read_text()},
      {'robot_description_kinematics':config('kinematics.yaml')},
      {'robot_description_planning':config('joint_limits.yaml')},
      {'planning_pipelines':['ompl'],'default_planning_pipeline':'ompl','ompl':config('ompl_planning.yaml')},
      config('moveit_controllers.yaml'),{'use_sim_time':sim,
      'allow_trajectory_execution':True,'publish_robot_description_semantic':True,
      'publish_robot_description':True,'publish_planning_scene':True,
      'publish_geometry_updates':True,'publish_state_updates':True,'publish_transforms_updates':True}]
    publisher=Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[robot,{'use_sim_time':sim}],output='screen')
    broadcaster=Node(package='controller_manager',executable='spawner',arguments=['joint_state_broadcaster','--controller-manager-timeout','120'],output='screen')
    controller=Node(package='controller_manager',executable='spawner',arguments=['arm_controller','--controller-manager-timeout','120'],output='screen')
    gripper=Node(package='controller_manager',executable='spawner',arguments=['gripper_controller','--controller-manager-timeout','120'],output='screen')
    move_group=Node(package='moveit_ros_move_group',executable='move_group',parameters=params,output='screen')
    ready=[move_group,Node(package='scara_bringup',executable='scene_objects.py',parameters=[{'use_sim_time':sim,'dynamic_scene':sim}],output='screen')]
    if sim:
        ready.append(Node(package='scara_bringup',executable='simulation_grasp.py',parameters=[{'use_sim_time':True}],output='screen'))
    if rviz:ready.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(moveit/'config/scara.rviz')],parameters=params,output='screen'))
    def after_success(next_actions):
        def callback(event,context):
            if event.returncode!=0:
                return [LogInfo(msg='ERROR: SCARA startup failed; inspect the preceding process error.'),EmitEvent(event=Shutdown(reason='SCARA startup failure'))]
            return next_actions
        return callback
    actions=[SetEnvironmentVariable('GAZEBO_MODEL_PATH',os.pathsep.join(filter(None,[str(bringup/'models'),os.environ.get('GAZEBO_MODEL_PATH','')]))),
             RegisterEventHandler(OnProcessExit(target_action=broadcaster,on_exit=after_success([controller]))),
             RegisterEventHandler(OnProcessExit(target_action=controller,on_exit=after_success([gripper]))),
             RegisterEventHandler(OnProcessExit(target_action=gripper,on_exit=after_success(ready))),publisher]
    if sim:
        gazebo=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('gazebo_ros'))/'launch/gazebo.launch.py')),
            launch_arguments={'world':str(world_path),'gui':gui,'verbose':'false','pause':'false'}.items())
        # Color the final SDF after fixed-joint reduction, not just the RViz URDF.
        converter=runpy.run_path(str(bringup/'launch/gazebo_materials.py'))
        sdf=converter['build_gazebo_sdf'](urdf)
        model_directory=tempfile.TemporaryDirectory(prefix='scara-gazebo-')
        model_path=Path(model_directory.name)/'scara.sdf'
        model_path.write_text(sdf)
        def cleanup_model(context):
            model_directory.cleanup()
            return []
        actions.append(RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup_model)])))
        spawn=Node(package='gazebo_ros',executable='spawn_entity.py',arguments=['-entity','scara','-file',str(model_path),'-timeout','120'],output='screen')
        actions += [RegisterEventHandler(OnProcessExit(target_action=spawn,on_exit=after_success([broadcaster]))),gazebo,spawn]
    else:
        actions += [Node(package='controller_manager',executable='ros2_control_node',parameters=[robot,controllers,{'use_sim_time':False}],output='screen'),broadcaster]
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('mode',default_value='gazebo',choices=['gazebo','mock']),
       DeclareLaunchArgument('gui',default_value='true'),DeclareLaunchArgument('rviz',default_value='true'),DeclareLaunchArgument('world',default_value='scara.world'),OpaqueFunction(function=setup)])
