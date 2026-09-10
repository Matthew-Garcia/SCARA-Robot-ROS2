#!/usr/bin/env python3
"""Integration check: controller activation, FK/IK plugin, MoveIt plan and execution."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetPlanningScene
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PlanningSceneComponents
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

NAMES=['shoulder_joint','z_joint','elbow_joint','wrist_joint']

def main():
    rclpy.init();node=Node('scara_check_stack');latest=[None]
    sub=node.create_subscription(JointState,'/joint_states',lambda msg:latest.__setitem__(0,msg),10)
    def wait(future,timeout=60):
        rclpy.spin_until_future_complete(node,future,timeout_sec=timeout)
        if not future.done():raise RuntimeError('ROS response timed out')
        return future.result()
    def service(kind,name):
        client=node.create_client(kind,name)
        if not client.wait_for_service(timeout_sec=120):raise RuntimeError('Missing service '+name)
        return client
    try:
        manager=service(ListControllers,'/controller_manager/list_controllers')
        deadline=time.monotonic()+120
        while time.monotonic()<deadline:
            states={c.name:c.state for c in wait(manager.call_async(ListControllers.Request())).controller}
            if all(states.get(n)=='active' for n in ['joint_state_broadcaster','arm_controller','gripper_controller']):break
            rclpy.spin_once(node,timeout_sec=.5)
        else:raise RuntimeError('Controllers did not activate')
        fk=service(GetPositionFK,'/compute_fk');ik=service(GetPositionIK,'/compute_ik')
        scene_client=service(GetPlanningScene,'/get_planning_scene')
        scene_request=GetPlanningScene.Request()
        scene_request.components.components=PlanningSceneComponents.WORLD_OBJECT_NAMES
        expected={'work_surface','placement_tray','pickup_cube','pickup_sphere','pickup_cylinder','pickup_hex_prism','hanoi_stand','hanoi_ring_large','hanoi_ring_medium','hanoi_ring_small'}
        deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            scene_response=wait(scene_client.call_async(scene_request))
            if expected.issubset({o.id for o in scene_response.scene.world.collision_objects}):break
            rclpy.spin_once(node,timeout_sec=.2)
        else:raise RuntimeError('Manipulation props missing from MoveIt collision world')
        target=[.15,.01,-.3,.15]
        request=GetPositionFK.Request();request.header.frame_id='base_link';request.fk_link_names=['tcp_link']
        request.robot_state.joint_state.name=NAMES;request.robot_state.joint_state.position=target
        result=wait(fk.call_async(request));assert result.error_code.val==1,result.error_code
        pose=result.pose_stamped[0]
        req=GetPositionIK.Request();req.ik_request.group_name='arm';req.ik_request.ik_link_name='tcp_link'
        req.ik_request.pose_stamped=pose;req.ik_request.robot_state=request.robot_state
        req.ik_request.avoid_collisions=True;req.ik_request.timeout.sec=2
        sol=wait(ik.call_async(req));assert sol.error_code.val==1,sol.error_code
        values=dict(zip(sol.solution.joint_state.name,sol.solution.joint_state.position))
        assert all(abs(values[n]-q)<1e-5 for n,q in zip(NAMES,target)),values
        # The four-DOF plugin must reject an unreachable orientation.
        req.ik_request.pose_stamped.pose.orientation.x=math.sin(.2)
        req.ik_request.pose_stamped.pose.orientation.z=0.
        req.ik_request.pose_stamped.pose.orientation.w=math.cos(.2)
        rejected=wait(ik.call_async(req));assert rejected.error_code.val!=1
        action=ActionClient(node,MoveGroup,'/move_action');assert action.wait_for_server(timeout_sec=30)
        goal=MoveGroup.Goal();goal.request.group_name='arm';goal.request.pipeline_id='ompl'
        goal.request.allowed_planning_time=10.;goal.request.num_planning_attempts=3
        goal.request.max_velocity_scaling_factor=.5;goal.request.max_acceleration_scaling_factor=.5
        goal.request.start_state.is_diff=True
        constraints=Constraints()
        for name,value in zip(NAMES,target):
            c=JointConstraint();c.joint_name=name;c.position=value;c.tolerance_above=.0001;c.tolerance_below=.0001;c.weight=1.
            constraints.joint_constraints.append(c)
        goal.request.goal_constraints=[constraints];goal.planning_options.plan_only=False
        goal.planning_options.planning_scene_diff.is_diff=True
        goal.planning_options.planning_scene_diff.robot_state.is_diff=True
        handle=wait(action.send_goal_async(goal));assert handle.accepted
        result=wait(handle.get_result_async(),120).result;assert result.error_code.val==1,result.error_code
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.2)
            if latest[0]:
                actual=dict(zip(latest[0].name,latest[0].position))
                if all(abs(actual.get(n,1e9)-v)<(.005 if n=='z_joint' else .02) for n,v in zip(NAMES,target)):break
        else:raise RuntimeError('Joint states did not reach commanded target')
        gripper=ActionClient(node,FollowJointTrajectory,'/gripper_controller/follow_joint_trajectory')
        assert gripper.wait_for_server(timeout_sec=30)
        for positions in ([0.,0.],[.025,.025]):
            grip_goal=FollowJointTrajectory.Goal();grip_goal.trajectory.joint_names=['left_finger_joint','right_finger_joint']
            point=JointTrajectoryPoint();point.positions=positions;point.time_from_start.sec=1;grip_goal.trajectory.points=[point]
            grip_handle=wait(gripper.send_goal_async(grip_goal));assert grip_handle.accepted
            grip_result=wait(grip_handle.get_result_async(),15).result;assert grip_result.error_code==0,grip_result.error_string
        print('PASS: ten scene models, arm and gripper controllers, FK/IK, rejected tilt, MoveIt plan, execution and gripper motion.')
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
