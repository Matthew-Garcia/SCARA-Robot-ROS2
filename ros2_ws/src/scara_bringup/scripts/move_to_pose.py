#!/usr/bin/env python3
"""Solve Cartesian IK, then ask MoveIt to collision-check, plan and execute."""
import argparse
import math
import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from sensor_msgs.msg import JointState


def wait(node,future,seconds=30):
    rclpy.spin_until_future_complete(node,future,timeout_sec=seconds)
    if not future.done():raise RuntimeError('Timed out waiting for ROS response')
    return future.result()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ['x','y','z']:parser.add_argument('--'+key,type=float,required=True,help='metres in base_link')
    parser.add_argument('--yaw',type=float,default=0.,help='radians; TCP roll and pitch remain zero')
    parser.add_argument('--plan-only',action='store_true')
    args,ros_args=parser.parse_known_args()
    if not all(math.isfinite(v) for v in [args.x,args.y,args.z,args.yaw]):parser.error('Pose must be finite')
    rclpy.init(args=ros_args);node=Node('scara_move_to_pose')
    try:
        latest=[]
        sub=node.create_subscription(JointState,'/joint_states',lambda msg:latest.append(msg),10)
        import time
        deadline=time.monotonic()+20
        while not latest and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.2)
        if not latest:raise RuntimeError('No joint states: launch the simulation first')
        client=node.create_client(GetPositionIK,'/compute_ik')
        if not client.wait_for_service(timeout_sec=30):raise RuntimeError('MoveIt IK service unavailable')
        request=GetPositionIK.Request();ik=request.ik_request
        ik.group_name='arm';ik.ik_link_name='tcp_link';ik.avoid_collisions=True
        ik.robot_state.joint_state=latest[-1];ik.robot_state.is_diff=True
        ik.pose_stamped.header.frame_id='base_link';ik.timeout.sec=2
        p=ik.pose_stamped.pose;p.position.x=args.x;p.position.y=args.y;p.position.z=args.z
        p.orientation.z=math.sin(args.yaw/2);p.orientation.w=math.cos(args.yaw/2)
        response=wait(node,client.call_async(request))
        if response.error_code.val!=1:raise RuntimeError(f'No collision-free IK solution ({response.error_code.val})')
        joints=dict(zip(response.solution.joint_state.name,response.solution.joint_state.position))
        print('IK solution:',joints)
        goal=MoveGroup.Goal();goal.request.group_name='arm';goal.request.pipeline_id='ompl'
        goal.request.allowed_planning_time=10.;goal.request.num_planning_attempts=5
        goal.request.max_velocity_scaling_factor=.3;goal.request.max_acceleration_scaling_factor=.3
        goal.request.start_state.is_diff=True
        constraints=Constraints()
        for name in ['shoulder_joint','z_joint','elbow_joint','wrist_joint']:
            c=JointConstraint();c.joint_name=name;c.position=joints[name]
            c.tolerance_above=c.tolerance_below=.0001 if name=='z_joint' else .001;c.weight=1.;constraints.joint_constraints.append(c)
        goal.request.goal_constraints=[constraints];goal.planning_options.plan_only=args.plan_only
        goal.planning_options.planning_scene_diff.is_diff=True
        goal.planning_options.planning_scene_diff.robot_state.is_diff=True
        action=ActionClient(node,MoveGroup,'/move_action')
        if not action.wait_for_server(timeout_sec=30):raise RuntimeError('MoveIt action unavailable')
        handle=wait(node,action.send_goal_async(goal))
        if not handle.accepted:raise RuntimeError('MoveIt rejected goal')
        result=wait(node,handle.get_result_async(),120).result
        if result.error_code.val!=1:raise RuntimeError(f'MoveIt planning/execution failed ({result.error_code.val})')
        print('Plan complete.' if args.plan_only else 'Motion complete.')
    finally:
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
