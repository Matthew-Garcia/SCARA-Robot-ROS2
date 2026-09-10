#!/usr/bin/env python3
"""Open or close the simulated two-finger SCARA gripper."""
import argparse
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ['left_finger_joint', 'right_finger_joint']
POSITIONS = {'open': [0.025, 0.025], 'close': [0.0, 0.0]}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('command', choices=POSITIONS)
    args, _ = parser.parse_known_args()
    rclpy.init()
    node = Node('scara_gripper_command')
    client = ActionClient(node, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
    try:
        if not client.wait_for_server(timeout_sec=15):
            raise RuntimeError('gripper_controller action is unavailable')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = POSITIONS[args.command]
        point.time_from_start.sec = 1
        goal.trajectory.points = [point]
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future)
        handle = future.result()
        if not handle or not handle.accepted:
            raise RuntimeError('gripper command was rejected')
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result, timeout_sec=10)
        if not result.done() or result.result().result.error_code != 0:
            raise RuntimeError('gripper did not reach the requested position')
        print(f'Gripper {args.command} complete.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
