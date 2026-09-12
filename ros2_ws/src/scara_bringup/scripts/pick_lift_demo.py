#!/usr/bin/env python3
"""Gazebo-only fast pick, lift, carry and release demonstration."""
import argparse
import math
import time
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = ['shoulder_joint', 'z_joint', 'elbow_joint', 'wrist_joint']
FINGERS = ['left_finger_joint', 'right_finger_joint']
OBJECTS = {
    'cube': (0.255, 0.105, 0.060, 0.180),
    'sphere': (0.310, 0.120, 0.061, 0.240),
    'cylinder': (0.205, 0.135, 0.062, 0.300),
    'hex': (0.160, 0.095, 0.061, 0.360),
}


class Demo(Node):
    def __init__(self):
        super().__init__('scara_pick_lift_demo')
        self.arm = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.model_z = {}
        self.create_subscription(ModelStates, '/gazebo/model_states', self.models_callback, 10)

    def models_callback(self, message):
        self.model_z = {name: pose.position.z for name, pose in zip(message.name, message.pose)}

    def send(self, client, joints, positions, seconds):
        # DEMO ONLY: fixed wrist orientation.
        # -44 degrees = -0.767945 radians.
        if list(joints) == list(ARM) and len(positions) >= 4:
            positions = list(positions)
            positions[3] = math.radians(-44.0)
            self.get_logger().info(
                'DEMO wrist fixed at -44.0 degrees'
            )
        if not client.wait_for_server(timeout_sec=20):
            raise RuntimeError('trajectory controller action is unavailable')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        point = JointTrajectoryPoint()
        point.positions = positions
        whole = int(seconds)
        point.time_from_start.sec = whole
        point.time_from_start.nanosec = int((seconds - whole) * 1e9)
        goal.trajectory.points = [point]
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle or not handle.accepted:
            raise RuntimeError('trajectory was rejected')
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=seconds + 15)
        if not result.done() or result.result().result.error_code != 0:
            raise RuntimeError('trajectory execution failed')

    @staticmethod
    def joints_for(x, y, tcp_z):
        l1, l2 = 0.228, 0.1365
        cosine = (x*x + y*y - l1*l1 - l2*l2) / (2*l1*l2)
        if not -1 <= cosine <= 1:
            raise ValueError('target is outside the planar workspace')
        elbow = math.acos(cosine)
        shoulder = math.atan2(y, x) - math.atan2(l2*math.sin(elbow), l1+l2*math.cos(elbow))
        wrist = -shoulder-elbow
        return [shoulder, tcp_z - 0.0725550818996708, elbow, wrist]

    def run(self, object_name, position):
        x, y, object_z, tray_x = position
        gazebo_name = 'pickup_hex_prism' if object_name == 'hex' else f'pickup_{object_name}'
        deadline = time.monotonic() + 10
        while gazebo_name not in self.model_z and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if gazebo_name not in self.model_z:
            raise RuntimeError('Gazebo model states are unavailable; this demo requires mode:=gazebo')
        initial_z = self.model_z[gazebo_name]
        self.send(self.gripper, FINGERS, [0.025, 0.025], 0.5)
        self.send(self.arm, ARM, self.joints_for(x, y, 0.115), 1.4)
        self.send(self.arm, ARM, self.joints_for(x, y, object_z), 0.9)
        self.send(self.gripper, FINGERS, [0.0235, 0.0235], 0.8)
        time.sleep(0.4)
        self.send(self.arm, ARM, self.joints_for(x, y, 0.115), 0.9)
        if self.model_z.get(gazebo_name, initial_z) < initial_z + 0.025:
            raise RuntimeError(f'{gazebo_name} did not rise with the closed gripper')
        self.send(self.arm, ARM, self.joints_for(tray_x, -0.035, 0.115), 1.2)
        self.send(self.arm, ARM, self.joints_for(tray_x, -0.035, 0.080), 0.7)
        self.send(self.gripper, FINGERS, [0.025, 0.025], 0.5)
        self.send(self.arm, ARM, self.joints_for(tray_x, -0.035, 0.115), 0.7)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--object', choices=OBJECTS, default='cube')
    args, _ = parser.parse_known_args()
    rclpy.init()
    node = Demo()
    try:
        node.run(args.object, OBJECTS[args.object])
        print(f'PASS: picked, lifted, carried and released {args.object}.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
