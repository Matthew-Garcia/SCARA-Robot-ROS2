#!/usr/bin/env python3
"""Repeated vision-confirmed conveyor pick-and-place demonstration."""
import argparse
import json
import math
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint


ARM = ['shoulder_joint', 'z_joint', 'elbow_joint', 'wrist_joint']
FINGERS = ['left_finger_joint', 'right_finger_joint']
COLORS = ('red', 'green', 'blue')
BIN_X = {'red': 0.19, 'green': 0.26, 'blue': 0.33}


class ConveyorDemo(Node):
    def __init__(self):
        super().__init__('scara_conveyor_sort_demo')
        self.arm = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.set_model = self.create_client(SetModelState, '/gazebo/set_model_state')
        self.models = {}
        self.detections = {}
        self.create_subscription(ModelStates, '/gazebo/model_states', self.models_callback, 10)
        self.create_subscription(String, '/conveyor/vision/detection', self.vision_callback, 10)

    def models_callback(self, message):
        self.models = dict(zip(message.name, message.pose))

    def vision_callback(self, message):
        try:
            value = json.loads(message.data)
            self.detections[value['color']] = (time.monotonic(), value)
        except (KeyError, ValueError, TypeError):
            self.get_logger().warning('Ignored malformed vision detection.')

    def send(self, client, joints, positions, seconds):
        if not client.wait_for_server(timeout_sec=45):
            raise RuntimeError('trajectory controller action is unavailable')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(seconds)
        point.time_from_start.nanosec = int((seconds-int(seconds))*1e9)
        goal.trajectory.points = [point]
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle or not handle.accepted:
            raise RuntimeError('trajectory was rejected')
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=seconds+15)
        if not result.done() or result.result().result.error_code != 0:
            raise RuntimeError('trajectory execution failed')

    @staticmethod
    def joints_for(x, y, tcp_z):
        l1, l2 = 0.228, 0.1365
        cosine = (x*x+y*y-l1*l1-l2*l2)/(2*l1*l2)
        if not -1 <= cosine <= 1:
            raise ValueError('target is outside the SCARA workspace')
        elbow = math.acos(cosine)
        shoulder = math.atan2(y, x)-math.atan2(l2*math.sin(elbow), l1+l2*math.cos(elbow))
        return [shoulder, tcp_z-0.0725550818996708, elbow, -shoulder-elbow]

    def move_model(self, name, x, y, z):
        if not self.set_model.wait_for_service(timeout_sec=30):
            raise RuntimeError('/gazebo/set_model_state is unavailable')
        request = SetModelState.Request()
        request.model_state = ModelState()
        request.model_state.model_name = name
        request.model_state.reference_frame = 'world'
        request.model_state.pose.position.x = x
        request.model_state.pose.position.y = y
        request.model_state.pose.position.z = z
        request.model_state.pose.orientation.w = 1.0
        future = self.set_model.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)
        if not future.done() or not future.result().success:
            raise RuntimeError(f'could not move {name} on the conveyor')

    def wait_for_model(self, name, timeout=10.0):
        deadline = time.monotonic()+timeout
        while name not in self.models and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if name not in self.models:
            raise RuntimeError(f'Gazebo model state is unavailable for {name}')
        return self.models[name]

    def wait_for_height(self, name, minimum_z, timeout=3.0):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            pose = self.wait_for_model(name, timeout=0.2)
            if pose.position.z >= minimum_z:
                return pose
            rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError(f'{name} did not rise with the gripper')

    def feed_until_seen(self, color, name):
        self.detections.pop(color, None)
        for step in range(31):
            y = 0.22-step*(0.11/30.0)
            self.move_model(name, 0.26, y, 0.072)
            deadline = time.monotonic()+0.10
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.02)
        detection = self.detections.get(color)
        if detection is None or time.monotonic()-detection[0] > 4.0:
            raise RuntimeError(f'OpenCV did not detect the {color} conveyor cube')
        self.get_logger().info(f'OpenCV confirmed {color} cube at conveyor pickup point.')

    def sort_one(self, color):
        name = f'conveyor_cube_{color}'
        self.wait_for_model(name)
        for other in COLORS:
            if other != color:
                self.move_model(f'conveyor_cube_{other}', 0.50, 0.18+0.04*COLORS.index(other), 0.072)
        self.feed_until_seen(color, name)
        initial_z = self.wait_for_model(name).position.z
        self.send(self.gripper, FINGERS, [0.025, 0.025], 0.30)
        self.send(self.arm, ARM, self.joints_for(0.26, 0.11, 0.115), 0.80)
        self.send(self.arm, ARM, self.joints_for(0.26, 0.11, 0.072), 0.45)
        self.send(self.gripper, FINGERS, [0.0, 0.0], 0.35)
        self.send(self.arm, ARM, self.joints_for(0.26, 0.11, 0.115), 0.50)
        self.wait_for_height(name, initial_z+0.025)
        target_x = BIN_X[color]
        self.send(self.arm, ARM, self.joints_for(target_x, -0.12, 0.115), 0.85)
        self.send(self.arm, ARM, self.joints_for(target_x, -0.12, 0.062), 0.45)
        self.send(self.gripper, FINGERS, [0.025, 0.025], 0.30)
        self.send(self.arm, ARM, self.joints_for(target_x, -0.12, 0.115), 0.45)
        deadline = time.monotonic()+1.5
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        pose = self.wait_for_model(name).position
        if abs(pose.x-target_x) > 0.035 or abs(pose.y+0.12) > 0.035:
            raise RuntimeError(f'{name} missed its {color} sorting bin')
        self.get_logger().info(f'Placed {color} cube into the {color} bin.')

    def run(self, cycles):
        index = 0
        while rclpy.ok() and (cycles == 0 or index < cycles):
            color = COLORS[index % len(COLORS)]
            self.sort_one(color)
            index += 1
        print(f'PASS: completed {index} vision-confirmed conveyor pick-and-place cycles.')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cycles', type=int, default=0,
                        help='number of cubes to sort; 0 repeats until Ctrl+C')
    args, _ = parser.parse_known_args()
    rclpy.init()
    node = ConveyorDemo()
    try:
        node.run(args.cycles)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
