#!/usr/bin/env python3
"""Deterministic Gazebo-only grasp coupling for the functional gripper demo."""
import math
import time

import rclpy
from gazebo_msgs.msg import LinkStates, ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import JointState


TARGETS = {
    'pickup_cube', 'pickup_sphere', 'pickup_cylinder', 'pickup_hex_prism',
    'hanoi_ring_large', 'hanoi_ring_medium', 'hanoi_ring_small',
    'conveyor_cube_red', 'conveyor_cube_green', 'conveyor_cube_blue',
}


def quaternion(pose):
    return (pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w)


def multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx+ax*bw+ay*bz-az*by,
            aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw,
            aw*bw-ax*bx-ay*by-az*bz)


def conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def rotate(q, xyz):
    value = multiply(multiply(q, (*xyz, 0.0)), conjugate(q))
    return value[:3]


def relative(parent, child):
    inverse = conjugate(quaternion(parent))
    delta = (child.position.x-parent.position.x,
             child.position.y-parent.position.y,
             child.position.z-parent.position.z)
    xyz = rotate(inverse, delta)
    q = multiply(inverse, quaternion(child))
    result = Pose()
    result.position.x, result.position.y, result.position.z = xyz
    result.orientation.x, result.orientation.y = q[0], q[1]
    result.orientation.z, result.orientation.w = q[2], q[3]
    return result


def compose(parent, child):
    xyz = rotate(quaternion(parent),
                 (child.position.x, child.position.y, child.position.z))
    q = multiply(quaternion(parent), quaternion(child))
    result = Pose()
    result.position.x = parent.position.x+xyz[0]
    result.position.y = parent.position.y+xyz[1]
    result.position.z = parent.position.z+xyz[2]
    result.orientation.x, result.orientation.y = q[0], q[1]
    result.orientation.z, result.orientation.w = q[2], q[3]
    return result


class SimulationGrasp(Node):
    def __init__(self):
        super().__init__('scara_simulation_grasp')
        self.gripper_pose = None
        self.models = {}
        self.fingers = {'left_finger_joint': 0.025, 'right_finger_joint': 0.025}
        self.attached = None
        self.offset = None
        self.pending = None
        self.last_warning = 0.0
        self.set_model = self.create_client(SetModelState, '/gazebo/set_model_state')
        self.create_subscription(LinkStates, '/gazebo/link_states', self.links, 10)
        self.create_subscription(ModelStates, '/gazebo/model_states', self.model_states, 10)
        self.create_subscription(JointState, '/joint_states', self.joints, 10)
        self.create_timer(0.02, self.update)
        self.get_logger().info('Gazebo grasp coupling ready for standard and conveyor props.')

    def links(self, message):
        fingers = [pose for name, pose in zip(message.name, message.pose)
                   if name.endswith('::left_finger_link') or
                   name.endswith('::right_finger_link')]
        if len(fingers) != 2:
            return
        midpoint = Pose()
        midpoint.position.x = sum(p.position.x for p in fingers)/2.0
        midpoint.position.y = sum(p.position.y for p in fingers)/2.0
        midpoint.position.z = sum(p.position.z for p in fingers)/2.0
        midpoint.orientation = fingers[0].orientation
        self.gripper_pose = midpoint

    def model_states(self, message):
        self.models = dict(zip(message.name, message.pose))

    def joints(self, message):
        for name, position in zip(message.name, message.position):
            if name in self.fingers:
                self.fingers[name] = position

    def update(self):
        if self.gripper_pose is None:
            return
        closed = max(self.fingers.values()) <= 0.006
        opened = max(self.fingers.values()) >= 0.016
        if self.attached and opened:
            self.get_logger().info(f'Released {self.attached} at the commanded placement pose.')
            self.attached, self.offset = None, None
            return
        if not self.attached and closed:
            choices = []
            for name in TARGETS:
                pose = self.models.get(name)
                if pose is None:
                    continue
                dx = pose.position.x-self.gripper_pose.position.x
                dy = pose.position.y-self.gripper_pose.position.y
                dz = pose.position.z-self.gripper_pose.position.z
                choices.append((math.sqrt(dx*dx+dy*dy+dz*dz), name, pose))
            if choices:
                distance, name, pose = min(choices)
                if distance <= 0.090:
                    self.attached = name
                    self.offset = relative(self.gripper_pose, pose)
                    self.get_logger().info(f'Attached {name} at {distance:.3f} m.')
        if not self.attached:
            return
        if self.pending is not None and not self.pending.done():
            return
        if self.pending is not None and self.pending.done():
            response = self.pending.result()
            if (response is None or not response.success) and time.monotonic()-self.last_warning > 2.0:
                self.last_warning = time.monotonic()
                self.get_logger().warning('Gazebo rejected an attached-object pose update.')
        if not self.set_model.service_is_ready():
            return
        request = SetModelState.Request()
        request.model_state = ModelState()
        request.model_state.model_name = self.attached
        request.model_state.reference_frame = 'world'
        request.model_state.pose = compose(self.gripper_pose, self.offset)
        self.pending = self.set_model.call_async(request)


def main():
    rclpy.init()
    node = SimulationGrasp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
