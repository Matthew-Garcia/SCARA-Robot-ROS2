#!/usr/bin/env python3
"""Kinematic Gazebo grasp helper for the two-finger SCARA demo.

This node is deliberately simulation-only.  It attaches the nearest approved
prop when both fingers close around it, carries the prop in the gripper frame,
and releases it back to Gazebo physics when the fingers open.
"""
import math

import rclpy
from gazebo_msgs.msg import LinkStates, ModelState, ModelStates
from geometry_msgs.msg import Pose
from rclpy.node import Node


TARGETS = {
    'pickup_cube', 'pickup_sphere', 'pickup_cylinder', 'pickup_hex_prism',
    'hanoi_ring_large', 'hanoi_ring_medium', 'hanoi_ring_small',
}


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    )


def conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def rotate(q, vector):
    value = quat_multiply(quat_multiply(q, (*vector, 0.0)), conjugate(q))
    return value[:3]


def quaternion(pose):
    return (pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w)


def relative_pose(parent, child):
    inverse = conjugate(quaternion(parent))
    delta = (child.position.x-parent.position.x,
             child.position.y-parent.position.y,
             child.position.z-parent.position.z)
    xyz = rotate(inverse, delta)
    q = quat_multiply(inverse, quaternion(child))
    result = Pose()
    result.position.x, result.position.y, result.position.z = xyz
    result.orientation.x, result.orientation.y = q[0], q[1]
    result.orientation.z, result.orientation.w = q[2], q[3]
    return result


def compose(parent, child):
    xyz = rotate(quaternion(parent),
                 (child.position.x, child.position.y, child.position.z))
    q = quat_multiply(quaternion(parent), quaternion(child))
    result = Pose()
    result.position.x = parent.position.x + xyz[0]
    result.position.y = parent.position.y + xyz[1]
    result.position.z = parent.position.z + xyz[2]
    result.orientation.x, result.orientation.y = q[0], q[1]
    result.orientation.z, result.orientation.w = q[2], q[3]
    return result


class SimulationGrasp(Node):
    def __init__(self):
        super().__init__('scara_simulation_grasp')
        self.gripper_pose = None
        self.left_finger_pose = None
        self.right_finger_pose = None
        self.models = {}
        self.attached_name = None
        self.gripper_to_object = None
        self.publisher = self.create_publisher(
            ModelState, '/gazebo/set_model_state', 10)
        self.create_subscription(LinkStates, '/gazebo/link_states',
                                 self.links_callback, 10)
        self.create_subscription(ModelStates, '/gazebo/model_states',
                                 self.models_callback, 10)
        self.create_timer(0.02, self.update)
        self.get_logger().info('Simulation grasp helper ready.')

    def links_callback(self, message):
        for name, pose in zip(message.name, message.pose):
            if name.endswith('::gripper_base_link'):
                self.gripper_pose = pose
            elif name.endswith('::left_finger_link'):
                self.left_finger_pose = pose
            elif name.endswith('::right_finger_link'):
                self.right_finger_pose = pose

    def models_callback(self, message):
        self.models = dict(zip(message.name, message.pose))

    def update(self):
        if (self.gripper_pose is None or self.left_finger_pose is None or
                self.right_finger_pose is None):
            return
        dx = self.left_finger_pose.position.x-self.right_finger_pose.position.x
        dy = self.left_finger_pose.position.y-self.right_finger_pose.position.y
        dz = self.left_finger_pose.position.z-self.right_finger_pose.position.z
        finger_separation = math.sqrt(dx*dx+dy*dy+dz*dz)
        if self.attached_name and finger_separation >= 0.075:
            self.get_logger().info(f'Released {self.attached_name}.')
            self.attached_name = None
            self.gripper_to_object = None
            return
        if self.attached_name is None and finger_separation <= 0.060:
            candidates = []
            for name in TARGETS:
                pose = self.models.get(name)
                if pose is None:
                    continue
                dx = pose.position.x-self.gripper_pose.position.x
                dy = pose.position.y-self.gripper_pose.position.y
                dz = pose.position.z-self.gripper_pose.position.z
                candidates.append((math.sqrt(dx*dx+dy*dy+dz*dz), name, pose))
            if candidates:
                distance, name, pose = min(candidates)
                if distance <= 0.085:
                    self.attached_name = name
                    self.gripper_to_object = relative_pose(self.gripper_pose, pose)
                    self.get_logger().info(
                        f'Attached {name} at {distance:.3f} m.')
        if self.attached_name:
            state = ModelState()
            state.model_name = self.attached_name
            state.reference_frame = 'world'
            state.pose = compose(self.gripper_pose, self.gripper_to_object)
            self.publisher.publish(state)


def main():
    rclpy.init()
    node = SimulationGrasp()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
