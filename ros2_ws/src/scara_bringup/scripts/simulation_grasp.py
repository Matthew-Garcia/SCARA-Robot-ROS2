#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from gazebo_msgs.msg import ModelStates, LinkStates, EntityState
from gazebo_msgs.srv import SetEntityState


TARGETS = [
    'pickup_cube',
    'pickup_sphere',
    'pickup_cylinder',
    'pickup_hex_prism',
    'hanoi_ring_large',
    'hanoi_ring_medium',
    'hanoi_ring_small',
    'conveyor_cube_red',
    'conveyor_cube_green',
    'conveyor_cube_blue',
]

# Link-origin separation is about 94 mm when completely open.
# Attach before the CAD fingers physically squeeze the object.
ATTACH_SEPARATION = 0.0926
RELEASE_SEPARATION = 0.0935

# The finger link origins are above the actual gripping region.
GRASP_Z_OFFSET = -0.090

MAX_XY_DISTANCE = 0.055
MAX_Z_DISTANCE = 0.080


class SimulationGrasp(Node):

    def __init__(self):
        super().__init__('scara_simulation_grasp')

        self.models = {}
        self.links = {}

        self.attached = None
        self.hold_orientation = None
        self.pending = None

        self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_callback,
            10,
        )

        self.create_subscription(
            LinkStates,
            '/gazebo/link_states',
            self.link_callback,
            10,
        )

        self.set_state = self.create_client(
            SetEntityState,
            '/gazebo/set_entity_state',
        )

        self.create_timer(0.02, self.update)

        self.get_logger().info(
            'Simulation grasp helper ready - gentle pre-contact grasp enabled.'
        )

    def model_callback(self, msg):
        self.models = dict(zip(msg.name, msg.pose))

    def link_callback(self, msg):
        self.links = dict(zip(msg.name, msg.pose))

    def find_link(self, link_name):
        for name, pose in self.links.items():
            if name == link_name or name.endswith('::' + link_name):
                return pose
        return None

    @staticmethod
    def distance(a, b):
        return math.sqrt(
            (a.position.x - b.position.x) ** 2 +
            (a.position.y - b.position.y) ** 2 +
            (a.position.z - b.position.z) ** 2
        )

    def update(self):
        left = self.find_link('left_finger_link')
        right = self.find_link('right_finger_link')

        if left is None or right is None:
            return

        separation = self.distance(left, right)

        grasp_x = (left.position.x + right.position.x) * 0.5
        grasp_y = (left.position.y + right.position.y) * 0.5
        grasp_z = (
            (left.position.z + right.position.z) * 0.5
            + GRASP_Z_OFFSET
        )

        # RELEASE
        if self.attached is not None:
            if separation >= RELEASE_SEPARATION:
                self.get_logger().info(
                    f'Released {self.attached}'
                )
                self.attached = None
                self.hold_orientation = None
                return

            self.hold_object(
                grasp_x,
                grasp_y,
                grasp_z,
            )
            return

        # Gripper still too open.
        if separation > ATTACH_SEPARATION:
            return

        # Find the object closest to the gripping center.
        candidate = None
        best_distance = 999.0

        for name in TARGETS:
            pose = self.models.get(name)

            if pose is None:
                continue

            dx = pose.position.x - grasp_x
            dy = pose.position.y - grasp_y
            dz = pose.position.z - grasp_z

            xy = math.hypot(dx, dy)

            if (
                xy <= MAX_XY_DISTANCE
                and abs(dz) <= MAX_Z_DISTANCE
            ):
                score = xy + 0.25 * abs(dz)

                if score < best_distance:
                    best_distance = score
                    candidate = name

        if candidate is None:
            return

        self.attached = candidate
        pose = self.models[candidate]

        self.hold_orientation = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

        self.get_logger().info(
            f'Attached {candidate} before finger contact '
            f'(finger separation {separation:.3f} m)'
        )

        self.hold_object(
            grasp_x,
            grasp_y,
            grasp_z,
        )

    def hold_object(self, x, y, z):
        if self.attached is None:
            return

        if not self.set_state.service_is_ready():
            return

        if self.pending is not None and not self.pending.done():
            return

        state = EntityState()

        state.name = self.attached
        state.reference_frame = 'world'

        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z

        if self.hold_orientation is not None:
            (
                state.pose.orientation.x,
                state.pose.orientation.y,
                state.pose.orientation.z,
                state.pose.orientation.w,
            ) = self.hold_orientation
        else:
            state.pose.orientation.w = 1.0

        request = SetEntityState.Request()
        request.state = state

        self.pending = self.set_state.call_async(request)


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
