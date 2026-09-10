#!/usr/bin/env python3
"""Mirror Gazebo props into MoveIt's collision world without breaking the SRDF ACM."""
from pathlib import Path
import copy
import math
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data,
)
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    PlanningScene, CollisionObject, AllowedCollisionEntry, PlanningSceneComponents,
)
from moveit_msgs.srv import GetPlanningScene
from shape_msgs.msg import SolidPrimitive


GRASPABLE_OBJECTS = {
    'pickup_cube', 'pickup_sphere', 'pickup_cylinder', 'pickup_hex_prism',
    'hanoi_ring_large', 'hanoi_ring_medium', 'hanoi_ring_small',
}
GRIPPER_CONTACT_LINKS = {
    'left_finger_link', 'right_finger_link',
}


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (
        w*X + x*W + y*Z - z*Y,
        w*Y - x*Z + y*W + z*X,
        w*Z + x*Y - y*X + z*W,
        w*W - x*X - y*Y - z*Z,
    )


def from_xyz_rpy(values):
    x, y, z, roll, pitch, yaw = values
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = float(z)
    p.orientation.x = sr*cp*cy - cr*sp*sy
    p.orientation.y = cr*sp*cy + sr*cp*sy
    p.orientation.z = cr*cp*sy - sr*sp*cy
    p.orientation.w = cr*cp*cy + sr*sp*sy
    return p


def compose(a, b):
    qa = (a.orientation.x, a.orientation.y, a.orientation.z, a.orientation.w)
    qb = (b.orientation.x, b.orientation.y, b.orientation.z, b.orientation.w)
    v = multiply(
        multiply(qa, (b.position.x, b.position.y, b.position.z, 0)),
        (-qa[0], -qa[1], -qa[2], qa[3]),
    )
    p = Pose()
    p.position.x = a.position.x + v[0]
    p.position.y = a.position.y + v[1]
    p.position.z = a.position.z + v[2]
    q = multiply(qa, qb)
    p.orientation.x, p.orientation.y = q[0], q[1]
    p.orientation.z, p.orientation.w = q[2], q[3]
    return p


def ensure_acm_name(matrix, name):
    """Add one row/column to an AllowedCollisionMatrix without losing old entries."""
    if name in matrix.entry_names:
        return matrix.entry_names.index(name)

    old_size = len(matrix.entry_names)
    matrix.entry_names.append(name)

    # Existing rows gain one disabled column.
    for row in matrix.entry_values:
        row.enabled.append(False)

    # New row starts disabled against every entry, including itself.
    row = AllowedCollisionEntry()
    row.enabled = [False] * (old_size + 1)
    matrix.entry_values.append(row)
    return old_size


def extend_acm_for_grasping(matrix):
    """Preserve MoveIt's SRDF ACM, then allow only finger<->graspable contacts."""
    for name in sorted(GRIPPER_CONTACT_LINKS | GRASPABLE_OBJECTS):
        ensure_acm_name(matrix, name)

    for finger in GRIPPER_CONTACT_LINKS:
        i = matrix.entry_names.index(finger)
        for obj in GRASPABLE_OBJECTS:
            j = matrix.entry_names.index(obj)
            matrix.entry_values[i].enabled[j] = True
            matrix.entry_values[j].enabled[i] = True
    return matrix


class SceneObjects(Node):
    def __init__(self):
        super().__init__('scara_scene_objects')
        self.declare_parameter('dynamic_scene', True)

        path = (
            Path(get_package_share_directory('scara_bringup'))
            / 'config/scene_objects.yaml'
        )
        self.config = yaml.safe_load(path.read_text())
        self.poses = {}

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(
            PlanningScene, '/planning_scene', qos
        )
        self.subscription = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.update,
            qos_profile_sensor_data,
        )

        # Do NOT invent a replacement ACM. Fetch MoveIt's current matrix first;
        # it already contains the SRDF "disable_collisions" pairs such as
        # base_link<->shoulder_link and gripper_base_link<->finger links.
        self.acm_client = self.create_client(
            GetPlanningScene, '/get_planning_scene'
        )
        self.acm_future = None
        self.acm = None

        self.timer = self.create_timer(0.5, self.publish)

    def update(self, msg):
        self.poses = dict(zip(msg.name, msg.pose))

    def request_acm(self):
        if self.acm is not None or self.acm_future is not None:
            return
        if not self.acm_client.service_is_ready():
            return

        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        self.acm_future = self.acm_client.call_async(request)
        self.acm_future.add_done_callback(self.receive_acm)

    def receive_acm(self, future):
        try:
            response = future.result()
            matrix = copy.deepcopy(response.scene.allowed_collision_matrix)
            self.acm = extend_acm_for_grasping(matrix)
            self.get_logger().info(
                'Loaded MoveIt ACM and added gripper-to-object contact allowances.'
            )
        except Exception as exc:
            self.get_logger().warning(
                f'Could not load MoveIt allowed-collision matrix yet: {exc}'
            )
        finally:
            self.acm_future = None

    def publish(self):
        self.request_acm()

        scene = PlanningScene()
        scene.is_diff = True

        for item in self.config['objects']:
            if (
                self.get_parameter('dynamic_scene').value
                and item['name'] not in self.poses
            ):
                continue

            world_pose = self.poses.get(
                item['name'], from_xyz_rpy(item['pose'])
            )
            obj = CollisionObject()
            obj.header.frame_id = self.config['frame_id']
            obj.id = item['name']
            obj.operation = CollisionObject.ADD

            for shape in item['shapes']:
                primitive = SolidPrimitive()
                if shape['type'] == 'box':
                    primitive.type = SolidPrimitive.BOX
                    primitive.dimensions = [
                        float(v) for v in shape['size']
                    ]
                elif shape['type'] == 'sphere':
                    primitive.type = SolidPrimitive.SPHERE
                    primitive.dimensions = [float(shape['radius'])]
                else:
                    primitive.type = SolidPrimitive.CYLINDER
                    primitive.dimensions = [
                        float(shape['length']), float(shape['radius'])
                    ]

                obj.primitives.append(primitive)
                obj.primitive_poses.append(
                    compose(world_pose, from_xyz_rpy(shape['pose']))
                )

            scene.world.collision_objects.append(obj)

        # Only publish an ACM after we have fetched and extended MoveIt's
        # existing matrix. This preserves all SRDF self-collision exemptions.
        if self.acm is not None:
            scene.allowed_collision_matrix = copy.deepcopy(self.acm)

        if scene.world.collision_objects:
            self.publisher.publish(scene)


def main():
    rclpy.init()
    node = SceneObjects()
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
