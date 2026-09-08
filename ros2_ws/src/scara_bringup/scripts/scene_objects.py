#!/usr/bin/env python3
"""Mirror the Gazebo props into MoveIt's collision world; mock mode uses initial poses."""
from pathlib import Path
import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive


def multiply(a,b):
    x,y,z,w=a;X,Y,Z,W=b
    return (w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W,w*W-x*X-y*Y-z*Z)


def from_xyz_rpy(values):
    x,y,z,roll,pitch,yaw=values
    cr,sr=math.cos(roll/2),math.sin(roll/2);cp,sp=math.cos(pitch/2),math.sin(pitch/2);cy,sy=math.cos(yaw/2),math.sin(yaw/2)
    p=Pose();p.position.x=float(x);p.position.y=float(y);p.position.z=float(z)
    p.orientation.x=sr*cp*cy-cr*sp*sy;p.orientation.y=cr*sp*cy+sr*cp*sy
    p.orientation.z=cr*cp*sy-sr*sp*cy;p.orientation.w=cr*cp*cy+sr*sp*sy
    return p


def compose(a,b):
    qa=(a.orientation.x,a.orientation.y,a.orientation.z,a.orientation.w)
    qb=(b.orientation.x,b.orientation.y,b.orientation.z,b.orientation.w)
    v=multiply(multiply(qa,(b.position.x,b.position.y,b.position.z,0)),(-qa[0],-qa[1],-qa[2],qa[3]))
    p=Pose();p.position.x=a.position.x+v[0];p.position.y=a.position.y+v[1];p.position.z=a.position.z+v[2]
    q=multiply(qa,qb);p.orientation.x=q[0];p.orientation.y=q[1];p.orientation.z=q[2];p.orientation.w=q[3]
    return p


class SceneObjects(Node):
    def __init__(self):
        super().__init__('scara_scene_objects')
        self.declare_parameter('dynamic_scene',True)
        path=Path(get_package_share_directory('scara_bringup'))/'config/scene_objects.yaml'
        self.config=yaml.safe_load(path.read_text());self.poses={}
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.publisher=self.create_publisher(PlanningScene,'/planning_scene',qos)
        self.subscription=self.create_subscription(ModelStates,'/gazebo/model_states',self.update,qos_profile_sensor_data)
        self.timer=self.create_timer(1.,self.publish)
    def update(self,msg):
        self.poses=dict(zip(msg.name,msg.pose))
    def publish(self):
        scene=PlanningScene();scene.is_diff=True
        for item in self.config['objects']:
            if self.get_parameter('dynamic_scene').value and item['name'] not in self.poses:continue
            world_pose=self.poses.get(item['name'],from_xyz_rpy(item['pose']))
            obj=CollisionObject();obj.header.frame_id=self.config['frame_id'];obj.id=item['name'];obj.operation=CollisionObject.ADD
            for shape in item['shapes']:
                primitive=SolidPrimitive()
                if shape['type']=='box':primitive.type=SolidPrimitive.BOX;primitive.dimensions=[float(v) for v in shape['size']]
                elif shape['type']=='sphere':primitive.type=SolidPrimitive.SPHERE;primitive.dimensions=[float(shape['radius'])]
                else:primitive.type=SolidPrimitive.CYLINDER;primitive.dimensions=[float(shape['length']),float(shape['radius'])]
                obj.primitives.append(primitive);obj.primitive_poses.append(compose(world_pose,from_xyz_rpy(shape['pose'])))
            scene.world.collision_objects.append(obj)
        if scene.world.collision_objects:self.publisher.publish(scene)


def main():
    rclpy.init();node=SceneObjects()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
