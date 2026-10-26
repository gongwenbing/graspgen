#!/usr/bin/env python3
"""Mirror compound SDF collision geometry using Gazebo's actual link poses."""
import math
import xml.etree.ElementTree as ET

import rclpy
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.msg import CollisionObject, ObjectColor
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from std_srvs.srv import SetBool


def sdf_pose(element: ET.Element | None) -> Pose:
    values = [float(v) for v in element.text.split()] if element is not None and element.text else [0.0] * 6
    if len(values) != 6:
        raise ValueError('SDF collision poses must contain xyz and roll/pitch/yaw.')
    x, y, z, roll, pitch, yaw = values
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation = Quaternion(
        x=sr * cp * cy - cr * sp * sy, y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy, w=cr * cp * cy + sr * sp * sy)
    return pose


def compose_pose(parent: Pose, local: Pose) -> Pose:
    """Transform a collision pose from its link frame into the world frame."""
    q, p = parent.orientation, local.position
    tx = 2 * (q.y * p.z - q.z * p.y)
    ty = 2 * (q.z * p.x - q.x * p.z)
    tz = 2 * (q.x * p.y - q.y * p.x)
    result = Pose()
    result.position.x = parent.position.x + p.x + q.w * tx + q.y * tz - q.z * ty
    result.position.y = parent.position.y + p.y + q.w * ty + q.z * tx - q.x * tz
    result.position.z = parent.position.z + p.z + q.w * tz + q.x * ty - q.y * tx
    r = local.orientation
    result.orientation = Quaternion(
        x=q.w * r.x + q.x * r.w + q.y * r.z - q.z * r.y,
        y=q.w * r.y - q.x * r.z + q.y * r.w + q.z * r.x,
        z=q.w * r.z + q.x * r.y - q.y * r.x + q.z * r.w,
        w=q.w * r.w - q.x * r.x - q.y * r.y - q.z * r.z)
    return result


def read_collisions(scene_file: str):
    world = ET.parse(scene_file).getroot().find('world')
    if world is None:
        raise ValueError('Scene file must contain an SDF world.')
    links = []
    names = ['pick_table', 'pick_cube'] + [m.get('name') for m in world.findall('model')
        if m.get('name', '').startswith('pick_cube_') or m.get('name') == 'pick_basket']
    for name in names:
        model = world.find(f"model[@name='{name}']")
        if model is None:
            raise ValueError(f'Scene model {name} is missing.')
        count = 0
        for link in model.findall('link'):
            shapes = []
            for collision in link.findall('collision'):
                pose_element = collision.find('pose')
                if pose_element is not None and pose_element.get('relative_to', '') not in ('', link.get('name')):
                    raise ValueError('Collision poses relative to named external frames are unsupported.')
                geometry = collision.find('geometry')
                if geometry is None:
                    raise ValueError(f'{name}: collision has no geometry.')
                primitive = SolidPrimitive()
                if geometry.find('box') is not None:
                    primitive.type = SolidPrimitive.BOX
                    primitive.dimensions = [float(v) for v in geometry.findtext('box/size', '').split()]
                    expected = 3
                elif geometry.find('cylinder') is not None:
                    primitive.type = SolidPrimitive.CYLINDER
                    primitive.dimensions = [float(geometry.findtext('cylinder/length', '0')),
                                            float(geometry.findtext('cylinder/radius', '0'))]
                    expected = 2
                elif geometry.find('sphere') is not None:
                    primitive.type = SolidPrimitive.SPHERE
                    primitive.dimensions = [float(geometry.findtext('sphere/radius', '0'))]
                    expected = 1
                else:
                    raise ValueError(f'{name}: supported collision shapes are box, cylinder and sphere.')
                if len(primitive.dimensions) != expected or not all(math.isfinite(v) and v > 0 for v in primitive.dimensions):
                    raise ValueError(f'{name}: invalid collision dimensions.')
                shapes.append((primitive, sdf_pose(pose_element)))
            if shapes:
                links.append((name, f"{name}::{link.get('name')}", shapes))
                count += len(shapes)
        if count == 0:
            raise ValueError(f'{name}: no collision shapes found.')
    return links


class PickSceneBridge(Node):
    def __init__(self):
        super().__init__('pick_scene_bridge')
        self.declare_parameter('scene_file', '')
        scene_file = self.get_parameter('scene_file').value
        if not isinstance(scene_file, str):
            raise ValueError('scene_file must be a path string.')
        self.links = read_collisions(scene_file)
        self.state_client = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.pending = None
        self.paused = False
        self.pause_service = self.create_service(SetBool, '/pick_scene/pause', self.pause)
        self.objects = {}
        self.remaining = []
        self.reported = False
        self.timer = self.create_timer(0.5, self.update)
        self.get_logger().info('Waiting for Gazebo and MoveIt scene services...')

    def update(self):
        if self.paused or self.pending is not None:
            return
        if not (self.state_client.service_is_ready() and self.scene_client.service_is_ready()):
            return
        self.objects = {}
        self.remaining = list(self.links)
        self.query_next()

    def query_next(self):
        if self.paused:
            self.pending = None
            return
        if not self.remaining:
            request = ApplyPlanningScene.Request()
            request.scene.is_diff = True
            request.scene.robot_state.is_diff = True
            request.scene.world.collision_objects = list(self.objects.values())
            for name, rgba in {
                'pick_cube': (0.9, 0.12, 0.08, 1.0),
                'pick_cube_blue': (0.08, 0.25, 0.95, 1.0),
                'pick_cube_green': (0.08, 0.8, 0.2, 1.0),
                'pick_basket': (0.65, 0.48, 0.22, 1.0),
            }.items():
                request.scene.object_colors.append(ObjectColor(
                    id=name, color=ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])))
            self.pending = self.scene_client.call_async(request)
            self.pending.add_done_callback(self.applied)
            return
        name, link, shapes = self.remaining.pop(0)
        request = GetEntityState.Request()
        request.name = link
        request.reference_frame = 'world'
        self.pending = self.state_client.call_async(request)
        self.pending.add_done_callback(lambda future: self.received(name, shapes, future))

    def pause(self, request, response):
        self.paused = request.data
        response.success = not self.paused or self.pending is None
        response.message = 'Paused' if response.success else 'Draining scene update; retry'
        return response

    def received(self, name, shapes, future):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().warn(f'Gazebo could not locate a collision link for {name}; will retry.')
                self.pending = None
                return
            if name not in self.objects:
                obj = CollisionObject()
                obj.header.frame_id = 'world'
                obj.id = name
                obj.operation = CollisionObject.ADD
                self.objects[name] = obj
            obj = self.objects[name]
            for primitive, local in shapes:
                obj.primitives.append(primitive)
                obj.primitive_poses.append(compose_pose(response.state.pose, local))
        except Exception as exc:
            self.get_logger().error(f'Gazebo state query failed: {exc}')
            self.pending = None
            return
        self.query_next()

    def applied(self, future):
        try:
            if future.result().success:
                if not self.reported:
                    self.get_logger().info('Table and cube synchronized to MoveIt (2 Hz).')
                    self.reported = True
            else:
                self.get_logger().warn('MoveIt rejected the scene update; will retry.')
        except Exception as exc:
            self.get_logger().error(f'MoveIt scene update failed: {exc}')
        finally:
            self.pending = None


def main(args=None):
    rclpy.init(args=args)
    node = PickSceneBridge()
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
