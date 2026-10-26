"""Attach measured camera/mount envelope to MoveIt; never sends motion."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene, PlanningSceneComponents, ObjectColor
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
from .camera_geometry import check_transform, envelope

OBJECT_ID = 'graspgen_camera_and_mount'


def transform_matrix(message):
    t = message.transform.translation
    q = message.transform.rotation
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    result[:3, 3] = [t.x, t.y, t.z]
    return check_transform(result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(Path(get_package_share_directory('piper_moveit_bridge')) / 'config/workcell_measurements.yaml'))
    parser.add_argument('--calibration', default=str(Path.home() / 'GraspGen/calibration/handeye_result.yaml'))
    parser.add_argument('--report', default=str(Path.home() / 'GraspGen/calibration/camera_collision_report.json'))
    args = parser.parse_args()
    config = yaml.safe_load(Path(args.config).read_text())['camera_and_mount']
    calibration = yaml.safe_load(Path(args.calibration).read_text())
    if calibration.get('accepted') is not True or calibration.get('schema') != 1 or calibration.get('gripper_frame') != 'link6':
        raise ValueError('Require accepted link6 hand-eye calibration')
    calibrated = check_transform(calibration['gripper_camera_optical'])
    rclpy.init()
    node = rclpy.create_node('graspgen_add_camera_collision')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    latest = {}
    subscriptions = []
    for key, topic in [('hardware', '/piper_hardware/status'), ('bridge', '/piper_bridge/readiness')]:
        subscriptions.append(node.create_subscription(String, topic, lambda m, k=key: latest.update({k: (time.monotonic(), json.loads(m.data))}), 10))
    clients = {name: node.create_client(kind, name) for kind, name in [
        (ApplyPlanningScene, '/apply_planning_scene'), (GetPlanningScene, '/get_planning_scene'),
        (GetStateValidity, '/check_state_validity')]}
    def spin_for(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.03)
    def gates_closed():
        if any(k not in latest or time.monotonic() - latest[k][0] > 1 for k in ['hardware', 'bridge']):
            raise RuntimeError('No fresh hardware/bridge status; keep hardware launch running')
        h = latest['hardware'][1]; b = latest['bridge'][1]
        if h.get('outputs_enabled', True) or b.get('execution_enabled', True) or b.get('active', True):
            raise RuntimeError('Close both gates and stop active execution before changing collision geometry')
    def call(name, request):
        future = clients[name].call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'{name} response timed out')
        return future.result()
    try:
        for name, client in clients.items():
            if not client.wait_for_service(timeout_sec=10):
                raise RuntimeError(f'{name} unavailable')
        spin_for(3)
        gates_closed()
        root = transform_matrix(buffer.lookup_transform('link6', 'camera_link', Time()))
        optical = transform_matrix(buffer.lookup_transform('link6', calibration['camera_frame'], Time()))
        delta = np.linalg.inv(calibrated) @ optical
        if np.linalg.norm(delta[:3, 3]) > .002 or Rotation.from_matrix(delta[:3, :3]).magnitude() > np.deg2rad(.2):
            raise RuntimeError('Live hand-eye TF differs from calibration; do not attach with an inconsistent mount')
        dimensions, position, quaternion = envelope(root, config)
        obj = CollisionObject()
        obj.id = OBJECT_ID; obj.header.frame_id = 'link6'; obj.operation = CollisionObject.ADD
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=dimensions.tolist())]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, position)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, quaternion)
        obj.primitive_poses = [pose]
        attached = AttachedCollisionObject()
        attached.link_name = 'link6'; attached.object = obj
        # Only the mounting links may contact the assembly; retain collision
        # checks against moving fingers, other arm links and the table.
        attached.touch_links = ['link6', 'gripper_base']
        scene = PlanningScene(); scene.is_diff = True; scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached]
        color = ObjectColor(); color.id = OBJECT_ID
        color.color.r = 1.; color.color.g = .55; color.color.b = .05; color.color.a = .65
        scene.object_colors = [color]
        gates_closed()
        if not call('/apply_planning_scene', ApplyPlanningScene.Request(scene=scene)).success:
            raise RuntimeError('MoveIt rejected attached camera collision object')
        query = GetPlanningScene.Request()
        query.components.components = PlanningSceneComponents.ROBOT_STATE | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        readback = call('/get_planning_scene', query).scene.robot_state
        found = [o for o in readback.attached_collision_objects if o.object.id == OBJECT_ID]
        if len(found) != 1 or found[0].link_name != 'link6' or len(found[0].object.primitives) != 1 or not np.allclose(found[0].object.primitives[0].dimensions, dimensions):
            raise RuntimeError('Attached camera geometry failed scene readback')
        check = GetStateValidity.Request(); check.robot_state = readback; check.group_name = 'arm'
        validity = call('/check_state_validity', check)
        contacts = sorted(set((c.contact_body_1, c.contact_body_2) for c in validity.contacts))
        report = {'attached': True, 'object_id': OBJECT_ID, 'link': 'link6', 'dimensions_m': dimensions.tolist(),
            'center_link6_m': position.tolist(), 'quaternion_link6_xyzw': quaternion.tolist(),
            'physical_placement_verified': False, 'current_state_collision_free': validity.valid,
            'contacts': contacts, 'padding_each_side_m': float(config['padding_each_side_m']),
            'no_motion_requested': True, 'model': 'combined measured camera/mount envelope; nominal D455 case offset'}
        path = Path(args.report).expanduser(); path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2) + '\n')
        print('相机及支架整体碰撞包围盒已附着到link6，并从MoveIt读回确认。')
        print(f"含每侧{float(config['padding_each_side_m']) * 1000:g}mm余量，尺寸(前后/左右/上下)：" + str((dimensions * 1000).round(1).tolist()) + ' mm')
        print(f'当前姿态无碰撞：{validity.valid}；接触对：{contacts}；报告：{path}')
        print('仅修改规划场景，未请求运动。请在MoveIt RViz核对包围盒覆盖实物；MoveIt重启后需重新添加。')
        if not validity.valid:
            raise RuntimeError('Camera attached, but current state is in collision; inspect geometry/contacts before planning')
    except Exception as exc:
        node.get_logger().error(str(exc))
        raise SystemExit(1)
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
