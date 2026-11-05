"""Independent point-cloud candidate node. No motion or actuator clients."""
import copy
import json
import hashlib
import yaml
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point
from std_msgs.msg import String, Header
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener
from .frame_pairing import FramePairer
from .generators import generate
from .bottle_geometry import fit_bottle, cloud_diagnostics


class GraspGenerator(Node):
    def __init__(self):
        super().__init__('graspgen_grasp_generator')
        self.method = self.declare_parameter('generator', 'auto').value
        if self.method not in ('auto', 'pca', 'known_cube', 'bottle_cylinder'):
            raise ValueError('Unknown generator')
        self.max_opening = float(self.declare_parameter('max_opening', .07).value)
        self.max_age = float(self.declare_parameter('max_frame_age', 1.5).value)
        if not 0 < self.max_opening <= .07 or not 0 < self.max_age <= 3:
            raise ValueError('Invalid opening/age')
        self.config_path = Path(self.declare_parameter('bottle_config', str(
            Path.home()/'GraspGen/calibration/bottle_geometry.json')).value).expanduser()
        self.handeye_path=Path(self.declare_parameter('handeye_calibration',str(Path.home()/'GraspGen/calibration/handeye_result.yaml')).value).expanduser()
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.pairer = FramePairer(); self.pending = None; self.last_pair = 0.
        self.pub_status = self.create_publisher(String, '/graspgen/status', 1)
        self.pub_poses = self.create_publisher(PoseArray, '/graspgen/grasp_poses', 1)
        self.pub_selected = self.create_publisher(PoseStamped, '/graspgen/selected_grasp', 1)
        self.pub_markers = self.create_publisher(MarkerArray, '/graspgen/markers', 1)
        self.create_subscription(PointCloud2, '/graspgen/object_cloud', self.on_cloud, 5)
        self.create_subscription(String, '/graspgen/segmentation/objects', self.on_objects, 5)
        self.create_timer(.05, self.tick)
        self.get_logger().info(f'Independent grasp generator={self.method}; perception only')

    def clear(self, reason, header=None):
        header = header or Header()
        delete = Marker(header=header, action=Marker.DELETEALL)
        self.pub_markers.publish(MarkerArray(markers=[delete]))
        self.pub_poses.publish(PoseArray(header=header))
        self.pub_status.publish(String(data=json.dumps(dict(valid=False, reason=reason,
            objects=[], selected_id=None, frame=header.frame_id))))

    def accept(self, kind, key, message):
        try:
            result = self.pairer.put(kind, key, message)
            if result:
                self.pending = result
                self.last_pair = time.monotonic()
        except (ValueError, TypeError) as exc:
            self.pending = None
            self.clear(str(exc))

    def on_cloud(self, message):
        h = message.header
        self.accept('cloud', (h.frame_id, h.stamp.sec, h.stamp.nanosec), message)

    def on_objects(self, message):
        try:
            data = json.loads(message.data)
            if data['schema_version'] != 1 or not isinstance(data['objects'], list):
                raise ValueError('Invalid segmentation metadata')
            stamp = data['stamp']
            self.accept('objects', (data['frame'], stamp['sec'], stamp['nanosec']), data)
        except (ValueError, KeyError, TypeError) as exc:
            self.pending = None
            self.clear('Invalid segmentation metadata: '+str(exc))

    def transform(self, target, header):
        tf = self.tf.lookup_transform(target, header.frame_id, Time.from_msg(header.stamp))
        q = tf.transform.rotation; t = tf.transform.translation
        return Rotation.from_quat([q.x, q.y, q.z, q.w]), np.array([t.x, t.y, t.z])

    def tick(self):
        if self.pending is None:
            if time.monotonic()-self.last_pair > self.max_age:
                self.clear('No fresh matched object cloud/metadata')
            return
        cloud, data = self.pending; h = cloud.header
        stamp = h.stamp.sec+h.stamp.nanosec*1e-9
        if not -.1 <= self.get_clock().now().nanoseconds*1e-9-stamp <= self.max_age:
            self.pending = None; self.clear('Stale object cloud', h); return
        method = self.method
        if method == 'auto':
            method = 'known_cube' if data['backend'] == 'color_cube' else 'pca'
        try:
            config = None; up = None; rotation = None; translation = None; handeye_digest=None
            if method == 'bottle_cylinder':
                # Validate dimensions even if TF is not available yet.
                from .bottle_geometry import validate_bottle_config
                config = json.loads(self.config_path.read_text())
                validate_bottle_config(config)
                contents=self.handeye_path.read_bytes();cal=yaml.safe_load(contents)
                if cal.get('accepted') is not True or cal.get('schema')!=1 or cal.get('camera_frame')!=h.frame_id:
                    raise ValueError('Invalid hand-eye calibration/frame')
                live_r,live_t=self.transform(cal['gripper_frame'],h)
                expected=np.array(cal['gripper_camera_optical'],dtype=float)
                if expected.shape!=(4,4) or not np.isfinite(expected).all():raise ValueError('Invalid hand-eye matrix')
                error_r=Rotation.from_matrix(expected[:3,:3]).inv()*live_r
                if np.linalg.norm(live_t-expected[:3,3])>.0005 or error_r.magnitude()>np.deg2rad(.1):
                    raise ValueError('Live camera TF differs from selected hand-eye calibration; restart handeye_mount and re-add camera collision model')
                handeye_digest=hashlib.sha256(contents).hexdigest()
                rotation, translation = self.transform('base_link', h)
            elif method == 'known_cube':
                rotation, _ = self.transform('world', h)
                up = rotation.inv().apply([0., 0., 1.])
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.pending = None; self.clear(str(exc), h); return
        except Exception as exc:
            # TF may arrive after the image. Retry this exact frame until expiry.
            self.clear('Waiting for image-time TF: '+str(exc), h); return
        self.pending = None
        try:
            fields = {f.name for f in cloud.fields}
            if cloud.width*cloud.height and not {'x', 'y', 'z', 'object_id'} <= fields:
                raise ValueError('Object cloud requires x/y/z/object_id fields')
            rows = list(point_cloud2.read_points(cloud, field_names=('x','y','z','object_id'), skip_nans=True)) if cloud.width*cloud.height else []
            groups = {}
            for row in rows:
                groups.setdefault(int(row[3]), []).append([float(row[i]) for i in range(3)])
            records = []; candidates = []; ids = set()
            for source in data['objects']:
                record = dict(source); record['valid'] = False; records.append(record)
                oid = source['id']
                if oid in ids:
                    raise ValueError('Duplicate object_id in frame')
                ids.add(oid)
                if not source.get('cloud_valid'):
                    continue
                try:
                    p = np.asarray(groups.get(oid, []), dtype=float)
                    if len(p) != source['point_count']:
                        raise ValueError('Point count differs from matching metadata')
                    if method == 'bottle_cylinder':
                        if source['class'] != 'bottle':
                            raise ValueError('Bottle generator accepts bottle only')
                        record['handeye_sha256']=handeye_digest
                        base_points = rotation.apply(p)+translation
                        record['geometry_diagnostics'] = cloud_diagnostics(base_points, config)
                        grasps = [fit_bottle(base_points, config)]
                    else:
                        if method == 'known_cube':
                            record.update(cube_size_m=data.get('cube_size_m',.04),
                                confidence_semantics='color_rule_match_not_probability')
                        grasps = generate(p, method, max_opening=self.max_opening, up=up,
                                          cube_size=data.get('cube_size_m', .04))
                    for grasp in grasps:
                        if grasp['opening'] > self.max_opening:
                            raise ValueError('Candidate exceeds gripper opening')
                        record.update(valid=True, generator=method,
                            position_semantics=grasp['position_semantics'],
                            opening_m=float(grasp['opening']), center_m=grasp['center'].tolist(),
                            quaternion_xyzw=grasp['quaternion'].tolist(), observed_extent_m=grasp['extent'].tolist())
                        if method == 'bottle_cylinder':
                            record.update({k:grasp[k] for k in ('fit_rms_m','arc_deg','diameter_m',
                                'total_height_m','grasp_height_m','body_bottom_m','body_top_m','table_z_m')})
                        candidates.append((source['confidence'], oid, grasp))
                except ValueError as exc:
                    record['reason'] = str(exc)
            out = copy.deepcopy(h)
            if method == 'bottle_cylinder':
                out.frame_id = 'base_link'
            self.publish(out, records, candidates, method, data.get('reason',''))
        except Exception as exc:
            self.clear(str(exc), h)

    def publish(self, header, records, candidates, method, reason):
        poses = PoseArray(header=header); markers = [Marker(header=header, action=Marker.DELETEALL)]
        selected = None
        for index, (score, oid, g) in enumerate(candidates):
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, g['center'])
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, g['quaternion'])
            poses.poses.append(pose)
            if selected is None or score > selected[0]:
                selected = (score, oid, pose)
            box = Marker(header=header, ns='observed_obb', id=index, type=Marker.CUBE, pose=copy.deepcopy(pose))
            box.scale.x, box.scale.y, box.scale.z = map(float, np.maximum(g['extent'], .002))
            # Cylinder candidate position is a chosen grip height, not box centre.
            if method == 'bottle_cylinder':
                box.pose.position.z = g.get('table_z_m',0.)+g['total_height_m']/2
            box.color.g=1.; box.color.a=.25; box.lifetime.sec=2; markers.append(box)
            for axis in range(3):
                arrow=Marker(header=header, ns=f'grasp_axis_{axis}', id=index, type=Marker.ARROW)
                a=g['center']; b=a+g['axes'][:,axis]*.06
                arrow.points=[Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in (a,b)]
                arrow.scale.x=.003; arrow.scale.y=.007; arrow.scale.z=.01
                arrow.color.r=float(axis==0); arrow.color.g=float(axis==1); arrow.color.b=float(axis==2); arrow.color.a=1.
                arrow.lifetime.sec=2; markers.append(arrow)
        self.pub_poses.publish(poses); self.pub_markers.publish(MarkerArray(markers=markers))
        if selected:
            self.pub_selected.publish(PoseStamped(header=header, pose=selected[2]))
        if not selected and not reason:
            reason='; '.join(str(r['reason']) for r in records if r.get('reason')) or 'No eligible grasp candidate'
        stamp=header.stamp.sec+header.stamp.nanosec*1e-9
        self.pub_status.publish(String(data=json.dumps(dict(valid=bool(selected), reason='' if selected else reason or 'No eligible grasp candidate',
            frame=header.frame_id, stamp_sec=stamp, selected_id=selected[1] if selected else None,
            objects=records, generator=method, processing_age_sec=self.get_clock().now().nanoseconds*1e-9-stamp))))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node=GraspGenerator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
