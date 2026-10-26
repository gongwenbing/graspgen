"""Publish only link6 -> camera_link, respecting the camera driver's TF subtree."""
import numpy as np
import yaml
from pathlib import Path
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer,TransformListener,StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from .core import matrix,check_transform


class Mount(Node):
    def __init__(self):
        super().__init__('handeye_mount')
        path=self.declare_parameter('calibration_file','').value
        self.data=yaml.safe_load(Path(path).expanduser().read_text())
        if self.data.get('schema')!=1 or self.data.get('accepted') is not True:raise ValueError('Calibration did not pass quality checks')
        self.x=check_transform(self.data['gripper_camera_optical'])
        self.root=self.declare_parameter('camera_root','camera_link').value
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.broadcaster=StaticTransformBroadcaster(self);self.sent=False
        self.create_timer(.5,self.publish)

    def publish(self):
        if self.sent:return
        try:
            # Camera driver already owns root -> ... -> optical. Do not assign
            # a second parent to the optical frame.
            tf=self.buffer.lookup_transform(self.root,self.data['camera_frame'],Time())
            t=tf.transform.translation;q=tf.transform.rotation
            root_opt=matrix(Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix(),[t.x,t.y,t.z])
            mount=self.x@np.linalg.inv(root_opt)
            msg=TransformStamped();msg.header.stamp=self.get_clock().now().to_msg()
            msg.header.frame_id=self.data['gripper_frame'];msg.child_frame_id=self.root
            msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z=map(float,mount[:3,3])
            msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w=map(float,Rotation.from_matrix(mount[:3,:3]).as_quat())
            self.broadcaster.sendTransform(msg);self.sent=True
            self.get_logger().info(f'Published calibrated {msg.header.frame_id} -> {self.root}; independently verify before grasping')
        except Exception as e:self.get_logger().warning(f'Waiting for camera driver TF: {e}',throttle_duration_sec=5.)


def main(args=None):
    rclpy.init(args=args);node=Mount()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
