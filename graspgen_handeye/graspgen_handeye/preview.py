"""Display-only provisional model TCP and top-down pregrasp. No motion API."""
import json,math,time
from collections import deque
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.signals import SignalHandlerOptions
from tf2_ros import Buffer,TransformListener,StaticTransformBroadcaster
from geometry_msgs.msg import PoseStamped,TransformStamped,Point
from visualization_msgs.msg import Marker,MarkerArray
from std_msgs.msg import String,Header
from .core import matrix


def pose_matrix(pose):
    p=pose.position;q=pose.orientation
    return matrix(Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix(),[p.x,p.y,p.z])


def top_down_pregrasp(surface,tcp_offset,height):
    """Candidate closing Y projected into base XY; TCP +Z points down.
    Hover is base +Z, not camera's estimated surface normal.
    """
    y=surface[:3,1].copy();y[2]=0
    if np.linalg.norm(y)<.2:raise ValueError('Closing axis too vertical for top-down preview')
    y/=np.linalg.norm(y);z=np.array([0.,0.,-1.]);x=np.cross(y,z)
    tcp=matrix(np.column_stack((x,y,z)),surface[:3,3]+[0,0,height])
    flange=tcp@np.linalg.inv(tcp_offset)
    return tcp,flange


def fill_pose(t):
    from geometry_msgs.msg import Pose
    pose=Pose();pose.position.x,pose.position.y,pose.position.z=map(float,t[:3,3])
    pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=map(float,Rotation.from_matrix(t[:3,:3]).as_quat())
    return pose


def transform_matrix(tf):
    t=tf.transform.translation;q=tf.transform.rotation
    return matrix(Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix(),[t.x,t.y,t.z])


class Preview(Node):
    def __init__(self):
        super().__init__('grasp_preview')
        self.base='base_link';self.tip='link6'
        self.z=float(self.declare_parameter('tcp_z',.1358).value)
        self.height=float(self.declare_parameter('hover_height',.10).value)
        if not math.isfinite(self.z) or not .05<=self.z<=.25:raise ValueError('Invalid provisional TCP offset')
        if not math.isfinite(self.height) or not .05<=self.height<=.3:raise ValueError('Hover height must be 0.05..0.3m')
        config_path=Path(self.declare_parameter('tcp_config_file',str(Path.home()/'GraspGen/calibration/tcp_config.json')).value).expanduser()
        self.tcp_translation=[0.,0.,self.z];self.tcp_translation_verified=False
        if config_path.is_file():
            data=json.loads(config_path.read_text());translation=data['translation_m']
            if len(translation)!=3 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in translation) or not .05<=np.linalg.norm(translation)<=.25:
                raise ValueError('Invalid TCP translation config')
            self.tcp_translation=list(map(float,translation));self.tcp_translation_verified=bool(data.get('translation_verified',False))
        self.offset=matrix(np.eye(3),self.tcp_translation);self.pose=None;self.status=None;self.status_at=0
        self.candidates=deque(maxlen=30)
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.static=StaticTransformBroadcaster(self)
        tf=TransformStamped();tf.header.stamp=self.get_clock().now().to_msg();tf.header.frame_id=self.tip;tf.child_frame_id='piper_tcp_preview'
        tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=self.tcp_translation;tf.transform.rotation.w=1.;self.static.sendTransform(tf)
        self.create_subscription(PoseStamped,'/graspgen/selected_grasp',self.on_pose,1)
        self.create_subscription(String,'/graspgen/status',self.on_status,1)
        self.markers=self.create_publisher(MarkerArray,'/grasp_preview/markers',1)
        self.current=self.create_publisher(PoseStamped,'/grasp_preview/current_tcp',1)
        self.target=self.create_publisher(PoseStamped,'/grasp_preview/pregrasp_tcp',1)
        self.flange=self.create_publisher(PoseStamped,'/grasp_preview/pregrasp_link6',1)
        self.info=self.create_publisher(String,'/grasp_preview/status',1)
        self.create_timer(.1,self.tick)
        self.get_logger().info(f'DISPLAY ONLY: TCP link6 translation {self.tcp_translation}m (orientation remains provisional); hover {self.height:.3f}m. No arm command/action clients.')

    def remember_candidate(self):
        s=self.status;p=self.pose
        if not s or not s.get('valid'):
            self.candidates.clear();return
        if p is None:return
        stamp=p.header.stamp.sec+p.header.stamp.nanosec/1e9
        if s.get('frame')==p.header.frame_id and abs(s.get('stamp_sec',0)-stamp)<=1e-6:
            if not self.candidates or self.candidates[-1][1] is not p:
                self.candidates.append((s,p,self.status_at))

    def on_pose(self,msg):
        self.pose=msg;self.remember_candidate()
    def on_status(self,msg):
        try:self.status=json.loads(msg.data);self.status_at=time.monotonic()
        except (ValueError,TypeError):self.status=None
        self.remember_candidate()

    def candidate_with_tf(self):
        # Keep a short history so continuously arriving images do not replace
        # every candidate before its timestamp has reached the TF buffer.
        # Always transform at the image timestamp, never at the latest TF.
        now=self.get_clock().now().nanoseconds/1e9
        for s,p,received in reversed(tuple(self.candidates)):
            stamp=p.header.stamp.sec+p.header.stamp.nanosec/1e9
            if time.monotonic()-received>.8 or not -.1<=now-stamp<=1.:continue
            if self.buffer.can_transform(self.base,p.header.frame_id,Time.from_msg(p.header.stamp)):
                return s,p
        raise ValueError('Waiting for image-time TF (up to 1s fresh candidate history)')

    def marker(self,header,name,kind,pose,color,scale):
        m=Marker();m.header=header;m.ns=name;m.id=0;m.type=kind;m.pose=pose
        m.color.r,m.color.g,m.color.b,m.color.a=map(float,color)
        m.scale.x,m.scale.y,m.scale.z=map(float,scale);m.lifetime.sec=1
        return m

    def tick(self):
        header=Header();header.frame_id=self.base;header.stamp=self.get_clock().now().to_msg()
        delete=Marker();delete.header=header;delete.action=Marker.DELETEALL;items=[delete]
        current_ok=False;reason='';target_ok=False
        try:
            tf=self.buffer.lookup_transform(self.base,self.tip,Time())
            stamp=tf.header.stamp.sec+tf.header.stamp.nanosec/1e9
            if stamp<=0 or not -.1<=self.get_clock().now().nanoseconds/1e9-stamp<=.5:raise ValueError('Stale robot TF')
            actual=transform_matrix(tf)@self.offset;pose=fill_pose(actual)
            self.current.publish(PoseStamped(header=header,pose=pose));current_ok=True
            items.append(self.marker(header,'current_model_tcp',Marker.SPHERE,pose,[0,1,1,1],[.012]*3))
            text=self.marker(header,'tcp_label',Marker.TEXT_VIEW_FACING,fill_pose(actual),[0,1,1,1],[0,0,.018])
            text.pose.position.z+=.025;text.text='MODEL TCP - verify physical tip';items.append(text)
        except Exception as e:reason=f'Robot TCP unavailable: {e}'
        try:
            s=self.status;p=self.pose
            if not s or time.monotonic()-self.status_at>.8 or not s.get('valid') or p is None:
                raise ValueError('Waiting for a fresh valid vision candidate')
            s,p=self.candidate_with_tf()
            stamp=p.header.stamp.sec+p.header.stamp.nanosec/1e9
            if stamp<=0 or not -.1<=self.get_clock().now().nanoseconds/1e9-stamp<=1.0:raise ValueError('Stale vision candidate')
            if s.get('frame')!=p.header.frame_id or abs(s.get('stamp_sec',0)-stamp)>1e-6:raise ValueError('Vision pose/status timestamps differ')
            if not current_ok:raise ValueError('Fresh robot feedback required')
            tf=self.buffer.lookup_transform(self.base,p.header.frame_id,Time.from_msg(p.header.stamp))
            surface=transform_matrix(tf)@pose_matrix(p.pose)
            tcp,flange=top_down_pregrasp(surface,self.offset,self.height)
            self.target.publish(PoseStamped(header=p.header.__class__(stamp=p.header.stamp,frame_id=self.base),pose=fill_pose(tcp)))
            self.flange.publish(PoseStamped(header=p.header.__class__(stamp=p.header.stamp,frame_id=self.base),pose=fill_pose(flange)))
            items.append(self.marker(header,'surface_center',Marker.SPHERE,fill_pose(surface),[1,.8,0,1],[.012]*3))
            items.append(self.marker(header,'hover_tcp',Marker.SPHERE,fill_pose(tcp),[0,1,0,1],[.016]*3))
            for i in range(3):
                arrow=self.marker(header,f'hover_axis_{i}',Marker.ARROW,fill_pose(np.eye(4)),[i==0,i==1,i==2,1],[.003,.007,.01])
                a=tcp[:3,3];b=a+tcp[:3,i]*.06
                arrow.points=[Point(x=float(v[0]),y=float(v[1]),z=float(v[2])) for v in (a,b)];items.append(arrow)
            line=self.marker(header,'hover_vertical',Marker.LINE_STRIP,fill_pose(np.eye(4)),[1,1,0,1],[.002,0,0])
            line.points=[Point(x=float(v[0]),y=float(v[1]),z=float(v[2])) for v in (surface[:3,3],tcp[:3,3])];items.append(line)
            target_ok=True
            self.info.publish(String(data=json.dumps({'display_only':True,'tcp_provisional':True,'tcp_translation_m':self.tcp_translation,'tcp_translation_verified':self.tcp_translation_verified,'current_tcp_valid':True,'target_valid':True,
                'hover_height_m':self.height,'surface_base_m':surface[:3,3].tolist(),'pregrasp_tcp_base_m':tcp[:3,3].tolist(),
                'pregrasp_link6_base_m':flange[:3,3].tolist(),'reason':''})))
        except Exception as e:
            self.info.publish(String(data=json.dumps({'display_only':True,'tcp_provisional':True,'tcp_translation_m':self.tcp_translation,'tcp_translation_verified':self.tcp_translation_verified,'current_tcp_valid':current_ok,'target_valid':False,'reason':reason or str(e)})))
        self.markers.publish(MarkerArray(markers=items))


def main(args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO);node=Preview()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
