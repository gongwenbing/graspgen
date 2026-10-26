"""ROS 2 perception only: no target_pose, joint commands or execution services."""
import json
import math
import threading
import time
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,CameraInfo,PointCloud2,PointField
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Pose,PoseArray,PoseStamped
from visualization_msgs.msg import Marker,MarkerArray
from std_msgs.msg import String
from .geometry import object_points,pca_grasp
from .cubes import CubeDetector,cube_grasp
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer,TransformListener


class SegmentGrasp(Node):
    def __init__(self):
        super().__init__('graspgen_vision')
        self.backend=self.declare_parameter('backend','yolo').value
        if self.backend not in ('yolo','color_cube'):raise ValueError('backend must be yolo or color_cube')
        self.cube_size=float(self.declare_parameter('cube_size',.04).value)
        if not .02<=self.cube_size<=.06:raise ValueError('cube_size must be 0.02..0.06m')
        self.model_path=self.declare_parameter('model','').value
        self.device=self.declare_parameter('device','cpu').value
        self.classes={s.strip() for s in self.declare_parameter('target_classes','bottle,cup,mouse,cell phone,orange').value.split(',') if s.strip()}
        self.conf=float(self.declare_parameter('confidence',.45).value)
        self.min_depth=float(self.declare_parameter('min_depth',.35).value)
        self.max_depth=float(self.declare_parameter('max_depth',1.5).value)
        self.max_age=float(self.declare_parameter('max_frame_age',1.5).value)
        self.period=float(self.declare_parameter('process_period',.25).value)
        self.max_opening=float(self.declare_parameter('max_opening',.07).value)
        if not (0<self.conf<=1 and 0<self.min_depth<self.max_depth and 0<self.period and 0<self.max_age and 0<self.max_opening<=.07):
            raise ValueError('Invalid confidence/depth/age/period/opening parameters')
        if self.backend=='yolo':
            import torch
            torch.set_num_threads(4)
            from ultralytics import YOLO
            if not Path(self.model_path).is_file():raise ValueError(f'Model missing: {self.model_path}; run scripts/setup_vision.sh')
            self.model=YOLO(self.model_path,task='segment')
        else:
            self.model=CubeDetector()
            self.tf=Buffer();self.tf_listener=TransformListener(self.tf,self)
        if self.model.task!='segment':raise ValueError('Instance segmentation weights required')
        unknown=self.classes-set(self.model.names.values())
        if unknown:raise ValueError(f'Target classes absent from model: {sorted(unknown)}')
        self.class_ids=[i for i,name in self.model.names.items() if name in self.classes] if self.classes else None
        self.bridge=CvBridge();self.lock=threading.Lock();self.latest=None;self.last_processed=None
        self.received_at=0.;self.last_error='';self.last_publish=0.
        self.pub_image=self.create_publisher(Image,'/graspgen/segmentation/image',1)
        self.pub_cloud=self.create_publisher(PointCloud2,'/graspgen/object_cloud',1)
        self.pub_markers=self.create_publisher(MarkerArray,'/graspgen/markers',1)
        self.pub_poses=self.create_publisher(PoseArray,'/graspgen/grasp_poses',1)
        self.pub_selected=self.create_publisher(PoseStamped,'/graspgen/selected_grasp',1)
        self.pub_status=self.create_publisher(String,'/graspgen/status',1)
        self.group=ReentrantCallbackGroup()
        kwargs={'qos_profile':qos_profile_sensor_data,'callback_group':self.group}
        self.subs=[Subscriber(self,kind,topic,**kwargs) for kind,topic in [
            (Image,'/camera/color/image_raw'),(Image,'/camera/aligned_depth_to_color/image_raw'),
            (CameraInfo,'/camera/color/camera_info'),(CameraInfo,'/camera/aligned_depth_to_color/camera_info')]]
        self.sync=ApproximateTimeSynchronizer(self.subs,queue_size=5,slop=.05)
        self.sync.registerCallback(self.input)
        self.create_timer(self.period,self.process)
        self.create_timer(1.,self.watchdog,callback_group=self.group)
        self.get_logger().info(f'{self.backend} segmentation ready; perception only, no arm output')

    def input(self,*msgs):
        with self.lock:self.latest=msgs;self.received_at=time.monotonic()

    def clear(self,header,reason):
        delete=Marker();delete.action=Marker.DELETEALL;delete.header=header
        self.pub_markers.publish(MarkerArray(markers=[delete]))
        self.pub_poses.publish(PoseArray(header=header))
        self.pub_cloud.publish(point_cloud2.create_cloud_xyz32(header,[]))
        self.pub_status.publish(String(data=json.dumps({'valid':False,'reason':reason,'objects':[],'selected_id':None,'frame':header.frame_id})))

    def watchdog(self):
        with self.lock:missing=time.monotonic()-self.received_at>2.;packet=self.latest
        if missing:
            from std_msgs.msg import Header
            self.clear(packet[0].header if packet else Header(),'No fresh synchronized RGB-D input')

    def process(self):
        with self.lock:packet=self.latest
        if packet is None or packet is self.last_processed:return
        self.last_processed=packet;c,d,info,di=packet;header=c.header
        try:
            stamp=header.stamp.sec+header.stamp.nanosec*1e-9
            age=self.get_clock().now().nanoseconds*1e-9-stamp
            if stamp<=0 or not -.1<=age<=self.max_age:raise ValueError('Stale RGB-D frame')
            if len({(m.width,m.height) for m in packet})!=1:raise ValueError('RGB-D dimensions mismatch')
            if not header.frame_id or any(m.header.frame_id!=header.frame_id for m in packet):raise ValueError('Optical frames differ')
            if info.distortion_model not in ('','plumb_bob','rational_polynomial'):raise ValueError('Unsupported distortion model')
            if not np.allclose(info.k,di.k,rtol=1e-5,atol=1e-5):raise ValueError('Aligned depth intrinsics mismatch')
            bgr=self.bridge.imgmsg_to_cv2(c,'bgr8')
            raw=self.bridge.imgmsg_to_cv2(d,'passthrough')
            if d.encoding=='16UC1':depth=raw.astype(np.float32)*.001
            elif d.encoding=='32FC1':depth=raw.astype(np.float32)
            else:raise ValueError('Unsupported depth encoding')
            k=np.asarray(info.k).reshape(3,3)
            result=self.model.predict(bgr,device=self.device,conf=self.conf,imgsz=640,
                                      retina_masks=True,verbose=False,max_det=15,classes=self.class_ids)[0]
            if self.get_clock().now().nanoseconds*1e-9-stamp>self.max_age:raise ValueError('Inference result too old; reduce inference load')
            overlay=result.plot();image=self.bridge.cv2_to_imgmsg(overlay,'bgr8');image.header=header
            self.pub_image.publish(image)
            poses=PoseArray(header=header);markers=[];clouds=[];records=[];selected=None
            delete=Marker();delete.action=Marker.DELETEALL;delete.header=header;markers.append(delete)
            if result.masks is not None:
                masks=result.masks.data.cpu().numpy();boxes=result.boxes
                for index,mask in enumerate(masks):
                    label=result.names[int(boxes.cls[index])];score=float(boxes.conf[index])
                    if self.classes and label not in self.classes:continue
                    record={'id':index,'class':label,'confidence':score,'valid':False};records.append(record)
                    try:
                        if mask.shape!=depth.shape:raise ValueError('Segmentation mask not at input resolution')
                        points,rgb=object_points(mask>.5,depth,bgr,k,info.d,self.min_depth,self.max_depth)
                        packed=(rgb[:,0].astype(np.uint32)<<16)|(rgb[:,1].astype(np.uint32)<<8)|rgb[:,2].astype(np.uint32)
                        clouds.extend((float(p[0]),float(p[1]),float(p[2]),int(color),index) for p,color in zip(points,packed))
                        record['point_count']=len(points)
                        if self.backend=='color_cube':
                            transform=self.tf.lookup_transform('world',header.frame_id,rclpy.time.Time.from_msg(header.stamp))
                            q=transform.transform.rotation
                            up=Rotation.from_quat([q.x,q.y,q.z,q.w]).inv().apply([0.,0.,1.])
                            grasp=cube_grasp(points,up,self.cube_size)
                            if grasp['opening']>self.max_opening:raise ValueError('Cube exceeds gripper opening')
                            record.update(position_semantics='known_size_cube_center',cube_size_m=self.cube_size,
                                          confidence_semantics='color_rule_match_not_probability')
                        else:grasp=pca_grasp(points,self.max_opening)
                        pose=Pose();pose.position.x,pose.position.y,pose.position.z=map(float,grasp['center'])
                        pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=map(float,grasp['quaternion'])
                        poses.poses.append(pose)
                        record.update(valid=True,opening_m=grasp['opening'],center_m=grasp['center'].tolist(),quaternion_xyzw=grasp['quaternion'].tolist(),observed_extent_m=grasp['extent'].tolist())
                        if selected is None or score>selected[0]:selected=(score,index,pose)
                        box=Marker();box.header=header;box.ns='observed_obb';box.id=index;box.type=Marker.CUBE;box.pose=pose
                        box.scale.x,box.scale.y,box.scale.z=map(float,np.maximum(grasp['extent'],.002))
                        box.color.r=.1;box.color.g=1.;box.color.b=.2;box.color.a=.3;box.lifetime.sec=2;markers.append(box)
                        for axis in range(3):
                            arrow=Marker();arrow.header=header;arrow.ns=f'grasp_axis_{axis}';arrow.id=index;arrow.type=Marker.ARROW
                            from geometry_msgs.msg import Point
                            a=grasp['center'];b=a+grasp['axes'][:,axis]*.06
                            arrow.points=[Point(x=float(v[0]),y=float(v[1]),z=float(v[2])) for v in (a,b)]
                            arrow.scale.x=.003;arrow.scale.y=.007;arrow.scale.z=.01
                            arrow.color.r=float(axis==0);arrow.color.g=float(axis==1);arrow.color.b=float(axis==2);arrow.color.a=1.;arrow.lifetime.sec=2;markers.append(arrow)
                        text=Marker();text.header=header;text.ns='labels';text.id=index;text.type=Marker.TEXT_VIEW_FACING;text.pose=pose
                        # A separate pose avoids mutating the grasp candidate.
                        import copy
                        text.pose=copy.deepcopy(pose);text.pose.position.z-=.03;text.scale.z=.018;text.color.r=text.color.g=text.color.b=text.color.a=1.
                        text.text=f'{label} {score:.2f} opening={grasp["opening"]:.3f}m';text.lifetime.sec=2;markers.append(text)
                    except ValueError as e:record['reason']=str(e)
            fields=[PointField(name=n,offset=4*i,datatype=PointField.FLOAT32 if i<3 else PointField.UINT32,count=1) for i,n in enumerate(('x','y','z','rgb','object_id'))]
            self.pub_cloud.publish(point_cloud2.create_cloud(header,fields,clouds))
            self.pub_markers.publish(MarkerArray(markers=markers));self.pub_poses.publish(poses)
            if selected:self.pub_selected.publish(PoseStamped(header=header,pose=selected[2]))
            status={'valid':bool(selected),'reason':'' if selected else 'No eligible grasp candidate','frame':header.frame_id,'stamp_sec':stamp,
                    'selected_id':selected[1] if selected else None,'objects':records,'processing_age_sec':self.get_clock().now().nanoseconds*1e-9-stamp}
            self.pub_status.publish(String(data=json.dumps(status)));self.last_publish=time.monotonic();self.last_error=''
        except Exception as e:
            self.clear(header,str(e))
            if str(e)!=self.last_error:self.get_logger().error(str(e));self.last_error=str(e)


def main(args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    node=SegmentGrasp();executor=MultiThreadedExecutor(num_threads=2);executor.add_node(node)
    try:executor.spin()
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
