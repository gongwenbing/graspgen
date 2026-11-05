"""ROS 2 perception only: no target_pose, joint commands or execution services."""
import json
import threading
import time
from pathlib import Path
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
from std_msgs.msg import String
from .geometry import object_points,mask_depth_statistics,scene_points
from .cubes import CubeDetector


class SegmentGrasp(Node):
    def __init__(self):
        super().__init__('graspgen_vision')
        self.backend=self.declare_parameter('backend','yolo').value
        if self.backend not in ('yolo','color_cube'):raise ValueError('backend must be yolo or color_cube')
        self.cube_size=float(self.declare_parameter('cube_size',.04).value)
        if not .02<=self.cube_size<=.06:raise ValueError('cube_size must be 0.02..0.06m')
        self.model_path=self.declare_parameter('model','').value
        self.device=self.declare_parameter('device','cpu').value
        self.imgsz=self.declare_parameter('imgsz',960).value
        if isinstance(self.imgsz,bool) or not isinstance(self.imgsz,int) or not 320<=self.imgsz<=1280 or self.imgsz%32:
            raise ValueError('imgsz must be an integer multiple of32 within320..1280')
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
        if self.model.task!='segment':raise ValueError('Instance segmentation weights required')
        unknown=self.classes-set(self.model.names.values())
        if unknown:raise ValueError(f'Target classes absent from model: {sorted(unknown)}')
        self.class_ids=[i for i,name in self.model.names.items() if name in self.classes] if self.classes else None
        self.bridge=CvBridge();self.lock=threading.Lock();self.latest=None;self.last_processed=None
        self.received_at=0.;self.last_error='';self.last_publish=0.
        self.pub_image=self.create_publisher(Image,'/graspgen/segmentation/image',1)
        self.pub_cloud=self.create_publisher(PointCloud2,'/graspgen/object_cloud',1)
        self.pub_scene=self.create_publisher(PointCloud2,'/graspgen/scene_cloud',2)
        self.pub_objects=self.create_publisher(String,'/graspgen/segmentation/objects',5)
        self.group=ReentrantCallbackGroup()
        kwargs={'qos_profile':qos_profile_sensor_data,'callback_group':self.group}
        self.subs=[Subscriber(self,kind,topic,**kwargs) for kind,topic in [
            (Image,'/camera/color/image_raw'),(Image,'/camera/aligned_depth_to_color/image_raw'),
            (CameraInfo,'/camera/color/camera_info'),(CameraInfo,'/camera/aligned_depth_to_color/camera_info')]]
        self.sync=ApproximateTimeSynchronizer(self.subs,queue_size=5,slop=.05)
        self.sync.registerCallback(self.input)
        self.create_timer(self.period,self.process)
        self.create_timer(1.,self.watchdog,callback_group=self.group)
        self.get_logger().info(f'{self.backend} segmentation ready; imgsz={self.imgsz}, confidence={self.conf}; perception only, no arm output')

    def input(self,*msgs):
        with self.lock:self.latest=msgs;self.received_at=time.monotonic()

    def clear(self,header,reason):
        self.pub_cloud.publish(point_cloud2.create_cloud_xyz32(header,[]))
        self.publish_objects(header,[],reason)

    def publish_objects(self,header,records,reason=''):
        self.pub_objects.publish(String(data=json.dumps({
            'schema_version':1,'stamp':{'sec':header.stamp.sec,'nanosec':header.stamp.nanosec},
            'frame':header.frame_id,'backend':self.backend,'cube_size_m':self.cube_size,
            'reason':reason,'objects':records})))

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
            result=self.model.predict(bgr,device=self.device,conf=self.conf,imgsz=self.imgsz,
                                      retina_masks=True,verbose=False,max_det=15,classes=self.class_ids)[0]
            if self.get_clock().now().nanoseconds*1e-9-stamp>self.max_age:raise ValueError('Inference result too old; reduce inference load')
            overlay=result.plot();image=self.bridge.cv2_to_imgmsg(overlay,'bgr8');image.header=header
            self.pub_image.publish(image)
            observed=scene_points(depth,k,info.d,self.min_depth,self.max_depth)
            self.pub_scene.publish(point_cloud2.create_cloud_xyz32(header,observed.tolist()))
            clouds=[];records=[]
            if result.masks is not None:
                masks=result.masks.data.cpu().numpy();boxes=result.boxes
                for index,mask in enumerate(masks):
                    label=result.names[int(boxes.cls[index])];score=float(boxes.conf[index])
                    if self.classes and label not in self.classes:continue
                    record={'id':index,'class':label,'confidence':score,'cloud_valid':False};records.append(record)
                    try:
                        if mask.shape!=depth.shape:raise ValueError('Segmentation mask not at input resolution')
                        record['depth_diagnostics']=mask_depth_statistics(mask>.5,depth,self.min_depth,self.max_depth)
                        points,rgb=object_points(mask>.5,depth,bgr,k,info.d,self.min_depth,self.max_depth)
                        packed=(rgb[:,0].astype(np.uint32)<<16)|(rgb[:,1].astype(np.uint32)<<8)|rgb[:,2].astype(np.uint32)
                        clouds.extend((float(p[0]),float(p[1]),float(p[2]),int(color),index) for p,color in zip(points,packed))
                        record.update(cloud_valid=True,point_count=len(points))
                    except ValueError as e:record['reason']=str(e)
            fields=[PointField(name=n,offset=4*i,datatype=PointField.FLOAT32 if i<3 else PointField.UINT32,count=1) for i,n in enumerate(('x','y','z','rgb','object_id'))]
            self.pub_cloud.publish(point_cloud2.create_cloud(header,fields,clouds))
            self.publish_objects(header,records)
            self.last_publish=time.monotonic();self.last_error=''
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
