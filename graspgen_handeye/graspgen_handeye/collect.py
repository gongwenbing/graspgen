"""Read-only eye-in-hand observation; Trigger captures, never moves the arm."""
import copy,json,threading,time
from collections import deque
from pathlib import Path
import cv2,numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from message_filters import Subscriber,ApproximateTimeSynchronizer
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from cv_bridge import CvBridge
from tf2_ros import Buffer,TransformListener
from .core import detect,matrix,distance


class Collector(Node):
    def __init__(self,name='graspgen_handeye',default_samples_file=None,topic_prefix='/handeye',extra_meta=None):
        super().__init__(name)
        self.marker=int(self.declare_parameter('marker_id',0).value)
        self.length=float(self.declare_parameter('marker_length',.095).value)
        self.dictionary=self.declare_parameter('dictionary','DICT_4X4_50').value
        self.base=self.declare_parameter('base_frame','base_link').value
        self.gripper=self.declare_parameter('gripper_frame','link6').value
        self.camera=self.declare_parameter('camera_frame','camera_color_optical_frame').value
        self.path=Path(self.declare_parameter('samples_file',str(default_samples_file or Path.cwd()/'calibration/handeye_samples.json')).value).expanduser()
        if not 0<self.length<1 or not 0<=self.marker<50:raise ValueError('Invalid marker parameters')
        self.meta={'schema':1,'base_frame':self.base,'gripper_frame':self.gripper,'camera_frame':self.camera,
                   'marker_id':self.marker,'marker_length_m':self.length,'dictionary':self.dictionary}
        self.meta.update(extra_meta or {})
        self.samples=[]
        if self.path.exists():
            stored=json.loads(self.path.read_text())
            if any(stored.get(k)!=v for k,v in self.meta.items()):raise ValueError('Existing sample metadata differs; choose another samples_file')
            self.samples=stored['samples']
        self.lock=threading.RLock();self.history=deque(maxlen=12);self.latest=None;self.observation=None;self.last_input=0.;self.last_process=0.;self.error='Waiting for camera'
        self.bridge=CvBridge();self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.pub_image=self.create_publisher(Image,topic_prefix+'/image',1)
        self.pub_pose=self.create_publisher(PoseStamped,topic_prefix+'/marker_pose',1)
        self.pub_status=self.create_publisher(String,topic_prefix+'/status',1)
        self.group=ReentrantCallbackGroup()
        self.subs=[Subscriber(self,t,topic,qos_profile=qos_profile_sensor_data,callback_group=self.group) for t,topic in [
            (Image,'/camera/color/image_raw'),(CameraInfo,'/camera/color/camera_info')]]
        self.sync=ApproximateTimeSynchronizer(self.subs,5,.03);self.sync.registerCallback(self.observe)
        self.create_service(Trigger,'~/capture',self.capture,callback_group=self.group)
        self.create_service(Trigger,'~/undo',self.undo,callback_group=self.group)
        self.create_timer(.2,self.status,callback_group=self.group)
        self.get_logger().info(f'Marker {self.marker}, {self.length:.3f} m, {self.dictionary}; {len(self.samples)} saved samples; NO arm control')

    def stable(self):
        if self.latest is None or time.monotonic()-self.last_input>.3:return False
        if len(self.history)<5 or self.history[-1][0]-self.history[0][0]<.5:return False
        g=self.latest['base_gripper'];c=self.latest['camera_target']
        return all(distance(g,x[1])[0]<.002 and distance(g,x[1])[1]<np.deg2rad(.5)
                   and distance(c,x[2])[0]<.003 and distance(c,x[2])[1]<np.deg2rad(1) for x in self.history)

    def observe(self,image,info):
        now=time.monotonic()
        if now-self.last_process<.08:return
        self.last_process=now;canvas=None;detected=False
        try:
            age=(self.get_clock().now()-Time.from_msg(image.header.stamp)).nanoseconds*1e-9
            if not -.1<=age<=.3 or image.header.stamp.sec==0:raise ValueError('Stale camera timestamp')
            if image.header.frame_id!=self.camera or info.header.frame_id!=self.camera:raise ValueError('Camera frame mismatch')
            if (image.width,image.height)!=(info.width,info.height):raise ValueError('Image/intrinsics size mismatch')
            if info.distortion_model not in ('','plumb_bob','rational_polynomial'):raise ValueError('Unsupported distortion model')
            k=np.array(info.k).reshape(3,3);d=np.array(info.d,dtype=float)
            if not np.isfinite(k).all() or k[0,0]<=0 or k[1,1]<=0:raise ValueError('Invalid camera intrinsics')
            canvas=self.bridge.imgmsg_to_cv2(image,'bgr8').copy()
            c,pixels,error,ambiguous=detect(cv2.cvtColor(canvas,cv2.COLOR_BGR2GRAY),k,d,self.marker,self.length,self.dictionary)
            detected=True
            with self.lock:self.observation=(now,error,ambiguous)
            cv2.polylines(canvas,[pixels.astype(np.int32)],True,(0,165,255) if ambiguous else (0,255,0),2)
            cv2.drawFrameAxes(canvas,k,d,cv2.Rodrigues(c[:3,:3])[0],c[:3,3],.04)
            cv2.putText(canvas,f'ID {self.marker} {self.length*1000:g}mm reproj={error:.2f}px'+(' AMBIGUOUS' if ambiguous else ''),(10,25),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,0),1)
            pose=PoseStamped();pose.header=image.header
            pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,c[:3,3])
            pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w=map(float,Rotation.from_matrix(c[:3,:3]).as_quat())
            self.pub_pose.publish(pose)
            if ambiguous:raise ValueError('Planar pose ambiguous: tilt view by 15-40 degrees')
            tf=self.buffer.lookup_transform(self.base,self.gripper,Time.from_msg(image.header.stamp),timeout=Duration(seconds=.05))
            q=tf.transform.rotation;t=tf.transform.translation
            g=matrix(Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix(),[t.x,t.y,t.z])
            with self.lock:
                self.latest={'base_gripper':g,'camera_target':c,'stamp_sec':image.header.stamp.sec+image.header.stamp.nanosec/1e9,
                    'stamp_ns':image.header.stamp.sec*1000000000+image.header.stamp.nanosec,'reprojection_error_px':error}
                self.last_input=now;self.history.append((now,g,c));self.error=''
        except Exception as e:
            with self.lock:
                self.error=str(e);self.latest=None;self.history.clear()
                if not detected:self.observation=None
        if canvas is not None:
            out=self.bridge.cv2_to_imgmsg(canvas,'bgr8');out.header=image.header;self.pub_image.publish(out)

    def persist(self):
        self.path.parent.mkdir(parents=True,exist_ok=True)
        temporary=self.path.with_suffix('.tmp')
        temporary.write_text(json.dumps(dict(self.meta,samples=self.samples),indent=2)+'\n');temporary.replace(self.path)

    def capture(self,request,response):
        with self.lock:
            if not self.stable():response.success=False;response.message=self.error or 'Wait stationary for >=1 second with a clear, tilted marker view';return response
            s=self.latest
            for old in self.samples:
                shift,angle=distance(np.array(old['base_gripper']),s['base_gripper'])
                if shift<.01 and angle<np.deg2rad(5):
                    response.success=False;response.message='Pose duplicates an existing sample (<10mm and <5deg)';return response
            saved=copy.deepcopy(s);saved['base_gripper']=saved['base_gripper'].tolist();saved['camera_target']=saved['camera_target'].tolist()
            self.samples.append(saved)
            try:self.persist()
            except Exception as e:
                self.samples.pop();response.success=False;response.message=f'Could not save: {e}';return response
            response.success=True;response.message=f'Saved sample {len(self.samples)} to {self.path}; aim for 20-25 diverse poses';return response

    def undo(self,request,response):
        with self.lock:
            if not self.samples:response.success=False;response.message='No saved sample';return response
            removed=self.samples.pop()
            try:self.persist()
            except Exception as e:
                self.samples.append(removed);response.success=False;response.message=str(e);return response
            response.success=True;response.message=f'Removed last sample; {len(self.samples)} remain';return response

    def status(self):
        with self.lock:
            fresh=self.latest is not None and time.monotonic()-self.last_input<=.3
            visible=self.observation is not None and time.monotonic()-self.observation[0]<=.3
            self.pub_status.publish(String(data=json.dumps({'marker_id':self.marker,'marker_length_m':self.length,'dictionary':self.dictionary,
                'marker_visible':visible,'pose_ambiguous':self.observation[2] if visible else None,
                'sample_count':len(self.samples),'capture_ready':bool(fresh and self.stable()),'reason':self.error if fresh else self.error or 'No fresh valid observation',
                'reprojection_error_px':self.observation[1] if visible else None})))


def main(args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    node=Collector();executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
    try:executor.spin()
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
