#!/usr/bin/env python3
"""Isolated ROS plumbing test with synthetic RGB-D and mocked inference; no hardware."""
import os
os.environ['ROS_DOMAIN_ID']='194';os.environ['ROS_LOCALHOST_ONLY']='1';os.environ['ROS_LOG_DIR']='/tmp/graspgen-vision-test'
import sys,json,time,threading
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo,PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PoseArray,PoseStamped
from std_msgs.msg import String
from cv_bridge import CvBridge

class Tensor:
    def __init__(self,a):self.a=a
    def cpu(self):return self
    def numpy(self):return self.a
class FakeYOLO:
    task='segment';detect=True
    names={39:'bottle',41:'cup',64:'mouse',67:'cell phone',49:'orange'}
    def predict(self,bgr,**kwargs):
        mask=np.zeros(bgr.shape[:2],np.float32);mask[30:90,30:130]=1
        return [S(masks=S(data=Tensor(mask[None])) if self.detect else None,
                  boxes=S(cls=[39],conf=[.9]),names={39:'bottle'},plot=lambda:bgr)]
net=FakeYOLO()
sys.modules['torch']=S(set_num_threads=lambda _:None)
sys.modules['ultralytics']=S(YOLO=lambda *a,**kw:net)
from graspgen_vision.node import SegmentGrasp
model=Path('/tmp/graspgen-vision-mock.pt');model.touch()
rclpy.init(args=['--ros-args','-p',f'model:={model}'])
vision=SegmentGrasp();test=Node('synthetic_rgbd_test');cv=CvBridge();seen={};publishing=True
pubs=[test.create_publisher(kind,topic,qos_profile_sensor_data) for kind,topic in [
 (Image,'/camera/color/image_raw'),(Image,'/camera/aligned_depth_to_color/image_raw'),
 (CameraInfo,'/camera/color/camera_info'),(CameraInfo,'/camera/aligned_depth_to_color/camera_info')]]
subs=[test.create_subscription(kind,topic,lambda msg,key=key:seen.update({key:msg}),10) for key,kind,topic in [
 ('status',String,'/graspgen/status'),('cloud',PointCloud2,'/graspgen/object_cloud'),
 ('poses',PoseArray,'/graspgen/grasp_poses'),('selected',PoseStamped,'/graspgen/selected_grasp')]]
def publish():
    if not publishing:return
    rgb=cv.cv2_to_imgmsg(np.full((120,160,3),100,np.uint8),'bgr8')
    depth=cv.cv2_to_imgmsg(np.full((120,160),500,np.uint16),'16UC1')
    info=CameraInfo();info.width=160;info.height=120;info.k=[500.,0.,80.,0.,500.,60.,0.,0.,1.];info.distortion_model='plumb_bob';info.d=[0.]*5
    stamp=test.get_clock().now().to_msg()
    for pub,msg in zip(pubs,[rgb,depth,info,info]):
        msg.header.stamp=stamp;msg.header.frame_id='camera_color_optical_frame';pub.publish(msg)
timer=test.create_timer(.1,publish)
executor=MultiThreadedExecutor(num_threads=4);executor.add_node(vision);executor.add_node(test)
thread=threading.Thread(target=executor.spin);thread.start()
def wait(pred,timeout=10):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if pred():return
        time.sleep(.05)
    raise AssertionError(str({k:(v.data if k=='status' else type(v).__name__) for k,v in seen.items()}))
try:
    wait(lambda:'selected' in seen and 'status' in seen and json.loads(seen['status'].data)['valid'])
    assert len(seen['poses'].poses)==1
    assert seen['cloud'].width>80 and seen['cloud'].header.frame_id=='camera_color_optical_frame'
    assert seen['selected'].pose.position.z==.5
    fields=[f.name for f in seen['cloud'].fields];assert fields==['x','y','z','rgb','object_id']
    data=list(point_cloud2.read_points(seen['cloud'],field_names=('z',),skip_nans=True));assert all(abs(float(p[0])-.5)<1e-6 for p in data)
    selected_stamp=seen['selected'].header.stamp
    net.detect=False
    wait(lambda:not json.loads(seen['status'].data)['valid'] and not seen['poses'].poses and seen['cloud'].width==0)
    time.sleep(.4);assert seen['selected'].header.stamp==selected_stamp,'Miss must not republish stale selected pose'
    publishing=False
    wait(lambda:'No fresh' in json.loads(seen['status'].data)['reason'],5)
    print('PASS: synchronized ROS2 mask -> metric RGB cloud -> PCA pose/status; empty detections and input timeout clear outputs; no stale selected pose replay')
finally:
    executor.shutdown();thread.join(timeout=3);vision.destroy_node();test.destroy_node();rclpy.shutdown();model.unlink(missing_ok=True)
