#!/usr/bin/env python3
"""Synthetic marker/TF and manual-service regression in domain195; no hardware."""
import os
os.environ['ROS_DOMAIN_ID']='195';os.environ['ROS_LOCALHOST_ONLY']='1';os.environ['ROS_LOG_DIR']='/tmp/graspgen-handeye-check'
import cv2,numpy as np,rclpy,time,threading,tempfile,json
from pathlib import Path
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
from graspgen_handeye.collect import Collector

tmp=tempfile.TemporaryDirectory(prefix='graspgen-handeye-test-');path=Path(tmp.name)/'samples.json'
rclpy.init(args=['--ros-args','-p',f'samples_file:={path}'])
collector=Collector();node=Node('synthetic_handeye_test');bridge=CvBridge();broadcaster=TransformBroadcaster(node)
image_pub=node.create_publisher(Image,'/camera/color/image_raw',qos_profile_sensor_data)
info_pub=node.create_publisher(CameraInfo,'/camera/color/camera_info',qos_profile_sensor_data)
seen={};sub=node.create_subscription(String,'/handeye/status',lambda m:seen.update(json.loads(m.data)),10)
k=np.array([[650.,0,320.],[0,650.,240.],[0,0,1.]])
h=.095/2;obj=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]])
pixels=cv2.projectPoints(obj,np.array([2.7,.2,.1]),np.array([.015,-.02,.55]),k,np.zeros(5))[0].reshape(4,2).astype(np.float32)
marker=cv2.aruco.drawMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),0,300)
warp=cv2.getPerspectiveTransform(np.array([[0,0],[299,0],[299,299],[0,299]],np.float32),pixels)
canvas=cv2.warpPerspective(marker,warp,(640,480),borderValue=255)
visible=True

def publish():
 stamp=node.get_clock().now().to_msg()
 tf=TransformStamped();tf.header.stamp=stamp;tf.header.frame_id='base_link';tf.child_frame_id='link6';tf.transform.rotation.w=1.;broadcaster.sendTransform(tf)
 image=bridge.cv2_to_imgmsg(cv2.cvtColor(canvas if visible else np.full_like(canvas,255),cv2.COLOR_GRAY2BGR),'bgr8')
 info=CameraInfo();info.width=640;info.height=480;info.k=k.ravel().tolist();info.d=[0.]*5;info.distortion_model='plumb_bob'
 for msg,pub in [(image,image_pub),(info,info_pub)]:
  msg.header.stamp=stamp;msg.header.frame_id='camera_color_optical_frame';pub.publish(msg)
timer=node.create_timer(.1,publish)
executor=MultiThreadedExecutor(num_threads=5);executor.add_node(collector);executor.add_node(node)
thread=threading.Thread(target=executor.spin);thread.start()
def wait(test,timeout=10):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  if test():return
  time.sleep(.05)
 raise AssertionError(str(seen))
def call(service):
 client=node.create_client(Trigger,service);assert client.wait_for_service(timeout_sec=3)
 future=client.call_async(Trigger.Request());wait(future.done)
 response=future.result();node.destroy_client(client);return response
try:
 wait(lambda:seen.get('capture_ready'))
 response=call('/graspgen_handeye/capture');assert response.success,response.message
 data=json.loads(path.read_text());assert len(data['samples'])==1
 assert data['marker_length_m']==.095 and data['marker_id']==0
 assert np.linalg.norm(np.array(data['samples'][0]['camera_target'])[:3,3]-[.015,-.02,.55])<.005
 response=call('/graspgen_handeye/capture');assert not response.success,'Duplicate pose should be rejected'
 visible=False;wait(lambda:not seen.get('capture_ready') and 'not detected' in seen.get('reason',''))
 assert not call('/graspgen_handeye/capture').success
 assert call('/graspgen_handeye/undo').success
 assert not json.loads(path.read_text())['samples']
 print('PASS: real ArUco detection + synchronized synthetic TF, stationary Trigger capture/persist, duplicate/missing-marker rejection, undo')
finally:
 executor.shutdown();thread.join(timeout=3);collector.destroy_node();node.destroy_node();rclpy.shutdown();tmp.cleanup()
