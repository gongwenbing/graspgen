#!/usr/bin/env python3
"""Synthetic TF/candidates in isolated domain198, no camera/CAN or actions."""
import os
os.environ['ROS_DOMAIN_ID']='198';os.environ['ROS_LOCALHOST_ONLY']='1';os.environ['ROS_LOG_DIR']='/tmp/graspgen-preview-check'
import json,time,threading,tempfile
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import TransformStamped,PoseStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from graspgen_handeye.preview import Preview
config_dir=tempfile.TemporaryDirectory(prefix='graspgen-preview-')
config_path=Path(config_dir.name)/'tcp.json';config_path.write_text(json.dumps({'translation_m':[0,0,.1308],'translation_verified':False}))
rclpy.init(args=['--ros-args','-p',f'tcp_config_file:={config_path}']);preview=Preview();node=Node('synthetic_preview_test');broadcaster=TransformBroadcaster(node)
pose_pub=node.create_publisher(PoseStamped,'/graspgen/selected_grasp',1);status_pub=node.create_publisher(String,'/graspgen/status',1)
seen={};valid=True
subs=[node.create_subscription(t,topic,lambda m,k=k:seen.update({k:m}),10) for k,t,topic in [
 ('status',String,'/grasp_preview/status'),('current',PoseStamped,'/grasp_preview/current_tcp'),
 ('tcp',PoseStamped,'/grasp_preview/pregrasp_tcp'),('flange',PoseStamped,'/grasp_preview/pregrasp_link6')]]
def publish():
 stamp=node.get_clock().now().to_msg()
 for child,z in [('link6',.2),('camera_color_optical_frame',0.)]:
  tf=TransformStamped();tf.header.stamp=stamp;tf.header.frame_id='base_link';tf.child_frame_id=child;tf.transform.rotation.w=1.;tf.transform.translation.z=z;broadcaster.sendTransform(tf)
 p=PoseStamped();p.header.stamp=stamp;p.header.frame_id='camera_color_optical_frame';p.pose.orientation.w=1.;p.pose.position.x=.3;p.pose.position.z=.4
 pose_pub.publish(p);status_pub.publish(String(data=json.dumps({'valid':valid,'frame':p.header.frame_id,'stamp_sec':stamp.sec+stamp.nanosec/1e9})))
timer=node.create_timer(.1,publish);ex=MultiThreadedExecutor(num_threads=3);ex.add_node(preview);ex.add_node(node)
thread=threading.Thread(target=ex.spin);thread.start()
def wait(pred):
 end=time.monotonic()+8
 while time.monotonic()<end:
  if pred():return
  time.sleep(.05)
 raise AssertionError(str(seen.get('status')))
try:
 wait(lambda:'status' in seen and json.loads(seen['status'].data).get('target_valid'))
 assert abs(seen['current'].pose.position.z-.3308)<1e-5
 assert abs(seen['tcp'].pose.position.z-.5)<1e-5
 assert abs(seen['flange'].pose.position.z-.6308)<1e-5
 valid=False;wait(lambda:not json.loads(seen['status'].data).get('target_valid'))
 print('PASS: provisional current TCP, base-Z hover, link6 conversion, invalid-candidate clearing; no hardware execution')
finally:
 ex.shutdown();thread.join(timeout=3);preview.destroy_node();node.destroy_node();rclpy.shutdown();config_dir.cleanup()
