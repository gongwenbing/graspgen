#!/usr/bin/env python3
"""Isolated ROS cloud+metadata+TF check. Synthetic bottle; no camera/CAN/actions."""
import os
os.environ['ROS_DOMAIN_ID']='196';os.environ['ROS_LOCALHOST_ONLY']='1';os.environ['ROS_LOG_DIR']='/tmp/graspgen-bottle-generator-test'
import json
import time
import threading
import tempfile
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String,Header
from sensor_msgs.msg import PointCloud2,PointField
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PoseStamped,TransformStamped
from tf2_ros import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation
from graspgen_vision.grasp_generator_node import GraspGenerator

temporary=tempfile.TemporaryDirectory(prefix='graspgen-bottle-generator-')
config_path=Path(temporary.name)/'bottle.json'
config_path.write_text(json.dumps(dict(diameter_m=.032,total_height_m=.12,body_bottom_m=.045,body_top_m=.075,grasp_height_m=.06,table_z_m=-.011)))
rclpy.init(args=['--ros-args','-p','generator:=bottle_cylinder','-p',f'bottle_config:={config_path}','-p',f'handeye_calibration:={Path(temporary.name)/"handeye.yaml"}'])
generator=GraspGenerator();probe=Node('bottle_generator_test')
cloud_pub=probe.create_publisher(PointCloud2,'/graspgen/object_cloud',5)
meta_pub=probe.create_publisher(String,'/graspgen/segmentation/objects',5)
seen={}
subs=[probe.create_subscription(kind,topic,lambda m,k=k:seen.update({k:m}),5) for k,kind,topic in [
    ('status',String,'/graspgen/status'),('selected',PoseStamped,'/graspgen/selected_grasp')]]
rotation=Rotation.from_euler('xyz',[2.8,.2,.4]);translation=np.array([.15,-.2,.4])
tf=TransformStamped();tf.header.frame_id='base_link';tf.child_frame_id='test_camera';tf.header.stamp=probe.get_clock().now().to_msg()
tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=translation
tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=rotation.as_quat()
broadcaster=StaticTransformBroadcaster(probe)
gripper=TransformStamped();gripper.header.frame_id='base_link';gripper.child_frame_id='link6';gripper.header.stamp=probe.get_clock().now().to_msg();gripper.transform.rotation.w=1.
broadcaster.sendTransform([tf,gripper])
x=np.eye(4);x[:3,:3]=rotation.as_matrix();x[:3,3]=translation
(Path(temporary.name)/'handeye.yaml').write_text(yaml.safe_dump(dict(schema=1,accepted=True,gripper_frame='link6',camera_frame='test_camera',gripper_camera_optical=x.tolist())))
a,z=np.meshgrid(np.deg2rad(np.linspace(-70,70,60)),np.linspace(.048,.072,35))
base=np.column_stack((.08+.016*np.cos(a.ravel()),-.30+.016*np.sin(a.ravel()),z.ravel()-.011))
points=rotation.inv().apply(base-translation)
fields=[PointField(name=n,offset=4*i,datatype=PointField.FLOAT32 if i<3 else PointField.UINT32,count=1) for i,n in enumerate(['x','y','z','rgb','object_id'])]
mode='valid';publishing=True

def publish():
    if not publishing:return
    stamp=probe.get_clock().now().to_msg();header=Header(stamp=stamp,frame_id='test_camera')
    records=[dict(id=0,**{'class':'bottle'},confidence=.9,cloud_valid=True,point_count=len(points))]
    if mode=='wrong_class':records[0]['class']='cup'
    data=dict(schema_version=1,stamp=dict(sec=stamp.sec,nanosec=stamp.nanosec),frame='test_camera',backend='yolo',objects=records)
    # Metadata intentionally arrives first, and no color images/YOLO are needed.
    meta_pub.publish(String(data=json.dumps(data)))
    cloud_pub.publish(point_cloud2.create_cloud(header,fields,[(*p,0,0) for p in points]))

timer=probe.create_timer(.15,publish)
executor=MultiThreadedExecutor(num_threads=2);executor.add_node(generator);executor.add_node(probe)
thread=threading.Thread(target=executor.spin);thread.start()
def wait(predicate,timeout=8):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if predicate():return
        time.sleep(.03)
    raise AssertionError(seen.get('status'))
try:
    wait(lambda:'selected' in seen and 'status' in seen and json.loads(seen['status'].data)['valid'])
    data=json.loads(seen['status'].data);obj=data['objects'][0]
    assert data['frame']=='base_link' and obj['position_semantics']=='bottle_grasp_center'
    np.testing.assert_allclose(obj['center_m'],[.08,-.30,.049],atol=.0001)
    np.testing.assert_allclose(Rotation.from_quat(obj['quaternion_xyzw']).apply([0,0,1]),[0,0,-1],atol=1e-6)
    mode='wrong_class'
    wait(lambda:not json.loads(seen['status'].data)['valid'])
    time.sleep(.3);saved=seen['selected'].header.stamp
    time.sleep(.3);assert seen['selected'].header.stamp==saved
    publishing=False
    wait(lambda:'No fresh' in json.loads(seen['status'].data)['reason'],5)
    print('PASS: independent generator, exact frame pairing, image-time TF, metric cylinder centre, wrong-class invalidation and timeout; no actuator clients')
finally:
    executor.shutdown();thread.join(timeout=3);generator.destroy_node();probe.destroy_node();rclpy.shutdown();temporary.cleanup()
