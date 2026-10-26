"""Read-only image/intrinsics check. Exit nonzero for missing/invalid RGB-D."""
import argparse
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--timeout',type=float,default=15)
    args,ros_args=parser.parse_known_args()
    if not 0 < args.timeout <= 120:parser.error('timeout must be in (0,120]')
    rclpy.init(args=ros_args)
    node=Node('graspgen_rgbd_check');bridge=CvBridge();received={};subscriptions=[]
    for key,topic,kind in [
        ('color','/camera/color/image_raw',Image),
        ('depth','/camera/aligned_depth_to_color/image_raw',Image),
        ('info','/camera/color/camera_info',CameraInfo),
        ('depth_info','/camera/aligned_depth_to_color/camera_info',CameraInfo)]:
        subscriptions.append(node.create_subscription(kind,topic,
            lambda msg,key=key:received.update({key:(msg,time.monotonic())}),qos_profile_sensor_data))
    error='Timed out waiting for fresh RGB-D';end=time.monotonic()+args.timeout
    try:
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.1)
            if len(received)!=4 or any(time.monotonic()-t>1 for _,t in received.values()):continue
            c,d,i,di=[received[k][0] for k in ('color','depth','info','depth_info')]
            if len({(m.width,m.height) for m in (c,d,i,di)})!=1:
                error='Color/depth/intrinsics dimensions mismatch';continue
            if any(m.k[0]<=0 or m.k[4]<=0 for m in (i,di)):
                error='Invalid camera focal lengths';continue
            if len({m.header.frame_id for m in (c,d,i,di)})!=1 or not c.header.frame_id:
                error='Aligned RGB-D optical frames mismatch';continue
            stamps=[m.header.stamp.sec+m.header.stamp.nanosec/1e9 for m in (c,d)]
            if min(stamps)<=0 or abs(stamps[0]-stamps[1])>.1:
                error='RGB/depth timestamps not synchronized within 100 ms';continue
            if d.encoding not in ('16UC1','32FC1'):
                error=f'Unsupported depth encoding {d.encoding}';continue
            data=bridge.imgmsg_to_cv2(d,desired_encoding='passthrough')
            valid=data[np.isfinite(data)&(data>0)]
            if not valid.size:error='Depth image contains no valid depth';continue
            scale=.001 if d.encoding=='16UC1' else 1
            print(f'PASS: {c.width}x{c.height}; color={c.encoding}; depth={d.encoding}; frame={c.header.frame_id}')
            print(f'Valid depth {valid.size/data.size:.1%}; median {float(np.median(valid))*scale:.3f} m; RGB/depth delta {abs(stamps[0]-stamps[1]):.4f} s')
            print('RGB-D reception verified; physical alignment, hand-eye calibration and object recognition are not verified.')
            return
        missing=set(('color','depth','info','depth_info'))-received.keys()
        raise RuntimeError(f'{error}; missing={sorted(missing)}')
    finally:
        node.destroy_node();rclpy.shutdown()
