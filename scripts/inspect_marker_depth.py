#!/usr/bin/env python3
"""Read-only ArUco/PnP vs aligned-depth vs image-time TF comparison.

No services, actions, CAN, motor, mode or calibration writes. Only a report.
"""
import argparse
import json
import time
from collections import Counter, OrderedDict
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation
from graspgen_handeye.core import detect


def stamp_key(message):
    return message.header.stamp.sec*10**9+message.header.stamp.nanosec


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=15.)
    parser.add_argument('--output',default=str(Path.home()/'GraspGen/calibration/marker_depth_inspection.json'))
    args=parser.parse_args()
    rclpy.init();node=rclpy.create_node('marker_depth_readonly_inspector')
    buffer=Buffer();listener=TransformListener(buffer,node);bridge=CvBridge()
    queues={key:OrderedDict() for key in ('rgb','depth')};infos={};status={};subs=[]
    def receive_image(key,message):
        queues[key][stamp_key(message)]=message
        while len(queues[key])>40:queues[key].popitem(last=False)
    for key,topic in [('rgb','/camera/color/image_raw'),('depth','/camera/aligned_depth_to_color/image_raw')]:
        subs.append(node.create_subscription(Image,topic,lambda m,key=key:receive_image(key,m),qos_profile_sensor_data))
    for key,topic in [('rgb','/camera/color/camera_info'),('depth','/camera/aligned_depth_to_color/camera_info')]:
        subs.append(node.create_subscription(CameraInfo,topic,lambda m,key=key:infos.update({key:m}),qos_profile_sensor_data))
    for key,topic in [('hardware','/piper_hardware/status'),('vision','/graspgen/status')]:
        subs.append(node.create_subscription(String,topic,lambda m,key=key:status.update({key:json.loads(m.data)}),5))
    samples=[];processed=set();failures=Counter();end=time.monotonic()+args.seconds;last_attempt=0.;last_image=None
    try:
        while time.monotonic()<end and len(samples)<20:
            rclpy.spin_once(node,timeout_sec=.01)
            if time.monotonic()-last_attempt<.10:continue
            if len(infos)!=2:continue
            keys=sorted((set(queues['rgb'])&set(queues['depth']))-processed,reverse=True)
            if not keys:continue
            key=keys[0];rgb=queues['rgb'][key];depth=queues['depth'][key]
            if not -.1<=node.get_clock().now().nanoseconds*1e-9-key/1e9<=1.:processed.add(key);continue
            if not buffer.can_transform('base_link',rgb.header.frame_id,Time.from_msg(rgb.header.stamp)):continue
            processed.add(key);last_attempt=time.monotonic()
            try:
                info=infos['rgb'];di=infos['depth']
                if len({m.header.frame_id for m in (rgb,depth,info,di)})!=1 or len({(m.width,m.height) for m in (rgb,depth,info,di)})!=1:
                    raise ValueError('RGB/depth/info frame or dimensions mismatch')
                if not np.allclose(info.k,di.k,rtol=1e-5,atol=1e-5):raise ValueError('Aligned depth intrinsics mismatch')
                k=np.array(info.k).reshape(3,3);d=np.array(info.d)
                bgr=bridge.imgmsg_to_cv2(rgb,'bgr8');last_image=bgr
                marker,pixels,error,ambiguous=detect(cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY),k,d,0,.095)
                raw=bridge.imgmsg_to_cv2(depth,'passthrough')
                if depth.encoding not in ('16UC1','32FC1'):raise ValueError('Unsupported depth encoding')
                z=raw.astype(float)*(.001 if depth.encoding=='16UC1' else 1.)
                mask=np.zeros(z.shape,np.uint8);cv2.fillConvexPoly(mask,pixels.astype(np.int32),1)
                mask=cv2.erode(mask,np.ones((7,7),np.uint8)).astype(bool)
                valid=mask&np.isfinite(z)&(z>.2)&(z<1.5);v,u=np.nonzero(valid)
                if len(u)<80:raise ValueError('Insufficient marker depth')
                rays=cv2.undistortPoints(np.column_stack([u,v]).astype(float).reshape(-1,1,2),k,d).reshape(-1,2)
                rays=np.column_stack([rays,np.ones(len(rays))]);normal=marker[:3,2]
                denom=rays@normal
                if np.min(abs(denom))<.05:raise ValueError('Grazing marker view')
                predicted_z=(normal@marker[:3,3])/denom
                measured=rays*z[v,u,None];predicted=rays*predicted_z[:,None]
                tf=buffer.lookup_transform('base_link',rgb.header.frame_id,Time.from_msg(rgb.header.stamp))
                q=tf.transform.rotation;t=tf.transform.translation
                rotation=Rotation.from_quat([q.x,q.y,q.z,q.w]);translation=np.array([t.x,t.y,t.z])
                base_marker=rotation.apply(marker[:3,3])+translation
                base_measured=rotation.apply(measured)+translation
                base_predicted=rotation.apply(predicted)+translation
                base_camera=np.eye(4);base_camera[:3,:3]=rotation.as_matrix();base_camera[:3,3]=translation
                gripper_tf=buffer.lookup_transform('base_link','link6',Time.from_msg(rgb.header.stamp))
                gq=gripper_tf.transform.rotation;gt=gripper_tf.transform.translation
                base_gripper=np.eye(4);base_gripper[:3,:3]=Rotation.from_quat([gq.x,gq.y,gq.z,gq.w]).as_matrix();base_gripper[:3,3]=[gt.x,gt.y,gt.z]
                sample=dict(stamp_sec=key/1e9,reprojection_error_px=float(error),pose_ambiguous=bool(ambiguous),
                    camera_marker_matrix=marker.tolist(),base_camera_matrix=base_camera.tolist(),base_gripper_matrix=base_gripper.tolist(),
                    marker_center_base_m=base_marker.tolist(),marker_depth_coverage=float(valid.sum()/max(mask.sum(),1)),
                    marker_depth_z_base_median_mm=float(np.median(base_measured[:,2])*1000),
                    depth_minus_pnp_camera_z_median_mm=float(np.median(z[v,u]-predicted_z)*1000),
                    depth_minus_pnp_base_z_median_mm=float(np.median(base_measured[:,2]-base_predicted[:,2])*1000),
                    depth_plane_residual_median_mm=float(np.median((measured-marker[:3,3])@normal)*1000),
                    hardware=status.get('hardware'))
                samples.append(sample)
            except Exception as exc:failures[str(exc)]+=1
        reliable=[s for s in samples if not s['pose_ambiguous'] and s['reprojection_error_px']<=1.5 and s['marker_depth_coverage']>=.6]
        report=dict(read_only=True,marker_id=0,marker_length_m=.095,sample_count=len(samples),
                    reliable_sample_count=len(reliable),failure_counts=dict(failures),samples=samples,
                    latest_status=status,images_received={k:len(v) for k,v in queues.items()},camera_info_received=list(infos))
        if reliable:
            p=np.array([s['marker_center_base_m'] for s in reliable])*1000
            summary=dict(marker_pnp_base_z_median_mm=float(np.median(p[:,2])),
                marker_depth_base_z_median_mm=float(np.median([s['marker_depth_z_base_median_mm'] for s in reliable])),
                depth_minus_pnp_base_z_median_mm=float(np.median([s['depth_minus_pnp_base_z_median_mm'] for s in reliable])),
                depth_minus_pnp_camera_z_median_mm=float(np.median([s['depth_minus_pnp_camera_z_median_mm'] for s in reliable])),
                marker_position_peak_to_peak_mm=np.ptp(p,axis=0).tolist(),
                median_reprojection_error_px=float(np.median([s['reprojection_error_px'] for s in reliable])))
            report['summary']=summary
        path=Path(args.output).expanduser();path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        if last_image is not None:cv2.imwrite(str(path.with_suffix('.jpg')),last_image)
        print(json.dumps({k:v for k,v in report.items() if k not in ('samples','latest_status')},ensure_ascii=False,indent=2))
        print('Report:',path)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
