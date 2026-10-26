"""Manual independent fixed-target validation; reuses stationary detection gates."""
import argparse,copy,hashlib,json
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger
from .collect import Collector
from .core import check_transform,distance,mean_transform,matrix


def drift_report(samples,reference):
    transforms=[check_transform(s['base_target']) for s in samples]
    if not transforms:raise ValueError('No independent validation samples')
    center=mean_transform(transforms)
    deviations=np.array([distance(center,t) for t in transforms])
    reference_errors=np.array([distance(reference,t) for t in transforms])
    pairs=[distance(a,b) for i,a in enumerate(transforms) for b in transforms[i+1:]]
    pair_max=np.max(pairs,axis=0) if pairs else np.zeros(2)
    rms=np.sqrt(np.mean(deviations**2,axis=0))
    accepted=bool(len(samples)>=3 and pair_max[0]<=.01 and pair_max[1]<=np.deg2rad(3)
                  and np.max(reference_errors[:,0])<=.015 and np.max(reference_errors[:,1])<=np.deg2rad(5))
    return {'sample_count':len(samples),'accepted':accepted,'enough_samples':len(samples)>=3,
            'max_pairwise_translation_mm':float(pair_max[0]*1000),
            'max_pairwise_rotation_deg':float(np.rad2deg(pair_max[1])),
            'translation_rms_about_mean_mm':float(rms[0]*1000),
            'rotation_rms_about_mean_deg':float(np.rad2deg(rms[1])),
            'max_error_from_calibration_reference_mm':float(np.max(reference_errors[:,0])*1000),
            'max_rotation_from_calibration_reference_deg':float(np.rad2deg(np.max(reference_errors[:,1]))),
            'positions_base_m':[t[:3,3].tolist() for t in transforms]}


class Validator(Collector):
    def __init__(self,calibration,training):
        calibration=Path(calibration).expanduser().resolve()
        data=yaml.safe_load(calibration.read_text())
        if data.get('accepted') is not True or data.get('schema')!=1:raise ValueError('Calibration has not passed fitting checks')
        self.x=check_transform(data['gripper_camera_optical']);self.reference=check_transform(data['base_target_reference'])
        original=json.loads(Path(training).expanduser().read_text())
        if original.get('purpose'):raise ValueError('Use original calibration samples, not validation records')
        self.original=[check_transform(s['base_gripper']) for s in original['samples']]
        super().__init__('handeye_validation',calibration.parent/'handeye_validation.json','/handeye_validation',
            {'purpose':'independent_validation','calibration_sha256':hashlib.sha256(calibration.read_bytes()).hexdigest()})
        for key in ('base_frame','gripper_frame','camera_frame','marker_id','marker_length_m','dictionary'):
            if self.meta[key]!=data[key] or original[key]!=data[key]:raise ValueError(f'Calibration metadata mismatch: {key}')
        self.create_service(Trigger,'~/report',self.report,callback_group=self.group)

    def capture(self,request,response):
        with self.lock:
            if not self.stable():
                response.success=False;response.message=self.error or 'Wait stationary for 1-2 seconds';return response
            s=self.latest
            for g in self.original:
                shift,angle=distance(g,s['base_gripper'])
                if shift<.005 and angle<np.deg2rad(2):
                    response.success=False;response.message='Too close to an original calibration pose; choose a NEW viewpoint';return response
            for old in self.samples:
                shift,angle=distance(np.array(old['base_gripper']),s['base_gripper'])
                if shift<.01 and angle<np.deg2rad(5):
                    response.success=False;response.message='Duplicate validation pose; change viewpoint';return response
            # Check the live TF mount agrees with the file, at the same image time.
            try:
                tf=self.buffer.lookup_transform(self.base,self.camera,Time(nanoseconds=s['stamp_ns']),timeout=Duration(seconds=.05))
                t=tf.transform.translation;q=tf.transform.rotation
                live=matrix(Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix(),[t.x,t.y,t.z])
                mismatch=distance(live,s['base_gripper']@self.x)
                if mismatch[0]>.002 or mismatch[1]>np.deg2rad(.2):raise ValueError('Live camera TF differs from calibration file')
            except Exception as e:
                response.success=False;response.message=f'Check publish_mount is running: {e}';return response
            saved=copy.deepcopy(s)
            saved['base_target']=(s['base_gripper']@self.x@s['camera_target']).tolist()
            saved['base_gripper']=saved['base_gripper'].tolist();saved['camera_target']=saved['camera_target'].tolist()
            self.samples.append(saved)
            try:self.persist()
            except Exception as e:
                self.samples.pop();response.success=False;response.message=f'Could not save: {e}';return response
            report=drift_report(self.samples,self.reference)
            response.success=True
            response.message=(f'Saved validation {len(self.samples)}; marker base XYZ(m)={saved["base_target"][0][3]:.4f}, '
                f'{saved["base_target"][1][3]:.4f}, {saved["base_target"][2][3]:.4f}; '
                f'max drift={report["max_pairwise_translation_mm"]:.2f} mm. Aim for 5 poses.')
            return response

    def report(self,request,response):
        with self.lock:
            if not self.samples:response.success=False;response.message='No validation samples';return response
            result=drift_report(self.samples,self.reference)
            path=self.path.with_name(self.path.stem+'_report.json')
            try:path.write_text(json.dumps(result,indent=2)+'\n')
            except Exception as e:response.success=False;response.message=f'Could not save report: {e}';return response
            response.success=result['accepted'];response.message=json.dumps(result)+f'; report={path}'
            return response


def main():
    p=argparse.ArgumentParser();p.add_argument('--calibration',required=True);p.add_argument('--training-samples',required=True)
    args,ros_args=p.parse_known_args()
    rclpy.init(args=ros_args,signal_handler_options=SignalHandlerOptions.NO)
    node=Validator(args.calibration,args.training_samples);executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
    try:executor.spin()
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
