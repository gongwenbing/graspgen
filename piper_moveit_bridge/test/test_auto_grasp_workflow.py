"""Exercise orchestration without any ROS graph or physical hardware."""
import copy
import json
from types import SimpleNamespace as NS
from pathlib import Path
import numpy as np
import pytest
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation
from piper_moveit_bridge.model import ARM
from piper_moveit_bridge.grasp_calibration import empirical_profile
import piper_moveit_bridge.grasp_workflow as workflow


@pytest.mark.parametrize('mode', ['plan','execute','moved_target','gripper_fail','hover_fail','closed_gripper','wrong_observation','stale_target','model_changed'])
def test_sequence_and_failure_stops(monkeypatch,tmp_path,mode):
    events=[];clock=[0.];surface=np.array([.1,-.34,.04]);rotation=Rotation.from_euler('x',180,degrees=True)
    tcp=surface+[0,0,.02];flange=tcp-rotation.apply([0,0,.1358])
    profile=empirical_profile(surface,flange+[0,0,.01],rotation,.01,.035,[0,0,.1358])
    profile['reference_observation_joints_rad']=dict.fromkeys(ARM,0.)
    profile['reference_observed_opening_m']=.033;profile['user_confirmed_held']=True
    path=tmp_path/'profile.json';path.write_text(json.dumps(profile))
    monkeypatch.setattr(workflow,'ROOT',tmp_path)
    monkeypatch.setattr(workflow.time,'monotonic',lambda:clock[0])
    for method in ['init','shutdown']:
        monkeypatch.setattr(workflow.rclpy,method,lambda *a,**k:None)
    monkeypatch.setattr(workflow.rclpy,'ok',lambda:False)
    logger=NS(error=lambda text:events.append('error:'+text))
    monkeypatch.setattr(workflow.rclpy,'create_node',lambda *a:NS(get_logger=lambda:logger,destroy_node=lambda:None))
    def check_profile(*args):
        if mode=='model_changed':raise ValueError('Hand-eye/robot model changed')
    monkeypatch.setattr(workflow,'validate_profile',check_profile)
    class Fake:
        def __init__(self,node,execute=False):
            events.append('construct_execute:'+str(execute));self.model_path=path
            self.runner=NS(check_scene=lambda:events.append('scene'))
            self.inputs_count=0;self.candidates=0;self.pose_value=workflow.make_pose(flange+[0,0,.08],rotation)
        def inputs(self):
            self.inputs_count+=1
            js=JointState();js.name=list(ARM)+['joint7'];js.position=[0.]*6+[.035]
            q=dict(zip(js.name,js.position))
            if mode=='closed_gripper':q['joint7']=.0175
            if mode=='wrong_observation':q['joint1']=.3
            return js,q
        def wait(self,*a,**k):clock[0]+=.1
        def candidate(self):
            self.candidates+=1
            if mode=='stale_target':raise ValueError('Stale image')
            point=surface.copy()
            if mode=='moved_target' and self.candidates>5:point[0]+=.02
            return {'surface_base_m':point.tolist(),'pregrasp_link6_base_m':[0,0,.3],
                'pregrasp_tcp_base_m':[0,0,.1642]}, {'stamp_sec':clock[0]}, {'class':'bottle','opening_m':.033}
        def hover_plan(self,start,target):
            events.append('plan_hover');self.pose_value=copy.deepcopy(target)
            t=RobotTrajectory();t.joint_trajectory.joint_names=list(ARM)
            t.joint_trajectory.points=[JointTrajectoryPoint(positions=[0.]*6)]
            return t
        def state_after(self,start,t):return copy.deepcopy(start)
        def pose(self,start):return copy.deepcopy(self.pose_value)
        def cart_plan(self,start,target):
            events.append('plan_cart');return self.hover_plan_no_event()
        def hover_plan_no_event(self):
            t=RobotTrajectory();t.joint_trajectory.joint_names=list(ARM);t.joint_trajectory.points=[JointTrajectoryPoint(positions=[0.]*6)]
            return t
        def valid_state(self,*a):pass
        def show(self,*a):events.append('display')
        def arm(self,*a):
            events.append('arm')
            if mode=='hover_fail':raise RuntimeError('Controller failed')
        def close_gripper(self,*a):
            events.append('close')
            if mode=='gripper_fail':raise RuntimeError('Gripper failure')
        def close(self):events.append('cleanup')
    monkeypatch.setattr(workflow,'Runtime',Fake)
    argv=['auto_grasp','--profile',str(path)]
    if mode!='plan':argv.append('--execute')
    monkeypatch.setattr('sys.argv',argv)
    if mode in ['plan','execute']:workflow.auto_main()
    else:
        with pytest.raises(SystemExit):workflow.auto_main()
    motions=[e for e in events if e in ['arm','close']]
    if mode=='execute':assert motions==['arm','arm','close','arm']
    elif mode=='gripper_fail':assert motions==['arm','arm','close']
    elif mode=='hover_fail':assert motions==['arm']
    else:assert not motions
    if mode=='plan':assert 'construct_execute:False' in events
    logs=list((tmp_path/'calibration/grasp_runs').glob('*.json'))
    assert len(logs)==1
    assert json.loads(logs[0].read_text())['physical_grasp_verified'] is False


def test_profile_is_bound_to_real_evidence_and_model(monkeypatch,tmp_path):
    hand=tmp_path/'hand.yaml';hand.write_text('calibration')
    model=tmp_path/'robot.urdf';model.write_text('robot')
    evidence=tmp_path/'evidence.json';evidence.write_text('{}')
    monkeypatch.setattr(workflow,'HAND_EYE',hand)
    monkeypatch.setattr(workflow,'load_tcp',lambda:{'translation_m':[0,0,.1358]})
    p={'schema_version':1,'method':'single_success_empirical','user_confirmed_held':True,
       'handeye_sha256':workflow.digest(hand),'urdf_sha256':workflow.digest(model),
       'evidence_file':str(evidence),'evidence_sha256':workflow.digest(evidence),
       'tcp_translation_m':[0,0,.1358],'correction_radial_tangent_z_m':[.01,0,.02],
       'reference_surface_base_m':[.1,-.34,.04],'closed_opening_m':.035}
    workflow.validate_profile(p,model)
    hand.write_text('changed')
    with pytest.raises(ValueError):workflow.validate_profile(p,model)
    hand.write_text('calibration');evidence.write_text('{"changed":true}')
    with pytest.raises(ValueError):workflow.validate_profile(p,model)
    evidence.write_text('{}');p['tcp_translation_m'][2]=.1438
    with pytest.raises(ValueError):workflow.validate_profile(p,model)


def test_candidate_reports_depth_coverage_instead_of_generic_preview_error():
    import time
    from collections import deque
    from std_msgs.msg import String
    from piper_moveit_bridge.grasp_runtime import Runtime
    rt=object.__new__(Runtime)
    rt.preview=deque([(time.monotonic(),String(data=json.dumps({'target_valid':False})))])
    rt.vision=deque([(time.monotonic(),String(data=json.dumps({'valid':False,'objects':[{'reason':'Insufficient mask depth coverage 55%; require 60%'}]})))])
    with pytest.raises(ValueError,match='coverage 55%'):
        rt.candidate()
