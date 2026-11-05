"""No ROS graph: verify plan-only behaviour and failure boundaries."""
import copy
import json
from types import SimpleNamespace as NS
import numpy as np
import pytest
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotTrajectory
from piper_moveit_bridge.model import ARM
import piper_moveit_bridge.bottle_workflow as workflow


@pytest.mark.parametrize('mode',['plan','execute','moved_target','hover_fail','close_fail','lift_plan_fail','closed_gripper'])
def test_bottle_orchestration_stops_and_retains_possible_held_volume(monkeypatch,tmp_path,mode):
    events=[];clock=[0.];center=np.array([.08,-.3,.06])
    folder=tmp_path/'calibration';folder.mkdir()
    config=dict(diameter_m=.032,total_height_m=.12,body_bottom_m=.045,body_top_m=.075,grasp_height_m=.06)
    for name,data in [('bottle_geometry.json',config),('tcp_config.json',{'translation_m':[0,0,.1358]}),('handeye_result.yaml',{})]:
        (folder/name).write_text(json.dumps(data))
    monkeypatch.setattr(workflow,'ROOT',tmp_path)
    monkeypatch.setattr(workflow,'load_tcp',lambda:{'translation_m':[0,0,.1358]})
    monkeypatch.setattr(workflow.time,'monotonic',lambda:clock[0])
    for name in ('init','shutdown'):monkeypatch.setattr(workflow.rclpy,name,lambda *a,**k:None)
    monkeypatch.setattr(workflow.rclpy,'ok',lambda:False)
    monkeypatch.setattr(workflow.rclpy,'create_node',lambda *a:NS(get_logger=lambda:NS(error=lambda e:events.append('error')),destroy_node=lambda:None))
    calls=[0]
    def candidate(rt,cfg):
        calls[0]+=1;point=center.copy()
        if mode=='moved_target' and calls[0]>5:point[0]+=.02
        return point,dict(stamp_sec=clock[0]),{}
    monkeypatch.setattr(workflow,'candidate',candidate)
    class Fake:
        def __init__(self,node,execute=False):
            events.append('execute_clients:'+str(execute));self.runner=NS(check_scene=lambda:None)
            self.hardware=[(0,NS(data=json.dumps({'gripper_enabled':True,'gripper_fault':False})))]
            self.ready=[(0,NS(data=json.dumps({'gripper_contact_tolerance_supported':True})))]
            self.pose_value=None;self.ends={}
        def inputs(self):
            js=JointState(name=list(ARM)+['joint7'],position=[0.]*6+[.035 if mode!='closed_gripper' else .01])
            return js,dict(zip(js.name,js.position))
        def wait(self,*a,**kw):clock[0]+=.1
        def pose_ik(self,start,target):return True,1
        def pose_plan(self,start,target,**kwargs):return self.plan(target)
        def plan(self,target):
            trajectory=RobotTrajectory();self.ends[id(trajectory)]=copy.deepcopy(target);self.pose_value=copy.deepcopy(target);return trajectory
        def state_after(self,start,trajectory):return copy.deepcopy(start)
        def cart_plan(self,start,target):
            if mode=='lift_plan_fail' and target.position.z>self.pose_value.position.z:raise ValueError('No lift path')
            return self.plan(target)
        def pose(self,start):return copy.deepcopy(self.pose_value)
        def valid_state(self,*args):pass
        def show(self,*args):pass
        def arm(self,trajectory):
            events.append('arm');self.pose_value=self.ends[id(trajectory)]
            if mode=='hover_fail':raise ValueError('Controller failure')
        def close_gripper(self,*args,**kw):
            events.append('close')
            if mode=='close_fail':raise ValueError('Contact not confirmed')
        def close(self):events.append('cancel_cleanup')
    class Scene:
        def __init__(self,*a,**kw):events.append('scene_create')
        def world(self):events.append('world')
        def source(self):events.append('source')
        def contacts(self):events.append('contacts')
        def attach(self,*a):events.append('attach')
        def cleanup(self):events.append('scene_cleanup')
    monkeypatch.setattr(workflow,'Runtime',Fake);monkeypatch.setattr(workflow,'BottleScene',Scene)
    if mode in ('plan','execute'):workflow.run(mode=='execute',pick_only=True)
    else:
        with pytest.raises(SystemExit):workflow.run(True,pick_only=True)
    motions=[e for e in events if e in ('arm','close')]
    expected={'execute':['arm','close','arm'],'hover_fail':['arm'],'close_fail':['arm','close']}.get(mode,[])
    assert motions==expected
    if mode=='plan':assert 'execute_clients:False' in events
    if mode in ('execute','close_fail'):assert 'scene_cleanup' not in events
    elif 'scene_create' in events:assert 'scene_cleanup' in events
    log=json.loads(next((folder/'grasp_runs').glob('*.json')).read_text())
    assert log['physical_grasp_verified'] is False
    assert log['possible_held_object']==(mode in ('execute','close_fail'))
