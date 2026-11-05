"""Plan-only, release gating and partial-failure state in full pick/place."""
import copy,json
from types import SimpleNamespace as NS
import numpy as np
import pytest
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotTrajectory
from piper_moveit_bridge.model import ARM
import piper_moveit_bridge.bottle_workflow as workflow


@pytest.mark.parametrize('mode',['plan','execute','place_unreachable','close_fail','carry_fail','open_fail','reach_only'])
def test_full_random_pick_place_never_releases_early(monkeypatch,tmp_path,mode):
    events=[];clock=[0.];folder=tmp_path/'calibration';folder.mkdir()
    config=dict(diameter_m=.032,total_height_m=.12,body_bottom_m=.045,body_top_m=.075,grasp_height_m=.06,table_z_m=-.0118)
    for name,data in [('bottle_geometry.json',config),('tcp_config.json',{'translation_m':[0,0,.1358]}),('handeye_result.yaml',{})]:(folder/name).write_text(json.dumps(data))
    monkeypatch.setattr(workflow,'ROOT',tmp_path);monkeypatch.setattr(workflow,'load_tcp',lambda:{'translation_m':[0,0,.1358]})
    monkeypatch.setattr(workflow.time,'monotonic',lambda:clock[0])
    for name in ('init','shutdown'):monkeypatch.setattr(workflow.rclpy,name,lambda *a,**k:None)
    monkeypatch.setattr(workflow.rclpy,'ok',lambda:False)
    monkeypatch.setattr(workflow.rclpy,'create_node',lambda *a:NS(get_logger=lambda:NS(error=lambda e:None),destroy_node=lambda:None))
    monkeypatch.setattr(workflow,'candidate',lambda rt,c:(np.array([.1,-.3,.0482]),{'stamp_sec':clock[0]},{}))
    monkeypatch.setattr(workflow,'free_table_candidates',lambda *a,**k:[[.3,-.3]])
    class Fake:
        def __init__(self,node,execute=False):
            events.append('clients:'+str(execute));self.runner=NS(check_scene=lambda:None);self.width=.07;self.ends={};self.pose_value=None;self.arm_count=0
            self.hardware=[(0,NS(data=json.dumps({'gripper_enabled':True,'gripper_fault':False})))];self.ready=[(0,NS(data=json.dumps({'gripper_contact_tolerance_supported':True})))]
        def inputs(self):
            events.append('fresh_inputs')
            js=JointState(name=list(ARM)+['joint7'],position=[0.]*6+[self.width/2]);return js,dict(zip(js.name,js.position))
        def wait(self,*a,**k):clock[0]+=.1
        def scene_points_at(self,*a):return np.zeros((300,3))
        def pose_ik(self,start,target):return (False,-31) if mode=='place_unreachable' and target.position.x>.2 else (True,1)
        def plan(self,target):
            events.append('plan_from_state')
            t=RobotTrajectory();self.ends[id(t)]=copy.deepcopy(target);self.pose_value=copy.deepcopy(target);return t
        def pose_plan(self,start,target,**kw):return self.plan(target)
        def cart_plan(self,start,target):return self.plan(target)
        def pose(self,start):return copy.deepcopy(self.pose_value)
        def state_after(self,start,t):return copy.deepcopy(start)
        def valid_state(self,*a):pass
        def show(self,*a):events.append('display')
        def arm(self,t):
            self.arm_count+=1;events.append('arm');self.pose_value=self.ends[id(t)]
            if mode=='carry_fail' and self.arm_count==4:raise ValueError('Carry failure')
        def close_gripper(self,*a,**k):
            events.append('close')
            if mode=='close_fail':raise ValueError('No contact')
            self.width=.032
        def open_gripper(self,*a):
            events.append('open')
            if mode=='open_fail':raise ValueError('Release unconfirmed')
            self.width=.04
        def close(self):pass
    class Scene:
        def __init__(self,*a,**k):self.attached=False
        def source(self):self.attached=False;events.append('source')
        def contacts(self):pass
        def attach(self,*a):self.attached=True;events.append('attach')
        def attachment(self):assert self.attached;return 'saved_attachment'
        def restore_attachment(self,saved):assert saved=='saved_attachment';self.attached=True;events.append('restore_attachment')
        def placed(self,*a):self.attached=False;events.append('placed')
        def cleanup(self):events.append('cleanup')
    monkeypatch.setattr(workflow,'Runtime',Fake);monkeypatch.setattr(workflow,'BottleScene',Scene)
    if mode in ('plan','execute','reach_only'):workflow.run(mode!='plan',reach_only=mode=='reach_only')
    else:
        with pytest.raises(SystemExit):workflow.run(True)
    motions=[e for e in events if e in ('arm','close','open')]
    if mode=='plan':assert not motions and 'clients:False' in events
    if mode=='execute':
        assert motions==['arm','close','arm','arm','arm','arm','open','arm']
        arms=[i for i,e in enumerate(events) if e=='arm']
        for previous,current in zip(arms[1:4],arms[2:5]):
            interval=events[previous+1:current]
            assert 'fresh_inputs' in interval and 'plan_from_state' in interval
            assert interval.index('fresh_inputs')<interval.index('plan_from_state')
    if mode=='place_unreachable':assert not motions
    if mode=='close_fail':assert motions==['arm','close']
    if mode=='carry_fail':assert motions==['arm','close','arm','arm','arm']
    if mode=='open_fail':assert motions==['arm','close','arm','arm','arm','arm','open']
    if mode=='reach_only':assert motions==['arm']
    log=json.loads(next((folder/'grasp_runs').glob('*.json')).read_text())
    assert log['physical_grasp_verified'] is False
    if mode in ('close_fail','carry_fail','open_fail'):assert log['possible_held_object'] and 'cleanup' not in events
    if mode=='execute':assert log['placed_object_retained'] and not log['possible_held_object']
