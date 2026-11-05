"""Recovery guards and permitted SDK call, without CAN or any physical motion."""
import runpy
from pathlib import Path
from types import SimpleNamespace as NS
import pytest

API=runpy.run_path(str(Path(__file__).resolve().parents[2]/'scripts/open_empty_gripper.py'))

def state():
    return dict(read_only=False,outputs_enabled=False,ctrl_mode=0,teach_status=0,err_code=0,
        motors_enabled=[True]*6,gripper_enabled=True,gripper_fault=False,
        gripper_opening_m=-.0011,gripper_feedback_age_sec=.01,joint_degrees=[0]*6,
        reason='Gripper feedback exceeds configured measurement tolerance'),dict(execution_enabled=False,active=False,reason='Gripper feedback exceeds configured measurement tolerance')

def test_accepts_bounded_closed_feedback_without_widening_normal_model():
    h,b=state();width,_=API['check_state'](h,b);assert width==-.0011

@pytest.mark.parametrize('key,value',[('outputs_enabled',True),('read_only',True),('teach_status',2),('err_code',1),('ctrl_mode',2),('gripper_fault',True),('gripper_enabled',False),('gripper_opening_m',-.003),('gripper_feedback_age_sec',.4),('reason','Incomplete or stale CAN feedback')])
def test_recovery_rejects_unrelated_faults_or_unclosed_gate(key,value):
    h,b=state();h[key]=value
    with pytest.raises(ValueError):API['check_state'](h,b)

@pytest.mark.parametrize('key',['execution_enabled','active'])
def test_recovery_rejects_bridge_gate_or_activity(key):
    h,b=state();b[key]=True
    with pytest.raises(ValueError):API['check_state'](h,b)


def test_only_gripper_opcode_no_zeroing_or_arm_call():
    calls=[];arm=NS(GripperCtrl=lambda *a:calls.append(a))
    API['send_open_target'](arm,.04)
    assert calls==[(40000,500,0x01,0x00)]
    for value in (-.001,.041,float('nan')):
        with pytest.raises(ValueError):API['send_open_target'](arm,value)
    assert len(calls)==1
