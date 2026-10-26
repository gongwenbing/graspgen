import runpy
from pathlib import Path
import pytest

api=runpy.run_path(str(Path(__file__).resolve().parents[2]/'scripts/enter_can_control.py'))


def ready():
    return {'read_only':False,'outputs_enabled':False,'reason':'','ctrl_mode':0,'teach_status':0,'err_code':0,'motors_enabled':[True]*6}, {'execution_enabled':False,'active':False}


def test_mode_only_no_joint_reset_enable():
    calls=[]
    class Fake:
        def MotionCtrl_2(self,*args):calls.append(args)
    api['send_mode_only'](Fake())
    assert calls==[(1,1,10)]


@pytest.mark.parametrize('field,value',[('outputs_enabled',True),('teach_status',2),('ctrl_mode',2),('read_only',True),('reason','stale'),('motors_enabled',[False]*6)])
def test_refuses_unsafe_mode_transition(field,value):
    h,b=ready();h[field]=value
    with pytest.raises(ValueError):api['check_state'](h,b)


def test_requires_inactive_bridge():
    h,b=ready();api['check_state'](h,b)
    b['active']=True
    with pytest.raises(ValueError):api['check_state'](h,b)


def test_transient_can_then_standby_is_not_confirmed():
    h,_=ready()
    can={'ctrl_mode':1,'teach_status':0,'err_code':0}
    assert not api['can_feedback_matches'](can,h)
    h['ctrl_mode']=1
    assert api['can_feedback_matches'](can,h)
    assert not api['can_feedback_matches']({**can,'ctrl_mode':0},h)
