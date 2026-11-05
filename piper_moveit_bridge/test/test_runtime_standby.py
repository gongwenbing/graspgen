from collections import deque
from pathlib import Path
from types import SimpleNamespace as NS
import json,time
import pytest
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from piper_moveit_bridge.grasp_runtime import Runtime
from piper_moveit_bridge.model import Model
from piper_moveit_bridge.execute_pregrasp import HoverExecution

@pytest.mark.parametrize('mode',[0,1])
def test_runtime_accepts_healthy_standby_and_can_for_plan_only(mode):
    now=time.time();received=time.monotonic();rt=object.__new__(Runtime);rt.execute=False
    rt.node=NS(count_publishers=lambda _:1,get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(now*1e9))))
    rt.model=Model(Path(__file__).resolve().parents[2]/'piper_ros/src/piper_description/urdf/piper_description.urdf')
    js=JointState(name=[f'joint{i}' for i in range(1,8)],position=[0,.8,-.7,0,.4,0,.02])
    js.header.stamp.sec=int(now);js.header.stamp.nanosec=int((now-int(now))*1e9)
    h=dict(ctrl_mode=mode,teach_status=0,err_code=0,reason='',read_only=False,outputs_enabled=False,motors_enabled=[True]*6)
    rt.states=deque([(received,js)]);rt.hardware=deque([(received,String(data=json.dumps(h)))])
    rt.ready=deque([(received,String(data=json.dumps(dict(active=False,execution_enabled=False))))])
    assert rt.inputs()[1]['joint7']==.02
    # Standby support never skips the execution gate checks.
    rt.execute=True;rt.runner=NS(check_hardware=HoverExecution.check_hardware)
    with pytest.raises(ValueError,match='Hardware not ready'):rt.inputs()
