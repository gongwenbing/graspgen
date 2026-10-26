import copy
import json
import time
from collections import deque
from std_msgs.msg import String
import pytest
from piper_moveit_bridge.grasp_runtime import Runtime
from piper_moveit_bridge.grasp_resume import check_resume_log


def test_result_waits_for_new_idle_and_fresh_state_before_next_stage():
    rt=object.__new__(Runtime);old=time.monotonic()-1
    rt.ready=deque([(old,String(data=json.dumps({'active':True})))])
    rt.hardware=deque([(old,None)]);rt.states=deque([(old,None)]);checked=[]
    rt.inputs=lambda:checked.append('validated')
    def wait(predicate,timeout,error):
        assert timeout==3 and not predicate()
        rt.ready.append((time.monotonic()+1,String(data=json.dumps({'active':False}))))
        assert not predicate()  # old hardware/joints are insufficient
        rt.hardware.append((time.monotonic()+1,None));rt.states.append((time.monotonic()+1,None))
        assert predicate()
    rt.wait=wait;rt.wait_idle_after_result();assert checked==['validated']


def test_handoff_timeout_does_not_validate_or_submit_next_action():
    rt=object.__new__(Runtime);rt.ready=deque();rt.hardware=deque();rt.states=deque()
    rt.inputs=lambda:pytest.fail('Must not continue on missing idle feedback')
    def wait(predicate,timeout,error):
        assert not predicate();raise RuntimeError(error)
    rt.wait=wait
    with pytest.raises(RuntimeError,match='no next stage'):rt.wait_idle_after_result()


def test_resume_only_after_matching_completed_descent():
    record={'execute_requested':True,'profile_sha256':'same','stages':[{'stage':'hover'},{'stage':'descent','confirmed_unix':100}], 'trajectories':[{}, {}, {}]}
    check_resume_log(record,'same',101)
    for bad in [dict(record,execute_requested=False),dict(record,profile_sha256='other'),
                dict(record,stages=record['stages']+[{'stage':'close'}]),dict(record,trajectories=[])]:
        with pytest.raises(ValueError):check_resume_log(bad,'same',101)
    with pytest.raises(ValueError):check_resume_log(record,'same',2000)
