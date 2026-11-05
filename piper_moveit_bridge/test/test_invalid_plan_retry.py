from types import SimpleNamespace as NS
import pytest
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from piper_moveit_bridge.grasp_runtime import Runtime


@pytest.mark.parametrize('code,expected',[(-2,3),(-31,1)])
def test_only_invalid_paths_are_retried_bounded_without_execution(code,expected):
    runtime=Runtime.__new__(Runtime);runtime.motion=object();calls=[]
    def call(client,request):
        calls.append(request)
        return NS(motion_plan_response=NS(error_code=NS(val=code)))
    runtime.call=call
    with pytest.raises(ValueError,match='planning failed'):
        runtime.pose_plan(JointState(),Pose())
    assert len(calls)==expected
    assert all(request.motion_plan_request.goal_constraints for request in calls)
