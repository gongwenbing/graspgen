import pytest
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from piper_moveit_bridge.model import ARM
from piper_moveit_bridge.cartesian_step import retime
from piper_moveit_bridge import speed_settings as speed


def test_stage_speeds_remain_inside_existing_gateway_limits():
    assert .2*speed.APPROACH_VELOCITY_SCALE<=speed.ARM_EXECUTION_CAP<=.2
    assert 0<speed.CARTESIAN_JOINT_SPEED<=speed.ARM_EXECUTION_CAP
    assert 0<speed.GRIPPER_TOTAL_SPEED<=.01
    assert 0<speed.APPROACH_ACCELERATION_SCALE<=1


def test_new_cartesian_timing_is_four_times_faster_without_changing_positions():
    t=JointTrajectory();t.joint_names=list(ARM)
    t.points=[JointTrajectoryPoint(positions=[0.]*6),JointTrajectoryPoint(positions=[.04]+[0.]*5)]
    slow=retime(t,dict.fromkeys(ARM,0.),joint_speed=.01)
    fast=retime(t,dict.fromkeys(ARM,0.))
    assert slow==pytest.approx(4*fast)
    assert t.points[-1].positions[0]==.04
