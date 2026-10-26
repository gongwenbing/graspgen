import pytest
from piper_moveit_bridge.grasp_contact import contact_matches,validate_contact_config

CONFIG={'command_opening_m':.035,'observed_opening_m':.0395,'observed_effort_nm':-1.044}
GOOD={'gripper_fault':False,'gripper_enabled':True,'gripper_opening_m':.0395,
      'gripper_effort_nm':-1.0,'gripper_feedback_age_sec':.02}


def test_taught_contact_requires_position_effort_and_fresh_fault_free_driver():
    validate_contact_config(CONFIG)
    assert contact_matches(GOOD,CONFIG)
    assert not contact_matches({},CONFIG)


@pytest.mark.parametrize('key,value',[
    ('gripper_opening_m',.0445),('gripper_opening_m',.035),
    ('gripper_effort_nm',0.),('gripper_effort_nm',1.),
    ('gripper_fault',True),('gripper_enabled',False),
    ('gripper_feedback_age_sec',1.),('gripper_feedback_age_sec',-.1),
    ('gripper_effort_nm',float('nan'))])
def test_position_alone_or_bad_feedback_never_confirms_contact(key,value):
    assert not contact_matches({**GOOD,key:value},CONFIG)


@pytest.mark.parametrize('key,value',[
    ('command_opening_m',.0),('observed_opening_m',.05),
    ('observed_effort_nm',0.),('observed_effort_nm',float('nan'))])
def test_bad_reference_is_rejected(key,value):
    with pytest.raises(ValueError):validate_contact_config({**CONFIG,key:value})


def test_gripper_contact_endpoint_tolerance_is_explicit_and_bounded():
    from control_msgs.msg import JointTolerance
    from piper_moveit_bridge.trajectory_bridge import bounded_goal_tolerances
    assert bounded_goal_tolerances(('joint7',),.0005,.01,[])['joint7']==.0005
    t=JointTolerance(name='joint7',position=.00275)
    assert bounded_goal_tolerances(('joint7',),.0005,.01,[t])['joint7']==.00275
    t.position=.1
    assert bounded_goal_tolerances(('joint7',),.0005,.01,[t])['joint7']==.003
    t=JointTolerance(name='joint1',position=.1)
    assert bounded_goal_tolerances(('joint1',),.0005,.01,[t])['joint1']==.01
