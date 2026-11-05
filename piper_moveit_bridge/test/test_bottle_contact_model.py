import pytest
from piper_moveit_bridge.bottle_planning import contact_model
from piper_moveit_bridge.grasp_contact import contact_matches


def config():
    return dict(diameter_m=.032,gripper_contact=dict(user_confirmed_body_grasp=True,diameter_m=.032,tilt_deg=20.,command_opening_m=.0372,observed_opening_m=.0392,observed_effort_nm=-1.05))


def test_confirmed_contact_accepts_live_observation_but_not_free_close():
    c=config();model=contact_model(c)
    assert c['diameter_m']==.032
    h=dict(gripper_enabled=True,gripper_fault=False,gripper_feedback_age_sec=.003,gripper_opening_m=.0392,gripper_effort_nm=-1.04)
    assert contact_matches(h,model)
    assert not contact_matches(dict(h,gripper_effort_nm=-.01),model)
    assert not contact_matches(dict(h,gripper_opening_m=.0372),model)
    assert not contact_matches(dict(h,gripper_feedback_age_sec=.4),model)


@pytest.mark.parametrize('field,value',[('user_confirmed_body_grasp',False),('diameter_m',.04),('tilt_deg',10.),('command_opening_m',.03),('observed_effort_nm',1.05)])
def test_reject_unconfirmed_or_incompatible_contact(field,value):
    c=config();c['gripper_contact'][field]=value
    with pytest.raises(ValueError):contact_model(c)
