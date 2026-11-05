import pytest
from piper_moveit_bridge.gripper_timing import gripper_points


def test_subsecond_duration_preserves_nanoseconds():
    points,duration,speed=gripper_points(.0392,.0472,.01,.5)
    assert duration==pytest.approx(.8,abs=2e-9)
    assert points[-1].time_from_start.sec==0
    assert points[-1].time_from_start.nanosec==pytest.approx(800000000,abs=1)
    assert list(points[-1].positions)==[.0472/2]


def test_overspeed_requests_slow_to_gateway_cap():
    _,duration,speed=gripper_points(.0692,.0372,.1,.5)
    assert speed==.02
    assert duration==pytest.approx(1.6,abs=2e-9)


def test_model_cap_and_minimum_duration():
    _,duration,speed=gripper_points(.04,.041,.01,.5,.008)
    assert duration==.5 and speed==.008


@pytest.mark.parametrize('speed,minimum',[(0,.5),(-.1,.5),(float('nan'),.5),(.01,0)])
def test_invalid_timing_rejected(speed,minimum):
    with pytest.raises(ValueError):gripper_points(.04,.05,speed,minimum)


def test_updated_open_close_speed_halves_movement_time():
    from piper_moveit_bridge.speed_settings import GRIPPER_OPEN_SPEED,GRIPPER_CLOSE_SPEED,GRIPPER_MIN_DURATION
    for opening,target,speed in [(.0392,.0472,GRIPPER_OPEN_SPEED),(.0472,.0372,GRIPPER_CLOSE_SPEED)]:
        points,duration,_=gripper_points(opening,target,speed,GRIPPER_MIN_DURATION)
        assert duration==pytest.approx(abs(opening-target)/.02,abs=2e-9)
        assert duration<1
