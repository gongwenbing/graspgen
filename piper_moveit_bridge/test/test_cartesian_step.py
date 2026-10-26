import copy
from types import SimpleNamespace
import pytest
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from piper_moveit_bridge.cartesian_step import retime,check_line
from piper_moveit_bridge.model import ARM


def test_timing_limits_and_clears_old_derivatives():
    t=JointTrajectory();t.joint_names=list(ARM)
    t.points=[JointTrajectoryPoint(positions=[0.]*6),JointTrajectoryPoint(positions=[.02]+[0.]*5,velocities=[1.]*6)]
    assert retime(t,dict.fromkeys(ARM,0.),joint_speed=.01)==2.
    assert t.points[-1].time_from_start.sec==2
    assert not t.points[-1].velocities


def test_rejects_joint_jump():
    t=JointTrajectory();t.joint_names=list(ARM);t.points=[JointTrajectoryPoint(positions=[.051]*6)]
    with pytest.raises(ValueError):retime(t,dict.fromkeys(ARM,0.))


def test_fixed_orientation_base_z_line():
    a=Pose();a.orientation.w=1.;a.position.z=.3
    b=copy.deepcopy(a);b.position.z-=.01
    check_line(a,b,10.)
    b.position.x=.002
    with pytest.raises(ValueError):check_line(a,b,10.)
    b=copy.deepcopy(a);b.position.z-=.02
    with pytest.raises(ValueError):check_line(a,b,10.)
    b=copy.deepcopy(a);b.orientation.w=0.;b.orientation.x=1.
    with pytest.raises(ValueError):check_line(a,b,10.)


def test_outward_from_tcp_projection_preserves_height():
    from piper_moveit_bridge.cartesian_step import radial_displacement,check_translation
    a=Pose();a.position.x=.3;a.position.y=-.4;a.position.z=.2;a.orientation.w=1.
    d=radial_displacement(a,20.)
    assert d==pytest.approx((.012,-.016,0.))
    b=copy.deepcopy(a);b.position.x+=d[0];b.position.y+=d[1]
    check_translation(a,b,d)
    b.position.z+=.002
    with pytest.raises(ValueError):check_translation(a,b,d)


def test_outward_direction_uses_tilted_tcp_not_flange():
    import math
    from piper_moveit_bridge.cartesian_step import radial_displacement
    a=Pose();a.position.y=.2
    a.orientation.y=math.sin(math.pi/4);a.orientation.w=math.cos(math.pi/4)
    dx,dy,dz=radial_displacement(a,20.)
    assert dx>0 and dy>0 and dz==0
    a=Pose();a.orientation.w=1.
    with pytest.raises(ValueError):radial_displacement(a,20.)


def test_inward_is_reverse_radial_translation():
    from piper_moveit_bridge.cartesian_step import radial_displacement
    a=Pose();a.position.x=.3;a.position.y=-.4;a.orientation.w=1.
    assert radial_displacement(a,-10.)==pytest.approx((-.006,.008,0.))


def test_holding_only_allows_small_vertical_lift():
    from piper_moveit_bridge.cartesian_step import check_gripper_motion
    check_gripper_motion(.035,True,-10.,None)
    for down,outward in [(10.,None),(-11.,None),(None,10.),(0.,None)]:
        with pytest.raises(ValueError):check_gripper_motion(.035,True,down,outward)
    with pytest.raises(ValueError):check_gripper_motion(.035,False,-10.,None)
    with pytest.raises(ValueError):check_gripper_motion(0.,True,-10.,None)
