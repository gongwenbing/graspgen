from pathlib import Path
from types import SimpleNamespace as S
import math
import pytest
from piper_moveit_bridge.model import ARM, Model, validate_trajectory, interpolate

MODEL=Path(__file__).resolve().parents[2]/'piper_ros/src/piper_description/urdf/piper_description.urdf'

def point(q,t):
    return S(positions=q,velocities=[],accelerations=[],effort=[],time_from_start=S(sec=int(t),nanosec=round((t-int(t))*1e9)))

def current():
    return dict(zip(ARM+('joint7',),[0,.8,-.7,0,.4,0,.01]))

def test_units_and_negative_gripper_noise():
    model=Model(MODEL);q=current();msg=S(name=list(ARM)+['gripper'],position=[q[n] for n in ARM]+[-.0002])
    assert model.feedback(msg)['joint7']==0
    msg.position[-1]=.02
    assert model.feedback(msg)['joint7']==.01
    msg.position[-1]=-.001
    with pytest.raises(ValueError):model.feedback(msg)
    msg.position[-1]=.02;msg.position[1]=-.001
    with pytest.raises(ValueError):model.feedback(msg)

def test_trajectory_reordering_and_interpolation():
    m=Model(MODEL);q=current();names=ARM[::-1]
    a=[q[n] for n in names];b=a.copy();b[-1]+=.02
    trajectory=S(joint_names=list(names),points=[point(a,0),point(b,1)])
    times,points=validate_trajectory(trajectory,ARM,q,m,.2,.005,.02)
    assert interpolate(times,points,.5)[0]==pytest.approx(.01)
    assert interpolate(times,points,5)[0]==pytest.approx(.02)

@pytest.mark.parametrize('degrees',[.008,.047])
def test_arm_boundary_measurement_only(degrees):
    model=Model(MODEL);q=current()
    msg=S(name=list(ARM)+['gripper'],position=[q[n] for n in ARM]+[.02])
    msg.position[2]=math.radians(degrees)
    measured=model.feedback(msg)
    assert measured['joint3']==0
    assert msg.position[2]==math.radians(degrees)
    invalid=dict(measured);invalid['joint3']=math.radians(degrees)
    with pytest.raises(ValueError):model.check(invalid)
    trajectory=S(joint_names=list(ARM),points=[point([invalid[n] for n in ARM],1)])
    with pytest.raises(ValueError):validate_trajectory(trajectory,ARM,measured,model,.2,.005,.02)
    msg.position[2]=math.radians(.051)
    with pytest.raises(ValueError):model.feedback(msg)

@pytest.mark.parametrize('bad',['speed','nan','limits','time','names','start','effort'])
def test_bad_trajectories_rejected(bad):
    q=current();a=[q[n] for n in ARM];b=a.copy();b[0]+=.02
    t=S(joint_names=list(ARM),points=[point(a,0),point(b,1)])
    if bad=='speed':t.points[-1]=point(b,.01)
    if bad=='nan':t.points[-1].positions[0]=math.nan
    if bad=='limits':t.points[-1].positions[1]=-1
    if bad=='time':t.points[-1].time_from_start=S(sec=0,nanosec=0)
    if bad=='names':t.joint_names[0]='joint2'
    if bad=='start':t.points[0].positions[0]=.5
    if bad=='effort':t.points[-1].effort=[1]*6
    with pytest.raises(ValueError):validate_trajectory(t,ARM,q,Model(MODEL),.2,.005,.02)
