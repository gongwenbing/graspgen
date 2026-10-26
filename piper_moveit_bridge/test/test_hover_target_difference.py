import math
import pytest
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from piper_moveit_bridge.execute_pregrasp import pose_difference,check_surface_above_table,HoverExecution


def pose(rotation):
    p=PoseStamped();q=rotation.as_quat()
    p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w=map(float,q)
    return p


def test_hover_does_not_follow_pca_yaw_noise():
    down=Rotation.from_euler('x',math.pi)
    first=pose(down)
    other=pose(down*Rotation.from_euler('z',44,degrees=True))
    distance,angle=pose_difference(first,other)
    assert distance==0 and angle==pytest.approx(0,abs=1e-7)


def test_real_translation_and_approach_change_still_detected():
    down=Rotation.from_euler('x',math.pi)
    first=pose(down);other=pose(down*Rotation.from_euler('x',10,degrees=True))
    other.pose.position.x=.02
    distance,angle=pose_difference(first,other)
    assert distance==pytest.approx(.02)
    assert angle==pytest.approx(math.radians(10))


def test_reject_table_depth_mistaken_for_object():
    with pytest.raises(ValueError,match='at/below'):
        check_surface_above_table({'surface_base_m':[.0448,-.3454,-.02016]})
    with pytest.raises(ValueError,match='Missing'):
        check_surface_above_table({})
    check_surface_above_table({'surface_base_m':[.04,-.34,.08]})


def test_ready_standby_supported_but_teach_still_rejected():
    h={'read_only':False,'outputs_enabled':True,'reason':'','ctrl_mode':0,'teach_status':0,'err_code':0,'motors_enabled':[True]*6}
    b={'execution_enabled':True,'active':False}
    HoverExecution.check_hardware(h,b)
    h['ctrl_mode']=1;HoverExecution.check_hardware(h,b)
    h['ctrl_mode']=2
    with pytest.raises(ValueError):HoverExecution.check_hardware(h,b)
    h['ctrl_mode']=0;h['teach_status']=2
    with pytest.raises(ValueError):HoverExecution.check_hardware(h,b)
