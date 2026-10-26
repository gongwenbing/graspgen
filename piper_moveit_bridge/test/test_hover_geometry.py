import math
import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from piper_moveit_bridge.hover_geometry import tilt_hover


def unpack(p):
    a=p.pose.position;q=p.pose.orientation
    r=Rotation.from_quat([q.x,q.y,q.z,q.w])
    return np.array([a.x,a.y,a.z]),r


def test_tilt_preserves_tcp_instead_of_flange():
    p=PoseStamped();p.header.frame_id='base_link';p.header.stamp.sec=123
    p.pose.position.x=.1;p.pose.position.y=-.34;p.pose.position.z=.272
    p.pose.orientation.x=1.;p.pose.orientation.w=0.
    result=tilt_hover(p,10)
    a,ra=unpack(p);b,rb=unpack(result)
    assert np.allclose(a+ra.apply([0,0,.1358]),b+rb.apply([0,0,.1358]))
    assert not np.allclose(a,b)
    assert math.degrees((ra.inv()*rb).magnitude())==pytest.approx(10)
    assert np.linalg.norm(b[:2])<np.linalg.norm(a[:2])
    assert result.header==p.header and p.pose.position.z==.272


def test_zero_and_invalid_tilt():
    p=PoseStamped()
    assert tilt_hover(p,0)==p
    for value in [-1,21,float('nan')]:
        with pytest.raises(ValueError):tilt_hover(p,value)
