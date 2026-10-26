import numpy as np
import pytest
from graspgen_handeye.core import matrix
from graspgen_handeye.preview import top_down_pregrasp


def test_top_down_height_and_flange_conversion():
    surface=matrix(np.eye(3),[.3,-.2,.02]);offset=matrix(np.eye(3),[0,0,.1358])
    tcp,flange=top_down_pregrasp(surface,offset,.1)
    assert np.allclose(tcp[:3,3],[.3,-.2,.12])
    assert np.allclose(tcp[:3,2],[0,0,-1])
    assert np.linalg.det(tcp[:3,:3])==pytest.approx(1)
    assert np.allclose(flange@offset,tcp)
    assert flange[2,3]==pytest.approx(.2558)


def test_vertical_closing_axis_rejected():
    surface=matrix(np.array([[1,0,0],[0,0,-1],[0,1,0]]),[0,0,.2])
    with pytest.raises(ValueError):top_down_pregrasp(surface,np.eye(4),.1)


def test_delayed_tf_uses_buffered_image_timestamp_and_expires():
    from collections import deque
    from types import SimpleNamespace
    import time
    from geometry_msgs.msg import PoseStamped
    from graspgen_handeye.preview import Preview
    old=PoseStamped();old.header.frame_id='camera_color_optical_frame'
    old.header.stamp.sec=10;old.header.stamp.nanosec=100000000
    newest=PoseStamped();newest.header.frame_id=old.header.frame_id
    newest.header.stamp.sec=10;newest.header.stamp.nanosec=200000000
    queried=[]
    def available(base,frame,stamp):
        queried.append(stamp.nanoseconds)
        return stamp.nanoseconds<=10100000000
    now=SimpleNamespace(nanoseconds=10300000000)
    fake=SimpleNamespace(base='base_link',
        candidates=deque([({},old,time.monotonic()),({},newest,time.monotonic())]),
        get_clock=lambda:SimpleNamespace(now=lambda:now),
        buffer=SimpleNamespace(can_transform=available))
    assert Preview.candidate_with_tf(fake)[1] is old
    assert queried==[10200000000,10100000000]
    now.nanoseconds=12000000000
    with pytest.raises(ValueError,match='Waiting for image-time TF'):
        Preview.candidate_with_tf(fake)


def test_invalid_vision_clears_waiting_candidates():
    from collections import deque
    from types import SimpleNamespace
    from graspgen_handeye.preview import Preview
    fake=SimpleNamespace(status={'valid':False},pose=None,candidates=deque([1]))
    Preview.remember_candidate(fake)
    assert not fake.candidates
