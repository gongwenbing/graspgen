import numpy as np
from pathlib import Path
from types import SimpleNamespace as NS
from piper_moveit_bridge.bottle_candidates import direct_candidates
from piper_moveit_bridge.grasp_workflow import make_pose
from piper_moveit_bridge.reachability import wrist_reach_bound


def test_all_candidates_preserve_tcp_no_hover_and_horizontal_closing_axis():
    center=np.array([.134646302,-.47047486,.0482]);tcp=np.array([0,0,.1358])
    cases=direct_candidates(center,.032,.12,tcp,-.0118)
    assert len(cases)>=5
    for c in cases:
        np.testing.assert_allclose(c['grasp_flange']+c['rotation'].apply(tcp),center)
        assert abs(c['rotation'].as_matrix()[2,1])<1e-10
        assert c['opening_m']==.04
    model=Path(__file__).resolve().parents[2]/'piper_ros/src/piper_description/urdf/piper_description.urdf'
    pose=NS(pose=make_pose(cases[0]['grasp_flange'],cases[0]['rotation']))
    assert not wrist_reach_bound(pose,model)['outside_geometric_bound']
    # A target farther than50cm is not rejected by a made-up workspace sphere.
    assert direct_candidates([.1,-.6,.0482],.032,.12,tcp,-.0118)
