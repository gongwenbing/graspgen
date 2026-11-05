import numpy as np
import pytest
from piper_moveit_bridge.bottle_targets import targets


@pytest.mark.parametrize("tcp_z",[.1308,.1358])
def test_tcp_reconstruction_hover_tip_clearance_and_workspace(tcp_z):
    tcp=np.array([0,0,tcp_z]);center=np.array([.08,-.3,.06])
    hover,grasp,rotation,clearance,opening=targets(center,.032,.12,tcp)
    np.testing.assert_allclose(grasp+rotation.apply(tcp),center)
    assert clearance==pytest.approx(.1)
    assert hover[2]-grasp[2]==pytest.approx(.1)
    assert opening==pytest.approx(.04)
    assert center[2]+clearance-max(0.,.1358-tcp_z)>.12
    with pytest.raises(ValueError):targets([0,0,.06],.032,.12,tcp)
    # Reachability is decided by MoveIt, not an arbitrary radial cap.
    far=targets([.1,-.8,.06],.032,.12,tcp)
    assert np.isfinite(far[1]).all()


def test_lower_table_uses_relative_clearance_and_absolute_tcp_endpoint():
    tcp=np.array([0,0,.1358]);center=np.array([.08,-.3,.049])
    hover,grasp,rotation,clearance,opening=targets(center,.032,.12,tcp,table_z=-.011)
    np.testing.assert_allclose(grasp+rotation.apply(tcp),center)
    assert clearance==pytest.approx(.1)
    assert grasp[2]==pytest.approx(.1848)
    assert hover[2]==pytest.approx(.2848)
    assert opening==pytest.approx(.04)
