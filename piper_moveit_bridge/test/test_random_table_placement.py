import numpy as np
import pytest
from piper_moveit_bridge.table_placement import free_table_candidates


def tabletop():
    x,y=np.meshgrid(np.arange(.15,.55,.003),np.arange(-.55,-.15,.003))
    return np.column_stack([x.ravel(),y.ravel(),np.full(x.size,-.0118)])


def test_random_points_have_observed_edge_margin_and_avoid_source_and_obstacle():
    table=tabletop()
    obstacle=table[(table[:,0]>.33)&(table[:,0]<.43)&(table[:,1]>-.43)&(table[:,1]<-.33)].copy();obstacle[:,2]+=.05
    points=np.vstack([table,obstacle]);source=np.array([.2,-.5])
    candidates=free_table_candidates(points,source,-.0118,.032,seed=7)
    assert candidates
    for x,y in candidates:
        assert .20<x<.50 and -.50<y<-.20
        assert np.linalg.norm(np.array([x,y])-source)>=.12
        assert not (.28<x<.48 and -.48<y<-.28)
    assert candidates==free_table_candidates(points,source,-.0118,.032,seed=7)


def test_no_random_point_on_unknown_floor_or_tiny_patch():
    with pytest.raises(ValueError):free_table_candidates(np.zeros((50,3)),[.2,-.3],-.0118,.032)
    tiny=tabletop();tiny=tiny[(tiny[:,0]<.18)&(tiny[:,1]<-.52)]
    with pytest.raises(ValueError):free_table_candidates(tiny,[.2,-.3],-.0118,.032)
