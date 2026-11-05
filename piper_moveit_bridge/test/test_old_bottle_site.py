import numpy as np
from piper_moveit_bridge.table_placement import old_bottle_site_is_clear


def table():
    x,y=np.meshgrid(np.arange(-.06,.061,.002),np.arange(-.06,.061,.002))
    return np.column_stack([x.ravel()+.2,y.ravel()-.4,np.full(x.size,-.0118)])


def test_visible_empty_old_site_can_be_updated():
    assert old_bottle_site_is_clear(table(),[.2,-.4],-.0118,.032)


def test_occupied_old_site_is_preserved_even_with_table_points():
    points=np.vstack([table(),[.2,-.4,.0482]])
    assert not old_bottle_site_is_clear(points,[.2,-.4],-.0118,.032)


def test_unknown_or_partially_occluded_site_is_not_empty():
    points=table();points=points[points[:,0]>.205]
    assert not old_bottle_site_is_clear(points,[.2,-.4],-.0118,.032)
    assert not old_bottle_site_is_clear([], [.2,-.4],-.0118,.032)
