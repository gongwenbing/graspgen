import pytest
from moveit_msgs.msg import AllowedCollisionMatrix,AllowedCollisionEntry
from piper_moveit_bridge.scene_contacts import allow_table_mount


def test_only_fixed_mount_pair_is_allowed_and_original_is_unchanged():
    original=AllowedCollisionMatrix(entry_names=['link7','base_link','link6'],entry_values=[
        AllowedCollisionEntry(enabled=[False,False,True]),
        AllowedCollisionEntry(enabled=[False,False,False]),
        AllowedCollisionEntry(enabled=[True,False,False])])
    r=allow_table_mount(original)
    assert r.entry_names==['link7','base_link','link6','graspgen_table']
    assert len(original.entry_names)==3
    assert r.entry_values[1].enabled[3] and r.entry_values[3].enabled[1]
    assert r.entry_values[0].enabled[2]  # original exemption preserved
    assert not r.entry_values[0].enabled[3] and not r.entry_values[2].enabled[3]
    assert allow_table_mount(r)==r
    assert not r.default_entry_names


def test_rejects_malformed_matrix():
    m=AllowedCollisionMatrix(entry_names=['base_link'])
    with pytest.raises(ValueError):allow_table_mount(m)
