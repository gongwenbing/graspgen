from moveit_msgs.msg import AllowedCollisionMatrix, AllowedCollisionEntry
from piper_moveit_bridge.bottle_scene import allow_pairs, OBJECT_ID


def test_only_fingers_and_support_permitted_existing_permissions_preserved():
    original=AllowedCollisionMatrix(entry_names=['base_link','graspgen_table','link6'],
        entry_values=[AllowedCollisionEntry(enabled=[False,True,False]),
                      AllowedCollisionEntry(enabled=[True,False,False]),
                      AllowedCollisionEntry(enabled=[False,False,False])])
    def allowed(m,a,b):return m.entry_values[m.entry_names.index(a)].enabled[m.entry_names.index(b)]
    approach=allow_pairs(original,'support')
    assert allowed(approach,OBJECT_ID,'graspgen_table')
    assert allowed(approach,OBJECT_ID,'graspgen_base_height_guard')
    assert not allowed(approach,'link7','graspgen_base_height_guard')
    assert not allowed(approach,OBJECT_ID,'link7')
    contact=allow_pairs(approach,'contact')
    assert allowed(contact,OBJECT_ID,'link7') and allowed(contact,OBJECT_ID,'link8')
    assert not allowed(contact,OBJECT_ID,'link6')
    cleared=allow_pairs(contact,False)
    assert not allowed(cleared,OBJECT_ID,'link7')
    assert allowed(cleared,'base_link','graspgen_table')
    assert original.entry_names==['base_link','graspgen_table','link6']
