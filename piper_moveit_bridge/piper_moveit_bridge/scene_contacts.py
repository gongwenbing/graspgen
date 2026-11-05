"""Permit the fixed mounting contact only; preserve every other ACM entry."""
import copy
from moveit_msgs.msg import AllowedCollisionEntry


def allow_table_mount(matrix,table_name='graspgen_table'):
    result=copy.deepcopy(matrix)
    count=len(result.entry_names)
    if len(result.entry_values)!=count or any(len(row.enabled)!=count for row in result.entry_values):
        raise ValueError('Invalid allowed-collision matrix')
    for name in ['base_link',table_name]:
        if name not in result.entry_names:
            result.entry_names.append(name)
            for row in result.entry_values:row.enabled.append(False)
            result.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(result.entry_names)))
    base=result.entry_names.index('base_link');table=result.entry_names.index(table_name)
    result.entry_values[base].enabled[table]=True;result.entry_values[table].enabled[base]=True
    return result
