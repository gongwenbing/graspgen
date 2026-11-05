from pathlib import Path
import pytest
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject
from piper_moveit_bridge.table_geometry import load_table,verify_table_object


def table(top):
    obj=CollisionObject();obj.header.frame_id='base_link'
    obj.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[2.,2.,.05])]
    p=Pose();p.orientation.w=1.;p.position.z=top-.025;obj.primitive_poses=[p]
    return obj


def test_measured_table_top_and_box_centre_are_consistent():
    cfg=Path(__file__).resolve().parents[1]/'config/workcell_measurements.yaml'
    dims,center,top=load_table(cfg)
    assert top==pytest.approx(-.0118)
    assert center[2]+dims[2]/2==pytest.approx(top)
    assert verify_table_object(table(-.011),-.011)==pytest.approx(-.011)
    with pytest.raises(ValueError,match='differs'):verify_table_object(table(0),-.011)
    invalid=table(-.011);invalid.primitive_poses[0].orientation.x=.1;invalid.primitive_poses[0].orientation.w=(1-.1**2)**.5
    with pytest.raises(ValueError,match='Tilted'):verify_table_object(invalid,-.011)


def test_moveit_roundtrip_object_pose_contains_translation():
    obj=table(-.0118)
    obj.pose.orientation.w=1.
    obj.pose.position.z=-.0368
    obj.primitive_poses[0].position.z=0.
    assert verify_table_object(obj,-.0118)==pytest.approx(-.0118)
    # Both transforms can be nonzero; neither may be dropped.
    obj.pose.position.z=-.02
    obj.primitive_poses[0].position.z=-.0168
    assert verify_table_object(obj,-.0118)==pytest.approx(-.0118)
    obj.pose.position.z=float('nan')
    with pytest.raises(ValueError,match='Nonfinite'):verify_table_object(obj,-.0118)
