from pathlib import Path
from geometry_msgs.msg import PoseStamped
from piper_moveit_bridge.reachability import wrist_reach_bound
MODEL=Path(__file__).resolve().parents[2]/'piper_ros/src/piper_description/urdf/piper_description.urdf'


def goal(x,y,z):
    p=PoseStamped();p.pose.position.x=x;p.pose.position.y=y;p.pose.position.z=z
    p.pose.orientation.x=1.;p.pose.orientation.w=0.;return p


def test_observed_failure_is_outside_orientation_dependent_bound():
    r=wrist_reach_bound(goal(.0542,-.5575,.3295),MODEL)
    assert r['outside_geometric_bound']
    assert .633<r['wrist_distance_min_m']<.635
    assert .536<r['main_arm_length_sum_m']<.538


def test_closer_pose_only_passes_necessary_bound():
    r=wrist_reach_bound(goal(.05,-.35,.30),MODEL)
    assert not r['outside_geometric_bound']
