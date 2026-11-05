"""Precise gripper timing under the existing total-opening speed cap."""
import math
from .speed_settings import GRIPPER_OPENING_CAP
from trajectory_msgs.msg import JointTrajectoryPoint


def gripper_points(opening,target,speed,min_duration,model_total_cap=GRIPPER_OPENING_CAP):
    values=(opening,target,speed,min_duration,model_total_cap)
    if not all(math.isfinite(v) for v in values) or speed<=0 or min_duration<=0 or model_total_cap<=0:
        raise ValueError('Invalid gripper timing settings')
    effective=min(speed,GRIPPER_OPENING_CAP,model_total_cap)
    duration=max(min_duration,abs(opening-target)/effective)
    nanoseconds=math.ceil(duration*1e9)
    a=JointTrajectoryPoint(positions=[opening/2])
    b=JointTrajectoryPoint(positions=[target/2])
    b.time_from_start.sec,b.time_from_start.nanosec=divmod(nanoseconds,1000000000)
    return [a,b],nanoseconds/1e9,effective
