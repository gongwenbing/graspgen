"""Explicit tilted hover pose about a fixed provisional TCP."""
import copy
import math
import numpy as np
from scipy.spatial.transform import Rotation


def tilt_hover(pose, degrees, tcp_z=.1358, tcp_xyz=None):
    if not math.isfinite(degrees) or not 0<=degrees<=20:
        raise ValueError('Hover tilt must be within 0..20 degrees')
    result=copy.deepcopy(pose)
    if degrees==0:return result
    p=pose.pose.position;q=pose.pose.orientation
    rotation=Rotation.from_quat([q.x,q.y,q.z,q.w])
    offset=np.array([0.,0.,tcp_z] if tcp_xyz is None else tcp_xyz)
    tcp=np.array([p.x,p.y,p.z])+rotation.apply(offset)
    radial=np.array([tcp[0],tcp[1],0.])
    if np.linalg.norm(radial)<.05:raise ValueError('Target too close to base axis for radial hover tilt')
    radial/=np.linalg.norm(radial)
    axis=np.cross(radial,[0.,0.,1.])
    tilted=Rotation.from_rotvec(axis*math.radians(degrees))*rotation
    position=tcp-tilted.apply(offset)
    result.pose.position.x,result.pose.position.y,result.pose.position.z=map(float,position)
    result.pose.orientation.x,result.pose.orientation.y,result.pose.orientation.z,result.pose.orientation.w=map(float,tilted.as_quat())
    return result
