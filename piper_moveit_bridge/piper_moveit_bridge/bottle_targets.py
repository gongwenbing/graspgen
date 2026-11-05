"""Top-down targets for a measured upright cylinder and selected model TCP."""
import numpy as np
from scipy.spatial.transform import Rotation


def targets(center, diameter, total_height, tcp, table_z=0.):
    center=np.asarray(center,dtype=float);tcp=np.asarray(tcp,dtype=float)
    if center.shape!=(3,) or tcp.shape!=(3,) or not np.isfinite(np.r_[center,tcp]).all():
        raise ValueError('Invalid bottle/TCP coordinates')
    if not np.isfinite(table_z) or not -.15<=table_z<=.15:raise ValueError('Invalid table reference')
    relative_z=center[2]-table_z
    if not .01<=diameter<=.06 or not .03<=total_height<=.35 or not .02<relative_z<total_height-.005:
        raise ValueError('Invalid bottle dimensions/grasp height')
    radius=np.linalg.norm(center[:2])
    if radius<.05:raise ValueError('Cannot define grasp axes near base axis')
    if relative_z<.025:raise ValueError('Grasp too close to table')
    y=np.r_[center[:2]/radius,0.];z=np.array([0.,0.,-1.]);x=np.cross(y,z)
    rotation=Rotation.from_matrix(np.column_stack((x,y,z)))
    flange=center-rotation.apply(tcp)
    # Model fingertip level is Z=135.8mm; account for any tip beyond the selected TCP.
    tip_below_tcp=max(0.,.1358-tcp[2])
    if relative_z-tip_below_tcp<.015:raise ValueError('Insufficient finger/table clearance')
    clearance=max(.10,total_height-relative_z+tip_below_tcp+.03)
    if clearance>.25:raise ValueError('Required hover height too large')
    opening=diameter+.008
    if opening>.07:raise ValueError('Bottle exceeds gripper opening')
    return flange+[0,0,clearance],flange,rotation,clearance,opening
