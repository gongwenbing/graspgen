"""Direct open-gripper targets at one fixed TCP centre; no global reach radius."""
import numpy as np
from scipy.spatial.transform import Rotation


def direct_candidates(center, diameter, height, tcp, table_z=0.):
    center=np.asarray(center,dtype=float);tcp=np.asarray(tcp,dtype=float)
    if center.shape!=(3,) or tcp.shape!=(3,) or not np.isfinite(np.r_[center,tcp,table_z]).all():
        raise ValueError('Invalid bottle/TCP coordinates')
    if not .01<=diameter<=.06 or not .03<=height<=.35 or not .025<center[2]-table_z<height-.005:
        raise ValueError('Invalid measured bottle grip height')
    radius=np.linalg.norm(center[:2])
    if radius<.05:raise ValueError('Cannot define grasp axes near base axis')
    radial=np.r_[center[:2]/radius,0.];tangent=np.cross([0.,0.,1.],radial)
    cases=[('vertical_radial',radial,np.array([0.,0.,-1.]),0.),
           ('vertical_tangent',tangent,np.array([0.,0.,-1.]),0.)]
    # Both fingers stay at equal height: closing axis is horizontal/tangential.
    for angle in (10.,15.,20.,-10.,-15.,-20.):
        a=np.deg2rad(angle);z=np.sin(a)*radial+np.array([0.,0.,-np.cos(a)])
        cases.append((f'tilt_{angle:+.0f}',tangent,z,angle))
    result=[]
    for name,y,z,angle in cases:
        x=np.cross(y,z);rotation=Rotation.from_matrix(np.column_stack([x,y,z]))
        tip_extra=max(0.,.1358-tcp[2])*abs(z[2])+.028*abs(np.sin(np.deg2rad(angle)))
        if center[2]-table_z-tip_extra<.015:continue
        result.append(dict(name=name,rotation=rotation,grasp_tcp=center.copy(),
            grasp_flange=center-rotation.apply(tcp),tilt_deg=angle,opening_m=diameter+.008))
    return result
