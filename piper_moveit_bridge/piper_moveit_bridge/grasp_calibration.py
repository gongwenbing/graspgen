"""Geometry and validation for empirical grasp reproduction and TCP pivot fitting."""
import math
import numpy as np
from scipy.spatial.transform import Rotation


def pose_array(pose):
    p=pose.position;q=pose.orientation
    return np.array([p.x,p.y,p.z]),Rotation.from_quat([q.x,q.y,q.z,q.w])


def radial_basis(position):
    p=np.asarray(position,dtype=float)
    r=math.hypot(p[0],p[1])
    if not np.all(np.isfinite(p)) or r<.05:raise ValueError('Cannot define radial correction near base axis')
    x=np.array([p[0]/r,p[1]/r,0.]);y=np.cross([0.,0.,1.],x)
    return np.column_stack((x,y,[0.,0.,1.]))


def empirical_profile(surface, held_position, held_rotation, lift_m, opening_m, tcp_xyz):
    if not 0<=lift_m<=.02:raise ValueError('Recorded lift must be 0..20mm')
    if not .005<=opening_m<=.06:raise ValueError('Unexpected successful gripper opening')
    surface=np.asarray(surface,dtype=float);tcp=np.asarray(tcp_xyz,dtype=float)
    grasp=np.asarray(held_position,dtype=float)-[0.,0.,lift_m]
    grasp_tcp=grasp+held_rotation.apply(tcp)
    basis=radial_basis(surface);offset=basis.T@(grasp_tcp-surface)
    if not np.all(np.isfinite(offset)) or np.linalg.norm(offset[:2])>.05 or not -.02<=offset[2]<=.15:
        raise ValueError('Successful pose differs too far from historical surface; check reference/lift')
    return {'schema_version':1,'method':'single_success_empirical','tcp_verified':False,
            'tcp_translation_m':list(map(float,tcp)),
            'reference_surface_base_m':surface.tolist(),
            'correction_radial_tangent_z_m':offset.tolist(),
            'orientation_in_radial_xyzw':(Rotation.from_matrix(basis).inv()*held_rotation).as_quat().tolist(),
            'closed_opening_m':float(opening_m),'approach_height_m':.08,'test_lift_m':.01,
            'maximum_reference_distance_m':.05,'object_class':'bottle','sample_count':1,
            'scope':'same bottle, same fixed observation pose, near taught position; not a general TCP calibration'}


def corrected_targets(profile,surface):
    surface=np.asarray(surface,dtype=float);reference=np.array(profile['reference_surface_base_m'])
    if not np.all(np.isfinite(surface)) or surface.shape!=(3,) or surface[2]<=0:
        raise ValueError('Visual surface must be finite and above table')
    if np.linalg.norm(surface-reference)>min(.05,float(profile['maximum_reference_distance_m'])):
        raise ValueError('Target outside empirical profile 50mm neighborhood; collect more successful examples')
    basis=radial_basis(surface)
    rotation=Rotation.from_matrix(basis)*Rotation.from_quat(profile['orientation_in_radial_xyzw'])
    tcp=surface+basis@np.asarray(profile['correction_radial_tangent_z_m'])
    grasp=tcp-rotation.apply(profile['tcp_translation_m'])
    height=float(profile['approach_height_m']);lift=float(profile['test_lift_m'])
    if not .05<=height<=.10 or not 0<lift<=.01:raise ValueError('Invalid approach/lift configuration')
    if tcp[2]<.025:raise ValueError('Corrected TCP is too close to table')
    return grasp+[0,0,height],grasp,grasp+[0,0,lift],rotation


def solve_pivot(samples):
    """Fit R_i*t + p_i = fixed_point; no orientation calibration claim."""
    if len(samples)<8:raise ValueError('At least 8 diverse fixed-point contact poses required')
    rotations=[Rotation.from_quat(s['quaternion_xyzw']).as_matrix() for s in samples]
    positions=[np.asarray(s['position_m'],dtype=float) for s in samples]
    a=np.vstack([np.column_stack((r,-np.eye(3))) for r in rotations]);b=-np.concatenate(positions)
    if not np.all(np.isfinite(a)) or not np.all(np.isfinite(b)):raise ValueError('Nonfinite pivot data')
    x,_,rank,singular=np.linalg.lstsq(a,b,rcond=None)
    if rank<6 or singular[0]/singular[-1]>100:raise ValueError('Insufficient rotational diversity for TCP calibration')
    errors=np.linalg.norm(np.array([r@x[:3]+p-x[3:] for r,p in zip(rotations,positions)]),axis=1)
    rms=float(np.sqrt(np.mean(errors**2)))
    if rms>.002 or max(errors)>.005:raise ValueError('Pivot fit exceeds 2mm RMS / 5mm maximum error')
    if not .05<=np.linalg.norm(x[:3])<=.25:raise ValueError('Fitted TCP outside expected tool size')
    return {'translation_m':x[:3].tolist(),'fixed_point_base_m':x[3:].tolist(),
            'rms_m':rms,'max_error_m':float(max(errors)),'sample_count':len(samples),
            'method':'fixed_point_pivot','translation_verified':True,'orientation_verified':False}
