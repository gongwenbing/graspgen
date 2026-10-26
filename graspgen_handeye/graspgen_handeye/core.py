"""Transforms T_a_b map points in b into a. Hand-eye X = T_gripper_camera_optical."""
import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def matrix(r,t):
    m=np.eye(4);m[:3,:3]=r;m[:3,3]=np.asarray(t).reshape(3);return m


def check_transform(t):
    a=np.asarray(t,dtype=float)
    if a.shape!=(4,4) or not np.isfinite(a).all() or not np.allclose(a[3],[0,0,0,1]):raise ValueError('Invalid transform')
    if not np.allclose(a[:3,:3].T@a[:3,:3],np.eye(3),atol=1e-5) or not np.isclose(np.linalg.det(a[:3,:3]),1,atol=1e-5):raise ValueError('Invalid rotation')
    return a


def distance(a,b):
    return float(np.linalg.norm(a[:3,3]-b[:3,3])),float(np.linalg.norm(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).as_rotvec()))


def mean_transform(items):
    return matrix(Rotation.from_matrix(np.array([t[:3,:3] for t in items])).mean().as_matrix(),np.mean([t[:3,3] for t in items],axis=0))


def detect(gray,k,d,marker_id=0,length=.095,dictionary='DICT_4X4_50'):
    if dictionary not in ('DICT_4X4_50','DICT_4X4_100','DICT_4X4_250','DICT_4X4_1000'):raise ValueError('Unsupported dictionary')
    book=cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,dictionary))
    params=cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
    corners,ids,_=cv2.aruco.detectMarkers(gray,book,parameters=params)
    if ids is None or marker_id not in ids.flatten():raise ValueError(f'Marker ID {marker_id} not detected')
    indices=np.flatnonzero(ids.flatten()==marker_id)
    if len(indices)!=1:raise ValueError('Duplicate marker ID in view')
    pixels=corners[int(indices[0])].reshape(4,2).astype(np.float64)
    if abs(cv2.contourArea(pixels.astype(np.float32)))<900:raise ValueError('Marker too small: move closer')
    h=length/2;obj=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]],float)
    ok,rs,ts,_=cv2.solvePnPGeneric(obj,pixels,k,d,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    candidates=[]
    if ok:
        for r,t in zip(rs,ts):
            R=cv2.Rodrigues(r)[0]
            if not np.isfinite(R).all() or not np.isfinite(t).all():continue
            if np.min((obj@R.T+t.reshape(3))[:,2])<=0:continue
            pred=cv2.projectPoints(obj,r,t,k,d)[0].reshape(4,2)
            error=float(np.sqrt(np.mean(np.sum((pred-pixels)**2,axis=1))))
            candidates.append((error,matrix(R,t)))
    candidates.sort(key=lambda v:v[0])
    if not candidates or candidates[0][0]>1.5:raise ValueError('PnP reprojection error >1.5 px')
    ambiguous=len(candidates)>1 and candidates[1][0]-candidates[0][0]<.25
    return candidates[0][1],pixels,candidates[0][0],ambiguous


def solve_samples(samples):
    if len(samples)<16:raise ValueError('At least 16 samples required; recommend 20-25')
    gs=[check_transform(s['base_gripper']) for s in samples]
    cs=[check_transform(s['camera_target']) for s in samples]
    # Interleaved held-out samples are never used to fit X.
    held=[i for i in range(len(samples)) if i%4==3];train=[i for i in range(len(samples)) if i not in held]
    rotations=np.array([Rotation.from_matrix(gs[train[0]][:3,:3].T@gs[i][:3,:3]).as_rotvec() for i in train[1:]])
    singular=np.linalg.svd(rotations,compute_uv=False)
    if singular[1]<.25 or singular[0]<.6:raise ValueError('Insufficient rotation diversity: rotate around at least two axes')
    candidates=[]
    for method,name in [(cv2.CALIB_HAND_EYE_PARK,'PARK'),(cv2.CALIB_HAND_EYE_TSAI,'TSAI')]:
        try:
            R,t=cv2.calibrateHandEye([gs[i][:3,:3] for i in train],[gs[i][:3,3] for i in train],
                [cs[i][:3,:3] for i in train],[cs[i][:3,3] for i in train],method=method)
            X=check_transform(matrix(R,t))
            if np.linalg.norm(X[:3,3])>.5:continue
            target=mean_transform([gs[i]@X@cs[i] for i in train])
            errors=np.array([distance(target,gs[i]@X@cs[i]) for i in range(len(samples))])
            rms=np.sqrt(np.mean(errors[held]**2,axis=0))
            candidates.append((rms[0]+rms[1]*.1,name,X,target,errors,rms))
        except (ValueError,cv2.error,np.linalg.LinAlgError):continue
    if not candidates:raise ValueError('Hand-eye fit failed or camera offset >0.5 m')
    _,name,X,target,errors,rms=min(candidates,key=lambda v:v[0])
    # Engineering acceptance gates, not a manufacturer accuracy specification.
    accepted=bool(rms[0]<=.005 and rms[1]<=np.deg2rad(2) and np.max(errors[:,0])<=.015 and np.max(errors[:,1])<=np.deg2rad(5))
    return {'method':name,'accepted':accepted,'sample_count':len(samples),'training_indices':train,'held_out_indices':held,
            'gripper_camera_optical':X.tolist(),'base_target_reference':target.tolist(),
            'held_out_translation_rms_m':float(rms[0]),'held_out_rotation_rms_deg':float(np.rad2deg(rms[1])),
            'max_translation_error_m':float(errors[:,0].max()),'max_rotation_error_deg':float(np.rad2deg(errors[:,1].max())),
            'rotation_singular_values':singular.tolist()}
