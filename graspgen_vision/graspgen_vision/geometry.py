"""Metric RGB-D geometry only. No robot commands or invented hand-eye transform."""
import cv2
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def object_points(mask, depth, bgr, k, distortion=(), min_depth=.15, max_depth=1.5,
                  depth_band=.08, max_points=12000, min_points=80,
                  min_valid_fraction=.6, min_component_fraction=.3):
    if mask.shape != depth.shape or bgr.shape[:2] != depth.shape:
        raise ValueError('RGB/depth/mask dimensions differ')
    if not np.isfinite(k).all() or k[0,0]<=0 or k[1,1]<=0:
        raise ValueError('Invalid intrinsics')
    interior=cv2.erode(mask.astype(np.uint8),np.ones((3,3),np.uint8)).astype(bool)
    valid=interior & np.isfinite(depth) & (depth>=min_depth) & (depth<=max_depth)
    area=int(np.count_nonzero(interior))
    coverage=np.count_nonzero(valid)/max(area,1)
    if coverage<min_valid_fraction:
        raise ValueError(f'Insufficient mask depth coverage {coverage:.1%}; require {min_valid_fraction:.0%} within {min_depth:.2f}..{max_depth:.2f}m. Move to a valid observation distance; do not use isolated background depths')
    if np.count_nonzero(valid)<min_points:raise ValueError('Insufficient valid mask depth')
    median=float(np.median(depth[valid]))
    valid &= np.abs(depth-median)<=depth_band
    count,labels,stats,_=cv2.connectedComponentsWithStats(valid.astype(np.uint8),8)
    if count<=1:raise ValueError('No connected depth component')
    valid=labels==(1+np.argmax(stats[1:,cv2.CC_STAT_AREA]))
    if np.count_nonzero(valid)/max(area,1)<min_component_fraction:
        raise ValueError('Dominant depth component covers too little of the object mask')
    v,u=np.nonzero(valid)
    if len(u)<min_points:raise ValueError('Depth component too small')
    step=max(1,int(np.ceil(len(u)/max_points)));u=u[::step];v=v[::step]
    uv=np.stack((u,v),axis=1).astype(np.float64).reshape(-1,1,2)
    normalized=cv2.undistortPoints(uv,np.asarray(k,dtype=np.float64),
                                  np.asarray(distortion,dtype=np.float64) if len(distortion) else None).reshape(-1,2)
    z=depth[v,u];points=np.column_stack((normalized*z[:,None],z))
    rgb=bgr[v,u,::-1].copy()
    distances,_=cKDTree(points).query(points,k=min(16,len(points)))
    mean=distances[:,1:].mean(axis=1)
    keep=mean<=mean.mean()+1.5*mean.std()
    points=points[keep];rgb=rgb[keep]
    if len(points)<min_points:raise ValueError('Too few points after outlier filtering')
    return points,rgb


def pca_grasp(points,max_opening=.07,clearance=.005):
    """Observed surface OBB; local +Z is camera-facing approach into surface,
    +Y closing axis in tangent plane, +X major tangent. No hidden thickness claim.
    """
    p=np.asarray(points,dtype=np.float64)
    if p.ndim!=2 or p.shape[1]!=3 or len(p)<30 or not np.isfinite(p).all():
        raise ValueError('Invalid PCA cloud')
    origin=p.mean(axis=0)
    values,vectors=np.linalg.eigh(np.cov((p-origin).T))
    if values[1]<1e-8:raise ValueError('Degenerate line/point cloud')
    if values[0]/values[1]>.5:raise ValueError('No stable surface normal')
    if values[2]/values[1]<1.1:raise ValueError('Ambiguous tangent direction')
    z=vectors[:,0]
    if np.dot(z,origin)<0:z=-z
    x=vectors[:,2]
    if x[np.argmax(np.abs(x))]<0:x=-x
    y=np.cross(z,x);y/=np.linalg.norm(y);x=np.cross(y,z)
    axes=np.column_stack((x,y,z))
    local=(p-origin)@axes
    low,high=np.quantile(local,[.01,.99],axis=0)
    extent=high-low;center=origin+axes@((high+low)/2)
    opening=float(extent[1]+clearance)
    if opening>max_opening:raise ValueError(f'Observed closing width {opening:.3f} m exceeds gripper {max_opening:.3f} m')
    return {'center':center,'quaternion':Rotation.from_matrix(axes).as_quat(),
            'extent':extent,'opening':opening,'axes':axes}
