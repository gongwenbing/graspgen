"""Measured tabletop height and a conservative keep-out up to base Z=0."""
from pathlib import Path
import math
import yaml
import numpy as np
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory

GUARD_ID='graspgen_base_height_guard'


def load_table(path=None):
    path=Path(path) if path else Path(get_package_share_directory('piper_moveit_bridge'))/'config/workcell_measurements.yaml'
    cfg=yaml.safe_load(path.read_text())['table'];dims=list(cfg['collision_size']);top=float(cfg['top_z'])
    if len(dims)!=3 or not all(isinstance(v,(int,float)) and math.isfinite(v) and v>0 for v in dims) or not math.isfinite(top) or not -.15<=top<=.15:
        raise ValueError('Invalid measured table geometry')
    center=list(cfg.get('collision_center',[0.,0.,top-dims[2]/2]))
    if len(center)!=3 or not all(math.isfinite(v) for v in center) or abs(center[2]+dims[2]/2-top)>1e-6:
        raise ValueError('Table box centre disagrees with measured top_z')
    return dims,center,top


def verify_table_object(obj,expected_top):
    if obj.header.frame_id!='base_link' or len(obj.primitives)!=1 or len(obj.primitive_poses)!=1:
        raise ValueError('Invalid table collision geometry; re-run add_table with gates closed')
    box=obj.primitives[0]
    if box.type!=box.BOX or len(box.dimensions)!=3 or not all(math.isfinite(v) and v>0 for v in box.dimensions):
        raise ValueError('Table requires finite positive box dimensions')
    def transform(pose, allow_identity_default=False):
        q=pose.orientation;p=pose.position
        quat=np.array([q.x,q.y,q.z,q.w]);xyz=np.array([p.x,p.y,p.z])
        if not np.isfinite(np.r_[quat,xyz]).all():raise ValueError('Nonfinite table pose')
        norm=np.linalg.norm(quat)
        # Legacy outgoing CollisionObject has a zero/default object quaternion;
        # MoveIt interprets it as identity. Returned poses should be normalized.
        if allow_identity_default and norm<1e-12:quat=np.array([0.,0.,0.,1.])
        elif abs(norm-1)>1e-6:raise ValueError('Invalid table quaternion')
        result=np.eye(4);result[:3,:3]=Rotation.from_quat(quat).as_matrix();result[:3,3]=xyz
        return result
    # CollisionObject.pose is relative to header; primitive_poses is relative
    # to the object, and MoveIt may move the first shape transform into pose.
    world=transform(obj.pose,True)@transform(obj.primitive_poses[0])
    if not np.allclose(world[:3,:3],np.eye(3),atol=1e-6):
        raise ValueError('Tilted table not supported by upright bottle mode')
    actual_top=float(world[2,3]+box.dimensions[2]/2)
    if not math.isfinite(actual_top) or abs(actual_top-expected_top)>1e-6:
        raise ValueError(f'Table scene Z={actual_top:.6f} differs from bottle reference Z={expected_top:.6f}; re-run add_table with gates closed')
    return actual_top
