"""Random placement only on observed, fully supported, empty table patches."""
import numpy as np
from scipy.ndimage import binary_erosion


def free_table_candidates(points, source_xy, table_z, diameter, *, seed=None, limit=8):
    p=np.asarray(points,dtype=float);source=np.asarray(source_xy,dtype=float)
    if p.ndim!=2 or p.shape[1]!=3 or not np.isfinite(p).all() or source.shape!=(2,):
        raise ValueError('Invalid scene point cloud')
    cell=.01
    surface=p[np.abs(p[:,2]-table_z)<=.005]
    if len(surface)<200:raise ValueError('Not enough observed tabletop for random placement')
    low=np.floor(surface[:,:2].min(0)/cell).astype(int)-1
    high=np.floor(surface[:,:2].max(0)/cell).astype(int)+1
    shape=high-low+1
    if max(shape)>500:raise ValueError('Unbounded/invalid tabletop extent')
    support=np.zeros(tuple(shape),dtype=int)
    xy=np.floor(surface[:,:2]/cell).astype(int)-low
    np.add.at(support,(xy[:,0],xy[:,1]),1)
    clear=support>=2
    # Seen objects, including their projected footprints, exclude placement.
    obstacles=p[p[:,2]>table_z+.008]
    if len(obstacles):
        ix=np.floor(obstacles[:,:2]/cell).astype(int)-low
        good=np.all((ix>=0)&(ix<shape),axis=1);ix=ix[good]
        clear[ix[:,0],ix[:,1]]=False
    radius=diameter/2+.035
    cells=int(np.ceil(radius/cell));u,v=np.mgrid[-cells:cells+1,-cells:cells+1]
    disk=(u*u+v*v)<=cells*cells
    clear=binary_erosion(clear,structure=disk,border_value=0)
    locations=(np.argwhere(clear)+low+.5)*cell
    if not len(locations):raise ValueError('No fully observed clear table patch; show more empty tabletop to camera')
    # Distinct support area, and avoid the physical base footprint.
    locations=locations[(np.linalg.norm(locations-source,axis=1)>=.12)&(np.linalg.norm(locations,axis=1)>=.18)]
    if not len(locations):raise ValueError('No separated safe random placement patch')
    rng=np.random.default_rng(seed);order=rng.permutation(len(locations));selected=[]
    for i in order:
        point=locations[i]
        if any(np.linalg.norm(point-x)<.06 for x in selected):continue
        selected.append(point)
        if len(selected)>=limit:break
    return [x.tolist() for x in selected]


def old_bottle_site_is_clear(points,xy,table_z,diameter):
    """Require observed table across the old footprint; unknown never means empty."""
    p=np.asarray(points,dtype=float);xy=np.asarray(xy,dtype=float)
    if p.ndim!=2 or p.shape[1]!=3 or xy.shape!=(2,) or not np.isfinite(p).all() or not np.isfinite(xy).all():
        return False
    if not np.isfinite(table_z) or not .01<=diameter<=.06:return False
    cell=.01;cells=int(np.ceil((diameter/2+.015)/cell))
    relative=p[:,:2]-xy
    indices=np.floor(relative/cell+.5).astype(int)
    inside=np.all(np.abs(indices)<=cells,axis=1)
    local=p[inside];indices=indices[inside]
    if np.any((np.linalg.norm(local[:,:2]-xy,axis=1)<=cells*cell)&(local[:,2]>table_z+.008)):
        return False
    counts=np.zeros((2*cells+1,2*cells+1),dtype=int)
    table=np.abs(local[:,2]-table_z)<=.005
    ids=indices[table]+cells
    np.add.at(counts,(ids[:,0],ids[:,1]),1)
    x,y=np.mgrid[-cells:cells+1,-cells:cells+1]
    footprint=x*x+y*y<=cells*cells
    return bool(np.all(counts[footprint]>=2))
