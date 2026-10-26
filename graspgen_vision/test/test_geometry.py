import numpy as np
import pytest
from graspgen_vision.geometry import object_points,pca_grasp


def test_projection_units_mask_and_invalid_depth():
    h,w=80,120;depth=np.full((h,w),.5,np.float32);mask=np.zeros((h,w),bool);mask[15:65,20:100]=True
    depth[20:25,30:35]=np.nan;depth[30:35,40:45]=0
    rgb=np.zeros((h,w,3),np.uint8);rgb[:,:,2]=255
    k=np.array([[500,0,60],[0,500,40],[0,0,1]],float)
    points,colors=object_points(mask,depth,rgb,k)
    assert np.allclose(points[:,2],.5)
    assert np.max(np.abs(points[:,0]))<.041
    assert np.all(colors[:,0]==255)
    assert len(points)<mask.sum()


def plane(width=.03):
    x,y=np.meshgrid(np.linspace(-.06,.06,100),np.linspace(-width/2,width/2,40))
    angle=.6;c,s=np.cos(angle),np.sin(angle)
    return np.column_stack((c*x.ravel()-s*y.ravel(),s*x.ravel()+c*y.ravel(),np.full(x.size,.5)))


def test_pca_pose_right_handed_and_width():
    g=pca_grasp(plane())
    assert np.allclose(g['center'],[0,0,.5],atol=.001)
    assert np.linalg.det(g['axes'])==pytest.approx(1)
    assert np.linalg.norm(g['quaternion'])==pytest.approx(1)
    assert .03<g['opening']<.04
    assert g['axes'][2,2]>.99
    assert np.allclose(g['axes'].T@g['axes'],np.eye(3),atol=1e-6)


def test_wide_surface_and_degeneracy_rejected():
    with pytest.raises(ValueError):pca_grasp(plane(.09))
    with pytest.raises(ValueError):pca_grasp(np.zeros((100,3)))
    with pytest.raises(ValueError):object_points(np.ones((10,10)),np.zeros((10,10)),np.zeros((10,10,3)),np.eye(3))


def test_sparse_background_patch_cannot_become_object_candidate():
    depth=np.zeros((100,100),np.float32);depth[20:40,20:40]=.5
    with pytest.raises(ValueError,match='coverage'):
        object_points(np.ones((100,100),bool),depth,np.zeros((100,100,3),np.uint8),np.array([[500.,0,50],[0,500,50],[0,0,1]]))


def test_d455_near_range_rejected_with_operating_limit():
    depth=np.full((40,40),.25,np.float32)
    with pytest.raises(ValueError,match='coverage'):
        object_points(np.ones((40,40),bool),depth,np.zeros((40,40,3),np.uint8),np.array([[500.,0,20],[0,500,20],[0,0,1]]),min_depth=.35)
