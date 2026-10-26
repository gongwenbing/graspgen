import cv2,numpy as np,pytest
from scipy.spatial.transform import Rotation
from graspgen_handeye.core import matrix,solve_samples,detect,distance


def dataset():
    rng=np.random.default_rng(4)
    x=matrix(Rotation.from_euler('xyz',[.2,-.3,.1]).as_matrix(),[.035,.02,.08])
    target=matrix(Rotation.from_euler('xyz',[2.8,.1,.2]).as_matrix(),[.45,.1,.2])
    samples=[]
    for _ in range(24):
        g=matrix(Rotation.from_rotvec(rng.uniform(-.8,.8,3)).as_matrix(),rng.uniform([.1,-.2,.3],[.4,.2,.6]))
        c=np.linalg.inv(g@x)@target
        samples.append({'base_gripper':g.tolist(),'camera_target':c.tolist()})
    return samples,x


def test_handeye_direction_and_held_out_quality():
    samples,x=dataset();result=solve_samples(samples)
    assert result['accepted']
    assert np.allclose(result['gripper_camera_optical'],x,atol=1e-8)
    assert set(result['training_indices']).isdisjoint(result['held_out_indices'])
    assert len(result['held_out_indices'])==6
    assert result['held_out_translation_rms_m']<1e-8


def test_moved_target_rejected():
    samples,_=dataset()
    for i in range(3,24,4):samples[i]['camera_target'][0][3]+=.08
    assert not solve_samples(samples)['accepted']


def test_insufficient_and_single_axis_poses_rejected():
    samples,_=dataset()
    with pytest.raises(ValueError):solve_samples(samples[:8])
    for i in range(24):samples[i]['base_gripper']=samples[0]['base_gripper']
    with pytest.raises(ValueError):solve_samples(samples)


def test_real_aruco_detector_and_metric_size():
    ar=cv2.aruco
    # ID0 is shared by these nested 4x4 dictionaries.
    book=ar.getPredefinedDictionary(ar.DICT_4X4_50)
    for name in ('DICT_4X4_100','DICT_4X4_250','DICT_4X4_1000'):
        assert np.array_equal(book.bytesList[0],ar.getPredefinedDictionary(getattr(ar,name)).bytesList[0])
    image=ar.drawMarker(book,0,300)
    k=np.array([[650.,0,320.],[0,650.,240.],[0,0,1.]])
    h=.095/2;obj=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]])
    r=np.array([2.7,.2,.1]);t=np.array([.015,-.02,.55])
    pixels=cv2.projectPoints(obj,r,t,k,np.zeros(5))[0].reshape(4,2).astype(np.float32)
    warp=cv2.getPerspectiveTransform(np.array([[0,0],[299,0],[299,299],[0,299]],np.float32),pixels)
    canvas=cv2.warpPerspective(image,warp,(640,480),borderValue=255)
    c,corners,error,ambiguous=detect(canvas,k,np.zeros(5))
    assert error<1.5 and not ambiguous
    assert np.linalg.norm(c[:3,3]-t)<.005
    assert distance(c,matrix(cv2.Rodrigues(r)[0],t))[1]<np.deg2rad(2)
    with pytest.raises(ValueError):detect(np.full_like(canvas,255),k,np.zeros(5))


def test_camera_root_composition():
    _,x=dataset()
    root_opt=matrix(Rotation.from_euler('xyz',[-np.pi/2,0,-np.pi/2]).as_matrix(),[.015,0,0])
    mount=x@np.linalg.inv(root_opt)
    assert np.allclose(mount@root_opt,x)
