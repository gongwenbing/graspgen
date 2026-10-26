import math
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from piper_moveit_bridge.grasp_calibration import empirical_profile,corrected_targets,solve_pivot


def profile():
    surface=np.array([.1,-.34,.04]);rotation=Rotation.from_euler('xy',[170,5],degrees=True)
    desired_tcp=surface+np.array([.003,-.009,.02])
    flange=desired_tcp-rotation.apply([0,0,.1358])
    p=empirical_profile(surface,flange+[0,0,.01],rotation,.01,.035,[0,0,.1358])
    return p,flange,rotation


def test_record_removes_test_lift_and_reproduces_successful_flange():
    p,flange,rotation=profile();hover,grasp,lift,r=corrected_targets(p,p['reference_surface_base_m'])
    assert grasp==pytest.approx(flange)
    assert hover-grasp==pytest.approx([0,0,.08])
    assert lift-grasp==pytest.approx([0,0,.01])
    assert (r.inv()*rotation).magnitude()<1e-8
    assert not p['tcp_verified']


def test_correction_rotates_with_radial_direction():
    p,flange,rotation=profile();old=np.array(p['reference_surface_base_m'])
    rz=Rotation.from_euler('z',4,degrees=True);new=rz.apply(old)
    _,grasp,_,r=corrected_targets(p,new)
    assert grasp==pytest.approx(rz.apply(flange))
    assert (r.inv()*(rz*rotation)).magnitude()<1e-8


@pytest.mark.parametrize('target',[[.5,-.34,.04],[.1,-.34,-.01],[float('nan'),0,.1]])
def test_rejects_unvalidated_targets(target):
    p,_,_=profile()
    with pytest.raises(ValueError):corrected_targets(p,target)


def test_eight_mm_tcp_change_cannot_replace_eighty_mm_descent():
    r=Rotation.from_euler('y',10,degrees=True)
    assert np.linalg.norm(r.apply([0,0,.1438-.1358]))==pytest.approx(.008)


def pivot_samples():
    tcp=np.array([.002,-.003,.1358]);point=np.array([.2,-.3,.15])
    rng=np.random.default_rng(42);samples=[]
    for r in Rotation.random(16,random_state=rng):
        samples.append({'position_m':(point-r.apply(tcp)).tolist(),'quaternion_xyzw':r.as_quat().tolist()})
    return samples,tcp


def test_pivot_recovers_translation_without_claiming_orientation():
    samples,tcp=pivot_samples();result=solve_pivot(samples)
    assert result['translation_m']==pytest.approx(tcp)
    assert result['rms_m']<1e-10
    assert not result['orientation_verified']


def test_pivot_rejects_no_orientation_diversity_and_bad_contact():
    samples,_=pivot_samples()
    with pytest.raises(ValueError):solve_pivot(samples[:7])
    with pytest.raises(ValueError):solve_pivot([samples[0]]*8)
    samples[0]['position_m'][0]+=.03
    with pytest.raises(ValueError):solve_pivot(samples)
