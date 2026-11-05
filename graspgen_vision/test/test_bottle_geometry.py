import numpy as np
import pytest
from graspgen_vision.bottle_geometry import fit_bottle, validate_bottle_config

CONFIG=dict(diameter_m=.032,total_height_m=.12,body_bottom_m=.045,body_top_m=.075,grasp_height_m=.06)

def cloud(center=(.08,-.3), angle=150, tilt=0., radius=.016):
    a,z=np.meshgrid(np.deg2rad(np.linspace(-angle/2,angle/2,60)),np.linspace(.048,.072,35))
    p=np.column_stack((center[0]+radius*np.cos(a.ravel())+tilt*(z.ravel()-.06),
                       center[1]+radius*np.sin(a.ravel()),z.ravel()))
    return p

@pytest.mark.parametrize('xy',[(.08,-.3),(-.1,-.25),(.2,-.2)])
def test_recovers_hidden_axis_from_partial_surface_at_different_positions(xy):
    p=cloud(xy);p+=np.random.default_rng(42).normal(0,.0001,p.shape)
    result=fit_bottle(p,CONFIG)
    np.testing.assert_allclose(result['center'],[*xy,.06],atol=.0005)
    assert result['opening']==pytest.approx(.04)
    np.testing.assert_allclose(result['axes'][:,2],[0,0,-1])
    assert np.linalg.norm(p.mean(0)[:2]-xy)>.008  # Surface mean would be biased.

def test_rejects_short_arc_wrong_radius_and_tilt():
    for p in (cloud(angle=20),cloud(radius=.025),cloud(tilt=.6)):
        with pytest.raises(ValueError):fit_bottle(p,CONFIG)

def test_rejects_plane_missing_grasp_band_and_unknown_dimensions():
    p=cloud();p[:,0]=.1
    with pytest.raises(ValueError):fit_bottle(p,CONFIG)
    p=cloud();p[:,2]+=.05
    with pytest.raises(ValueError):fit_bottle(p,CONFIG)
    with pytest.raises(ValueError,match='Measure bottle'):validate_bottle_config({})


def test_height_offset_is_reported_without_silently_shifting_cloud():
    from graspgen_vision.bottle_geometry import cloud_diagnostics
    p=cloud()
    p[:,2]=(p[:,2]-.048)/.024*.12-.0195
    original=p.copy()
    diagnostics=cloud_diagnostics(p,CONFIG)
    assert diagnostics['z_quantiles_mm'][1]<-15
    assert diagnostics['below_table_allowance_fraction']>.05
    with pytest.raises(ValueError,match='Bottle height mismatch.*p01=.*TCP does not affect'):
        fit_bottle(p,CONFIG)
    np.testing.assert_array_equal(p,original)


def test_measured_lower_table_changes_reference_not_cloud_or_grip_above_table():
    from graspgen_vision.bottle_geometry import cloud_diagnostics
    config=dict(CONFIG,table_z_m=-.011);p=cloud();p[:,2]-=.011;original=p.copy()
    g=fit_bottle(p,config)
    np.testing.assert_allclose(g['center'],[.08,-.3,.049],atol=1e-6)
    assert g['grasp_height_m']==pytest.approx(.06)
    assert g['table_z_m']==pytest.approx(-.011)
    assert g['body_bottom_m']==pytest.approx(.045)
    stats=cloud_diagnostics(p,config)
    assert stats['grasp_band_point_count']>80
    assert stats['below_table_allowance_fraction']==0
    np.testing.assert_array_equal(p,original)
    with pytest.raises(ValueError):fit_bottle(p,dict(config,table_z_m=float('nan')))
