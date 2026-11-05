"""Known-radius upright bottle fitting in base_link. No empirical offsets."""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def validate_bottle_config(config):
    keys = ('diameter_m', 'total_height_m', 'body_bottom_m', 'body_top_m', 'grasp_height_m')
    if any(not isinstance(config.get(k), (int, float)) or isinstance(config[k], bool)
           or not np.isfinite(config[k]) for k in keys):
        raise ValueError('Measure bottle first: set diameter_m, total_height_m, body_bottom_m, body_top_m, grasp_height_m in calibration/bottle_geometry.json')
    d, h, lo, hi, z = (float(config[k]) for k in keys)
    if not .01 <= d <= .06 or not .03 <= h <= .35 or not 0 <= lo < z < hi <= h:
        raise ValueError('Invalid bottle dimensions/body/grasp height (metres)')
    if min(z-lo, hi-z) < .01:
        raise ValueError('Grasp band needs >=10mm margin from body ends')
    table_z = config.get('table_z_m', 0.)
    if isinstance(table_z,bool) or not isinstance(table_z,(int,float)) or not np.isfinite(table_z) or not -.15<=table_z<=.15:
        raise ValueError('Invalid measured table_z_m')
    return d, h, lo, hi, z


def cloud_diagnostics(points, config):
    """Report measured support before rejection; never shifts or rescales points."""
    p = np.asarray(points, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3 or not len(p) or not np.isfinite(p).all():
        raise ValueError('Invalid bottle cloud')
    height = float(config['total_height_m']); table_z=float(config.get('table_z_m',0.)); grasp = table_z+float(config['grasp_height_m'])
    quantiles = np.quantile(p[:, 2], [0., .01, .05, .5, .95, .99, 1.])
    return {
        'frame': 'base_link', 'point_count': len(p), 'table_z_m':table_z,
        'z_above_table_quantiles_mm':((quantiles-table_z)*1000).tolist(),
        'z_quantile_levels': [0., .01, .05, .5, .95, .99, 1.],
        'z_quantiles_mm': (quantiles*1000).tolist(),
        'xyz_min_mm': (p.min(axis=0)*1000).tolist(),
        'xyz_max_mm': (p.max(axis=0)*1000).tolist(),
        'below_table_allowance_fraction': float(np.mean(p[:, 2] < table_z-.005)),
        'above_height_allowance_fraction': float(np.mean(p[:, 2] > table_z+height+.005)),
        'grasp_band_point_count': int(np.count_nonzero(abs(p[:, 2]-grasp) <= .01)),
    }


def fit_bottle(points, config):
    d, height, bottom, top, grasp_z = validate_bottle_config(config)
    table_z=float(config.get('table_z_m',0.));bottom+=table_z;top+=table_z;grasp_z+=table_z
    p = np.asarray(points, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3 or len(p) < 80 or not np.isfinite(p).all():
        raise ValueError('Invalid bottle cloud')
    if np.quantile(p[:, 2], .01) < table_z-.005 or np.quantile(p[:, 2], .99) > table_z+height+.005:
        diagnostics = cloud_diagnostics(p, config)
        low, high = diagnostics['z_quantiles_mm'][1], diagnostics['z_quantiles_mm'][-2]
        raise ValueError(f'Bottle height mismatch in base_link: Z p01={low:.1f}mm, p99={high:.1f}mm; allowed [{table_z*1000-5:.1f},{(table_z+height)*1000+5:.1f}]mm (table Z={table_z*1000:.1f}mm). Check table plane / image-time hand-eye TF; TCP does not affect this cloud transform')
    band = p[(abs(p[:, 2]-grasp_z) <= .01) & (p[:, 2] >= bottom) & (p[:, 2] <= top)]
    if len(band) < 60 or np.ptp(band[:, 2]) < .008:
        raise ValueError('Insufficient visible straight body at requested grasp height')
    xy = band[:, :2]
    _, indices = np.unique(np.round(xy/.0005), axis=0, return_index=True)
    xy = xy[indices]
    if len(xy) < 20:
        raise ValueError('Too little independent bottle arc support')
    radius = d/2
    a = np.column_stack((2*xy, np.ones(len(xy))))
    solution, _, rank, _ = np.linalg.lstsq(a, (xy*xy).sum(axis=1), rcond=None)
    if rank < 3:
        raise ValueError('Degenerate bottle arc')
    initial = solution[:2]
    fitted_radius = np.sqrt(max(0., solution[2]+initial@initial))
    if abs(fitted_radius-radius) > max(.002, radius*.2):
        raise ValueError('Observed curvature differs from measured bottle radius')
    fit = least_squares(lambda center: np.linalg.norm(xy-center, axis=1)-radius,
                        initial, loss='soft_l1', f_scale=.001)
    residual = np.abs(np.linalg.norm(xy-fit.x, axis=1)-radius)
    inliers = residual <= .003
    if inliers.mean() < .85 or np.sqrt(np.mean(residual**2)) > .002:
        raise ValueError('Bottle circle fit residual too large')
    directions = xy[inliers]-fit.x
    angles = np.sort(np.mod(np.arctan2(directions[:, 1], directions[:, 0]), 2*np.pi))
    arc = 2*np.pi-np.max(np.diff(np.r_[angles, angles[0]+2*np.pi]))
    if arc < np.deg2rad(45):
        raise ValueError('Visible bottle arc <45deg; change observation view')
    # Two height bands must agree: rejects appreciably tapered/tilted bodies.
    centers = []
    for segment in (band[band[:, 2] < grasp_z], band[band[:, 2] >= grasp_z]):
        if len(segment) < 20:
            raise ValueError('Insufficient upper/lower bottle support')
        center = least_squares(lambda c: np.linalg.norm(segment[:, :2]-c, axis=1)-radius,
                               fit.x, loss='soft_l1', f_scale=.001).x
        centers.append(center)
    if np.linalg.norm(centers[0]-centers[1]) > .002:
        raise ValueError('Bottle not sufficiently upright/cylindrical in grasp band')
    if np.mean(np.linalg.norm(p[:, :2]-fit.x, axis=1) > radius+.004) > .1:
        raise ValueError('Cloud extends outside configured bottle envelope')
    center = np.r_[fit.x, grasp_z]
    if np.linalg.norm(center[:2]) < .08:
        raise ValueError('Bottle too close to base axis')
    # Model tool +Z approaches down, +Y closes radially in base XY.
    y = np.r_[center[:2]/np.linalg.norm(center[:2]), 0.]
    z = np.array([0., 0., -1.]); x = np.cross(y, z)
    axes = np.column_stack((x, y, z))
    return dict(center=center, quaternion=Rotation.from_matrix(axes).as_quat(), axes=axes,
                extent=np.array([d, d, height]), opening=d+.008,
                generator='bottle_cylinder', position_semantics='bottle_grasp_center',
                fit_rms_m=float(np.sqrt(np.mean(residual**2))), arc_deg=float(np.rad2deg(arc)),
                diameter_m=d, total_height_m=height, grasp_height_m=grasp_z-table_z,table_z_m=table_z,
                body_bottom_m=bottom-table_z, body_top_m=top-table_z)
