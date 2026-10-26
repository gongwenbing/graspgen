"""Camera/mount envelope in link6; no ROS or hardware side effects."""
import numpy as np
from scipy.spatial.transform import Rotation


def check_transform(value):
    t = np.asarray(value, dtype=float)
    if t.shape != (4, 4) or not np.isfinite(t).all():
        raise ValueError('Invalid camera transform')
    r = t[:3, :3]
    if not np.allclose(t[3], [0, 0, 0, 1], atol=1e-6) or not np.allclose(r.T @ r, np.eye(3), atol=1e-5) or np.linalg.det(r) < .999:
        raise ValueError('Camera transform is not rigid')
    return t


def envelope(root_transform, config):
    """Returns dimensions, center and quaternion, all attached to link6."""
    t = check_transform(root_transform)
    size = np.asarray(config['size_left_right_front_back_up_down'], dtype=float)
    center = np.asarray(config['center_camera_link_m'], dtype=float)
    margin = float(config['padding_each_side_m'])
    if size.shape != (3,) or center.shape != (3,) or not np.isfinite(size).all() or not np.isfinite(center).all() or np.any(size <= 0) or not np.isfinite(margin) or not 0 <= margin <= .02:
        raise ValueError('Invalid camera envelope dimensions/offset/padding')
    # camera_link: X forward, Y left, Z up. User dimensions: left-right,
    # front-back, up-down. Root is at the depth sensor, not body center.
    dimensions = size[[1, 0, 2]] + 2 * margin
    position = t[:3, :3] @ center + t[:3, 3]
    quaternion = Rotation.from_matrix(t[:3, :3]).as_quat()
    return dimensions, position, quaternion
