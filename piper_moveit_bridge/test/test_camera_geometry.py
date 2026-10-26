import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from piper_moveit_bridge.camera_geometry import envelope


def config():
    return {'size_left_right_front_back_up_down': [.125, .03, .05],
            'center_camera_link_m': [-.01045, -.0475, -.0105], 'padding_each_side_m': .003}


def test_camera_sensor_origin_is_not_box_center():
    dims, center, q = envelope(np.eye(4), config())
    assert np.allclose(dims, [.036, .131, .056])
    # Nominal camera top is 14.5mm above root; envelope adds 3mm.
    assert np.isclose(center[2] + dims[2] / 2, .0175)
    assert np.isclose(center[2] - dims[2] / 2, -.0385)
    assert center[1] == -.0475


def test_full_mount_rotation_and_translation_applied():
    t = np.eye(4)
    t[:3, :3] = Rotation.from_euler('xyz', [.2, -.5, 1.2]).as_matrix()
    t[:3, 3] = [.1, -.02, .07]
    dims, center, q = envelope(t, config())
    assert np.allclose(np.linalg.inv(t) @ np.r_[center, 1], np.r_[config()['center_camera_link_m'], 1])
    assert np.allclose(Rotation.from_quat(q).as_matrix(), t[:3, :3])


def test_reject_nonrigid_or_invalid_size():
    t = np.eye(4); t[0, 0] = 2
    with pytest.raises(ValueError): envelope(t, config())
    c = config(); c['size_left_right_front_back_up_down'][0] = -1
    with pytest.raises(ValueError): envelope(np.eye(4), c)
