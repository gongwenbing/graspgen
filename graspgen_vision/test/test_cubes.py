import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from graspgen_vision.cubes import CubeDetector, cube_grasp


def test_color_components_do_not_require_yolo_cube_class():
    image = np.full((120, 180, 3), 80, np.uint8)
    image[20:55, 15:50] = [10, 20, 230]
    image[20:55, 65:100] = [230, 30, 10]
    image[20:55, 115:150] = [10, 220, 20]
    result = CubeDetector().predict(image)[0]
    assert result.boxes.cls == [0, 1, 2]
    assert len(result.masks.data.numpy()) == 3
    assert CubeDetector().predict(image, classes=[1])[0].boxes.cls == [1]


def test_square_top_fit_recovers_known_cube_center_in_rotated_camera():
    u, v = np.meshgrid(np.linspace(-.02, .02, 35), np.linspace(-.02, .02, 35))
    points = np.column_stack((u.ravel(), v.ravel(), np.full(u.size, .02)))
    rotation = Rotation.from_euler('xyz', [.5, -.8, .3])
    center = np.array([.03, .05, .5])
    result = cube_grasp(rotation.apply(points)+center, rotation.apply([0, 0, 1]))
    np.testing.assert_allclose(result['center'], center, atol=1e-6)
    assert result['opening'] == pytest.approx(.045)
    axes = Rotation.from_quat(result['quaternion']).as_matrix()
    np.testing.assert_allclose(axes[:, 2], -rotation.apply([0, 0, 1]), atol=1e-6)
    with pytest.raises(ValueError, match='does not match'):
        cube_grasp(rotation.apply(points*2)+center, rotation.apply([0, 0, 1]))
