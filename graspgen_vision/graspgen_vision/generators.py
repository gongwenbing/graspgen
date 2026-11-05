"""Metric candidate backends. No ROS, learned model, or actuator interfaces."""
from .geometry import pca_grasp
from .cubes import cube_grasp


def generate(points, method, *, max_opening=.07, up=None, cube_size=.04):
    if method == 'pca':
        result = pca_grasp(points, max_opening)
        result['position_semantics'] = 'observed_surface_center'
    elif method == 'known_cube':
        if up is None:
            raise ValueError('Known cube needs image-time world up transform')
        result = cube_grasp(points, up, cube_size)
        if result['opening'] > max_opening:
            raise ValueError('Cube exceeds gripper opening')
        result['position_semantics'] = 'known_size_cube_center'
    else:
        raise ValueError(f'Unsupported generator: {method}')
    result['generator'] = method
    # List interface permits additional candidates without coupling to YOLO.
    return [result]
