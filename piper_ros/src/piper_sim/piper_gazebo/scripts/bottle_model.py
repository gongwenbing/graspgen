"""Read shared bottle dimensions and observation geometry from its SDF."""
import math
import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory


def model_path():
    local = Path(__file__).resolve().parents[1]/'worlds/bottle.sdf'
    return local if local.is_file() else Path(get_package_share_directory('piper_gazebo'))/'worlds/bottle.sdf'


def dimensions():
    root = ET.parse(model_path()).getroot()
    body = root.find("model/link/collision[@name='body']/geometry/cylinder")
    bounds = []
    for collision in root.findall('model/link/collision'):
        xyz = [float(x) for x in collision.findtext('pose', '0 0 0 0 0 0').split()[:3]]
        g = collision.find('geometry')
        half = float(g.findtext('cylinder/length'))/2 if g.find('cylinder') is not None else float(g.findtext('sphere/radius'))
        bounds.extend([xyz[2]-half, xyz[2]+half])
    return {'radius': float(body.findtext('radius')), 'body_height': float(body.findtext('length')),
            'height': max(bounds)-min(bounds), 'center_z': (max(bounds)+min(bounds))/2}


@lru_cache(maxsize=1)
def centered_surface_points():
    root = ET.parse(model_path()).getroot()
    center = dimensions()['center_z']
    points = []
    for collision in root.findall('model/link/collision'):
        pose = [float(x) for x in collision.findtext('pose', '0 0 0 0 0 0').split()]
        if any(abs(x) > 1e-9 for x in pose[3:]):
            raise ValueError('Observation sampling requires unrotated bottle collision primitives.')
        g = collision.find('geometry')
        if g.find('cylinder') is not None:
            r, h = float(g.findtext('cylinder/radius')), float(g.findtext('cylinder/length'))
            rings = [(r, -h/2), (r, h/2)]
        else:
            r = float(g.findtext('sphere/radius'))
            rings = [(r*math.cos(a), r*math.sin(a)) for a in np.linspace(-math.pi/2, math.pi/2, 9)]
        for r, z in rings:
            for a in np.linspace(0, 2*math.pi, 16, endpoint=False):
                points.append([pose[0]+r*math.cos(a), pose[1]+r*math.sin(a), pose[2]+z-center])
    return np.array(points)
