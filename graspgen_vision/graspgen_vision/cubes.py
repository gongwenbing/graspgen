"""RGB-D detection of known-size, upright coloured cubes (no Gazebo poses)."""
from types import SimpleNamespace
import cv2
import numpy as np
from scipy.spatial.transform import Rotation


class Array:
    def __init__(self, value):
        self.value = value

    def cpu(self):
        return self

    def numpy(self):
        return self.value


class CubeDetector:
    task = 'segment'
    names = {0: 'cube_red', 1: 'cube_blue', 2: 'cube_green'}

    def predict(self, bgr, classes=None, **kwargs):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        masks, labels = [], []
        overlay = bgr.copy()
        for index, ranges in enumerate(([(0, 12), (170, 179)], [(95, 135)], [(35, 90)])):
            if classes is not None and index not in classes:
                continue
            mask = np.zeros(bgr.shape[:2], np.uint8)
            for low, high in ranges:
                mask |= cv2.inRange(hsv, (low, 100, 45), (high, 255, 255))
            count, components, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
            for component in range(1, count):
                x, y, w, h, area = stats[component]
                if area < 60 or min(w, h) < 6 or max(w, h)/min(w, h) > 3:
                    continue
                masks.append((components == component).astype(np.float32))
                labels.append(index)
                cv2.rectangle(overlay, (x, y), (x+w, y+h), (0, 255, 255), 1)
                cv2.putText(overlay, self.names[index], (x, max(y-4, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 255, 255), 1)
        return [SimpleNamespace(names=self.names,
            masks=SimpleNamespace(data=Array(np.array(masks))) if masks else None,
            boxes=SimpleNamespace(cls=labels, conf=[1.]*len(labels)), plot=lambda: overlay)]


def cube_grasp(points, up, size=.04):
    """Fit the visible horizontal top face; infer centre using known cube size.

    A square has no unique PCA tangent. Its face edges provide a closing axis
    modulo 90 degrees. Reject incomplete top faces and non-cube dimensions.
    """
    points = np.asarray(points, dtype=float)
    up = np.asarray(up, dtype=float)
    up /= np.linalg.norm(up)
    heights = points @ up
    top_height = np.quantile(heights, .98)
    top = points[np.abs(heights-top_height) < .002]
    if len(top) < 50:
        raise ValueError('Cube top face is not sufficiently visible')
    x = np.cross(up, [0., 0., 1.])
    if np.linalg.norm(x) < .1:
        x = np.cross(up, [0., 1., 0.])
    x /= np.linalg.norm(x)
    y = np.cross(up, x)
    flat = np.column_stack((top@x, top@y)).astype(np.float32)
    (cx, cy), (width, height), angle = cv2.minAreaRect(flat)
    if not all(.65*size <= extent <= 1.2*size for extent in (width, height)):
        raise ValueError(f'Visible top face does not match {size:.3f}m cube: {width:.3f} x {height:.3f}')
    a = np.deg2rad(angle)
    closing = np.cos(a)*x+np.sin(a)*y
    z = -up
    axes = np.column_stack((np.cross(closing, z), closing, z))
    center = cx*x+cy*y+(float(np.median(top@up))-size/2)*up
    return dict(center=center, quaternion=Rotation.from_matrix(axes).as_quat(),
                extent=np.array([size]*3), opening=size+.005, axes=axes)
