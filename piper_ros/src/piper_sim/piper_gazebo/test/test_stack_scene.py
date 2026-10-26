"""Random layouts stay separated and held-object transforms are reversible."""
import math
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from geometry_msgs.msg import Pose, Quaternion

scripts = str(Path(__file__).resolve().parents[1] / 'scripts')
sys.path.insert(0, scripts)
try:
    from stack_cubes import inverse_pose
    from stack_scene import randomize_world
    from sync_pick_scene import compose_pose
finally:
    sys.path.remove(scripts)


class StackSceneTest(unittest.TestCase):
    def test_seeded_layouts_are_reachable_and_do_not_overlap(self):
        source = Path(__file__).resolve().parents[1] / 'worlds' / 'pick_cube.world'
        with tempfile.TemporaryDirectory() as directory:
            output = str(Path(directory) / 'scene.world')
            for seed in range(100):
                points = list(randomize_world(str(source), output, seed).values())
                self.assertEqual(points[0], (0.4, 0.0))
                for x, y in points[1:]:
                    self.assertLessEqual(math.hypot(x, y), 0.395)
                    self.assertLess(y, -0.059)
                for i, j in ((0, 1), (0, 2), (1, 2)):
                    self.assertGreaterEqual(math.dist(points[i], points[j]), 0.105)
            world = ET.parse(output).getroot().find('world')
            basket = world.find("model[@name='pick_basket']")
            self.assertEqual(len(basket.findall('link/collision')), 5)
            self.assertEqual(len(world.findall('model/link/sensor')), 3)

    def test_relative_payload_transform_reconstructs_world_pose(self):
        wrist = Pose()
        wrist.position.x, wrist.position.y, wrist.position.z = 0.31, -0.14, 1.20
        wrist.orientation = Quaternion(y=1.0, w=0.0)
        cube = Pose()
        cube.position.x, cube.position.y, cube.position.z = 0.311, -0.139, 1.075
        relative = compose_pose(inverse_pose(wrist), cube)
        restored = compose_pose(wrist, relative)
        self.assertAlmostEqual(restored.position.x, cube.position.x)
        self.assertAlmostEqual(restored.position.y, cube.position.y)
        self.assertAlmostEqual(restored.position.z, cube.position.z)
        self.assertAlmostEqual(restored.orientation.w, 1.0)


if __name__ == '__main__':
    unittest.main()
