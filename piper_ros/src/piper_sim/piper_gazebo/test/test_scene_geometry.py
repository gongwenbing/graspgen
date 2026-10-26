"""Regression for a tabletop and four cylindrical legs saved by Gazebo."""
import importlib.util
import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from geometry_msgs.msg import Pose, Quaternion
from shape_msgs.msg import SolidPrimitive

spec = importlib.util.spec_from_file_location(
    'sync_pick_scene', Path(__file__).resolve().parents[1] / 'scripts' / 'sync_pick_scene.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class SceneGeometryTest(unittest.TestCase):
    def test_world_has_no_saved_robot_or_duplicate_link_elements(self):
        path = Path(__file__).resolve().parents[1] / 'worlds' / 'pick_cube.world'
        world = ET.parse(path).getroot().find('world')
        self.assertIsNone(world.find("model[@name='piper']"))
        self.assertIsNone(world.find('state'))
        for link in world.findall('model/link'):
            names = [element.get('name') for element in link
                     if element.tag in ('collision', 'visual', 'sensor')]
            self.assertEqual(len(names), len(set(names)))

    def test_all_table_collisions_and_offsets_are_preserved(self):
        path = Path(__file__).resolve().parents[1] / 'worlds' / 'pick_cube.world'
        links = module.read_collisions(str(path))
        table = next(item for item in links if item[0] == 'pick_table')
        self.assertEqual(table[1], 'pick_table::link')
        self.assertEqual(len(table[2]), 5)
        top, local = table[2][0]
        self.assertEqual(top.type, SolidPrimitive.BOX)
        self.assertEqual(list(top.dimensions), [1.5, 0.8, 0.03])
        self.assertAlmostEqual(local.position.z + top.dimensions[2] / 2, 1.015)
        for leg, pose in table[2][1:]:
            self.assertEqual(leg.type, SolidPrimitive.CYLINDER)
            self.assertEqual(list(leg.dimensions), [1.0, 0.02])
            self.assertAlmostEqual(pose.position.z, 0.5)

    def test_link_rotation_is_applied_to_collision_offset(self):
        parent = Pose()
        parent.position.x = 1.0
        parent.orientation = Quaternion(z=math.sqrt(0.5), w=math.sqrt(0.5))
        local = Pose()
        local.position.x = 0.4
        local.position.z = 1.0
        result = module.compose_pose(parent, local)
        self.assertAlmostEqual(result.position.x, 1.0)
        self.assertAlmostEqual(result.position.y, 0.4)
        self.assertAlmostEqual(result.position.z, 1.0)
        self.assertAlmostEqual(result.orientation.z, math.sqrt(0.5))


if __name__ == '__main__':
    unittest.main()
