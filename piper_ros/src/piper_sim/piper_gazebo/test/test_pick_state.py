"""Regression for the Gazebo mimic name that caused MoveIt's IK service to abort."""
import importlib.util
import unittest
from pathlib import Path
from unittest.mock import Mock

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

spec = importlib.util.spec_from_file_location(
    'pick_cube', Path(__file__).resolve().parents[1] / 'scripts' / 'pick_cube.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class PickStateTest(unittest.TestCase):
    def test_numeric_parameters_have_a_float_result(self):
        node = Mock()
        for value in (0.05, 1):
            node.get_parameter.return_value.value = value
            result = module.PickCube.parameter(node, 'lift_height')
            self.assertIsInstance(result, float)
            self.assertEqual(result, value)
        for value in (None, True, '0.05'):
            node.get_parameter.return_value.value = value
            with self.assertRaises(RuntimeError):
                module.PickCube.parameter(node, 'lift_height')

    def test_boolean_parameters_are_separate_from_numbers(self):
        node = Mock()
        node.get_parameter.return_value.value = True
        self.assertIs(module.PickCube.bool_parameter(node, 'stabilize_grasp'), True)
        node.get_parameter.return_value.value = 1
        with self.assertRaises(RuntimeError):
            module.PickCube.bool_parameter(node, 'stabilize_grasp')

    def test_grasp_quaternion_points_tool_down(self):
        cube = Pose()
        cube.position.z = 0.72
        target = module.grasp_pose(cube, 0.125)
        q = target.orientation
        self.assertAlmostEqual(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w, 1.0)
        self.assertAlmostEqual(1.0 - 2.0*(q.x*q.x + q.y*q.y), -1.0)
        self.assertAlmostEqual(target.position.z, 0.845)
        self.assertAlmostEqual(cube.position.z, 0.72)

    def test_gazebo_alias_and_nan_efforts_do_not_reach_moveit(self):
        message = JointState()
        message.name = ['joint8_mimic', 'joint7', 'joint3', 'joint1',
                        'joint6', 'joint2', 'joint5', 'joint4']
        message.position = [-0.035, 0.035, -1.5, 0.1, 0.2, 2.0, 1.1, 0.0]
        message.effort = [float('nan')] * 8
        state = module.moveit_state(message)
        self.assertNotIn('joint8_mimic', state.joint_state.name)
        self.assertEqual(state.joint_state.position[0], 0.1)
        self.assertEqual(state.joint_state.position[2], -1.5)
        self.assertEqual(state.joint_state.position[-1], -0.035)
        self.assertEqual(list(state.joint_state.effort), [])

    def test_bad_sensor_positions_stop_motion(self):
        message = JointState()
        message.name = [f'joint{i}' for i in range(1, 8)]
        message.position = [0.0] * 6 + [float('nan')]
        with self.assertRaises(RuntimeError):
            module.moveit_state(message)


if __name__ == '__main__':
    unittest.main()
