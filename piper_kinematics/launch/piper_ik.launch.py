"""Launch the Piper IK solver and six-axis interactive pose marker."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    defaults = {
        'model_path': (os.path.join(get_package_share_directory('piper_description'), 'urdf', 'piper_description.urdf'), str),
        'max_iterations': ('200', int),
        'damping_factor': ('0.03', float),
        'hardware_mode': ('false', bool),
        'feedback_timeout': ('0.5', float),
        'max_joint_velocity': ('0.2', float),
        'max_gripper_velocity': ('0.01', float),
        'tip_link': ('link6', str),
        'joint_command_topic': ('/ik/joint_commands', str),
        'publish_rate': ('30.0', float),
        'use_analytic_ik': ('false', bool),
        'joint_state_topic': ('/ik/joint_states', str),
        'target_pose_topic': ('/target_pose', str),
        'sub_real_joint_position_topic': ('/joint_states_single', str),
        'gripper_cmd_topic': ('/gripper_cmd_topic', str),
    }
    arguments = [DeclareLaunchArgument(name, default_value=value)
                 for name, (value, _) in defaults.items()]
    parameters = {name: ParameterValue(LaunchConfiguration(name), value_type=kind)
                  for name, (_, kind) in defaults.items()}
    return LaunchDescription(arguments + [
        Node(package='piper_kinematics', executable='piper_ik_node',
             output='screen', parameters=[parameters]),
        Node(package='piper_kinematics', executable='interactive_pose_marker.py',
             output='screen', parameters=[{'pose_topic': parameters['target_pose_topic']}]),
    ])
