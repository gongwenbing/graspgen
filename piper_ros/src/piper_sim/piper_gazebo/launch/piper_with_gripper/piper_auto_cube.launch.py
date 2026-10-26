"""One launch: scene, D455, MoveIt, RGB-D cubes, automatic pick and place."""
import os
import socket
from urllib.parse import urlparse

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    # A stale server used to cause cascaded RViz/MoveIt shutdown errors. Fail
    # before launching anything, with the actual conflicting master address.
    uri = urlparse(os.environ.get('GAZEBO_MASTER_URI', 'http://localhost:11345'))
    with socket.socket() as sock:
        sock.settimeout(.3)
        if sock.connect_ex((uri.hostname or 'localhost', uri.port or 11345)) == 0:
            raise RuntimeError('Gazebo master port is occupied. Stop the previous simulation first '
                               '(Ctrl+C, then pkill -TERM -x gzserver if needed).')
    share = get_package_share_directory('piper_gazebo')
    vision = get_package_share_directory('graspgen_vision')
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            share, 'launch/piper_with_gripper/piper_pick_scene.launch.py')),
            launch_arguments=dict(moveit='true', enable_camera='true',
                gui=LaunchConfiguration('gui'), rviz=LaunchConfiguration('rviz'), seed=LaunchConfiguration('seed')).items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(vision, 'launch/cube_vision.launch.py'))),
        Node(package='piper_gazebo', executable='auto_cube_task.py', name='auto_cube_task', output='screen', parameters=[{
            'use_sim_time': True,
            'count': ParameterValue(LaunchConfiguration('count'), value_type=int),
            'assisted_grasp': ParameterValue(LaunchConfiguration('assisted_grasp'), value_type=bool),
            'transport_height': ParameterValue(LaunchConfiguration('transport_height'), value_type=float),
        }]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('seed', default_value='455'),
        DeclareLaunchArgument('count', default_value='3'),
        DeclareLaunchArgument('assisted_grasp', default_value='true'),
        DeclareLaunchArgument('transport_height', default_value='0.09'),
        OpaqueFunction(function=setup),
    ])
