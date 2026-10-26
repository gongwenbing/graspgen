"""Launch a randomized cube scene and optionally MoveIt with the same base height."""
import importlib.util
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('base_height', default_value='1.015'),
        DeclareLaunchArgument('moveit', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('enable_camera', default_value='false'),
        DeclareLaunchArgument('seed', default_value='random'),
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context):
    share = get_package_share_directory('piper_gazebo')
    spec = importlib.util.spec_from_file_location('stack_scene', os.path.join(share, 'scripts', 'stack_scene.py'))
    if spec is None or spec.loader is None:
        raise RuntimeError('Cannot load scene generator.')
    generator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(generator)
    seed = LaunchConfiguration('seed').perform(context)
    with tempfile.NamedTemporaryFile(prefix='piper_stack_', suffix='.world', delete=False) as temporary:
        world = temporary.name
    generator.randomize_world(os.path.join(share, 'worlds', 'pick_cube.world'), world,
                              None if seed == 'random' else int(seed))
    base_height = LaunchConfiguration('base_height')

    def cleanup(_context):
        if os.path.exists(world):
            os.unlink(world)
        return []

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                share, 'launch', 'piper_with_gripper', 'piper_gazebo.launch.py')),
            launch_arguments={'world': world, 'base_height': base_height,
                              'enable_camera': LaunchConfiguration('enable_camera'),
                              'gui': LaunchConfiguration('gui')}.items(),
        ),
        Node(package='piper_gazebo', executable='sync_pick_scene.py', output='screen',
             parameters=[{'use_sim_time': True, 'scene_file': world}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('piper_with_gripper_moveit'), '/launch/piper_moveit.launch.py']),
            launch_arguments={'base_height': base_height, 'rviz': LaunchConfiguration('rviz'),
                              'enable_camera': LaunchConfiguration('enable_camera')}.items(),
            condition=IfCondition(LaunchConfiguration('moveit')),
        ),
    ]
