"""Known-size coloured cubes in the D455 Gazebo scene."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('cube_size', default_value='0.04'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('graspgen_vision'), 'launch', 'vision.launch.py')),
            launch_arguments={
                'backend': 'color_cube', 'python': '/usr/bin/python3',
                'target_classes': 'cube_red,cube_blue,cube_green',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'cube_size': LaunchConfiguration('cube_size'),
            }.items()),
    ])
