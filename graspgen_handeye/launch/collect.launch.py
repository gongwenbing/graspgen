from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    workspace=Path(get_package_share_directory('graspgen_handeye')).parents[3]
    return LaunchDescription([
        DeclareLaunchArgument('samples_file',default_value=str(workspace/'calibration/handeye_samples.json')),
        DeclareLaunchArgument('dictionary',default_value='DICT_4X4_50'),
        Node(package='graspgen_handeye',executable='collect',output='screen',parameters=[{
            'marker_id':0,'marker_length':.095,'base_frame':'base_link','gripper_frame':'link6',
            'camera_frame':'camera_color_optical_frame',
            'dictionary':LaunchConfiguration('dictionary'),
            'samples_file':LaunchConfiguration('samples_file'),
        }]),
    ])
