"""ROS 2 RGB-D only; no arm, calibration or grasp execution."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_no',default_value="''"),
        DeclareLaunchArgument('rgb_camera.color_profile',default_value='640,480,30'),
        DeclareLaunchArgument('depth_module.depth_profile',default_value='640,480,30'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('realsense2_camera'))/'launch/rs_launch.py')),
            launch_arguments={
                'camera_namespace':'', 'camera_name':'camera',
                'serial_no':LaunchConfiguration('serial_no'),
                'enable_color':'true','enable_depth':'true',
                'align_depth.enable':'true','enable_sync':'true',
                'rgb_camera.color_profile':LaunchConfiguration('rgb_camera.color_profile'),
                'depth_module.depth_profile':LaunchConfiguration('depth_module.depth_profile'),
                'pointcloud.enable':'false','publish_tf':'true',
            }.items()),
    ])
