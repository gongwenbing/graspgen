from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share=Path(get_package_share_directory('graspgen_handeye'))
    return LaunchDescription([
        DeclareLaunchArgument('tcp_z',default_value='0.1358'),
        DeclareLaunchArgument('hover_height',default_value='0.10'),
        DeclareLaunchArgument('rviz',default_value='true'),
        Node(package='graspgen_handeye',executable='preview',output='screen',parameters=[{
            'tcp_z':ParameterValue(LaunchConfiguration('tcp_z'),value_type=float),
            'hover_height':ParameterValue(LaunchConfiguration('hover_height'),value_type=float)}]),
        Node(package='rviz2',executable='rviz2',condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d',str(share/'config/preview.rviz')]),
    ])
