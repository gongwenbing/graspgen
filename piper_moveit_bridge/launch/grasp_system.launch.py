"""Passive complete system startup; never enables motors, gates, or grasp actions."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,ExecuteProcess,TimerAction,RegisterEventHandler,LogInfo
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    root=Path.home()/'GraspGen'
    def include(package,file,arguments=None,condition=None):
        kwargs={'launch_arguments':(arguments or {}).items()}
        if condition is not None:kwargs['condition']=condition
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory(package))/'launch'/file)),**kwargs)
    table=ExecuteProcess(cmd=['ros2','run','piper_moveit_bridge','add_table'],output='screen')
    camera=ExecuteProcess(cmd=['ros2','run','piper_moveit_bridge','add_camera'],output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('start_camera',default_value='true'),
        DeclareLaunchArgument('can_port',default_value='can0'),
        DeclareLaunchArgument('sdk_path',default_value=str(Path.home()/'piper_sdk')),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('generator',default_value='auto'),
        DeclareLaunchArgument('imgsz',default_value='960'),
        include('graspgen_camera','rgbd.launch.py',condition=IfCondition(LaunchConfiguration('start_camera'))),
        include('piper_moveit_bridge','moveit_hardware.launch.py',{
            'sdk_path':LaunchConfiguration('sdk_path'),'can_port':LaunchConfiguration('can_port'),
            'read_only':'false','rviz':LaunchConfiguration('rviz')}),
        include('graspgen_vision','vision.launch.py',{'generator':LaunchConfiguration('generator'),'imgsz':LaunchConfiguration('imgsz')}),
        Node(package='graspgen_handeye',executable='publish_mount',output='screen',parameters=[{'calibration_file':str(root/'calibration/handeye_result.yaml')}]),
        include('graspgen_handeye','preview.launch.py',{'rviz':'false'}),
        RegisterEventHandler(OnProcessExit(target_action=table,on_exit=lambda event,context:
            [camera] if event.returncode==0 else [LogInfo(msg='Table setup failed; inspect logs before planning or execution.')])),
        TimerAction(period=8.,actions=[table]),
    ])
