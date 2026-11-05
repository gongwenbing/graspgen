from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Isolated inference environment; ROS imports inherited from system site packages.
    workspace=Path(get_package_share_directory('graspgen_vision')).parents[3]
    return LaunchDescription([
        DeclareLaunchArgument('python',default_value=str(workspace/'.venv-vision/bin/python')),
        DeclareLaunchArgument('model',default_value=str(workspace/'models/yolov8n-seg.pt')),
        DeclareLaunchArgument('device',default_value='cpu'),
        DeclareLaunchArgument('imgsz',default_value='960'),
        DeclareLaunchArgument('backend',default_value='yolo'),
        DeclareLaunchArgument('generator',default_value='auto'),
        DeclareLaunchArgument('bottle_config',default_value=str(workspace/'calibration/bottle_geometry.json')),
        DeclareLaunchArgument('cube_size',default_value='0.04'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('target_classes',default_value='bottle,cup,mouse,cell phone,orange'),
        # D455 640x480 nominal Min-Z is 0.32m; leave 3cm operating margin.
        DeclareLaunchArgument('min_depth',default_value='0.35'),
        DeclareLaunchArgument('max_depth',default_value='1.5'),
        Node(package='graspgen_vision',executable='segment_grasp',output='screen',
             prefix=[LaunchConfiguration('python')],additional_env={
                 'YOLO_CONFIG_DIR':str(workspace/'.cache/ultralytics'),
                 'MPLCONFIGDIR':str(workspace/'.cache/matplotlib'),
                 'OMP_NUM_THREADS':'4','OPENBLAS_NUM_THREADS':'1',
             },parameters=[{
                 'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool),
                 'model':LaunchConfiguration('model'),'device':LaunchConfiguration('device'),
                 'imgsz':ParameterValue(LaunchConfiguration('imgsz'),value_type=int),
                 'backend':LaunchConfiguration('backend'),
                 'cube_size':ParameterValue(LaunchConfiguration('cube_size'),value_type=float),
                 'target_classes':ParameterValue(LaunchConfiguration('target_classes'),value_type=str),
                 'min_depth':ParameterValue(LaunchConfiguration('min_depth'),value_type=float),
                 'max_depth':ParameterValue(LaunchConfiguration('max_depth'),value_type=float),
             }]),
        Node(package='graspgen_vision',executable='generate_grasps',output='screen',
             parameters=[{
                 'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool),
                 'generator':LaunchConfiguration('generator'),
                 'bottle_config':LaunchConfiguration('bottle_config'),
             }]),
    ])
