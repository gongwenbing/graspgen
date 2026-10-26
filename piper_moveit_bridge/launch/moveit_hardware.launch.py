"""Real feedback and gated actions. Startup is passive, including read_only:=false."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    model_path=Path(get_package_share_directory('piper_description'))/'urdf/piper_description.urdf'
    config=MoveItConfigsBuilder('piper',package_name='piper_with_gripper_moveit').robot_description(file_path=str(model_path)).to_moveit_configs()
    config.sensors_3d={}
    # Match the bridge's strict arm bounds: do not prepend an out-of-bounds
    # measured state and report a plan that the hardware action must reject.
    for pipeline in config.planning_pipelines.get('planning_pipelines',[]):
        config.planning_pipelines[pipeline]['start_state_max_bounds_error']=1e-6
    config.joint_limits.update({'default_velocity_scaling_factor':0.32,'default_acceleration_scaling_factor':0.15})
    for name,limit in config.joint_limits['robot_description_planning']['joint_limits'].items():
        limit.update({'has_velocity_limits':True,'max_velocity':0.005 if name=='joint7' else 0.2,
                      'has_acceleration_limits':True,'max_acceleration':0.01 if name=='joint7' else 0.1})
    return LaunchDescription([
        DeclareLaunchArgument('can_port',default_value='can0'),
        DeclareLaunchArgument('sdk_path',default_value='',description='SDK source root; empty uses installed SDK'),
        DeclareLaunchArgument('read_only',default_value='true'),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('start_can_gateway',default_value='true'),
        Node(package='piper_moveit_bridge',executable='can_gateway',output='screen',
             condition=IfCondition(LaunchConfiguration('start_can_gateway')),
             parameters=[{'can_port':LaunchConfiguration('can_port'),
                          'sdk_path':ParameterValue(LaunchConfiguration('sdk_path'),value_type=str),
                          'read_only':ParameterValue(LaunchConfiguration('read_only'),value_type=bool),
                          'model_path':str(model_path)}]),
        Node(package='piper_moveit_bridge',executable='trajectory_bridge',output='screen',parameters=[{'model_path':str(model_path)}]),
        Node(package='robot_state_publisher',executable='robot_state_publisher',output='screen',parameters=[config.robot_description,{'use_sim_time':False}]),
        Node(package='moveit_ros_move_group',executable='move_group',output='screen',parameters=[config.to_dict(),
            {'use_sim_time':False,'allow_trajectory_execution':True,'publish_robot_description_semantic':True,
             'trajectory_execution.allowed_execution_duration_scaling':2.0,'trajectory_execution.allowed_goal_duration_margin':5.0}]),
        Node(package='rviz2',executable='rviz2',output='log',condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d',str(config.package_path/'config/moveit.rviz')],
             parameters=[config.robot_description,config.robot_description_semantic,
                         config.robot_description_kinematics,config.planning_pipelines,{'use_sim_time':False}]),
    ])
