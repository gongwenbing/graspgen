import os
import re
from xml.dom import minidom

import xacro
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty.world'),
        DeclareLaunchArgument('base_height', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('enable_camera', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context):
    robot_name_in_model = 'piper'
    package_name = 'piper_description'
    urdf_name = "piper_description_gazebo.xacro"

    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')

    # Start Gazebo server
    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gzserver', '--verbose', LaunchConfiguration('world'),
             '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen', additional_env={
            'GAZEBO_PLUGIN_PATH': os.pathsep.join(filter(None, [
                os.path.join(get_package_prefix('piper_gazebo'), 'lib'),
                os.environ.get('GAZEBO_PLUGIN_PATH', ''),
            ])),
        })
    # Manage both processes directly. The `gazebo` wrapper can exit while its
    # server survives, leaving port 11345 occupied after Ctrl+C.
    start_client_cmd = ExecuteProcess(
        cmd=['gzclient', '--verbose'], output='screen',
        condition=IfCondition(LaunchConfiguration('gui')))


    # 因为 urdf文件中有一句 $(find mybot) 需要用xacro进行编译一下才行
    xacro_file = urdf_model_path
    with open(xacro_file) as xacro_stream:
        doc = minidom.parse(xacro_stream)
    xacro.process_doc(doc, mappings={
        'enable_camera': LaunchConfiguration('enable_camera').perform(context),
        'base_height': LaunchConfiguration('base_height').perform(context)})
    params = {'robot_description': remove_comments(doc.toxml())}

    # 启动了robot_state_publisher节点后，该节点会发布 robot_description 话题，话题内容是模型文件urdf的内容？
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}, params, {"publish_frequency":15.0}],
        output='screen'
    )

    # Launch the robot, 通过robot_description话题进行模型内容获取从而在gazebo中生成模型
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model,  '-topic', 'robot_description'], output='screen')

    # 关节状态发布器
    load_joint_state_controller = Node(package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'],
        output='screen')

    load_joint_trajectory_controller = Node(package='controller_manager', executable='spawner',
        arguments=['arm_controller', '--controller-manager-timeout', '60'],
        output='screen')

    load_gripper_trajectory_controller = Node(package='controller_manager', executable='spawner',
        arguments=['gripper_controller', '--controller-manager-timeout', '60'],
        output='screen')
    

    close_evt1 =  RegisterEventHandler( 
            event_handler=OnProcessExit(
                target_action=spawn_entity_cmd,
                on_exit=lambda event, context: [] if context.is_shutdown else (
                    [load_joint_state_controller] if event.returncode == 0 else
                    [LogInfo(msg='ERROR: Robot spawn failed; controller startup skipped. '
                              'Close the previous Gazebo instance before restarting.')]),
            )
    )

    close_evt2 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller, 
                         load_gripper_trajectory_controller],
            )
    )


    ld = LaunchDescription()


    # Stop the whole launch (including an included MoveIt launch) if Gazebo
    # exits. In particular, a port conflict must not leave other nodes running
    # against an older Gazebo instance.
    ld.add_action(RegisterEventHandler(OnProcessExit(
        target_action=start_gazebo_cmd,
        on_exit=[
            LogInfo(msg='Gazebo exited; stopping this simulation launch.'),
            EmitEvent(event=Shutdown(
                reason='Gazebo exited; stopping this simulation launch.')),
        ],
    )))
    ld.add_action(close_evt1)
    ld.add_action(close_evt2)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_client_cmd)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(spawn_entity_cmd)

    return list(ld.entities)
