from glob import glob
from setuptools import setup
setup(name='piper_moveit_bridge', version='0.1.0', packages=['piper_moveit_bridge'],
      data_files=[('share/ament_index/resource_index/packages',['resource/piper_moveit_bridge']),
                  ('share/piper_moveit_bridge',['package.xml','README.md']),
                  ('share/piper_moveit_bridge/config',glob('config/*.yaml')),
                  ('share/piper_moveit_bridge/launch',glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='GraspGen', maintainer_email='maintainer@example.com', license='Apache-2.0',
      description='Gated FollowJointTrajectory bridge for the PiPER CAN arm',
      entry_points={'console_scripts':[
          'trajectory_bridge = piper_moveit_bridge.trajectory_bridge:main',
          'can_gateway = piper_moveit_bridge.can_gateway:main',
          'record_grasp = piper_moveit_bridge.grasp_workflow:record_main',
          'resume_grasp = piper_moveit_bridge.grasp_resume:main',
          'auto_grasp = piper_moveit_bridge.grasp_workflow:auto_main',
          'tcp_calibration = piper_moveit_bridge.tcp_calibration:main',
          'cartesian_step = piper_moveit_bridge.cartesian_step:main',
          'small_step = piper_moveit_bridge.small_step:main',
          'add_table = piper_moveit_bridge.add_table:main',
          'add_camera = piper_moveit_bridge.add_camera:main',
          'plan_pregrasp = piper_moveit_bridge.plan_pregrasp:main',
          'execute_pregrasp = piper_moveit_bridge.execute_pregrasp:main',
          'observation_pose = piper_moveit_bridge.observation_pose:main']})
