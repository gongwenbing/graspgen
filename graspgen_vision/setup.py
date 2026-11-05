from glob import glob
from setuptools import setup
setup(name='graspgen_vision',version='0.1.0',packages=['graspgen_vision'],
 data_files=[('share/ament_index/resource_index/packages',['resource/graspgen_vision']),
 ('share/graspgen_vision',['package.xml','README.md','requirements.txt']),
 ('share/graspgen_vision/launch',glob('launch/*.launch.py')),
 ('share/graspgen_vision/config',glob('config/*'))],install_requires=['setuptools'],
 zip_safe=True,maintainer='GraspGen',maintainer_email='maintainer@example.com',license='Apache-2.0',
 entry_points={'console_scripts':['segment_grasp = graspgen_vision.node:main','generate_grasps = graspgen_vision.grasp_generator_node:main']})
