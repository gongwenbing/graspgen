from glob import glob
from setuptools import setup
setup(name='graspgen_handeye',version='0.1.0',packages=['graspgen_handeye'],
 data_files=[('share/ament_index/resource_index/packages',['resource/graspgen_handeye']),
 ('share/graspgen_handeye',['package.xml','README.md']),('share/graspgen_handeye/launch',glob('launch/*.launch.py')),('share/graspgen_handeye/config',glob('config/*'))],
 install_requires=['setuptools'],zip_safe=True,maintainer='GraspGen',maintainer_email='maintainer@example.com',license='Apache-2.0',
 entry_points={'console_scripts':['preview = graspgen_handeye.preview:main','validate = graspgen_handeye.validate:main','collect = graspgen_handeye.collect:main','solve = graspgen_handeye.solve:main','publish_mount = graspgen_handeye.publish_mount:main']})
