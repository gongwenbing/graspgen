from glob import glob
from setuptools import setup
setup(name='graspgen_camera',version='0.1.0',packages=['graspgen_camera'],
      data_files=[('share/ament_index/resource_index/packages',['resource/graspgen_camera']),
                  ('share/graspgen_camera',['package.xml','README.md']),
                  ('share/graspgen_camera/launch',glob('launch/*.launch.py'))],
      install_requires=['setuptools'],zip_safe=True,
      maintainer='GraspGen',maintainer_email='maintainer@example.com',license='Apache-2.0',
      entry_points={'console_scripts':['check_rgbd = graspgen_camera.check_rgbd:main']})
