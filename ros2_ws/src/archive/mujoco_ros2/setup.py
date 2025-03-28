#!/home/jarvis/miniconda3/envs/magic/bin/python

from setuptools import setup
import os
from glob import glob

package_name = 'mujoco_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='MuJoCo environment with ROS 2 integration',
    license='MIT',
    #install_requires=['setuptools', 'moveit_commander', 'moveit_ros_planning_interface'],
    entry_points={
        'console_scripts': [
            'mujoco_node = mujoco_ros2.mujoco_node:main',
            'moveit_pose= mujoco_ros2.moveit_pose:main'
        ],
    },
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    options={'build_scripts': {'executable': '/home/jarvis/miniconda3/envs/magic/bin/python'}}
)

