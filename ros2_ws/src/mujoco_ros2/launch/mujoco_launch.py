#!/home/jarvis/miniconda3/envs/magic/bin/python

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mujoco_ros2',
            executable='mujoco_node',
            name='mujoco_sim',
            output='screen'
        )
    ])

