from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='moveit_go',  
            executable='moveit_go_left',
            name='moveit_go_left',
            output='log'
        ),
        Node(
            package='moveit_go',  
            executable='moveit_go_right',
            name='moveit_go_right',
            output='log'
        )
    ])
