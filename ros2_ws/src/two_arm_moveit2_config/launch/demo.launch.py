from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("gen3_robotiq_2f_85", package_name="two_arm_moveit2_config").to_moveit_configs()
    
    # Standard demo launch
    demo_launch = generate_demo_launch(moveit_config)
    
    # Add a joint state publisher with GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )
    
    # Combine launches
    ld = LaunchDescription()
    
    # Add the demo launch components
    for item in demo_launch.entities:
        ld.add_action(item)
    
    # Add our additional nodes
    ld.add_action(joint_state_publisher_gui_node)
    
    return ld