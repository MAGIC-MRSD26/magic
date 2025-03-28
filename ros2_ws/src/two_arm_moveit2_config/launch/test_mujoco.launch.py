# import os
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# from launch.event_handlers import OnProcessExit, OnProcessStart
# import xacro

# def generate_launch_description():
#     # Path to mujoco xml file
#     mujoco_model_path = os.path.join(
#         get_package_share_directory("two_arm_moveit2_config"),
#         "mujoco_model",
#         "dual_arm_setup.xml")
    
#     xacro_file = os.path.join(get_package_share_directory("two_arm_moveit2_config"),
#         "urdf",
#         "gen3_dual_arm.xacro")

#     doc = xacro.parse(open(xacro_file))
#     xacro.process_doc(doc)
#     robot_description = {'robot_description': doc.toxml()}

#     controller_config_file = os.path.join(get_package_share_directory("two_arm_moveit2_config"), 'config', 'ros2_controllers.yaml')
    
#     # Set up MuJoCo node
#     mujoco_ros2_control_node = Node(
#         package="mujoco_ros2_control",
#         executable="mujoco_ros2_control",
#         output="screen",
#         parameters=[
#             robot_description,
#             controller_config_file,
#             {"mujoco_model_path": mujoco_model_path},
#         ],
#     )

#     node_robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[robot_description]
#     )

#     # Load controllers based on the ros2_controllers.yaml file
#     load_joint_state_broadcaster = ExecuteProcess(
#         cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
#              'joint_state_broadcaster'],
#         output='screen'
#     )

#     load_left_arm_controller = ExecuteProcess(
#         cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
#              'left_arm_controller'],
#         output='screen'
#     )

#     load_right_arm_controller = ExecuteProcess(
#         cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
#              'right_arm_controller'],
#         output='screen'
#     )

#     load_twist_controller = ExecuteProcess(
#         cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
#              'twist_controller'],
#         output='screen'
#     )

#     load_fault_controller = ExecuteProcess(
#         cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
#              'fault_controller'],
#         output='screen'
#     )

#     load_left_robotiq_gripper_controller = ExecuteProcess(
#         cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
#              'left_robotiq_gripper_controller'],
#         output='screen'
#     )

#     load_right_robotiq_gripper_controller = ExecuteProcess(
#         cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
#              'right_robotiq_gripper_controller'],
#         output='screen'
#     )

#     # Create a chain of controller loading events
#     return LaunchDescription([
#         # Start the MuJoCo node and robot state publisher
#         mujoco_ros2_control_node,
#         node_robot_state_publisher,
        
#         # Set up the controller loading sequence
#         RegisterEventHandler(
#             event_handler=OnProcessStart(
#                 target_action=mujoco_ros2_control_node,
#                 on_start=[load_joint_state_broadcaster],
#             )
#         ),
#         RegisterEventHandler(
#             event_handler=OnProcessExit(
#                 target_action=load_joint_state_broadcaster,
#                 on_exit=[load_left_arm_controller],
#             )
#         ),
#         RegisterEventHandler(
#             event_handler=OnProcessExit(
#                 target_action=load_left_arm_controller,
#                 on_exit=[load_right_arm_controller],
#             )
#         ),
#         RegisterEventHandler(
#             event_handler=OnProcessExit(
#                 target_action=load_right_arm_controller,
#                 on_exit=[load_twist_controller],
#             )
#         ),
#         RegisterEventHandler(
#             event_handler=OnProcessExit(
#                 target_action=load_twist_controller,
#                 on_exit=[load_fault_controller],
#             )
#         ),
#         RegisterEventHandler(
#             event_handler=OnProcessExit(
#                 target_action=load_fault_controller,
#                 on_exit=[load_left_robotiq_gripper_controller],
#             )
#         ),
#         RegisterEventHandler(
#             event_handler=OnProcessExit(
#                 target_action=load_left_robotiq_gripper_controller,
#                 on_exit=[load_right_robotiq_gripper_controller],
#             )
#         ),
#     ])

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Define and declare parameters with explicit types
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Path to mujoco xml file
    mujoco_model_path = os.path.join(
        get_package_share_directory("two_arm_moveit2_config"),
        "mujoco_model",
        "gen3_dual_arm.xml")
    
    # Load URDF file with explicit simulation parameters
    xacro_file = os.path.join(
        get_package_share_directory("two_arm_moveit2_config"),
        "urdf",
        "gen3_dual_arm.xacro")

    # Process the xacro with explicit parameters
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings={
        'sim_mujoco': 'true',
        'sim_gazebo': 'false',
        'sim_ignition': 'false',
        'sim_isaac': 'false',
        'use_fake_hardware': 'false'
    })
    robot_description = {'robot_description': doc.toxml()}
    
    # Set up MuJoCo node with minimal parameters first
    mujoco_ros2_control_node = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        output="screen",
        parameters=[
            robot_description,
            {"mujoco_model_path": mujoco_model_path},
            {"use_sim_time": use_sim_time},
        ],
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    return LaunchDescription([
        mujoco_ros2_control_node,
        node_robot_state_publisher
    ])