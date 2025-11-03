import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulated clock",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "vision",
            default_value="false",
            description="Add vision module to URDF",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument("start_home", default_value="true", description="Start robot in home position?")
    )

    # Initialize Arguments
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    vision = LaunchConfiguration("vision")
    start_home = LaunchConfiguration("start_home")

    # Use fake hardware instead of Gazebo
    description_arguments = {
        "robot_ip": "xxx.yyy.zzz.www",
        "use_fake_hardware": "true",  # Use fake hardware
        "gripper": "robotiq_2f_85",
        "dof": "7",
        "sim_ignition": "false",  # No Gazebo
        "vision": vision,
    }

    moveit_config = (
        MoveItConfigsBuilder("gen3_dual_arm", package_name="two_arm_moveit2_config")
        .robot_description(mappings=description_arguments)
        .robot_description_semantic(file_path="config/gen3_dual_arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time}
        ],
    )

    # ros2_control node with fake hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("two_arm_moveit2_config"),
        "config",
        "ros2_controllers.yaml",
    )
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="log",
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Arm Controllers
    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "--controller-manager", "/controller_manager"],
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "--controller-manager", "/controller_manager"],
    )

    # Gripper Controllers
    left_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_robotiq_gripper_controller", "--controller-manager", "/controller_manager"],
    )

    right_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_robotiq_gripper_controller", "--controller-manager", "/controller_manager"],
    )

    # Move Group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="log",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time}
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            "fatal",
        ],
    )

    # RViz
    rviz_config_path = os.path.join(
        get_package_share_directory("two_arm_moveit2_config"),
        "config",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(launch_rviz),
    )

    # Home position publisher - publishes the initial joint states to set robot to home pose
    home_position_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="home_position_publisher",
        output="log",
        parameters=[{
            'robot_description': moveit_config.robot_description['robot_description'],
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(start_home),
        remappings=[('/joint_states', '/home_joint_states')]
    )

    # Delayed home position command - sends robot to home after controllers are loaded
    home_command_publisher = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--controller-manager", "/controller_manager",
            "--timeout", "60",
            "--controller-manager-timeout", "60"
        ],
        condition=IfCondition(start_home),
    )

    # Joint State publisher to initialize home pose
    joint_state_home_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_home_publisher',
        parameters=[{
            'source_list': ['/home_joint_states'],
            'rate': 50,
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(start_home),
    )

    # Timer to delay move group startup until controllers are ready
    delayed_move_group = TimerAction(
        period=3.0,
        actions=[move_group_node]
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            left_arm_controller_spawner,
            right_arm_controller_spawner, 
            left_gripper_controller_spawner,
            right_gripper_controller_spawner,
            delayed_move_group,
            rviz_node,
        ]
    )