import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    # Build moveit configuration
    moveit_config = (
        MoveItConfigsBuilder("gen3_dual_arm", package_name="two_arm_moveit2_config")
        .robot_description(
            mappings={
                "sim_mujoco": "true"
            },
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl"
        )
        .to_moveit_configs()
    )

    # Start move group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": False}]
    )

    # rviz with moveit configuration
    rviz_config_file = (
        get_package_share_directory("two_arm_moveit2_config")
        + "/config/moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": False}
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "left_base_link"],
        parameters=[{"use_sim_time": False}]
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, {"use_sim_time": False}],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("two_arm_moveit2_config"),
        "config",
        "ros2_controllers.yaml",
    )

    # Path to mujoco xml file
    mujoco_model_path = os.path.join(
        get_package_share_directory("two_arm_moveit2_config"),
        "mujoco_model",
        "gen3_dual_arm.xml")

    # Set up MuJoCo node
    node_mujoco_ros2_control = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
            {'mujoco_model_path': mujoco_model_path},
            {"use_sim_time": False}
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robot_left_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "-c", "/controller_manager",],
    )

    robot_right_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "-c", "/controller_manager"],
    )

    robot_pos_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["twist_controller", "--inactive", "-c", "/controller_manager"],
    )

    robot_left_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_robotiq_gripper_controller", "-c", "/controller_manager"],
    )

    robot_right_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_robotiq_gripper_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription(
        [
            # RegisterEventHandler(
            #     event_handler=OnProcessStart(
            #         target_action=node_mujoco_ros2_control,
            #         on_start=[joint_state_broadcaster_spawner],
            #     )
            # ),
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=joint_state_broadcaster_spawner,
            #         on_exit=[robot_left_traj_controller_spawner],
            #     )
            # ),
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=robot_left_traj_controller_spawner,
            #         on_exit=[robot_right_traj_controller_spawner],
            #     )
            # ),
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=robot_right_traj_controller_spawner,
            #         on_exit=[robot_left_hand_controller_spawner],
            #     )
            # ),
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=robot_left_hand_controller_spawner,
            #         on_exit=[robot_right_hand_controller_spawner],
            #     )
            # ),
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=robot_right_hand_controller_spawner,
            #         on_exit=[robot_pos_controller_spawner],
            #     )
            # ),
            node_mujoco_ros2_control,
            joint_state_broadcaster_spawner,
            robot_left_traj_controller_spawner,
            robot_right_traj_controller_spawner,
            robot_left_hand_controller_spawner,
            robot_right_hand_controller_spawner,
            robot_pos_controller_spawner,
            rviz_node,
            static_tf,
            robot_state_publisher,
            move_group_node,
        ]
    )