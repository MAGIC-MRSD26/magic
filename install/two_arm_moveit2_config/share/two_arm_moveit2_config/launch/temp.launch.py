import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_namespaced_moveit_nodes(namespace, planning_group, controller_name):
    moveit_config = (
        MoveItConfigsBuilder(robot_name="gen3_dual_arm", package_name="dual_kinova_moveit_config")
        .robot_description(file_path="config/gen3_dual_arm.urdf.xacro")
        .robot_description_semantic(file_path="config/gen3_dual_arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=namespace,
            output="both",
            parameters=[moveit_config.robot_description],
        ),

        Node(
            package="controller_manager",
            executable="ros2_control_node",
            name="ros2_control_node",
            namespace=namespace,
            parameters=[
                moveit_config.robot_description,
                os.path.join(get_package_share_directory("dual_kinova_moveit_config"), "config", "ros2_controllers.yaml")
            ],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            arguments=["joint_state_broadcaster", "--controller-manager", f"/{namespace}/controller_manager"],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            arguments=[controller_name, "--controller-manager", f"/{namespace}/controller_manager"],
        ),

        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            namespace=namespace,
            parameters=[
                moveit_config.to_dict(),
                {"move_group/planning_group": planning_group},
                {"move_group/default_controller": controller_name},
            ],
        ),
    ]


def generate_launch_description():
    left_arm_ns = "left_arm_ns"
    right_arm_ns = "right_arm_ns"

    left_arm_nodes = generate_namespaced_moveit_nodes(
        namespace=left_arm_ns,
        planning_group="left_arm",
        controller_name="left_arm_controller"
    )

    right_arm_nodes = generate_namespaced_moveit_nodes(
        namespace=right_arm_ns,
        planning_group="right_arm",
        controller_name="right_arm_controller"
    )

    return LaunchDescription(left_arm_nodes + right_arm_nodes)
