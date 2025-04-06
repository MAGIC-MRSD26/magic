import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context) == "true"

    # Initialize Arguments
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    gripper_max_velocity = LaunchConfiguration("gripper_max_velocity")
    gripper_max_force = LaunchConfiguration("gripper_max_force")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_internal_bus_gripper_comm = LaunchConfiguration("use_internal_bus_gripper_comm")
    use_gui = LaunchConfiguration('use_gui')

    launch_arguments = {
        "robot_ip": robot_ip,
        "use_fake_hardware": use_fake_hardware,
        "gripper": "robotiq_2f_85",
        "dof": "7",
        "gripper_max_velocity": gripper_max_velocity,
        "gripper_max_force": gripper_max_force,
        "use_internal_bus_gripper_comm": use_internal_bus_gripper_comm,
    }

    # Build moveit configuration
    moveit_config = (
        MoveItConfigsBuilder("gen3_dual_arm", package_name="two_arm_moveit2_config")
        .robot_description(mappings=launch_arguments)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "chomp"],
            default_planning_pipeline="ompl"
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
                namespace=namespace,
                parameters=[
                    moveit_config.robot_description,
                    os.path.join(get_package_share_directory("two_arm_moveit2_config"), "config", "ros2_controllers.yaml")
                ],
                remappings=[("/controller_manager/robot_description", f"/{namespace}/robot_description")],
                output="both",
            ),

            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,
                arguments=["joint_state_broadcaster", "--controller-manager", f"/{namespace}/controller_manager"],
                parameters=[{"rate": 30}],
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
                    {"use_sim_time": use_sim_time == "true"},
                ],
            ),
        ]

    nodes_to_start = []
    nodes_to_start += generate_namespaced_arm("left_arm_ns", "left_arm", "left_arm_controller")
    nodes_to_start += generate_namespaced_arm("right_arm_ns", "right_arm", "right_arm_controller")

    nodes_to_start.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_pub",
            output="log",
            arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"]
        )
        .to_moveit_configs()
    )  

    ompl_planning_yaml = os.path.join(
    get_package_share_directory("two_arm_moveit2_config"),
    "config",
    "ompl_planning.yaml"
    )
    if os.path.exists(ompl_planning_yaml):
        with open(ompl_planning_yaml, 'r') as f:
            ompl_config = yaml.safe_load(f)
            if not hasattr(moveit_config, 'planning_pipelines'):
                moveit_config.planning_pipelines = {}
            if 'ompl' not in moveit_config.planning_pipelines:
                moveit_config.planning_pipelines['ompl'] = {}
            moveit_config.planning_pipelines['ompl'].update(ompl_config)

    moveit_config.moveit_cpp.update({"use_sim_time": use_sim_time.perform(context) == "true"})

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"planning_pipeline": "ompl", 
            "default_planner_id": "RRTConnect"} 
        ],
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("two_arm_moveit2_config"), "config", "moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time == "true"}],
        #condition=IfCondition(launch_rviz),
    )

    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=nodes_to_start[2],  # left joint_state_broadcaster
            on_exit=[rviz_node],
        ),
        #condition=IfCondition(launch_rviz),
    )

    nodes_to_start.append(delay_rviz)

    return nodes_to_start


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("robot_ip", description="IP address by which the robot can be reached."),
        DeclareLaunchArgument("use_fake_hardware", default_value="false", description="Start robot with fake hardware mirroring command to its states."),
        DeclareLaunchArgument("gripper_max_velocity", default_value="100.0", description="Max velocity for gripper commands"),
        DeclareLaunchArgument("gripper_max_force", default_value="100.0", description="Max force for gripper commands"),
        DeclareLaunchArgument("use_internal_bus_gripper_comm", default_value="true", description="Use arm's internal gripper connection"),
        DeclareLaunchArgument("use_external_cable", default_value="false", description="Enable external gripper communication"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulated clock"),
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
