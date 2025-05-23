from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    sim_gazebo = LaunchConfiguration("sim_gazebo")
    sim_ignition = LaunchConfiguration("sim_ignition")
    robot_type = LaunchConfiguration("robot_type")
    dof = LaunchConfiguration("dof")
    vision = LaunchConfiguration("vision")
    # General arguments
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    robot_name = LaunchConfiguration("robot_name")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gripper = LaunchConfiguration("gripper")

    robot_controllers = PathJoinSubstitution(
        # https://answers.ros.org/question/397123/how-to-access-the-runtime-value-of-a-launchconfiguration-instance-within-custom-launch-code-injected-via-an-opaquefunction-in-ros2/
        [
            FindPackageShare(description_package),
            "config",
            controllers_file,
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "moveit.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "name:=",
            robot_name,
            " ",
            "arm:=",
            robot_type,
            " ",
            "dof:=",
            dof,
            " ",
            "vision:=",
            vision,
            " ",
            "sim_gazebo:=",
            sim_gazebo,
            " ",
            "sim_ignition:=",
            sim_ignition,
            " ",
            "simulation_controllers:=",
            robot_controllers,
            " ",
            "gripper:=",
            gripper,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content.perform(context)}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
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

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
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

    # robot_both_traj_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["both_arms_controller", "-c", "/controller_manager"],
    # )

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


    # Bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    # Gazebo nodes
    gzserver = ExecuteProcess(
        cmd=["gzserver", "-s", "libgazebo_ros_init.so", "-s", "libgazebo_ros_factory.so", ""],
        output="screen",
        condition=IfCondition(sim_gazebo),
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
        condition=IfCondition(sim_gazebo),
    )

    # gazebo = IncludeLaunchDescription(
    # PythonLaunchDescriptionSource(
    # [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
    # ),
    # launch_arguments={"verbose": "false"}.items(),
    # )

    # Spawn robot
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_robot",
        arguments=["-entity", robot_name, "-topic", "robot_description"],
        output="screen",
        condition=IfCondition(sim_gazebo),
    )

    ignition_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            robot_name,
            "-allow_renaming",
            "true",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.3",
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            "0.0",
        ],
        condition=IfCondition(sim_ignition),
    )

    ignition_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"ign_args": " -r -v 3 empty.sdf"}.items(),
        condition=IfCondition(sim_ignition),
    )

    # Bridge
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "/wrist_mounted_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/wrist_mounted_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/wrist_mounted_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            "/wrist_mounted_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        ],
        output="screen",
    )

    # Object to manipulate
    bin_model_path = PathJoinSubstitution(
        [FindPackageShare(description_package), "urdf", "bin.sdf"])

    gazebo_spawn_object = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_bin",
        output="screen",
        arguments=[
            "-file", bin_model_path,
            "-name", "bin",
            "-x", "-0.1905",
            "-y", "0.15875",
            "-z", "1.33",
            "-R", "1.5708",
            "-P", "0.0",
            "-Y", "0.0",
        ],
        condition=IfCondition(sim_ignition),
    )

    nodes_to_start = [
        bridge,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        robot_left_traj_controller_spawner,
        robot_right_traj_controller_spawner,
        # robot_both_traj_controller_spawner,
        robot_pos_controller_spawner,
        robot_left_hand_controller_spawner,
        robot_right_hand_controller_spawner,
        gzserver,
        gzclient,
        gazebo_spawn_robot,
        gazebo_spawn_object,
        ignition_launch_description,
        ignition_spawn_entity,
        gazebo_bridge,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # Simulation specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="true",
            description="Use Ignition for simulation",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo",
            default_value="false",
            description="Use Gazebo Classic for simulation",
        )
    )
    # Robot specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            description="Type/series of robot.",
            choices=["gen3", "gen3_lite"],
            default_value="gen3",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "dof",
            description="DoF of robot.",
            choices=["6", "7"],
            default_value="7",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "vision",
            description="Use arm mounted realsense",
            choices=["true", "false"],
            default_value="false",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ros2_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="two_arm_moveit2_config",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="gen3_dual_arm.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="gen3",
            description="Robot name.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulated clock",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper",
            default_value="robotiq_2f_85",
            choices=["robotiq_2f_85", "robotiq_2f_140", "gen3_lite_2f", ""],
            description="Gripper to use",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="false", description="Launch RViz?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
