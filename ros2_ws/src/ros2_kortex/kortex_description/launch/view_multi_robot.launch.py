# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    declared_arguments = []
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
            "gripper",
            default_value='robotiq_2f_85',
            description="Name of the gripper attached to the arm",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "dof",
            default_value="7",
            description="Robot's dof",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_prefix",
            default_value="left_",
            description="Prefix for the left arm",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_prefix",
            default_value="right_",
            description="Prefix for the right arm",
        )
    )

    robot_type = LaunchConfiguration("robot_type")
    gripper = LaunchConfiguration("gripper")
    dof = LaunchConfiguration("dof")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("kortex_description"), "multiple_robots", "kortex_dual_robots.xacro"]
            ),
            " ",
            "left_arm:=", robot_type,
            " ",
            "left_gripper:=", gripper,
            " ",
            "left_dof:=", dof,
            " ",
            "left_vision:=false",
            " ",
            "left_prefix:=left_",
            " ",
            "right_arm:=", robot_type,
            " ",
            "right_gripper:=", gripper,
            " ",
            "right_dof:=", dof,
            " ",
            "right_vision:=false",
            " ",
            "right_prefix:=right_",
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("kortex_description"), "rviz", "view_robot.rviz"]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    nodes_to_start = [
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
