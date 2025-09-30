#ifndef DUAL_ARM_PLANNER_HPP
#define DUAL_ARM_PLANNER_HPP

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

enum class State; // Forward declaration

class DualArmPlanner {
public:
    DualArmPlanner(
        const rclcpp::Node::SharedPtr& node,
        moveit::planning_interface::MoveGroupInterface& arm_A,
        moveit::planning_interface::MoveGroupInterface& arm_B,
        moveit::planning_interface::MoveGroupInterface& arm_dual);

    bool plantoTarget_dualarm(
        geometry_msgs::msg::Pose pose1, 
        geometry_msgs::msg::Pose pose2,
        State& current_state,
        State next_state,
        moveit::planning_interface::MoveGroupInterface::Plan& plan,
        const std::string& planning_message = "Planning succeeded!");

    bool executeMovement_dualarm(
        State& current_state,
        State next_state,
        moveit::planning_interface::MoveGroupInterface::Plan& plan,
        const std::string& success_message,
        const std::string& prompt_message = "");

    void rotate(
        double roll, 
        double pitch, 
        double yaw,
        geometry_msgs::msg::Pose& rotated_pose1,
        geometry_msgs::msg::Pose& rotated_pose2);

private:
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface& arm_move_group_A_;
    moveit::planning_interface::MoveGroupInterface& arm_move_group_B_;
    moveit::planning_interface::MoveGroupInterface& arm_move_group_dual_;

    char waitForKeyPress();
};

#endif // DUAL_ARM_PLANNER_HPP