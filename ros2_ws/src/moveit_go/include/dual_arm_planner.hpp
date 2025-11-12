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
        const std::string& planning_message = "Planning succeeded!",
        bool holding_object = false);

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

    char waitForKeyPress();

private:
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface& arm_move_group_A_;
    moveit::planning_interface::MoveGroupInterface& arm_move_group_B_;
    moveit::planning_interface::MoveGroupInterface& arm_move_group_dual_;
    
    bool validateGripperDistance(
        const moveit_msgs::msg::RobotTrajectory& traj_left,
        const moveit_msgs::msg::RobotTrajectory& traj_right,
        double expected_distance,
        double tolerance);

    void enforceGripperDistance(
        moveit_msgs::msg::RobotTrajectory& traj_left,
        moveit_msgs::msg::RobotTrajectory& traj_right,
        double target_distance);

    void constrainWaypointPairDistance(
        geometry_msgs::msg::Pose& waypoint1,
        geometry_msgs::msg::Pose& waypoint2,
        double target_distance);

    moveit_msgs::msg::RobotTrajectory resampleTrajectory(
        const moveit_msgs::msg::RobotTrajectory& traj, 
        size_t target_size);

    std::tuple<std::vector<geometry_msgs::msg::Pose>, std::vector<geometry_msgs::msg::Pose>, double> calculateWaypoints(
        geometry_msgs::msg::Pose& pose1,
        geometry_msgs::msg::Pose& pose2,
        double eef_step,
        bool holding_object);
};

#endif // DUAL_ARM_PLANNER_HPP