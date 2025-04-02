#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  auto node = rclcpp::Node::make_shared("moveit_go_dual", node_options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Move group for both arms
  static const std::string PLANNING_GROUP = "both_arms";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Wait for MoveGroup to be ready
  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(10);

  // Print active joints for debug
  auto active_joints = move_group.getActiveJoints();
  RCLCPP_INFO(node->get_logger(), "Active joints in both_arms:");
  for (const auto& joint : active_joints)
    RCLCPP_INFO(node->get_logger(), "  %s", joint.c_str());

  // Get current end effector names (update based on your SRDF)
  const std::string left_ee_link = "left_end_effector_link";
  const std::string right_ee_link = "right_end_effector_link";

  // Set target poses for both end effectors
  std::map<std::string, geometry_msgs::msg::Pose> target_poses;

  geometry_msgs::msg::Pose left_target;
  left_target.position.x = 0.5;
  left_target.position.y = 0.3;
  left_target.position.z = 0.4;
  left_target.orientation.w = 1.0;

  geometry_msgs::msg::Pose right_target;
  right_target.position.x = 0.5;
  right_target.position.y = -0.3;
  right_target.position.z = 0.4;
  right_target.orientation.w = 1.0;

  target_poses[left_ee_link] = left_target;
  target_poses[right_ee_link] = right_target;

  move_group.setPoseTargets(target_poses);

  // Plan
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success)
  {
    RCLCPP_INFO(node->get_logger(), "Planning successful. Executing...");
    move_group.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Planning failed.");
  }

  rclcpp::shutdown();
  return 0;
}