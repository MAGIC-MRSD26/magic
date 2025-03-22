#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char * argv[]){
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "moveit_go_left",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("moveit_go_left");

  // Wait for move_group to be available
  RCLCPP_INFO(logger, "Waiting for move_group capability...");
  rclcpp::sleep_for(std::chrono::seconds(2));

  // Create the MoveIt MoveGroup Interface
  moveit::planning_interface::MoveGroupInterface MoveGroupInterface(node, "left_arm");\

  // Define orientation in quaternion
  tf2::Quaternion tf2_quat;
  tf2_quat.setRPY(0, -3.14, 0);
  geometry_msgs::msg::Quaternion quat_orient;
  tf2::convert(tf2_quat, quat_orient);

  // Set target EE pose
  geometry_msgs::msg::Pose goal;
  goal.orientation = quat_orient;
  goal.position.x = 0.1868;
  goal.position.y = 0;
  goal.position.z = 1.4;
  //   goal.position.z = 1.1716; actual height should be

  MoveGroupInterface.setPoseTarget(goal);

  // Set target joint pose
  // std::vector<double> joint_group_positions = {0.0, 0.0, 0.0, 2.5, 0.0, 1.0, 0.0};  // Home position
  // MoveGroupInterface.setJointValueTarget(joint_group_positions);

  // Generate a plan
  MoveGroupInterface.setPlanningTime(15.0);  // Give the planner more time (15 seconds)
  moveit::planning_interface::MoveGroupInterface::Plan plan1;
  auto const plan_complete = static_cast<bool>(MoveGroupInterface.plan(plan1));
  
  // Execute the plan
  if(plan_complete) {
    RCLCPP_INFO(logger, "Executing successful pose target plan");
    MoveGroupInterface.execute(plan1);
    RCLCPP_INFO(logger, "Pose target plan executed successfully");
  } else {
    RCLCPP_ERROR(logger, "All pose planning attempts failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}