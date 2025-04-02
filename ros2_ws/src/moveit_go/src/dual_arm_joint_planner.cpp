// ik_printer_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ik_printer_node");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  rclcpp::sleep_for(std::chrono::seconds(2)); // Wait for MoveIt to initialize

  // Load Robot Model
  robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
  moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();

  const moveit::core::JointModelGroup* left_arm_group = kinematic_model->getJointModelGroup("arm");
  // const moveit::core::JointModelGroup* right_arm_group = kinematic_model->getJointModelGroup("right_arm");

  // Define goal poses
  geometry_msgs::msg::Pose left_target_pose;
  left_target_pose.orientation.w = 1.0;
  left_target_pose.position.x = 0.1868;
  left_target_pose.position.y = 0.0;
  left_target_pose.position.z = 1.4;

  // geometry_msgs::msg::Pose right_target_pose;
  // right_target_pose.orientation.w = 1.0;
  // right_target_pose.position.x = -0.1868;
  // right_target_pose.position.y = -0.0;
  // right_target_pose.position.z = 1.4;

  // Perform IK for left arm
  bool success_left = kinematic_state->setFromIK(
    left_arm_group, left_target_pose, 0.1,  // timeout
    moveit::core::GroupStateValidityCallbackFn());
  if (success_left)
  {
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(left_arm_group, joint_values);
    RCLCPP_INFO(node->get_logger(), "Left arm IK solution:");
    for (size_t i = 0; i < joint_values.size(); ++i)
    {
      RCLCPP_INFO(node->get_logger(), "  Joint %lu: %f", i + 1, joint_values[i]);
    }
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to find IK solution for left arm.");
  }

  // Perform IK for right arm
  // bool success_right = kinematic_state->setFromIK(
  //   right_arm_group, right_target_pose, 0.1,
  //   moveit::core::GroupStateValidityCallbackFn());
  // if (success_right)
  // {
  //   std::vector<double> joint_values;
  //   kinematic_state->copyJointGroupPositions(right_arm_group, joint_values);
  //   RCLCPP_INFO(node->get_logger(), "Right arm IK solution:");
  //   for (size_t i = 0; i < joint_values.size(); ++i)
  //   {
  //     RCLCPP_INFO(node->get_logger(), "  Joint %lu: %f", i + 1, joint_values[i]);
  //   }
  // }
  // else
  // {
  //   RCLCPP_ERROR(node->get_logger(), "Failed to find IK solution for right arm.");
  // }

  rclcpp::shutdown();
  return 0;
}
