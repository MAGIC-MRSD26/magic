#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_base/kinematics_base.h>

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "moveit_go_dual",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("moveit_go_dual");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // MoveIt interfaces
  moveit::planning_interface::MoveGroupInterface dual_group(node, "both_arms");
  auto robot_model = dual_group.getRobotModel();

  // Visual tools
  moveit_visual_tools::MoveItVisualTools visual_tools{
    node, "base_link_left", rviz_visual_tools::RVIZ_MARKER_TOPIC, robot_model
  };
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  auto const draw_title = [&visual_tools](const std::string& text) {
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  };

  auto const prompt = [&visual_tools](const std::string& text) {
    visual_tools.prompt(text);
  };

  // Define target poses for both end effectors
  tf2::Quaternion tf2_quat;
  tf2_quat.setRPY(0, -3.14, 0);
  geometry_msgs::msg::Quaternion quat_orient;
  tf2::convert(tf2_quat, quat_orient);

  geometry_msgs::msg::Pose pose_left, pose_right;
  pose_left.orientation = quat_orient;
  pose_left.position.x = 0.1868;
  pose_left.position.y = 0;
  pose_left.position.z = 1.4;

  pose_right.orientation = quat_orient;
  pose_right.position.x = -0.1868;
  pose_right.position.y = 0;
  pose_right.position.z = 1.4;

  // Solve IK inline using RobotState
  auto robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
  robot_state->setToDefaultValues();

  std::vector<double> left_joints, right_joints;

  bool found_left = robot_state->setFromIK(
    robot_model->getJointModelGroup("left_arm"),
    pose_left,
    "tool_frame_left",
    0.1,  // timeout
    moveit::core::GroupStateValidityCallbackFn(),  // no constraints
    kinematics::KinematicsQueryOptions()
  );

  if (!found_left) {
    RCLCPP_ERROR(logger, "Failed to solve IK for left arm");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }
  robot_state->copyJointGroupPositions("left_arm", left_joints);

  bool found_right = robot_state->setFromIK(
    robot_model->getJointModelGroup("right_arm"),
    pose_right,
    "tool_frame_right",
    0.1,  // timeout
    moveit::core::GroupStateValidityCallbackFn(),
    kinematics::KinematicsQueryOptions()
  );

  if (!found_right) {
    RCLCPP_ERROR(logger, "Failed to solve IK for right arm");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }
  robot_state->copyJointGroupPositions("right_arm", right_joints);

  // Combine both joint sets into one target
  std::vector<double> full_joint_target;
  full_joint_target.insert(full_joint_target.end(), left_joints.begin(), left_joints.end());
  full_joint_target.insert(full_joint_target.end(), right_joints.begin(), right_joints.end());

  dual_group.setPlanningTime(15.0);
  dual_group.setJointValueTarget(full_joint_target);

  // Plan
  moveit::planning_interface::MoveGroupInterface::Plan dual_plan;

  prompt("Press 'Next' in RViz to plan");
  draw_title("Planning");
  visual_tools.trigger();

  bool success = static_cast<bool>(dual_group.plan(dual_plan));

  if (success) {
    visual_tools.publishTrajectoryLine(
      dual_plan.trajectory_,
      robot_model->getJointModelGroup("dual_arms")
    );
    draw_title("Executing");
    visual_tools.trigger();

    prompt("Press 'Next' in RViz to execute");
    dual_group.execute(dual_plan);
    RCLCPP_INFO(logger, "Dual-arm motion executed successfully.");
  } else {
    RCLCPP_ERROR(logger, "Planning failed for dual_arms group.");
    draw_title("Planning Failed!");
    visual_tools.trigger();
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
