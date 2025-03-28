#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "moveit_go_dual",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto const logger = rclcpp::get_logger("moveit_go_dual");

  // Executor and spinner
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // MoveGroup interfaces for both arms
  moveit::planning_interface::MoveGroupInterface left_arm(node, "left_arm");
  moveit::planning_interface::MoveGroupInterface right_arm(node, "right_arm");

  // MoveItVisualTools for both arms (assuming same base frame and topic)
  moveit_visual_tools::MoveItVisualTools visual_tools(
    node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    left_arm.getRobotModel());
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  auto draw_title = [&visual_tools](const std::string &text) {
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  };

  auto prompt = [&visual_tools](const std::string &text) {
    visual_tools.prompt(text);
  };

  auto draw_trajectory = [&visual_tools](
    const moveit_msgs::msg::RobotTrajectory &trajectory,
    const moveit::core::JointModelGroup *jmg) {
    visual_tools.publishTrajectoryLine(trajectory, jmg);
  };

  // Define goals for both arms
  tf2::Quaternion tf2_quat;
  tf2_quat.setRPY(0, -3.14, 0); // downward pointing orientation
  geometry_msgs::msg::Quaternion quat_orient;
  tf2::convert(tf2_quat, quat_orient);

  geometry_msgs::msg::Pose left_goal;
  left_goal.orientation = quat_orient;
  left_goal.position.x = 0.1868;
  left_goal.position.y = 0;
  left_goal.position.z = 1.4;

  geometry_msgs::msg::Pose right_goal;
  right_goal.orientation = quat_orient;
  right_goal.position.x = -0.1868;
  right_goal.position.y = 0;
  right_goal.position.z = 1.4;

  left_arm.setPoseTarget(left_goal);
  right_arm.setPoseTarget(right_goal);

  left_arm.setPlanningTime(15.0);
  right_arm.setPlanningTime(15.0);

  draw_title("Planning both arms");
  prompt("Press 'Next' in RViz to start planning");
  visual_tools.trigger();

  moveit::planning_interface::MoveGroupInterface::Plan left_plan;
  moveit::planning_interface::MoveGroupInterface::Plan right_plan;

  bool left_success = static_cast<bool>(left_arm.plan(left_plan));
  bool right_success = static_cast<bool>(right_arm.plan(right_plan));

  if (left_success && right_success) {
    draw_trajectory(left_plan.trajectory_, left_arm.getRobotModel()->getJointModelGroup("left_arm"));
    draw_trajectory(right_plan.trajectory_, right_arm.getRobotModel()->getJointModelGroup("right_arm"));
    draw_title("Executing both plans");
    prompt("Press 'Next' to execute both");
    visual_tools.trigger();

    std::thread left_thread([&]() {
      left_arm.execute(left_plan);
    });

    std::thread right_thread([&]() {
      right_arm.execute(right_plan);
    });

    left_thread.join();
    right_thread.join();

    RCLCPP_INFO(logger, "Both trajectories executed in parallel.");
  } else {
    draw_title("Planning Failed");
    visual_tools.trigger();
    if (!left_success)
      RCLCPP_ERROR(logger, "Left arm planning failed");
    if (!right_success)
      RCLCPP_ERROR(logger, "Right arm planning failed");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
