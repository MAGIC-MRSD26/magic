#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char * argv[]){
  // Initialize ROS
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "moveit_cartesian_with_feedback",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("moveit_cartesian_with_feedback");

  // Wait for move_group to be available
  RCLCPP_INFO(logger, "Waiting for move_group...");
  rclcpp::sleep_for(std::chrono::seconds(2));

  // Start executor for ROS callbacks (needed for visual tools)
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Setup MoveGroupInterface for 'left_arm'
  moveit::planning_interface::MoveGroupInterface MoveGroupInterface(node, "left_arm");

  // Setup MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "left_base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    MoveGroupInterface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Draw helper closures
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto pose = Eigen::Isometry3d::Identity();
    pose.translation().z() = 1.0;
    moveit_visual_tools.publishText(pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };
  auto const draw_trajectory_tool_path =
    [&moveit_visual_tools, jmg = MoveGroupInterface.getRobotModel()->getJointModelGroup("left_arm")]
    (auto const trajectory) {
      moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
    };

  // ✅ Real-Time End-Effector Feedback (logs every 0.5s)
  rclcpp::TimerBase::SharedPtr timer = node->create_wall_timer(
    std::chrono::milliseconds(500),
    [&MoveGroupInterface, &logger]() {
      auto pose = MoveGroupInterface.getCurrentPose().pose;
      RCLCPP_INFO_STREAM_THROTTLE(
        rclcpp::get_clock(), 2000,
        logger,
        "EE Pose -> x: " << pose.position.x
                         << ", y: " << pose.position.y
                         << ", z: " << pose.position.z);
    }
  );

  // ✅ Get current pose
  geometry_msgs::msg::Pose start_pose = MoveGroupInterface.getCurrentPose().pose;
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(start_pose);

  // Waypoint 1 - move forward
  geometry_msgs::msg::Pose wp1 = start_pose;
  wp1.position.x += 0.1;
  waypoints.push_back(wp1);

  // Waypoint 2 - move up
  geometry_msgs::msg::Pose wp2 = wp1;
  wp2.position.z += 0.1;
  waypoints.push_back(wp2);

  // Waypoint 3 - move left
  geometry_msgs::msg::Pose wp3 = wp2;
  wp3.position.y += 0.1;
  waypoints.push_back(wp3);

  // Plan Cartesian path
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  draw_title("Planning Cartesian Path");
  prompt("Press 'Next' in RViz to plan");

  double fraction = MoveGroupInterface.computeCartesianPath(
    waypoints, eef_step, jump_threshold, trajectory);

  RCLCPP_INFO(logger, "Cartesian path %.2f%% planned", fraction * 100.0);

  // Execute if successful
  if (fraction > 0.9) {
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory;

    draw_trajectory_tool_path(trajectory);
    moveit_visual_tools.trigger();
    prompt("Press 'Next' in RViz to execute");

    draw_title("Executing Cartesian Path");
    moveit_visual_tools.trigger();

    MoveGroupInterface.execute(cartesian_plan);
    RCLCPP_INFO(logger, "Execution completed");
  } else {
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Cartesian path planning failed (only %.2f%%)", fraction * 100.0);
  }

  // Cleanup
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
