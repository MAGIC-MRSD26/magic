#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

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

  // Spin up a SingleThreadedExecutor for MoveItVisualTools to interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  moveit::planning_interface::MoveGroupInterface MoveGroupInterface(node, "left_arm");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "left_base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    MoveGroupInterface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create closures for visualization
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;  // Place text 1m above the base link
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                    rviz_visual_tools::XLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools,
      jmg = MoveGroupInterface.getRobotModel()->getJointModelGroup(
          "left_arm")](auto const trajectory) {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
      };

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
  // std::vector<double> joint_group_positions = {0.0, 0.0, 0.0, 2.5, 0.0, 1.0, 0.0};
  // MoveGroupInterface.setJointValueTarget(joint_group_positions);

  // Generate a plan
  MoveGroupInterface.setPlanningTime(15.0);  // Give the planner more time (15 seconds)
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning");
  moveit_visual_tools.trigger();
  moveit::planning_interface::MoveGroupInterface::Plan plan1;
  auto const plan_complete = static_cast<bool>(MoveGroupInterface.plan(plan1));
  
  // Execute the plan
  if(plan_complete) {
    RCLCPP_INFO(logger, "Executing successful pose target plan");
    draw_trajectory_tool_path(plan1.trajectory_);
    moveit_visual_tools.trigger();
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    draw_title("Executing");
    moveit_visual_tools.trigger();
    MoveGroupInterface.execute(plan1);
    RCLCPP_INFO(logger, "Pose target plan executed successfully");
  } else {
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "All pose planning attempts failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}