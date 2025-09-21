#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematic_constraints/utils.h>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <cmath>

class EnhancedMoveItController {
private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Logger logger_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  robot_model_loader::RobotModelLoader robot_model_loader_;
  moveit::core::RobotModelPtr kinematic_model_;
  const moveit::core::JointModelGroup* left_arm_group_;
  moveit::core::RobotStatePtr kinematic_state_;
  std::unique_ptr<moveit_visual_tools::MoveItVisualTools> moveit_visual_tools_;
  
  // Joint locking state
  std::vector<bool> locked_joints_;
  std::vector<double> locked_joint_values_;

public:
  EnhancedMoveItController(std::shared_ptr<rclcpp::Node> node) 
    : node_(node), 
      logger_(rclcpp::get_logger("enhanced_moveit_controller")),
      robot_model_loader_(node, "robot_description") {
    
    // Initialize MoveIt components
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node, "left_arm");
    kinematic_model_ = robot_model_loader_.getModel();
    left_arm_group_ = kinematic_model_->getJointModelGroup("left_arm");
    kinematic_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
    
    // Initialize visual tools
    moveit_visual_tools_ = std::make_unique<moveit_visual_tools::MoveItVisualTools>(
      node, "left_base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_->getRobotModel());
    moveit_visual_tools_->deleteAllMarkers();
    moveit_visual_tools_->loadRemoteControl();
    
    // Initialize joint locking vectors
    size_t num_joints = left_arm_group_->getVariableCount();
    locked_joints_.resize(num_joints, false);
    locked_joint_values_.resize(num_joints, 0.0);
    
    RCLCPP_INFO(logger_, "Enhanced MoveIt Controller initialized with %lu joints", num_joints);
    
    // Print joint names for reference
    const std::vector<std::string>& joint_names = left_arm_group_->getVariableNames();
    RCLCPP_INFO(logger_, "Joint names:");
    for (size_t i = 0; i < joint_names.size(); ++i) {
      RCLCPP_INFO(logger_, "  [%lu]: %s", i, joint_names[i].c_str());
    }
  }

  // Function to lock specific joints at their current values
  void lockJoints(const std::vector<int>& joint_indices) {
    std::vector<double> current_joint_values;
    move_group_->getCurrentState()->copyJointGroupPositions(left_arm_group_, current_joint_values);
    
    for (int idx : joint_indices) {
      if (idx >= 0 && idx < static_cast<int>(locked_joints_.size())) {
        locked_joints_[idx] = true;
        locked_joint_values_[idx] = current_joint_values[idx];
        RCLCPP_INFO(logger_, "Locked joint %d at value: %f", idx, current_joint_values[idx]);
      }
    }
  }

  // Function to unlock specific joints
  void unlockJoints(const std::vector<int>& joint_indices) {
    for (int idx : joint_indices) {
      if (idx >= 0 && idx < static_cast<int>(locked_joints_.size())) {
        locked_joints_[idx] = false;
        RCLCPP_INFO(logger_, "Unlocked joint %d", idx);
      }
    }
  }

  // Function to unlock all joints
  void unlockAllJoints() {
    std::fill(locked_joints_.begin(), locked_joints_.end(), false);
    RCLCPP_INFO(logger_, "All joints unlocked");
  }

  // Function to apply joint constraints for locked joints
  void applyJointConstraints() {
    move_group_->clearPathConstraints();
    
    const std::vector<std::string>& joint_names = left_arm_group_->getVariableNames();
    
    for (size_t i = 0; i < locked_joints_.size(); ++i) {
      if (locked_joints_[i]) {
        moveit_msgs::msg::JointConstraint joint_constraint;
        joint_constraint.joint_name = joint_names[i];
        joint_constraint.position = locked_joint_values_[i];
        joint_constraint.tolerance_above = 0.01;  // Small tolerance
        joint_constraint.tolerance_below = 0.01;
        joint_constraint.weight = 1.0;
        
        moveit_msgs::msg::Constraints constraints;
        constraints.joint_constraints.push_back(joint_constraint);
        move_group_->setPathConstraints(constraints);
        
        RCLCPP_INFO(logger_, "Applied constraint to joint %s at position %f", 
                   joint_names[i].c_str(), locked_joint_values_[i]);
      }
    }
  }

  // Function to rotate wrist joint (assuming last joint is wrist)
  bool rotateWrist(double angle_radians, bool execute_immediately = true) {
    const std::vector<std::string>& joint_names = left_arm_group_->getVariableNames();
    size_t wrist_joint_idx = joint_names.size() - 1;  // Assume last joint is wrist
    
    RCLCPP_INFO(logger_, "Rotating wrist joint (%s) by %f radians", 
               joint_names[wrist_joint_idx].c_str(), angle_radians);
    
    // Get current joint values
    std::vector<double> joint_group_positions;
    move_group_->getCurrentState()->copyJointGroupPositions(left_arm_group_, joint_group_positions);
    
    // Modify only the wrist joint
    joint_group_positions[wrist_joint_idx] += angle_radians;
    
    // Set the target
    move_group_->setJointValueTarget(joint_group_positions);
    
    // Apply constraints for locked joints
    applyJointConstraints();
    
    // Plan
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_->plan(plan));
    
    if (success) {
      RCLCPP_INFO(logger_, "Wrist rotation plan successful");
      if (execute_immediately) {
        move_group_->execute(plan);
        RCLCPP_INFO(logger_, "Wrist rotation executed");
      }
      return true;
    } else {
      RCLCPP_ERROR(logger_, "Wrist rotation planning failed");
      return false;
    }
  }

  // Function to continuously rotate wrist
  void continuousWristRotation(double angular_velocity_rad_per_sec, double duration_sec, 
                              double step_time_sec = 0.1) {
    double total_time = 0.0;
    double angle_per_step = angular_velocity_rad_per_sec * step_time_sec;
    
    RCLCPP_INFO(logger_, "Starting continuous wrist rotation for %f seconds", duration_sec);
    
    while (total_time < duration_sec) {
      if (!rotateWrist(angle_per_step, true)) {
        RCLCPP_WARN(logger_, "Wrist rotation step failed, stopping continuous rotation");
        break;
      }
      
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(step_time_sec)));
      total_time += step_time_sec;
    }
    
    RCLCPP_INFO(logger_, "Continuous wrist rotation completed");
  }

  // Enhanced planning function with joint locking support
  bool planToPoseWithLockedJoints(const geometry_msgs::msg::Pose& target_pose) {
    RCLCPP_INFO(logger_, "Planning to pose with locked joints");
    
    // Apply joint constraints
    applyJointConstraints();
    
    // Set pose target
    move_group_->setPoseTarget(target_pose);
    move_group_->setPlanningTime(15.0);
    
    // Plan
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_->plan(plan));
    
    if (success) {
      RCLCPP_INFO(logger_, "Planning successful with locked joints");
      
      // Visualize trajectory
      moveit_visual_tools_->publishTrajectoryLine(plan.trajectory_, left_arm_group_);
      moveit_visual_tools_->trigger();
      
      // Execute
      moveit_visual_tools_->prompt("Press 'Next' to execute the plan");
      move_group_->execute(plan);
      RCLCPP_INFO(logger_, "Plan executed successfully");
      
      return true;
    } else {
      RCLCPP_ERROR(logger_, "Planning failed with locked joints");
      return false;
    }
  }

  // Utility function to print current joint values
  void printCurrentJointValues() {
    std::vector<double> joint_values;
    move_group_->getCurrentState()->copyJointGroupPositions(left_arm_group_, joint_values);
    const std::vector<std::string>& joint_names = left_arm_group_->getVariableNames();
    
    RCLCPP_INFO(logger_, "Current joint values:");
    for (size_t i = 0; i < joint_values.size(); ++i) {
      std::string status = locked_joints_[i] ? " (LOCKED)" : "";
      RCLCPP_INFO(logger_, "  %s: %f%s", joint_names[i].c_str(), joint_values[i], status.c_str());
    }
  }

  // Get move group interface for direct access
  moveit::planning_interface::MoveGroupInterface* getMoveGroup() {
    return move_group_.get();
  }

  // Get visual tools for direct access
  moveit_visual_tools::MoveItVisualTools* getVisualTools() {
    return moveit_visual_tools_.get();
  }
};

int main(int argc, char * argv[]){
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "enhanced_moveit_controller",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("enhanced_moveit_controller");

  // Wait for move_group to be available
  RCLCPP_INFO(logger, "Waiting for move_group capability...");
  rclcpp::sleep_for(std::chrono::seconds(2));

  // Spin up a SingleThreadedExecutor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create enhanced controller
  EnhancedMoveItController controller(node);

  // Define target pose
  tf2::Quaternion tf2_quat;
  tf2_quat.setRPY(0, -3.14, 0);
  geometry_msgs::msg::Quaternion quat_orient;
  tf2::convert(tf2_quat, quat_orient);

  geometry_msgs::msg::Pose goal;
  goal.orientation = quat_orient;
  goal.position.x = 0.1868;
  goal.position.y = 0;
  goal.position.z = 1.4;

  // Print current joint values
  controller.printCurrentJointValues();

  // Example usage:
  
  // 1. Lock first two joints and plan to pose
  RCLCPP_INFO(logger, "\n=== Demo 1: Locking first two joints ===");
  controller.lockJoints({0, 1});  // Lock joints 0 and 1
  controller.planToPoseWithLockedJoints(goal);
  
  // 2. Unlock all joints and rotate wrist
  RCLCPP_INFO(logger, "\n=== Demo 2: Wrist rotation ===");
  controller.unlockAllJoints();
  
  // Rotate wrist by 90 degrees (π/2 radians)
  controller.rotateWrist(M_PI/2);
  
  // Wait a moment
  rclcpp::sleep_for(std::chrono::seconds(2));
  
  // Rotate wrist back by -90 degrees
  controller.rotateWrist(-M_PI/2);
  
  // 3. Continuous wrist rotation demo
  RCLCPP_INFO(logger, "\n=== Demo 3: Continuous wrist rotation ===");
  controller.getVisualTools()->prompt("Press 'Next' to start continuous wrist rotation");
  
  // Lock all joints except the wrist (last joint)
  std::vector<int> all_except_wrist;
  for (int i = 0; i < 6; ++i) {  // Assuming 7 joints total, lock first 6
    all_except_wrist.push_back(i);
  }
  controller.lockJoints(all_except_wrist);
  
  // Rotate wrist continuously for 5 seconds at 0.5 rad/sec
  controller.continuousWristRotation(0.5, 5.0, 0.2);
  
  // 4. Final demonstration: complex planning with locked joints
  RCLCPP_INFO(logger, "\n=== Demo 4: Complex planning with selective joint locking ===");
  controller.unlockAllJoints();
  
  // Lock middle joints, allow shoulder and wrist to move
  controller.lockJoints({2, 3, 4});  // Lock joints 2, 3, 4
  
  // Plan to a different pose
  geometry_msgs::msg::Pose goal2 = goal;
  goal2.position.x += 0.1;  // Move 10cm forward
  goal2.position.z -= 0.1;  // Move 10cm down
  
  controller.planToPoseWithLockedJoints(goal2);

  // Print final joint values
  controller.printCurrentJointValues();

  // Shutdown ROS
  RCLCPP_INFO(logger, "Shutting down...");
  rclcpp::shutdown();
  spinner.join();
  return 0;
}