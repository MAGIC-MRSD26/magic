#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>


class JointArmPlanner : public rclcpp::Node {
public:
  JointArmPlanner() : Node("moveit_go_dual") {
    RCLCPP_INFO(get_logger(), "JointArmPlanner node created");
  }
  
  void init_move_group(std::shared_ptr<rclcpp::Node> move_group_node) {
    // Store the move group node
    this->move_group_node_ = move_group_node;
    
    try {
      // Using the external node for MoveGroupInterface
      RCLCPP_INFO(get_logger(), "Initializing move groups using external node");
      move_group_left = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, "left_arm");
      move_group_right = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, "right_arm");
      move_group_both = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, "both_arms");
      
      //debug statements
      RCLCPP_INFO(get_logger(), "Move groups initialized successfully");

      // Add a brief delay to ensure move groups are ready
      std::this_thread::sleep_for(std::chrono::seconds(1));
      
      get_joint_positions();
      plan_both_arms();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Exception during move_group initialization: %s", e.what());
    }
  }
  
private:
  void get_joint_positions() {
    try {
      // Define orientation in quaternion
      tf2::Quaternion tf2_quat;
      tf2_quat.setRPY(-1.57, 0, 0);
      geometry_msgs::msg::Quaternion quat_orient;
      tf2::convert(tf2_quat, quat_orient);
  
      pose1.orientation = quat_orient;
      pose2.orientation = quat_orient;

      pose1.position.x = 0.1868;
      pose1.position.y = 0.0;
      pose1.position.z = 1.4; 

      pose2.position.x = -0.1868;
      pose2.position.y = 0.0;
      pose2.position.z = 1.4;

      // Get the current state with timeout
      RCLCPP_INFO(get_logger(), "Getting current robot state...");
      moveit::core::RobotStatePtr current_state_left = move_group_left->getCurrentState(5.0);
      if (!current_state_left) {
        RCLCPP_ERROR(get_logger(), "Failed to get current state for left arm");
        return;
      }
      
      moveit::core::RobotStatePtr current_state_right = move_group_right->getCurrentState(5.0);
      if (!current_state_right) {
        RCLCPP_ERROR(get_logger(), "Failed to get current state for right arm");
        return;
      }
      
      RCLCPP_INFO(get_logger(), "Successfully retrieved current states");

      // Get joint model
      const moveit::core::JointModelGroup* joint_model_group_left = current_state_left->getJointModelGroup("left_arm");
      const moveit::core::JointModelGroup* joint_model_group_right = current_state_right->getJointModelGroup("right_arm");

      // Print joint names for debugging
      std::vector<std::string> left_joint_names = joint_model_group_left->getVariableNames();
      RCLCPP_INFO(get_logger(), "Left arm has %zu joints", left_joint_names.size());

      //Compute IK
      RCLCPP_INFO(get_logger(), "Computing IK solutions...");
      bool ik_left = current_state_left->setFromIK(joint_model_group_left, pose1, move_group_left->getEndEffectorLink(), 10.0);
      bool ik_right = current_state_right->setFromIK(joint_model_group_right, pose2, move_group_right->getEndEffectorLink(), 10.0);
      
      if (ik_left && ik_right) {
        RCLCPP_INFO(this->get_logger(), "IK successful for both arms");
        
        // Initialize the vectors first, then pass them as output parameters
        std::vector<double> left_joint_positions;
        std::vector<double> right_joint_positions;
        
        // Get joint positions for left arm
        current_state_left->copyJointGroupPositions(joint_model_group_left, left_joint_positions);
        
        // Get joint positions for right arm
        current_state_right->copyJointGroupPositions(joint_model_group_right, right_joint_positions);
        
        // Combine the joint positions
        combined_joint_positions = left_joint_positions;
        combined_joint_positions.insert(combined_joint_positions.end(), 
                                      right_joint_positions.begin(), 
                                      right_joint_positions.end());
                                      
        RCLCPP_INFO(get_logger(), "Joint positions combined successfully, size=%zu", combined_joint_positions.size());
      } else {
        RCLCPP_WARN(this->get_logger(), "IK failed for one or both arms");
        if (!ik_left) RCLCPP_WARN(get_logger(), "Left arm IK failed");
        if (!ik_right) RCLCPP_WARN(get_logger(), "Right arm IK failed");
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Exception in get_joint_positions: %s", e.what());
    }
  }
  
  void plan_both_arms() {
    try {
      // Set the joint values for the combined group
      RCLCPP_INFO(get_logger(), "Setting joint target for both arms...");
      move_group_both->setJointValueTarget(combined_joint_positions);

      // Set planning time
      move_group_both->setPlanningTime(20.0);

      // Plan and execute the motion
      RCLCPP_INFO(get_logger(), "Planning trajectory...");
      moveit::planning_interface::MoveGroupInterface::Plan combined_plan;
      
      auto result = move_group_both->plan(combined_plan);
      bool success = (result == moveit::core::MoveItErrorCode::SUCCESS);

      if (success) {
        RCLCPP_INFO(this->get_logger(), "Combined plan successful, executing...");
        move_group_both->execute(combined_plan);
      } 
      else {
        RCLCPP_WARN(this->get_logger(), "Combined plan failed with error code: %d", result.val);
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Exception in plan_both_arms: %s", e.what());
    }
  }

  // Store the move group node
  std::shared_ptr<rclcpp::Node> move_group_node_;
  
  // MoveIt interfaces
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_left;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_right;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_both;

  // Poses and joint positions
  geometry_msgs::msg::Pose pose1;
  geometry_msgs::msg::Pose pose2;
  std::vector<double> combined_joint_positions;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  // Create a separate node for MoveIt with sim_time parameter
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("motion_planning_dual_arms", node_options);

  rclcpp::Parameter sim_time_param("use_sim_time", true);
  move_group_node->set_parameter(sim_time_param);

  // Wait for time to be properly initialized
  RCLCPP_INFO(move_group_node->get_logger(), "Waiting for ROS time to be initialized...");
  std::this_thread::sleep_for(std::chrono::seconds(2));

  move_group_node->declare_parameter("robot_description_kinematics.left_arm.kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
  move_group_node->declare_parameter("robot_description_kinematics.right_arm.kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
  
  // Spin this node in a separate thread to handle clock synchronization
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread spinner([&executor]() { 
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting executor for move_group_node");
    executor.spin(); 
  });
  
  // Give time for the executor to start
  std::this_thread::sleep_for(std::chrono::seconds(2));
  
  // Create the main node and pass the move_group_node to it
  auto node = std::make_shared<JointArmPlanner>();
  
  // Initialize move group using the external node
  RCLCPP_INFO(node->get_logger(), "Initializing move groups");
  node->init_move_group(move_group_node);
  
  // Spin the main node
  RCLCPP_INFO(node->get_logger(), "Spinning main node");
  rclcpp::spin(node);
  
  // Clean up
  spinner.join();
  rclcpp::shutdown();
  return 0;
}