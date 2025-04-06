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
      RCLCPP_INFO(get_logger(), "Node created, waiting for MoveGroupInterface to be initialized externally.");
    }
  
    void init_move_group() {
      move_group_left = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "left_arm");
      move_group_right = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "right_arm");
      move_group_both = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "both_arms");
  
      subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/arm_pose", 10,
        std::bind(&JointArmPlanner::pose_array_callback, this, std::placeholders::_1)
      );
      
      //debug statements
      RCLCPP_INFO(get_logger(), "initialized move_group");
      RCLCPP_INFO(get_logger(), "Ready to receive goals on /arm_pose");
      RCLCPP_INFO(get_logger(), move_group_left->getName().c_str());
      RCLCPP_INFO(get_logger(), move_group_left->getEndEffectorLink().c_str());
      RCLCPP_INFO(get_logger(), move_group_right->getName().c_str());
      RCLCPP_INFO(get_logger(), move_group_right->getEndEffectorLink().c_str());
      RCLCPP_INFO(get_logger(), move_group_both->getName().c_str());
      RCLCPP_INFO(get_logger(), move_group_both->getEndEffectorLink().c_str());
      RCLCPP_INFO(get_logger(), "Node initialized");
    }
  
  private:
    void pose_array_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
      //debug statement: checking incoming pose
      if (msg->poses.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "Received fewer than 2 poses.");
        return;
      }
    
      pose1 = msg->poses[0];
      pose2 = msg->poses[1];
      poses_received_=true;
    
      // Now you can use pose1 and pose2 for planning
      RCLCPP_INFO(this->get_logger(), "Received two poses.");
      RCLCPP_INFO(this->get_logger(), "Pose 1:");
      RCLCPP_INFO(this->get_logger(), "  Position -> x: %.3f, y: %.3f, z: %.3f",
              pose1.position.x, pose1.position.y, pose1.position.z);
      RCLCPP_INFO(this->get_logger(), "  Orientation -> x: %.3f, y: %.3f, z: %.3f, w: %.3f",
              pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w);
      RCLCPP_INFO(this->get_logger(), "Pose 2:");
      RCLCPP_INFO(this->get_logger(), "  Position -> x: %.3f, y: %.3f, z: %.3f",
              pose2.position.x, pose2.position.y, pose2.position.z);
      RCLCPP_INFO(this->get_logger(), "  Orientation -> x: %.3f, y: %.3f, z: %.3f, w: %.3f",
              pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w);
      
      // Set target pose for left arm
      move_group_left->setPoseTarget(pose1);
      move_group_left->setPlanningTime(15.0);
      moveit::planning_interface::MoveGroupInterface::Plan plan_left;
      bool success_left = (move_group_left->plan(plan_left) == moveit::core::MoveItErrorCode::SUCCESS);

      // Set target pose for right arm
      move_group_right->setPoseTarget(pose2);
      move_group_right->setPlanningTime(15.0);
      moveit::planning_interface::MoveGroupInterface::Plan plan_right;
      bool success_right = (move_group_right->plan(plan_right) == moveit::core::MoveItErrorCode::SUCCESS);
      
      //if both plans are successful
      if (success_left && success_right){
          const auto& joint_traj_left = plan_left.trajectory_.joint_trajectory;
          const auto& joint_traj_right = plan_right.trajectory_.joint_trajectory;
      
          // Extract joint positions and store them in the array
          for (const auto& point : joint_traj_left.points) {
              left_arm_trajectory.push_back(point.positions);
          }
          for (const auto& point : joint_traj_right.points) {
              right_arm_trajectory.push_back(point.positions);
          }

          //print that planning was successful
          RCLCPP_INFO(this->get_logger(), "Planning successful for both arms.");
      }

      else if (success_left) {
          RCLCPP_INFO(this->get_logger(), "Planning successful for left arm, but failed for right arm.");
      }
      else if (success_right) {
          RCLCPP_INFO(this->get_logger(), "Planning successful for right arm, but failed for left arm.");
      }
      else {
          RCLCPP_WARN(this->get_logger(), "Planning failed for both arms.");
      }

      if (left_arm_trajectory.empty() || right_arm_trajectory.empty()) {
        RCLCPP_WARN(this->get_logger(), "One or both trajectories are empty. Cannot proceed.");
        return;
    }

    // Get the sizes of the trajectories
    size_t left_size = left_arm_trajectory.size();
    size_t right_size = right_arm_trajectory.size();
    size_t max_size = std::max(left_size, right_size);

    // Loop through the trajectories
    for (size_t i = 0; i < max_size; ++i) {
      // Get the current joint positions for left and right arms
      const std::vector<double>& left_joint_positions = 
        (i < left_size) ? left_arm_trajectory[i] : left_arm_trajectory.back();
      const std::vector<double>& right_joint_positions = 
        (i < right_size) ? right_arm_trajectory[i] : right_arm_trajectory.back();

      // Concatenate the joint positions
      std::vector<double> combined_joint_positions;
      combined_joint_positions.insert(combined_joint_positions.end(), left_joint_positions.begin(), left_joint_positions.end());
      combined_joint_positions.insert(combined_joint_positions.end(), right_joint_positions.begin(), right_joint_positions.end());

      // Set the joint values for the combined group
      move_group_both->setJointValueTarget(combined_joint_positions);

      // Plan and execute the motion
      moveit::planning_interface::MoveGroupInterface::Plan combined_plan;
      bool success = (move_group_both->plan(combined_plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (success) {
          RCLCPP_INFO(this->get_logger(), "Combined plan successful for step %zu, executing...", i);
          move_group_both->execute(combined_plan);
      } 
      else {
          RCLCPP_WARN(this->get_logger(), "Combined plan failed for step %zu.", i);
      }
    }

          
      rclcpp::shutdown(); 
      // Uncomment this line if you want to shut down the node after executing the plan
    }


    // rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscriber_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_left;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_right;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_both;

    //variables for pose1 and pose2
    geometry_msgs::msg::Pose pose1;
    geometry_msgs::msg::Pose pose2;
    bool poses_received_ = false;  // optional flag for checking readiness

    //trajectories
    std::vector<std::vector<double>> left_arm_trajectory;
    std::vector<std::vector<double>> right_arm_trajectory;

  };
  
  int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointArmPlanner>();
  
    // Only call shared_from_this() AFTER node is fully constructed
    node->init_move_group();
  
    rclcpp::spin(node);
    //rclcpp::shutdown();
    //uncomment this line if you want to the node running continuously

    return 0;
  }
  