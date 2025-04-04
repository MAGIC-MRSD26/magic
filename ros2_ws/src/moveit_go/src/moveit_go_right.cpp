#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

class RightArmPlanner : public rclcpp::Node {
  public:
    RightArmPlanner() : Node("moveit_go_right") {
      RCLCPP_INFO(get_logger(), "Node created, waiting for MoveGroupInterface to be initialized externally.");
    }

    void init_move_group() {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "right_arm");

      subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/right_arm_pose", 10,
        std::bind(&RightArmPlanner::pose_callback, this, std::placeholders::_1)
      );

      RCLCPP_INFO(get_logger(), "Ready to receive goals on /right_arm_pose");
    }

  private:
    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
      move_group_->setPoseTarget(*msg);
      move_group_->setPlanningTime(15.0);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (success) {
        RCLCPP_INFO(this->get_logger(), "Planning successful, executing...");
        move_group_->execute(plan);
      } else {
        RCLCPP_WARN(this->get_logger(), "Planning failed.");
      }
      rclcpp::shutdown(); 
      // Uncomment this line if you want to shut down the node after executing the plan
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriber_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RightArmPlanner>();

  // Only call shared_from_this() AFTER node is fully constructed
  node->init_move_group();

  rclcpp::spin(node);
  //rclcpp::shutdown();
  //uncomment this line if you want to the node running continuously
  return 0;
}
