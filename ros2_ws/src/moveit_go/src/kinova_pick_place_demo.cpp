#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <mtc/kinova_pick_place_task.h>
#include <mtc/kinova_pick_place_demo_parameters.hpp>

using namespace kinova_pick_place_demo;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("kinova_pick_place_demo");
  
  // Create parameter instance and potentially load from ROS params
  kinova_pick_place_demo::Params params;
  
  // Uncomment to load parameters from ROS param server
  // loadParameters(node, params);
  
  // Set up the scene for the demo (add table and object)
  setupDemoScene(params);
  
  // Create and initialize the task
  KinovaPickPlaceTask pick_place_task("kinova_pick_place_demo");
  if (!pick_place_task.init(node, params)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize pick place task");
    return 1;
  }
  
  // Plan and execute
  if (pick_place_task.plan(10)) {
    RCLCPP_INFO(node->get_logger(), "Planning successful");
    if (pick_place_task.execute()) {
      RCLCPP_INFO(node->get_logger(), "Execution successful");
    } else {
      RCLCPP_ERROR(node->get_logger(), "Execution failed");
      return 1;
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), "Planning failed");
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}