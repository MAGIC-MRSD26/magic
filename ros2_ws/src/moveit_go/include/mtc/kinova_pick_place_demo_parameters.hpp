#pragma once

#include <string>
#include <vector>

namespace kinova_pick_place_demo {

struct Params {
  // Robot groups and links
  std::string arm_group_name = "manipulator";  // Planning group name for the Kinova arm
  std::string hand_group_name = "gripper";    // Planning group name for the Kinova gripper
  std::string eef_name = "gripper";           // End effector name
  std::string hand_frame = "robotiq_85_base_link"; // Name of the gripper frame
  
  // Home pose for arm
  std::vector<double> arm_home_pose = {0.0, -0.5, 0.0, -1.57, 0.0, 1.0, 0.78}; // Adjust for your Kinova arm
  
  // Gripper poses
  std::vector<double> hand_open_pose = {0.04};  // Adjust for your Kinova gripper joint values
  std::vector<double> hand_close_pose = {0.0};  // Adjust for your Kinova gripper joint values
  std::vector<double> grasp_frame_transform = {0.0, 0.0, 0.1, 0, 0, 0}; // Adjust for Kinova gripper TCP offset
  
  // Scene frames
  std::string world_frame = "world";
  std::string table_reference_frame = "world";
  std::string object_reference_frame = "world";
  bool spawn_table = true;
  
  // Surface properties
  std::string table_name = "table";
  std::string surface_link = "table";
  std::vector<double> table_dimensions = {0.6, 0.6, 0.05};  // Length, Width, Height
  std::vector<double> table_pose = {0.5, 0.0, 0.025, 0, 0, 0}; // X, Y, Z, Roll, Pitch, Yaw
  
  // Object properties
  std::string object_name = "object";
  std::vector<double> object_dimensions = {0.06, 0.02}; // Height, Radius for cylinder
  std::vector<double> object_pose = {0.5, 0.0, 0.06, 0, 0, 0};  // X, Y, Z, Roll, Pitch, Yaw
  std::vector<double> place_pose = {0.4, 0.3, 0.1, 0, 0, 0}; // X, Y, Z, Roll, Pitch, Yaw
  double place_surface_offset = 0.0001;  // Small offset from the target surface
  
  // Pick/place settings
  double approach_object_min_dist = 0.05;
  double approach_object_max_dist = 0.1;
  double lift_object_min_dist = 0.05;
  double lift_object_max_dist = 0.15;
};

} // namespace kinova_pick_place_demo