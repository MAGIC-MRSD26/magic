cmake_minimum_required(VERSION 3.5)
project(end_pose_cpp)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add executable
add_executable(end_pose_node src/end_pose.cpp)

# Link dependencies
ament_target_dependencies(end_pose_node rclcpp moveit_core moveit_ros_planning_interface moveit_ros_planning geometry_msgs)

# Install the executable
install(TARGETS
  end_pose_node
  DESTINATION lib/${PROJECT_NAME})

# Required for ament package
ament_package()
