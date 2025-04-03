cmake_minimum_required(VERSION 3.8)
project(moveit_go)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Make sure the compiler finds include files
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Original executables
add_executable(moveit_go_left src/moveit_go_left.cpp)
target_include_directories(moveit_go_left PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moveit_go_left PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  moveit_go_left
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

add_executable(moveit_go_right src/moveit_go_right.cpp)
target_include_directories(moveit_go_right PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moveit_go_right PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  moveit_go_right
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

add_executable(moveit_go_dual src/moveit_go_dual.cpp)
target_include_directories(moveit_go_dual PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moveit_go_dual PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  moveit_go_dual
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

# Add library for Kinova pick place task
add_library(kinova_pick_place_task
  src/kinova_pick_place_task.cpp
)
ament_target_dependencies(
  kinova_pick_place_task
  "moveit_task_constructor_core"
  "moveit_ros_planning_interface"
  "moveit_core"
  "moveit_ros_planning"
  "tf2_eigen"
  "tf2_geometry_msgs"
  "geometry_msgs"
  "rclcpp"
)

# New executable for Kinova pick place demo
add_executable(kinova_pick_place_demo src/kinova_pick_place_demo.cpp)
target_include_directories(kinova_pick_place_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(kinova_pick_place_demo PUBLIC cxx_std_17)
ament_target_dependencies(
  kinova_pick_place_demo
  "moveit_task_constructor_core"
  "moveit_ros_planning_interface"
  "rclcpp"
)
target_link_libraries(kinova_pick_place_demo
  kinova_pick_place_task
)

# Install targets
install(TARGETS 
  moveit_go_left 
  moveit_go_right 
  moveit_go_dual 
  kinova_pick_place_task
  kinova_pick_place_demo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(kinova_pick_place_task)
ament_export_dependencies(
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_task_constructor_core
  tf2_eigen
  tf2_geometry_msgs
  geometry_msgs
  moveit_task_constructor_msgs
)

ament_package()