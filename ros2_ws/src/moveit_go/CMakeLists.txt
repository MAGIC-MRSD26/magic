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
find_package(geometry_msgs REQUIRED)

add_executable(moveit_go_dual_test_IK src/moveit_go_dual_test_IK.cpp)
target_include_directories(moveit_go_dual_test_IK PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moveit_go_dual_test_IK PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveit_go_dual_test_IK
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

add_executable(motion_planning_single_arm_RRT src/motion_planning_single_arm_RRT.cpp)
target_include_directories(motion_planning_single_arm_RRT PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(motion_planning_single_arm_RRT PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  motion_planning_single_arm_RRT
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)
#adding executable info for dual_arm_fsm node
add_executable(dual_arm_fsm
  src/dual_arm_fsm.cpp
  src/object_definitions.cpp
  src/dual_arm_planner.cpp
)
target_include_directories(dual_arm_fsm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dual_arm_fsm PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  dual_arm_fsm
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
  "geometry_msgs"
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS dual_arm_fsm moveit_go_dual_test_IK
  DESTINATION lib/${PROJECT_NAME})

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


install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
