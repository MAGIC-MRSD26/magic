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

add_executable(moveit_go_left src/moveit_go_left.cpp)
target_include_directories(moveit_go_left PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moveit_go_left PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
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
target_compile_features(moveit_go_right PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
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
target_compile_features(moveit_go_dual PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveit_go_dual
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

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

add_executable(motion_planning src/motion_planning.cpp)
target_include_directories(motion_planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(motion_planning PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  motion_planning
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)
#adding executable info for motion_planning_dual node
add_executable(motion_planning_dual src/motion_planning_dual.cpp)
target_include_directories(motion_planning_dual PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(motion_planning_dual PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  motion_planning_dual
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
  "geometry_msgs"
)

add_executable(dual_arm_joint_planner src/dual_arm_joint_planner.cpp)
ament_target_dependencies(
  dual_arm_joint_planner
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)
install(TARGETS dual_arm_joint_planner DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS moveit_go_left moveit_go_right moveit_go_dual motion_planning motion_planning_dual moveit_go_dual_test_IK
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

ament_package()
