#include "dual_arm_planner.hpp"
#include "fsm_states.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("dual_arm_planner");

DualArmPlanner::DualArmPlanner(
    const rclcpp::Node::SharedPtr& node,
    moveit::planning_interface::MoveGroupInterface& arm_A,
    moveit::planning_interface::MoveGroupInterface& arm_B,
    moveit::planning_interface::MoveGroupInterface& arm_dual)
    : node_(node),
      arm_move_group_A_(arm_A),
      arm_move_group_B_(arm_B),
      arm_move_group_dual_(arm_dual) {
}

char DualArmPlanner::waitForKeyPress() {
    system("stty raw");
    char input = 0;
    
    while (true) {
        input = getchar();
        
        // Only accept spacebar, 'r', 'R', or Ctrl+C (ASCII 3)
        if (input == ' ' || input == 'r' || input == 'R' || input == 3) {
            break;
        }
    }
    
    system("stty cooked");
    std::cout << std::endl;

    if (input == 3) {
        rclcpp::shutdown();
        return 'q';
    }

    return input;
}

bool DualArmPlanner::plantoTarget_dualarm(
    geometry_msgs::msg::Pose pose1, 
    geometry_msgs::msg::Pose pose2,
    State& current_state,
    State next_state,
    moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const std::string& planning_message,
    bool holding_object) {

    static int plan_attempts = 0;
    const int max_plan_attempts = 5;

    RCLCPP_INFO(LOGGER, "Planning Cartesian path (attempt %d/%d)...", 
                plan_attempts + 1, max_plan_attempts);
    
    // Used to set start state
    moveit::core::RobotStatePtr current_robot_state = arm_move_group_dual_.getCurrentState(1.0);
    arm_move_group_dual_.setStartState(*current_robot_state);

    // Calculate waypoints
    std::vector<geometry_msgs::msg::Pose> waypoints_left;
    std::vector<geometry_msgs::msg::Pose> waypoints_right;
    double target_gripper_distance;
    double eef_step = 0.005;
    std::tie(waypoints_left, waypoints_right, target_gripper_distance) = calculateWaypoints(pose1, pose2, eef_step, holding_object);

    // Compute Cartesian paths
    moveit_msgs::msg::RobotTrajectory trajectory_left;
    double jump_threshold = 0.0;
    
    double fraction_left = arm_move_group_A_.computeCartesianPath(
        waypoints_left, eef_step, jump_threshold, trajectory_left);
    
    moveit_msgs::msg::RobotTrajectory trajectory_right;
    double fraction_right = arm_move_group_B_.computeCartesianPath(
        waypoints_right, eef_step, jump_threshold, trajectory_right);
    
    RCLCPP_INFO(LOGGER, "Cartesian path: Left %.2f%%, Right %.2f%%", 
                fraction_left * 100.0, fraction_right * 100.0);
        
    if (fraction_left < 0.99 || fraction_right < 0.99) {
        RCLCPP_ERROR(LOGGER, "Cartesian path incomplete");
        plan_attempts++;
        if (plan_attempts < max_plan_attempts) {
            return true;
        }
        plan_attempts = 0;
        current_state = State::FAILED;
        return true;
    }
    
    // Resample trajectories if sizes differ
    size_t left_traj_size = trajectory_left.joint_trajectory.points.size();
    size_t right_traj_size = trajectory_right.joint_trajectory.points.size();

    if (left_traj_size != right_traj_size) {
        RCLCPP_WARN(LOGGER, 
            "Trajectory sizes don't match (Left=%zu, Right=%zu). Resampling...", 
            left_traj_size, right_traj_size);
        
        // Resample the shorter trajectory to match the longer one
        if (left_traj_size < right_traj_size) {
            trajectory_left = resampleTrajectory(trajectory_left, right_traj_size);
        } else {
            trajectory_right = resampleTrajectory(trajectory_right, left_traj_size);
        }
        
        RCLCPP_INFO(LOGGER, "Resampled trajectories to size: %zu", 
                    trajectory_left.joint_trajectory.points.size());
    }

    // Validate gripper distance in computed trajectory
    if (holding_object) {
        if (!validateGripperDistance(trajectory_left, trajectory_right, 
                                    target_gripper_distance, 0.005)) {  // 5mm tolerance
            RCLCPP_ERROR(LOGGER, "Gripper distance constraint violated after Cartesian planning");
            plan_attempts++;
            if (plan_attempts < max_plan_attempts) {
                return true;
            }
            plan_attempts = 0;
            current_state = State::FAILED;
            return true;
        }
        
        RCLCPP_INFO(LOGGER, "✓ Gripper distance validated");
    }
    
    // Build combined trajectory and clear old traj
    plan.trajectory_.joint_trajectory.header.frame_id = "world";
    plan.trajectory_.joint_trajectory.header.stamp = node_->now();
    plan.trajectory_.joint_trajectory.points.clear();
    
    // Combine joint names
    plan.trajectory_.joint_trajectory.joint_names = trajectory_left.joint_trajectory.joint_names;
    plan.trajectory_.joint_trajectory.joint_names.insert(
        plan.trajectory_.joint_trajectory.joint_names.end(),
        trajectory_right.joint_trajectory.joint_names.begin(),
        trajectory_right.joint_trajectory.joint_names.end());
    
    // Merge waypoints
    size_t num_points = std::max(trajectory_left.joint_trajectory.points.size(),
                                trajectory_right.joint_trajectory.points.size());
    
    for (size_t i = 0; i < num_points; i++) {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        
        size_t left_idx = std::min(i, trajectory_left.joint_trajectory.points.size() - 1);
        auto& left_point = trajectory_left.joint_trajectory.points[left_idx];
        
        size_t right_idx = std::min(i, trajectory_right.joint_trajectory.points.size() - 1);
        auto& right_point = trajectory_right.joint_trajectory.points[right_idx];
        
        // Combine positions
        point.positions = left_point.positions;
        point.positions.insert(point.positions.end(), 
                            right_point.positions.begin(), 
                            right_point.positions.end());
        
        // Combine velocities
        if (!left_point.velocities.empty() && !right_point.velocities.empty()) {
            point.velocities = left_point.velocities;
            point.velocities.insert(point.velocities.end(),
                                right_point.velocities.begin(),
                                right_point.velocities.end());
        }
        
        // Time from original trajectories
        point.time_from_start = left_point.time_from_start;
        
        plan.trajectory_.joint_trajectory.points.push_back(point);
    }

    // Time parameterization
    robot_trajectory::RobotTrajectory robot_traj(
        arm_move_group_dual_.getRobotModel(), "both_arms");
    robot_traj.setRobotTrajectoryMsg(*current_robot_state, plan.trajectory_);
    
    trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
    time_param.computeTimeStamps(robot_traj, 0.5, 0.3);
    
    robot_traj.getRobotTrajectoryMsg(plan.trajectory_);
    
    plan_attempts = 0;
    RCLCPP_INFO(LOGGER, "%s (Cartesian path)", planning_message.c_str());
    RCLCPP_INFO(LOGGER, "\033[32m Press 'r' to replan, or any other key to execute \033[0m");
    char input = waitForKeyPress();
    
    if (input == 'q') {
        current_state = State::FAILED;
        return false;
    } else if (input == 'r' || input == 'R') {
        return true;
    } else {
        current_state = next_state;
        return true;
    }
}

bool DualArmPlanner::executeMovement_dualarm(
    State& current_state,
    State next_state,
    moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const std::string& success_message,
    const std::string& prompt_message) {

    arm_move_group_dual_.setMaxVelocityScalingFactor(0.2);
    arm_move_group_dual_.setMaxAccelerationScalingFactor(0.05);
    
    bool success = (arm_move_group_dual_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success) {
        RCLCPP_INFO(LOGGER, "%s", success_message.c_str());
        
        if (!prompt_message.empty()) {
            RCLCPP_INFO(LOGGER, "\033[32m %s\033[0m", prompt_message.c_str());
            waitForKeyPress();
        }
        current_state = next_state;
    } else {
        RCLCPP_ERROR(LOGGER, "Failed to execute movement");
        current_state = State::FAILED;
    }
    
    return true;
}

std::tuple<std::vector<geometry_msgs::msg::Pose>, std::vector<geometry_msgs::msg::Pose>, double> DualArmPlanner::calculateWaypoints(
    geometry_msgs::msg::Pose& pose1,
    geometry_msgs::msg::Pose& pose2,
    double eef_step,
    bool holding_object) {

    // Get fresh current state
    auto current_pose1 = arm_move_group_A_.getCurrentPose().pose;
    auto current_pose2 = arm_move_group_B_.getCurrentPose().pose;

    double dist1 = std::sqrt(
        std::pow(pose1.position.x - current_pose1.position.x, 2) +
        std::pow(pose1.position.y - current_pose1.position.y, 2) +
        std::pow(pose1.position.z - current_pose1.position.z, 2));

    double dist2 = std::sqrt(
        std::pow(pose2.position.x - current_pose2.position.x, 2) +
        std::pow(pose2.position.y - current_pose2.position.y, 2) +
        std::pow(pose2.position.z - current_pose2.position.z, 2));

    // Use the longer distance to determine number of waypoints
    double max_dist = std::max(dist1, dist2);
    int num_waypoints = std::max(2, static_cast<int>(max_dist / eef_step));

    RCLCPP_INFO(LOGGER, "Generating %d synchronized waypoints (dist1=%.3f, dist2=%.3f)", 
                num_waypoints, dist1, dist2);

    // Calculate initial gripper distance if holding object
    double target_gripper_distance = 0.0;
    if (holding_object) {
        target_gripper_distance = std::sqrt(
            std::pow(current_pose1.position.x - current_pose2.position.x, 2) +
            std::pow(current_pose1.position.y - current_pose2.position.y, 2) +
            std::pow(current_pose1.position.z - current_pose2.position.z, 2));
        
        RCLCPP_INFO(LOGGER, "Target gripper distance: %.3f m", target_gripper_distance);
    }

    // Synchronized waypoints
    std::vector<geometry_msgs::msg::Pose> waypoints_left;
    std::vector<geometry_msgs::msg::Pose> waypoints_right;

    for (int i = 0; i <= num_waypoints; i++) {
        double t = static_cast<double>(i) / num_waypoints;
        
        geometry_msgs::msg::Pose waypoint1, waypoint2;
        
        // Linear interpolation for positions
        waypoint1.position.x = current_pose1.position.x + t * (pose1.position.x - current_pose1.position.x);
        waypoint1.position.y = current_pose1.position.y + t * (pose1.position.y - current_pose1.position.y);
        waypoint1.position.z = current_pose1.position.z + t * (pose1.position.z - current_pose1.position.z);
        
        waypoint2.position.x = current_pose2.position.x + t * (pose2.position.x - current_pose2.position.x);
        waypoint2.position.y = current_pose2.position.y + t * (pose2.position.y - current_pose2.position.y);
        waypoint2.position.z = current_pose2.position.z + t * (pose2.position.z - current_pose2.position.z);
        
        // SLERP for orientations
        tf2::Quaternion q1_start, q1_end, q2_start, q2_end;
        tf2::convert(current_pose1.orientation, q1_start);
        tf2::convert(pose1.orientation, q1_end);
        tf2::convert(current_pose2.orientation, q2_start);
        tf2::convert(pose2.orientation, q2_end);
        
        tf2::Quaternion q1_interp = q1_start.slerp(q1_end, t);
        tf2::Quaternion q2_interp = q2_start.slerp(q2_end, t);
        
        tf2::convert(q1_interp, waypoint1.orientation);
        tf2::convert(q2_interp, waypoint2.orientation);

        if (holding_object) {
            constrainWaypointPairDistance(waypoint1, waypoint2, target_gripper_distance);
        }
        
        waypoints_left.push_back(waypoint1);
        waypoints_right.push_back(waypoint2);
    }

    return std::make_tuple(waypoints_left, waypoints_right, target_gripper_distance);
}

void DualArmPlanner::constrainWaypointPairDistance(
    geometry_msgs::msg::Pose& waypoint1,
    geometry_msgs::msg::Pose& waypoint2,
    double target_distance) {
    
    Eigen::Vector3d pos1(waypoint1.position.x, waypoint1.position.y, waypoint1.position.z);
    Eigen::Vector3d pos2(waypoint2.position.x, waypoint2.position.y, waypoint2.position.z);
    
    double current_distance = (pos1 - pos2).norm();
    
    if (std::abs(current_distance - target_distance) > 0.001) {  // 1mm threshold
        // Calculate center point
        Eigen::Vector3d center = (pos1 + pos2) / 2.0;
        
        // Calculate unit vectors from center to each gripper
        Eigen::Vector3d vec_to_pos1 = (pos1 - center).normalized();
        Eigen::Vector3d vec_to_pos2 = (pos2 - center).normalized();
        
        // Set corrected positions at exact target distance
        Eigen::Vector3d corrected_pos1 = center + vec_to_pos1 * (target_distance / 2.0);
        Eigen::Vector3d corrected_pos2 = center + vec_to_pos2 * (target_distance / 2.0);
        
        // Update waypoints
        waypoint1.position.x = corrected_pos1.x();
        waypoint1.position.y = corrected_pos1.y();
        waypoint1.position.z = corrected_pos1.z();
        
        waypoint2.position.x = corrected_pos2.x();
        waypoint2.position.y = corrected_pos2.y();
        waypoint2.position.z = corrected_pos2.z();
    }
}

bool DualArmPlanner::validateGripperDistance(
    const moveit_msgs::msg::RobotTrajectory& traj_left,
    const moveit_msgs::msg::RobotTrajectory& traj_right,
    double expected_distance,
    double tolerance) {
    
    size_t num_points = traj_left.joint_trajectory.points.size();
    
    if (num_points != traj_right.joint_trajectory.points.size()) {
        RCLCPP_ERROR(LOGGER, "Trajectory sizes don't match for validation");
        return false;
    }
    
    // Get robot model and kinematic states
    auto robot_model = arm_move_group_dual_.getRobotModel();
    moveit::core::RobotState robot_state(robot_model);
    
    const auto* jmg_left = robot_model->getJointModelGroup(arm_move_group_A_.getName());
    const auto* jmg_right = robot_model->getJointModelGroup(arm_move_group_B_.getName());
    
    if (!jmg_left || !jmg_right) {
        RCLCPP_ERROR(LOGGER, "Could not get joint model groups");
        return false;
    }
    
    std::string ee_link_left = arm_move_group_A_.getEndEffectorLink();
    std::string ee_link_right = arm_move_group_B_.getEndEffectorLink();
    
    double max_deviation = 0.0;
    
    // Check distance at each waypoint
    for (size_t i = 0; i < num_points; i++) {
        // Set joint positions for left arm
        const auto& left_point = traj_left.joint_trajectory.points[i];
        const auto& right_point = traj_right.joint_trajectory.points[i];
        
        robot_state.setJointGroupPositions(jmg_left, left_point.positions);
        robot_state.setJointGroupPositions(jmg_right, right_point.positions);
        robot_state.update();
        
        // Get end-effector positions
        const Eigen::Isometry3d& tf_left = robot_state.getGlobalLinkTransform(ee_link_left);
        const Eigen::Isometry3d& tf_right = robot_state.getGlobalLinkTransform(ee_link_right);
        
        Eigen::Vector3d pos_left = tf_left.translation();
        Eigen::Vector3d pos_right = tf_right.translation();
        
        double distance = (pos_left - pos_right).norm();
        double deviation = std::abs(distance - expected_distance);
        
        max_deviation = std::max(max_deviation, deviation);
        
        if (deviation > tolerance) {
            RCLCPP_ERROR(LOGGER, 
                "Waypoint %zu: Distance %.3fm (expected %.3fm, deviation %.3fm > %.3fm tolerance)",
                i, distance, expected_distance, deviation, tolerance);
            return false;
        }
    }
    
    RCLCPP_INFO(LOGGER, "Distance validation passed. Max deviation: %.4f m", max_deviation);
    return true;
}

moveit_msgs::msg::RobotTrajectory DualArmPlanner::resampleTrajectory(
    const moveit_msgs::msg::RobotTrajectory& traj, 
    size_t target_size) {
    
    moveit_msgs::msg::RobotTrajectory resampled = traj;
    size_t original_size = traj.joint_trajectory.points.size();

    resampled.joint_trajectory.points.clear();
    
    // Linear interpolation between points
    for (size_t i = 0; i < target_size; i++) {
        double t = static_cast<double>(i) / (target_size - 1);
        double original_index = t * (original_size - 1);
        
        size_t idx_low = static_cast<size_t>(std::floor(original_index));
        size_t idx_high = std::min(idx_low + 1, original_size - 1);
        double alpha = original_index - idx_low;
        
        trajectory_msgs::msg::JointTrajectoryPoint point;
        
        const auto& p_low = traj.joint_trajectory.points[idx_low];
        const auto& p_high = traj.joint_trajectory.points[idx_high];
        
        // Interpolate positions
        point.positions.resize(p_low.positions.size());
        for (size_t j = 0; j < p_low.positions.size(); j++) {
            point.positions[j] = (1 - alpha) * p_low.positions[j] + 
                                alpha * p_high.positions[j];
        }
        
        // Interpolate time
        point.time_from_start = rclcpp::Duration::from_seconds(
            (1 - alpha) * p_low.time_from_start.sec + 
            alpha * p_high.time_from_start.sec +
            (1 - alpha) * p_low.time_from_start.nanosec * 1e-9 + 
            alpha * p_high.time_from_start.nanosec * 1e-9
        );
        
        resampled.joint_trajectory.points.push_back(point);
    }
    
    return resampled;
}

void DualArmPlanner::rotate(
    double roll, 
    double pitch, 
    double yaw,
    geometry_msgs::msg::Pose& rotated_pose1,
    geometry_msgs::msg::Pose& rotated_pose2) {

    // Get current poses
    geometry_msgs::msg::PoseStamped current_pose1 = arm_move_group_A_.getCurrentPose();
    geometry_msgs::msg::PoseStamped current_pose2 = arm_move_group_B_.getCurrentPose();
    
    rotated_pose1 = current_pose1.pose;
    rotated_pose2 = current_pose2.pose;
    
    // Calculate bin's center point (midpoint between grippers)
    geometry_msgs::msg::Point object_center;
    object_center.x = (current_pose1.pose.position.x + current_pose2.pose.position.x) / 2.0;
    object_center.y = (current_pose1.pose.position.y + current_pose2.pose.position.y) / 2.0;
    object_center.z = (current_pose1.pose.position.z + current_pose2.pose.position.z) / 2.0;
    
    RCLCPP_INFO(LOGGER, "Object center: x=%.3f, y=%.3f, z=%.3f", 
                object_center.x, object_center.y, object_center.z);
    
    // Create rotation quaternion - define the rotation axis
    tf2::Quaternion rotation_quat;
    rotation_quat.setRPY(roll, pitch, yaw);
    
    // Calculate vectors from bin center to each gripper
    tf2::Vector3 vec_to_gripper1(
        current_pose1.pose.position.x - object_center.x,
        current_pose1.pose.position.y - object_center.y,
        current_pose1.pose.position.z - object_center.z
    );
    
    tf2::Vector3 vec_to_gripper2(
        current_pose2.pose.position.x - object_center.x,
        current_pose2.pose.position.y - object_center.y,
        current_pose2.pose.position.z - object_center.z
    );
    
    // Create rotation matrix from quaternion
    tf2::Matrix3x3 rotation_matrix(rotation_quat);
    
    // Rotate the vectors
    tf2::Vector3 rotated_vec1 = rotation_matrix * vec_to_gripper1;
    tf2::Vector3 rotated_vec2 = rotation_matrix * vec_to_gripper2;
    
    // Apply rotated vectors to get new positions
    rotated_pose1.position.x = object_center.x + rotated_vec1.x();
    rotated_pose1.position.y = object_center.y + rotated_vec1.y();
    rotated_pose1.position.z = object_center.z + rotated_vec1.z();
    
    rotated_pose2.position.x = object_center.x + rotated_vec2.x();
    rotated_pose2.position.y = object_center.y + rotated_vec2.y();
    rotated_pose2.position.z = object_center.z + rotated_vec2.z();
    
    // Convert current orientations to tf2
    tf2::Quaternion current_quat1, current_quat2;
    tf2::convert(current_pose1.pose.orientation, current_quat1);
    tf2::convert(current_pose2.pose.orientation, current_quat2);
    
    // Apply the same rotation to the orientations
    tf2::Quaternion new_quat1 = rotation_quat * current_quat1;
    tf2::Quaternion new_quat2 = rotation_quat * current_quat2;
    new_quat1.normalize();
    new_quat2.normalize();
    
    // Convert back to geometry_msgs
    tf2::convert(new_quat1, rotated_pose1.orientation);
    tf2::convert(new_quat2, rotated_pose2.orientation);
    
    RCLCPP_INFO(LOGGER, "Left arm new position: x=%.3f, y=%.3f, z=%.3f", 
                rotated_pose1.position.x, rotated_pose1.position.y, rotated_pose1.position.z);
    RCLCPP_INFO(LOGGER, "Right arm new position: x=%.3f, y=%.3f, z=%.3f", 
                rotated_pose2.position.x, rotated_pose2.position.y, rotated_pose2.position.z);
    
    // Set planning parameters specific for rotation
    arm_move_group_dual_.setMaxVelocityScalingFactor(0.2);
    arm_move_group_dual_.setMaxAccelerationScalingFactor(0.1);
    arm_move_group_dual_.setPlanningTime(15.0);
}
