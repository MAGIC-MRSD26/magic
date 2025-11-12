#include <moveit/move_group_interface/move_group_interface.h>
#include "geometry_msgs/msg/pose_array.hpp"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <future>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include "object_definitions.hpp"
#include <std_msgs/msg/bool.hpp>

// Helpers
#include "dual_arm_planner.hpp"
#include "fsm_states.hpp"

// This is a finite state machine for Kinova Gen3 7DOF arm
static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_planning");

class MotionPlanningFSM {
private:
    // mid fsm placement spot
    static constexpr double PLACEMENT_X = 0.0;
    static constexpr double PLACEMENT_Y = 0.0;
    static constexpr double PLACEMENT_ANGLE = 45.0;

public:
    MotionPlanningFSM(
        const rclcpp::Node::SharedPtr& node,
        const std::string& arm_planning_group_A_, 
        const std::string& gripper_planning_group_A_,
        const std::string& arm_planning_group_B_, 
        const std::string& gripper_planning_group_B_,
        const std::string& arm_planning_group_dual_, 
        const std::string& gripper_planning_group_dual_,
        ObjectType object_type = ObjectType::BIN)
        : node_(node),

        // Initialize planning groups
        arm_planning_group_A(arm_planning_group_A_),
        gripper_planning_group_A(gripper_planning_group_A_),
        arm_planning_group_B(arm_planning_group_B_),
        gripper_planning_group_B(gripper_planning_group_B_),
        arm_planning_group_dual(arm_planning_group_dual_),
        gripper_planning_group_dual(gripper_planning_group_dual_),

        // Initialize move groups
        arm_move_group_A(node,arm_planning_group_A_),
        gripper_move_group_A(node, gripper_planning_group_A_),
        arm_move_group_B(node,arm_planning_group_B_),
        gripper_move_group_B(node, gripper_planning_group_B_),
        arm_move_group_dual(node,arm_planning_group_dual_),
        gripper_move_group_dual(node, gripper_planning_group_dual_),
        current_state_(State::HOME),
        selected_object_type_(object_type) {
        
        // Initialize dual arm planner helper functions
        dual_arm_planner_ = std::make_unique<DualArmPlanner>(
            node_, arm_move_group_A, arm_move_group_B, arm_move_group_dual);
        
        // Create object parameters based on selected type
        arm_move_group_A.setMaxVelocityScalingFactor(0.6); // Increase from default
        arm_move_group_B.setMaxVelocityScalingFactor(0.6);
        arm_move_group_dual.setMaxVelocityScalingFactor(0.5); // More conservative for dual-arm

        // For kinematic chain movements specifically (after grasping)
        arm_move_group_dual.setMaxVelocityScalingFactor(0.4); // Safe but still faster
        arm_move_group_dual.setMaxAccelerationScalingFactor(0.3);
        
        // Create subscription to the object pose topic
        pose_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/cylinder_pose", 10, 
            std::bind(&MotionPlanningFSM::objectPoseCallback, this, std::placeholders::_1));

        pose_received_ = false;
        
        // Initialize 3D capture flag
        capture_active_ = false;
        
        // Create publisher for 3D capture flag (100Hz = 10ms period)
        capture_flag_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(
            "/capture_3d_active", 10);
        
        // Create timer to publish at 100Hz
        capture_flag_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MotionPlanningFSM::publishCaptureFlag, this));
        
        RCLCPP_INFO(LOGGER, "MotionPlanningFSM initialized");
    }

    bool execute() {
        switch (current_state_) {
            case State::HOME:
                return home();
     
            case State::PLAN_TO_OBJECT:
                return planToObject();

            case State::MOVE_TO_OBJECT:
                return moveToObject();

            case State::OPEN_GRIPPER:
                return opengripper();
                
            case State::PLAN_TO_GRASP:
                return planToGrasp(); 

            case State::MOVE_TO_GRASP:
                return moveToGrasp();

            case State::GRASP:
                return Grasp();

            case State::PLAN_TO_LIFT:
                return planToLift();

            case State::MOVE_TO_LIFT:
                return moveToLift();

            case State::PLAN_TO_CENTER:
                return planToCenter();

            case State::MOVE_TO_CENTER:
                return moveToCenter();

            case State::PLAN_TO_STRAIGHTEN:
                return planToStraighten();

            case State::MOVE_TO_STRAIGHTEN:
                return moveToStraighten();

            case State::ROTATE_EE:
                return rotateEndEffectors();

            case State::PLAN_TO_PLACE_XY:
                return planToPlaceXY();

            case State::MOVE_TO_PLACE_XY:
                return moveToPlaceXY();

            case State::PLAN_TO_PLACE:
                return planToPlace();

            case State::MOVE_TO_PLACE:
                return moveToPlace();

            case State::PLACE:
                return Place();

            case State::PLAN_RETRACT:
                return planToRetract();

            case State::MOVE_RETRACT:
                return moveRetract();

            case State::CLOSE_GRIPPER:
                return closegripper();

            case State::PLAN_TO_HOME:
                return planToHome();

            case State::MOVE_TO_HOME:
                return moveToHome();

            case State::SUCCEEDED:
                RCLCPP_INFO(LOGGER, "Motion planning succeeded.");
                return false;

            case State::FAILED:
                RCLCPP_ERROR(LOGGER, "Motion planning failed.");
                return false;
                
            default:
                RCLCPP_ERROR(LOGGER, "Unknown state.");
                current_state_ = State::FAILED;
                return false;
        }
    }

private:
    //retry counter variables
    int retry_count = 0;
    int centering_attempts = 0;
    //pointer for ros2 node
    rclcpp::Node::SharedPtr node_;

    //arm planning group names- variables
    std::string arm_planning_group_A;
    std::string gripper_planning_group_A;
    std::string arm_planning_group_B;
    std::string gripper_planning_group_B;
    std::string arm_planning_group_dual;
    std::string gripper_planning_group_dual;

    // Planner helper functions
    std::unique_ptr<DualArmPlanner> dual_arm_planner_;

    //creating moveit interfaces for arm and gripper groups
    //moveit groups for A
    moveit::planning_interface::MoveGroupInterface arm_move_group_A;
    moveit::planning_interface::MoveGroupInterface gripper_move_group_A;

    //moveit groups for B
    moveit::planning_interface::MoveGroupInterface arm_move_group_B;
    moveit::planning_interface::MoveGroupInterface gripper_move_group_B;

    //moveit groups for dual arms
    moveit::planning_interface::MoveGroupInterface arm_move_group_dual;
    moveit::planning_interface::MoveGroupInterface gripper_move_group_dual;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_dual;

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    //current state variable
    State current_state_;
    //target pose variables
    geometry_msgs::msg::Pose target_pose_A;
    geometry_msgs::msg::Pose target_pose_B;

    // for later when we reuse to go to other handles
    bool go_to_next_grasp = false;

    moveit_msgs::msg::AttachedCollisionObject attached_object;

    // Rotation policy variables
    geometry_msgs::msg::Pose rotated_pose1;
    geometry_msgs::msg::Pose rotated_pose2;
    int rotations = 0;

    // Object params
    ObjectType selected_object_type_;
    ObjectParameters object_params_;

    // Bin subscription 
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    geometry_msgs::msg::Pose object_pose_;
    double yaw_angle;
    bool pose_received_;
    
    // 3D capture flag
    bool capture_active_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr capture_flag_publisher_;
    rclcpp::TimerBase::SharedPtr capture_flag_timer_;
    
    
    // Callback for the object pose subscriber
    void objectPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        object_pose_ = msg->pose;
        pose_received_ = true;
        RCLCPP_INFO(LOGGER, "Received object pose: x=%f, y=%f, z=%f", 
                object_pose_.position.x,
                object_pose_.position.y,
                object_pose_.position.z);

        tf2::Quaternion q(object_pose_.orientation.x,
                            object_pose_.orientation.y,
                            object_pose_.orientation.z,
                            object_pose_.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        yaw_angle = yaw * (180.0 / M_PI);

        RCLCPP_INFO(LOGGER, "Object orientation (yaw): %f degrees", yaw_angle);

        pose_subscription_.reset();
    }

    void publishCaptureFlag() {
        std_msgs::msg::Bool msg;
        msg.data = capture_active_;
        capture_flag_publisher_->publish(msg);
    }

    ObjectParameters createPlacementParams() {
        ObjectParameters params;
        if (selected_object_type_ == ObjectType::BIN) {
            params = ObjectFactory::createBinParameters(PLACEMENT_X, PLACEMENT_Y);
        } else if (selected_object_type_ == ObjectType::CYLINDER_WITH_SPOKES) {
            params = ObjectFactory::createCylinderParameters(PLACEMENT_X, PLACEMENT_Y, PLACEMENT_ANGLE);
        } else if (selected_object_type_ == ObjectType::TBAR) {
            params = ObjectFactory::createTbarParameters(PLACEMENT_X, PLACEMENT_Y, PLACEMENT_ANGLE);
        } else {
            RCLCPP_ERROR(LOGGER, "Unknown object type!");
        }
        return params;
    }

    /*//////////////////////////////////////////

    START OF FSM FUNCTIONS

    *//////////////////////////////////////////

    bool home() {
        // Wait for robot state monitor to initialize
        RCLCPP_INFO(LOGGER, "Waiting for robot state...");
        rclcpp::sleep_for(std::chrono::seconds(2));
        
        // Verify we have joint states
        auto state = arm_move_group_dual.getCurrentState(2.0);
        if (!state) {
            RCLCPP_ERROR(LOGGER, "Failed to get current robot state");
            current_state_ = State::FAILED;
            return true;
        }
        RCLCPP_INFO(LOGGER, "Robot state ready");

        // Add object to the planning scene
        RCLCPP_INFO(LOGGER, "\033[32m Press any key to add object to planning scene \033[0m");
        dual_arm_planner_->waitForKeyPress();

        // Create object parameters based on type
        double x = 0.08, y = 0.14, yaw = 40.3;
        // At (0,0) yaw angle range is 35 - 50
        if (pose_received_) {
            x = object_pose_.position.x;
            y = object_pose_.position.y;
            yaw = yaw_angle;
        }
        
        // Use factory to create parameters and collision object
        if (selected_object_type_ == ObjectType::BIN) {
            object_params_ = ObjectFactory::createBinParameters(x, y);
        } else if (selected_object_type_ == ObjectType::CYLINDER_WITH_SPOKES) {
            object_params_ = ObjectFactory::createCylinderParameters(x, y, yaw);
        } else if (selected_object_type_ == ObjectType::TBAR) {
            object_params_ = ObjectFactory::createTbarParameters(x, y, yaw);
        }
        // Create and add collision object to scene
        moveit_msgs::msg::CollisionObject collision_object = 
            ObjectFactory::createObject(selected_object_type_, object_params_);
        
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.push_back(collision_object);
        planning_scene_interface_dual.applyCollisionObjects(collision_objects);
        
        RCLCPP_INFO(LOGGER, "Added %s to planning scene", object_params_.object_id.c_str());
        
        if (!pose_received_ && retry_count < 2) {
            RCLCPP_INFO_THROTTLE(LOGGER, *node_->get_clock(), 2000, 
                                "Waiting for pose on /aruco_poses...");
            retry_count++;
            return true;
        }
        current_state_ = State::PLAN_TO_OBJECT;
        return true;
    }

    bool planToObject() {

        if (go_to_next_grasp) {
            target_pose_A = object_params_.second_left_grasp_pose;
            target_pose_B = object_params_.second_right_grasp_pose;
        } else {
            // Use pre-calculated grasp poses from object_params_
            target_pose_A = object_params_.left_grasp_pose;
            target_pose_B = object_params_.right_grasp_pose;
        }

        // Adjust Z for approach
        target_pose_A.position.z += object_params_.approach_offset;
        target_pose_B.position.z += object_params_.approach_offset;

        RCLCPP_INFO(LOGGER, "Left arm target pose x: %f y: %f z: %f", target_pose_A.position.x, target_pose_A.position.y, target_pose_A.position.z);
        RCLCPP_INFO(LOGGER, "Right arm target pose x: %f y: %f z: %f", target_pose_B.position.x, target_pose_B.position.y, target_pose_B.position.z);

        RCLCPP_INFO(LOGGER, "\033[32m Press any key to plan to object \033[0m");
        dual_arm_planner_->waitForKeyPress();

        return dual_arm_planner_->plantoTarget_dualarm(target_pose_A, target_pose_B, current_state_, State::MOVE_TO_OBJECT, plan,
             "Planning to object succeeded!", false);
    }

    bool moveToObject() {
        //execute the planned trajectory
        return dual_arm_planner_->executeMovement_dualarm(current_state_, State::OPEN_GRIPPER, plan, "Successfully moved to object position", 
                                    "Press any key to open gripper");
    }

    bool opengripper() {
        //state for gripping action
        gripper_move_group_dual.setNamedTarget("Open");
        bool success = (gripper_move_group_dual.move() == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success) {
            RCLCPP_INFO(LOGGER, "Successfully opened gripper");
            current_state_ = State::PLAN_TO_GRASP;  
        } else {
            RCLCPP_ERROR(LOGGER, "Failed to open gripper");
            current_state_ = State::FAILED;
        }
        
        return true;
    }

    bool planToGrasp() {
        target_pose_A.position.z += object_params_.grasp_offset;
        target_pose_B.position.z += object_params_.grasp_offset;
        
        RCLCPP_INFO(LOGGER, "\033[32m Press any key to plan to grasp\033[0m");
        dual_arm_planner_->waitForKeyPress();
        return dual_arm_planner_->plantoTarget_dualarm(target_pose_A, target_pose_B, current_state_, State::MOVE_TO_GRASP, plan,
                          "Planning to grasp succeeded!", false);
    }

    bool moveToGrasp() {
        //state for executing the trajectory for moving to grasp point
        return dual_arm_planner_->executeMovement_dualarm(current_state_, State::GRASP, plan, "Successfully moved to grasp pose", 
                             "Press any key to grasp object");
    }

    bool Grasp() {

        // Now we attach the object
        attached_object.object.id = object_params_.object_id;
        //cannot link both end effectors so linking left arm alone
        attached_object.link_name = arm_move_group_A.getEndEffectorLink();
        
        // Define which links are allowed to touch the object
        std::vector<std::string> touch_links;
        // Add your specific gripper finger links here
        std::string prefix = "left_";
        touch_links.push_back(prefix + "robotiq_85_left_finger_link");
        touch_links.push_back(prefix + "robotiq_85_left_finger_tip_link");
        touch_links.push_back(prefix + "robotiq_85_left_inner_knuckle_link");
        touch_links.push_back(prefix + "robotiq_85_left_knuckle_link");
        touch_links.push_back(prefix + "robotiq_85_right_finger_link");
        touch_links.push_back(prefix + "robotiq_85_right_finger_tip_link");
        touch_links.push_back(prefix + "robotiq_85_right_inner_knuckle_link");
        touch_links.push_back(prefix + "robotiq_85_right_knuckle_link");
        prefix = "right_";
        touch_links.push_back(prefix + "robotiq_85_left_finger_link");
        touch_links.push_back(prefix + "robotiq_85_left_finger_tip_link");
        touch_links.push_back(prefix + "robotiq_85_left_inner_knuckle_link");
        touch_links.push_back(prefix + "robotiq_85_left_knuckle_link");
        touch_links.push_back(prefix + "robotiq_85_right_finger_link");
        touch_links.push_back(prefix + "robotiq_85_right_finger_tip_link");
        touch_links.push_back(prefix + "robotiq_85_right_inner_knuckle_link");
        touch_links.push_back(prefix + "robotiq_85_right_knuckle_link");

        // Add the end effector link of the other arm
        touch_links.push_back(arm_move_group_B.getEndEffectorLink());

        attached_object.touch_links = touch_links;
        
        // Define object as attached
        attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;
        bool attach_success_dual = (planning_scene_interface_dual.applyAttachedCollisionObject(attached_object) == moveit::core::MoveItErrorCode::SUCCESS);

        // Close gripper
        gripper_move_group_dual.setNamedTarget("Close");
        bool gripper_success = (gripper_move_group_dual.move() == moveit::core::MoveItErrorCode::SUCCESS);

        // Check both operations for overall success
        if (attach_success_dual && gripper_success) {
            RCLCPP_INFO(LOGGER, "Successfully grasped object");
            current_state_ = State::PLAN_TO_LIFT;
        } else {
            if (!attach_success_dual) {
                RCLCPP_ERROR(LOGGER, "Failed to attach object");
            }
            if (!gripper_success) {
                RCLCPP_ERROR(LOGGER, "Failed to close gripper");
            }
            current_state_ = State::FAILED;
        }
        
        return true;
    }

    bool planToLift() {
        // Get current poses - just lift straight up from where we are
        auto current_pose_A = arm_move_group_A.getCurrentPose().pose;
        auto current_pose_B = arm_move_group_B.getCurrentPose().pose;

        // STEP 1: Just lift straight up first (maintaining current x,y position)
        // This avoids complex diagonal motion that can fail when object is off-center
        geometry_msgs::msg::Pose lift_pose_A = current_pose_A;
        geometry_msgs::msg::Pose lift_pose_B = current_pose_B;

        lift_pose_A.position.z += 0.35;
        lift_pose_B.position.z += 0.35;

        RCLCPP_INFO(LOGGER, "\033[32m Press any key to plan to lift\033[0m");
        dual_arm_planner_->waitForKeyPress();
        return dual_arm_planner_->plantoTarget_dualarm(lift_pose_A, lift_pose_B, current_state_, State::MOVE_TO_LIFT, plan,
                            "Planning to lift succeeded!", true);
    }

    bool moveToLift() {
        return dual_arm_planner_->executeMovement_dualarm(current_state_, State::PLAN_TO_CENTER, plan, "Successfully lifted object",
                            "Press any key to center object");
    }

    bool planToCenter() {
        // STEP 2: Center the object at (0, 0) by moving both arms horizontally
        // Get current poses after lift
        auto current_pose_A = arm_move_group_A.getCurrentPose().pose;
        auto current_pose_B = arm_move_group_B.getCurrentPose().pose;

        // Calculate current object center and gripper distance
        double current_center_x = (current_pose_A.position.x + current_pose_B.position.x) / 2.0;
        double current_center_y = (current_pose_A.position.y + current_pose_B.position.y) / 2.0;

        double gripper_distance = std::sqrt(
            std::pow(current_pose_A.position.x - current_pose_B.position.x, 2) +
            std::pow(current_pose_A.position.y - current_pose_B.position.y, 2) +
            std::pow(current_pose_A.position.z - current_pose_B.position.z, 2)
        );

        // Calculate the offset needed to center the object
        double offset_x = -current_center_x;
        double offset_y = -current_center_y;
        double total_offset = std::sqrt(offset_x * offset_x + offset_y * offset_y);

        RCLCPP_INFO(LOGGER, "Centering object from (%.3f, %.3f) to (0.0, 0.0), gripper distance: %.3f, total offset: %.3f",
                    current_center_x, current_center_y, gripper_distance, total_offset);

        geometry_msgs::msg::Pose center_pose_A = current_pose_A;
        geometry_msgs::msg::Pose center_pose_B = current_pose_B;

        // Strategy 1: For very small offsets, try direct centering
        if (total_offset < 0.04) {
            center_pose_A.position.x += offset_x;
            center_pose_A.position.y += offset_y;
            center_pose_B.position.x += offset_x;
            center_pose_B.position.y += offset_y;

            RCLCPP_INFO(LOGGER, "Attempting direct centering (very small offset)");
            return dual_arm_planner_->plantoTarget_dualarm(center_pose_A, center_pose_B, current_state_, State::MOVE_TO_CENTER, plan,
                                "Planning to center succeeded!", true);
        }

        // Strategy 2: For small-medium offsets, use 20% incremental steps
        if (total_offset < 0.10) {
            center_pose_A.position.x += offset_x * 0.20;
            center_pose_A.position.y += offset_y * 0.20;
            center_pose_B.position.x += offset_x * 0.20;
            center_pose_B.position.y += offset_y * 0.20;

            RCLCPP_INFO(LOGGER, "Attempting incremental centering (20%% step, %.3fm)", total_offset * 0.20);
            return dual_arm_planner_->plantoTarget_dualarm(center_pose_A, center_pose_B, current_state_, State::MOVE_TO_CENTER, plan,
                                "Planning to center succeeded!", true);
        }

        // Strategy 3: For larger offsets, use very small fixed-size steps
        // Reduce step size further if we've had multiple attempts
        double base_step = 0.025;  // Reduced from 0.04
        if (centering_attempts > 3) {
            base_step = 0.015;  // Even smaller if struggling
        }

        double step_size = std::min(base_step, total_offset * 0.2);
        double scale_factor = step_size / total_offset;

        center_pose_A.position.x += offset_x * scale_factor;
        center_pose_A.position.y += offset_y * scale_factor;
        center_pose_B.position.x += offset_x * scale_factor;
        center_pose_B.position.y += offset_y * scale_factor;

        RCLCPP_INFO(LOGGER, "Attempting small-step centering (%.1f%% step, %.3fm)", scale_factor * 100, step_size);
        return dual_arm_planner_->plantoTarget_dualarm(center_pose_A, center_pose_B, current_state_, State::MOVE_TO_CENTER, plan,
                            "Planning to center succeeded!", true);
    }

    bool moveToCenter() {
        bool success = dual_arm_planner_->executeMovement_dualarm(current_state_, State::PLAN_TO_CENTER, plan, "Centering step completed",
                            "");

        if (!success) {
            return false;
        }

        centering_attempts++;

        // Check if we've reached the center (within tolerance)
        auto current_pose_A = arm_move_group_A.getCurrentPose().pose;
        auto current_pose_B = arm_move_group_B.getCurrentPose().pose;

        double current_center_x = (current_pose_A.position.x + current_pose_B.position.x) / 2.0;
        double current_center_y = (current_pose_A.position.y + current_pose_B.position.y) / 2.0;
        double distance_from_center = std::sqrt(current_center_x * current_center_x + current_center_y * current_center_y);

        RCLCPP_INFO(LOGGER, "Current center: (%.3f, %.3f), distance from target: %.3f (attempt %d)",
                    current_center_x, current_center_y, distance_from_center, centering_attempts);

        // Check for max attempts to prevent infinite loop
        if (centering_attempts > 20) {
            RCLCPP_WARN(LOGGER, "Reached max centering attempts. Proceeding with current position (%.3f m from center)",
                        distance_from_center);
            centering_attempts = 0;
            RCLCPP_INFO(LOGGER, "\033[32m Press any key to straighten arms\033[0m");
            dual_arm_planner_->waitForKeyPress();
            current_state_ = State::PLAN_TO_STRAIGHTEN;
            return true;
        }

        // If we're close enough to center (within 8cm), proceed to next state
        // Relaxed tolerance since precise centering isn't critical
        if (distance_from_center < 0.08) {
            RCLCPP_INFO(LOGGER, "Successfully centered object (within tolerance)");
            centering_attempts = 0;
            RCLCPP_INFO(LOGGER, "\033[32m Press any key to straighten arms\033[0m");
            dual_arm_planner_->waitForKeyPress();
            current_state_ = State::PLAN_TO_STRAIGHTEN;
        } else {
            // Continue centering with another iteration
            RCLCPP_INFO(LOGGER, "Continuing centering process (%.3f m remaining)", distance_from_center);
            current_state_ = State::PLAN_TO_CENTER;
        }

        return true;
    }

    bool planToStraighten() {
        // STEP 3: Rotate/straighten the arms for the 360-degree scan
        double yaw = object_params_.rotation_angle * M_PI / 180.0;

        // Straighten out the arms for 360 rotation
        if (go_to_next_grasp) {
            dual_arm_planner_->rotate(0, 0, M_PI/2 - yaw, rotated_pose1, rotated_pose2);
        } else {
            dual_arm_planner_->rotate(0, 0, -yaw, rotated_pose1, rotated_pose2);
        }

        // Re-center after rotation (just like the original planToLift does)
        // Calculate gripper distance after rotation
        double gripper_distance = std::sqrt(
            std::pow(rotated_pose1.position.x - rotated_pose2.position.x, 2) +
            std::pow(rotated_pose1.position.y - rotated_pose2.position.y, 2) +
            std::pow(rotated_pose1.position.z - rotated_pose2.position.z, 2)
        );

        // Center both grippers around (0, 0), separated by current distance
        double half_distance = gripper_distance / 2.0;
        rotated_pose1.position.x = half_distance;
        rotated_pose2.position.x = -half_distance;

        // Center in y
        rotated_pose1.position.y = 0.0;
        rotated_pose2.position.y = 0.0;

        RCLCPP_INFO(LOGGER, "Straightening arms for 360-degree rotation and re-centering");

        return dual_arm_planner_->plantoTarget_dualarm(rotated_pose1, rotated_pose2, current_state_, State::MOVE_TO_STRAIGHTEN, plan,
                            "Planning to straighten succeeded!", true);
    }

    bool moveToStraighten() {
        return dual_arm_planner_->executeMovement_dualarm(current_state_, State::ROTATE_EE, plan, "Successfully straightened arms",
                            "Press any key to start 3d capture");
    }

    bool rotateEndEffectors() {
        capture_active_ = true; // set capture active flag to true
        const int left_wrist_joint = 6;  
        const int right_wrist_joint = 13; 
       
        // Get current state
        auto current_state = arm_move_group_dual.getCurrentState(10.0);
        if (!current_state) {
            RCLCPP_ERROR(LOGGER, "Failed to get current state");
            current_state_ = State::FAILED;
            return true;
        }
        
        std::vector<double> start_joints = arm_move_group_dual.getCurrentJointValues();
        
        // Set conservative limits for rotation
        arm_move_group_dual.setMaxVelocityScalingFactor(0.15);
        arm_move_group_dual.setMaxAccelerationScalingFactor(0.15);
        
        const int steps = 16;
        const double rotation_per_step = (M_PI / 8.0);  // 22.5 degrees
       
        for (int step = 1; step <= steps; step++) {
            RCLCPP_INFO(LOGGER, "Rotation step %d/%d (%.1f degrees total)",
                       step, steps, (step * 22.5));
           
            // Get fresh joint values each iteration
            std::vector<double> target_joints = arm_move_group_dual.getCurrentJointValues();
            
            // Increment wrist joints
            target_joints[left_wrist_joint] += rotation_per_step;
            target_joints[right_wrist_joint] -= rotation_per_step;
    
            // Use setJointValueTarget and move() instead of plan/execute
            arm_move_group_dual.setJointValueTarget(target_joints);
            arm_move_group_dual.setPlanningTime(5.0);
            
            // Use the synchronous move() call instead of plan + execute
            moveit::core::MoveItErrorCode result = arm_move_group_dual.move();
            
            if (result != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_ERROR(LOGGER, "Failed to move in rotation step %d, error code: %d", 
                            step, result.val);
                current_state_ = State::FAILED;
                return true;
            }
            
            RCLCPP_INFO(LOGGER, "Step %d completed successfully", step);
        }
        
        // Restore normal velocity scaling
        arm_move_group_dual.setMaxVelocityScalingFactor(0.4);
        arm_move_group_dual.setMaxAccelerationScalingFactor(0.3);

        current_state_ = State::PLAN_TO_PLACE_XY;
        capture_active_ = false; // set capture active flag to false
        return true;
    }

    bool planToPlaceXY() {
        // STEP 1: Move to placement XY position at current height
        ObjectParameters place_params = createPlacementParams();

        if (go_to_next_grasp) {
            target_pose_A = place_params.second_left_grasp_pose;
            target_pose_B = place_params.second_right_grasp_pose;
        } else {
            target_pose_A = place_params.left_grasp_pose;
            target_pose_B = place_params.right_grasp_pose;
        }

        // Use current Z height, not placement Z yet
        auto current_pose_A = arm_move_group_A.getCurrentPose().pose;
        auto current_pose_B = arm_move_group_B.getCurrentPose().pose;

        target_pose_A.position.z = current_pose_A.position.z;
        target_pose_B.position.z = current_pose_B.position.z;

        RCLCPP_INFO(LOGGER, "Moving to placement XY position at current height z=%.4f", current_pose_A.position.z);
        RCLCPP_INFO(LOGGER, "Target XY poses:");
        RCLCPP_INFO(LOGGER, "  Left: x=%.4f, y=%.4f, z=%.4f",
                    target_pose_A.position.x, target_pose_A.position.y, target_pose_A.position.z);
        RCLCPP_INFO(LOGGER, "  Right: x=%.4f, y=%.4f, z=%.4f",
                    target_pose_B.position.x, target_pose_B.position.y, target_pose_B.position.z);

        RCLCPP_INFO(LOGGER, "\033[32m Press any key to move to placement XY\033[0m");
        dual_arm_planner_->waitForKeyPress();
        return dual_arm_planner_->plantoTarget_dualarm(target_pose_A, target_pose_B, current_state_, State::MOVE_TO_PLACE_XY, plan,
                             "Planning to placement XY succeeded!", true);
    }

    bool moveToPlaceXY() {
        return dual_arm_planner_->executeMovement_dualarm(current_state_, State::PLAN_TO_PLACE, plan, "Reached placement XY position",
                            "Press any key to lower to placement");
    }

    bool planToPlace() {
        // STEP 2: Now lower straight down to placement height
        ObjectParameters place_params = createPlacementParams();

        if (go_to_next_grasp) {
            target_pose_A = place_params.second_left_grasp_pose;
            target_pose_B = place_params.second_right_grasp_pose;
        } else {
            target_pose_A = place_params.left_grasp_pose;
            target_pose_B = place_params.right_grasp_pose;
        }

        target_pose_A.position.z += (place_params.approach_offset + place_params.grasp_offset);
        target_pose_B.position.z += (place_params.approach_offset + place_params.grasp_offset);

        RCLCPP_INFO(LOGGER, "Lowering to placement height z=%.4f", target_pose_A.position.z);
        RCLCPP_INFO(LOGGER, "Final placement poses:");
        RCLCPP_INFO(LOGGER, "  Left: x=%.4f, y=%.4f, z=%.4f",
                    target_pose_A.position.x, target_pose_A.position.y, target_pose_A.position.z);
        RCLCPP_INFO(LOGGER, "  Right: x=%.4f, y=%.4f, z=%.4f",
                    target_pose_B.position.x, target_pose_B.position.y, target_pose_B.position.z);

        RCLCPP_INFO(LOGGER, "\033[32m Press any key to plan to lower\033[0m");
        dual_arm_planner_->waitForKeyPress();
        return dual_arm_planner_->plantoTarget_dualarm(target_pose_A, target_pose_B, current_state_, State::MOVE_TO_PLACE, plan,
                             "Planning to place succeeded!", true);
    }

    bool moveToPlace() {
        return dual_arm_planner_->executeMovement_dualarm(current_state_, State::PLACE, plan, "Successfully moved to place position",
                             "Press any key to release object");
    }
    
    bool Place() {
        RCLCPP_INFO(LOGGER, "Releasing object...");

        // Open gripper to release object
        gripper_move_group_A.setNamedTarget("Open");
        gripper_move_group_B.setNamedTarget("Open");
        bool gripper_success_A = (gripper_move_group_A.move() == moveit::core::MoveItErrorCode::SUCCESS);
        bool gripper_success_B = (gripper_move_group_B.move() == moveit::core::MoveItErrorCode::SUCCESS);

        if (!gripper_success_A || !gripper_success_B) {
            RCLCPP_ERROR(LOGGER, "Failed to open gripper");
            current_state_ = State::FAILED;
            return true;
        }

        gripper_move_group_A.detachObject(attached_object.object.id);
        gripper_move_group_B.detachObject(attached_object.object.id);

        attached_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        planning_scene_interface_dual.applyAttachedCollisionObject(attached_object);

        ObjectParameters placed_params = createPlacementParams();

        object_params_ = placed_params;
        RCLCPP_INFO(LOGGER, "Updated object_params_ to reflect placement at (%.1f, %.1f, %.4f) with %.1f° rotation",
                    object_params_.x, object_params_.y, object_params_.z, object_params_.rotation_angle);

        current_state_ = State::PLAN_RETRACT;
        return true;
    }

    bool planToRetract() {
        
        // Get current poses and just lift straight up
        auto current_pose_A = arm_move_group_A.getCurrentPose().pose;
        auto current_pose_B = arm_move_group_B.getCurrentPose().pose;
        
        current_pose_A.position.z -= object_params_.grasp_offset - 0.13;
        current_pose_B.position.z -= object_params_.grasp_offset - 0.13;
        
        RCLCPP_INFO(LOGGER, "\033[32m Press any key to retract arms\033[0m");
        dual_arm_planner_->waitForKeyPress();
        
        return dual_arm_planner_->plantoTarget_dualarm(
            current_pose_A, current_pose_B, 
            current_state_, State::MOVE_RETRACT,
            plan, "Lifted from placement!",
            false);
    }

    bool moveRetract() {
        RCLCPP_INFO(LOGGER, "Executing lift movement...");

        // Reuse fsm to plan to next grasp
        if (go_to_next_grasp) {
            go_to_next_grasp = false;
        } else {
            go_to_next_grasp = true;
        }
        
        if (go_to_next_grasp) {
            return dual_arm_planner_->executeMovement_dualarm(current_state_, State::PLAN_TO_OBJECT,
                plan, "Arms lifted successfully",
                "Press any key to go to next grasp points");
        } else {
            return dual_arm_planner_->executeMovement_dualarm(current_state_, State::PLAN_TO_HOME,
                plan, "Arms lifted successfully",
                "Press any key to go to home");
        }
    }

    bool closegripper() {
        //state for gripping action
        gripper_move_group_dual.setNamedTarget("Close");
        bool success = (gripper_move_group_dual.move() == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success) {
            RCLCPP_INFO(LOGGER, "Successfully closed gripper");
            current_state_ = State::PLAN_TO_HOME;  
        } else {
            RCLCPP_ERROR(LOGGER, "Failed to closed gripper");
            current_state_ = State::FAILED;
        }
        
        return true;
    }

    bool planToHome() {

        // Close gripper
        gripper_move_group_dual.setNamedTarget("Close");
        gripper_move_group_dual.move();

        RCLCPP_INFO(LOGGER, "\033[32m Press any key to plan to home\033[0m");
        dual_arm_planner_->waitForKeyPress();

        // Give time for the robot state monitor to update
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // Clear any previous targets
        arm_move_group_dual.clearPoseTargets();

        // Get and verify current state
        auto current_state = arm_move_group_dual.getCurrentState(5.0);
        if (!current_state) {
            RCLCPP_ERROR(LOGGER, "Failed to get current robot state");
            current_state_ = State::FAILED;
            return true;
        }

        arm_move_group_dual.setStartState(*current_state);

        auto current_pose = arm_move_group_A.getCurrentPose().pose;
        RCLCPP_INFO(LOGGER, "Planning from current lifted position: z=%.4f", current_pose.position.z);

        arm_move_group_dual.setNamedTarget("Home");

        // Try multiple planners in order of preference
        std::vector<std::string> planners = {
            "RRTConnectkConfigDefault",
            "RRTstarkConfigDefault",
            "PRMkConfigDefault"
        };

        arm_move_group_dual.setPlanningTime(20.0);  // Increased timeout
        arm_move_group_dual.setNumPlanningAttempts(15);  // More attempts

        bool success = false;
        for (const auto& planner : planners) {
            arm_move_group_dual.setPlannerId(planner);
            RCLCPP_INFO(LOGGER, "Trying planner: %s", planner.c_str());

            success = (arm_move_group_dual.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (success) {
                RCLCPP_INFO(LOGGER, "Planning to home succeeded with %s!", planner.c_str());
                break;
            } else {
                RCLCPP_WARN(LOGGER, "Planning with %s failed, trying next planner...", planner.c_str());
            }
        }

        if (success) {
            RCLCPP_INFO(LOGGER, "\033[32m Press 'r' to replan, or any other key to execute \033[0m");
            char input = dual_arm_planner_->waitForKeyPress();

            if (input == 'r' || input == 'R') {
                // Stay in current state to replan
                return true;
            } else {
                current_state_ = State::MOVE_TO_HOME;
            }
        } else {
            RCLCPP_ERROR(LOGGER, "Planning to home failed with all planners!");
            current_state_ = State::FAILED;
        }

        return true;
    }

    bool moveToHome() {
        RCLCPP_INFO(LOGGER, "Executing movement to home...");
    
        auto result = arm_move_group_dual.execute(plan);
        
        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(LOGGER, "Successfully moved to home position");
            current_state_ = State::SUCCEEDED;
        } else {
            RCLCPP_ERROR(LOGGER, "Failed to execute movement to home");
            current_state_ = State::FAILED;
        }
        
        return true;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("motion_planning_dual_arms", node_options);

    rclcpp::Parameter sim_time_param("use_sim_time", true);
    move_group_node->set_parameter(sim_time_param);

    // Get the parameter value
    std::string object_type_str = "cylinder";  // default value
    if (move_group_node->has_parameter("object_type")) {
        move_group_node->get_parameter("object_type", object_type_str);
    }
    
    // Map string to ObjectType enum
    ObjectType object_type;
    if (object_type_str == "tbar") {
        object_type = ObjectType::TBAR;
    } else {
        object_type = ObjectType::CYLINDER_WITH_SPOKES;
    }

    move_group_node->declare_parameter("robot_description_kinematics.left_arm.kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
    move_group_node->declare_parameter("robot_description_kinematics.right_arm.kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread spinner([&executor]() { executor.spin(); });

    // Initialize MoveIt interfaces
    static const std::string PLANNING_GROUP_A = "left_arm";
    static const std::string GRIPPER_GROUP_A = "left_gripper";
    static const std::string PLANNING_GROUP_B = "right_arm";
    static const std::string GRIPPER_GROUP_B = "right_gripper";
    static const std::string PLANNING_GROUP_dual = "both_arms";
    static const std::string GRIPPER_GROUP_dual = "both_grippers";

    // Create and run FSM
    MotionPlanningFSM fsm(move_group_node, 
        PLANNING_GROUP_A, 
        GRIPPER_GROUP_A,
        PLANNING_GROUP_B,
        GRIPPER_GROUP_B,
        PLANNING_GROUP_dual,
        GRIPPER_GROUP_dual,
        object_type);
    
    // FSM execution loop
    while (rclcpp::ok() && fsm.execute()) {
        // Give time for things to process
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}