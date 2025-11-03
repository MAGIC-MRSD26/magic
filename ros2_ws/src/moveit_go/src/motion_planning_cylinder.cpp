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

#include "object_definitions.hpp"

// This is a finite state machine for Kinova Gen3 7DOF arm
static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_planning");

enum class State {
    HOME,
    PLAN_TO_OBJECT,
    MOVE_TO_OBJECT,
    OPEN_GRIPPER,
    PLAN_TO_GRASP,
    MOVE_TO_GRASP,
    GRASP,
    PLAN_TO_LIFT,
    MOVE_TO_LIFT,
    PLAN_TO_ROTATE_BACK,
    PLAN_TO_ROTATE_FRONT,
    MOVE_TO_ROTATE,
    PLAN_TO_PLACE,
    MOVE_TO_PLACE,
    PLACE,
    PLAN_TO_HOME,
    MOVE_TO_HOME,
    SUCCEEDED,
    FAILED
};

class MotionPlanningFSM {
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
        
        // Create object parameters based on selected type
        arm_move_group_A.setMaxVelocityScalingFactor(0.6); // Increase from default
        arm_move_group_B.setMaxVelocityScalingFactor(0.6);
        arm_move_group_dual.setMaxVelocityScalingFactor(0.5); // More conservative for dual-arm

        // For kinematic chain movements specifically (after grasping)
        arm_move_group_dual.setMaxVelocityScalingFactor(0.4); // Safe but still faster
        arm_move_group_dual.setMaxAccelerationScalingFactor(0.3);
        
        // Create subscription to the bin pose topic
        pose_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
            "/aruco_poses", 10, 
            std::bind(&MotionPlanningFSM::binPoseCallback, this, std::placeholders::_1));

        pose_received_ = false;
        
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

            case State::PLAN_TO_ROTATE_BACK:
                return planToRotateBack();

            case State::PLAN_TO_ROTATE_FRONT:
                return planToRotateFront();

            case State::MOVE_TO_ROTATE:
                return moveToRotate();

            case State::PLAN_TO_PLACE:
                return planToPlace();

            case State::MOVE_TO_PLACE:
                return moveToPlace();

            case State::PLACE:
                return Place();

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
    //pointer for ros2 node
    rclcpp::Node::SharedPtr node_;

    //arm planning group names- variables
    std::string arm_planning_group_A;
    std::string gripper_planning_group_A;
    std::string arm_planning_group_B;
    std::string gripper_planning_group_B;
    std::string arm_planning_group_dual;
    std::string gripper_planning_group_dual;

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

    // for later when we reuse go to Lift state
    bool go_to_home = false;

    moveit_msgs::msg::AttachedCollisionObject attached_object;

    // Rotation policy variables
    geometry_msgs::msg::Pose rotated_pose1;
    geometry_msgs::msg::Pose rotated_pose2;
    int rotations = 0;

    // Object params
    ObjectType selected_object_type_;
    ObjectParameters object_params_;

    // Bin subscription 
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_subscription_;
    geometry_msgs::msg::Pose object_pose_;
    bool pose_received_;
    
    
    // Callback for the bin pose subscriber
    void binPoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        object_pose_ = msg->poses[0];
        pose_received_ = true;
        RCLCPP_INFO(LOGGER, "Received object pose: x=%f, y=%f, z=%f", 
                object_pose_.position.x,
                object_pose_.position.y,
                object_pose_.position.z);

        pose_subscription_.reset();
    }

    char waitForKeyPress() {
        // Make sure stdin is in raw mode to get a single keypress
        system("stty raw");
        char input = getchar();
        system("stty cooked");
        
        std::cout << std::endl;  // Add newline after key press
        return input;
    }

    bool plantoTarget_dualarm(geometry_msgs::msg::Pose pose1, geometry_msgs::msg::Pose pose2, State next_state, const std::string& planning_message = "Planning succeeded!") {
        
        static int plan_attempts = 0;
        const int max_plan_attempts = 5;

        std::vector<double> combined_joint_positions;

        // Get the current state with timeout
        RCLCPP_INFO(LOGGER, "Getting current robot state...");
        moveit::core::RobotStatePtr current_state_left = arm_move_group_A.getCurrentState(5.0);
        if (!current_state_left) {
            RCLCPP_ERROR(LOGGER, "Failed to get current state for left arm");
            current_state_ = State::FAILED;
            return true;
        }
        
        moveit::core::RobotStatePtr current_state_right = arm_move_group_B.getCurrentState(5.0);
        if (!current_state_right) {
            RCLCPP_ERROR(LOGGER, "Failed to get current state for right arm");
            current_state_ = State::FAILED;
            return true;
        }
        
        RCLCPP_INFO(LOGGER, "Successfully retrieved current states");
        bool success = false;

        // Get joint model
        const moveit::core::JointModelGroup* joint_model_group_left = current_state_left->getJointModelGroup("left_arm");
        const moveit::core::JointModelGroup* joint_model_group_right = current_state_right->getJointModelGroup("right_arm");
 
        //Compute IK
        RCLCPP_INFO(LOGGER, "Computing IK solutions...");
        bool ik_left = current_state_left->setFromIK(joint_model_group_left, pose1, arm_move_group_A.getEndEffectorLink(), 15.0);
        bool ik_right = current_state_right->setFromIK(joint_model_group_right, pose2, arm_move_group_B.getEndEffectorLink(), 15.0);
        
        if (ik_left && ik_right) {
            RCLCPP_INFO(LOGGER, "IK successful for both arms");
            
            // Initialize the vectors first, then pass them as output parameters
            std::vector<double> left_joint_positions;
            std::vector<double> right_joint_positions;
            
            // Get joint positions for left arm
            current_state_left->copyJointGroupPositions(joint_model_group_left, left_joint_positions);
            
            // Get joint positions for right arm
            current_state_right->copyJointGroupPositions(joint_model_group_right, right_joint_positions);
            
            // Combine the joint positions
            combined_joint_positions = left_joint_positions;
            combined_joint_positions.insert(combined_joint_positions.end(), 
                                        right_joint_positions.begin(), 
                                        right_joint_positions.end());
                                        
            RCLCPP_INFO(LOGGER, "Joint positions combined successfully, size=%zu", combined_joint_positions.size());

            arm_move_group_dual.setJointValueTarget(combined_joint_positions);

            // Set planning parameters
            arm_move_group_dual.setPlanningTime(5.0 + (5.0 * plan_attempts));

            // Plan with the dual arm group directly
            success = (arm_move_group_dual.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        } else {
            RCLCPP_WARN(LOGGER, "IK failed for one or both arms");
            if (!ik_left) RCLCPP_WARN(LOGGER, "Left arm IK failed");
            if (!ik_right) RCLCPP_WARN(LOGGER, "Right arm IK failed");
        }

        if (success) {
            // Reset attempt counter on success
            plan_attempts = 0;

            RCLCPP_INFO(LOGGER, "%s", planning_message.c_str());

            RCLCPP_INFO(LOGGER, "\033[32m 1) Press 'r' to replan, OR 2) press any other key to execute the plan \033[0m");
            char input = waitForKeyPress();

            if (input == 'r' || input == 'R') {
                RCLCPP_INFO(LOGGER, "Replanning requested");
                return true; // Stay in same state for replanning
                } else {
                RCLCPP_INFO(LOGGER, "Executing plan");
                current_state_ = next_state;
                return true;
            }
        } else {
            // Planning failed
            plan_attempts++;

            if (plan_attempts < max_plan_attempts) {
                RCLCPP_WARN(LOGGER, "Planning attempt %d/%d, retrying...", 
                plan_attempts, max_plan_attempts);
                return true; // Stay in current state to try again
                } else {
                // After max attempts, ask the user what to do
                RCLCPP_ERROR(LOGGER, "Failed to plan after %d attempts", max_plan_attempts);
                current_state_ = State::FAILED;
                return true;
            }
        }
    }

    bool executeMovement_dualarm(State next_state, const std::string& success_message, const std::string& prompt_message = "") {
        bool success = (arm_move_group_dual.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success) {
            RCLCPP_INFO(LOGGER, "%s", success_message.c_str());
            
            if (!prompt_message.empty()) {
                RCLCPP_INFO(LOGGER, "\033[32m %s\033[0m", prompt_message.c_str());
                waitForKeyPress();
            }
            current_state_ = next_state;
        } else {
            RCLCPP_ERROR(LOGGER, "Failed to execute movement");
            current_state_ = State::FAILED;
        }
        
        return true;
    }

    void rotate(double roll, double pitch, double yaw) {

        // Get current poses
        geometry_msgs::msg::PoseStamped current_pose1 = arm_move_group_A.getCurrentPose();
        geometry_msgs::msg::PoseStamped current_pose2 = arm_move_group_B.getCurrentPose();
        
        rotated_pose1 = current_pose1.pose;
        rotated_pose2 = current_pose2.pose;
        
        // Calculate bin's center point (midpoint between grippers)
        geometry_msgs::msg::Point bin_center;
        bin_center.x = 0.0;
        bin_center.y = 0.0;
        bin_center.z = 1.194;
        
        RCLCPP_INFO(LOGGER, "Bin center: x=%.3f, y=%.3f, z=%.3f", 
                    bin_center.x, bin_center.y, bin_center.z);
        
        // Create rotation quaternion - define the rotation axis
        tf2::Quaternion rotation_quat;
        rotation_quat.setRPY(roll, pitch, yaw); // Rotate 30 degrees around X axis
        
        // Calculate vectors from bin center to each gripper
        tf2::Vector3 vec_to_gripper1(
            current_pose1.pose.position.x - bin_center.x,
            current_pose1.pose.position.y - bin_center.y,
            current_pose1.pose.position.z - bin_center.z
        );
        
        tf2::Vector3 vec_to_gripper2(
            current_pose2.pose.position.x - bin_center.x,
            current_pose2.pose.position.y - bin_center.y,
            current_pose2.pose.position.z - bin_center.z
        );
        
        // Create rotation matrix from quaternion
        tf2::Matrix3x3 rotation_matrix(rotation_quat);
        
        // Rotate the vectors - this moves the grip points around the fixed center
        tf2::Vector3 rotated_vec1 = rotation_matrix * vec_to_gripper1;
        tf2::Vector3 rotated_vec2 = rotation_matrix * vec_to_gripper2;
        
        // Apply rotated vectors to get new positions
        // The center stays the same, only the gripper positions change
        rotated_pose1.position.x = bin_center.x + rotated_vec1.x();
        rotated_pose1.position.y = bin_center.y + rotated_vec1.y();
        rotated_pose1.position.z = bin_center.z + rotated_vec1.z();
        
        rotated_pose2.position.x = bin_center.x + rotated_vec2.x();
        rotated_pose2.position.y = bin_center.y + rotated_vec2.y();
        rotated_pose2.position.z = bin_center.z + rotated_vec2.z();
        
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
        
        // Add diagnostic output for the rotated positions
        RCLCPP_INFO(LOGGER, "Left arm new position: x=%.3f, y=%.3f, z=%.3f", 
                    rotated_pose1.position.x, rotated_pose1.position.y, rotated_pose1.position.z);
        RCLCPP_INFO(LOGGER, "Right arm new position: x=%.3f, y=%.3f, z=%.3f", 
                    rotated_pose2.position.x, rotated_pose2.position.y, rotated_pose2.position.z);
        
        // Set planning parameters specific for rotation
        arm_move_group_dual.setMaxVelocityScalingFactor(0.2); // Slower for rotation
        arm_move_group_dual.setMaxAccelerationScalingFactor(0.1);
        arm_move_group_dual.setPlanningTime(15.0); // Give more planning time
    }


    /*//////////////////////////////////////////

    START OF FSM FUNCTIONS

    *//////////////////////////////////////////

    bool home() {
        // Add object to the planning scene
        RCLCPP_INFO(LOGGER, "\033[32m Press any key to add object to planning scene \033[0m");
        waitForKeyPress();

        // Create object parameters based on type
        double x = 0.0, y = 0.0;
        if (pose_received_) {
            x = object_pose_.position.x;
            y = object_pose_.position.y;
        }
        
        // Use factory to create parameters and collision object
        if (selected_object_type_ == ObjectType::BIN) {
            object_params_ = ObjectFactory::createBinParameters(x, y);
        } else {
            object_params_ = ObjectFactory::createCylinderParameters(x, y);
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
        //plans to the object
        /// Use pre-calculated grasp poses from object_params_
        target_pose_A = object_params_.left_grasp_pose;
        target_pose_B = object_params_.right_grasp_pose;
        
        // Adjust Z for approach
        target_pose_A.position.z -= 0.1;
        target_pose_B.position.z -= 0.1;
        RCLCPP_INFO(LOGGER, "Left arm target pose x: %f y: %f z: %f", target_pose_A.position.x, target_pose_A.position.y, target_pose_A.position.z);
        RCLCPP_INFO(LOGGER, "Right arm target pose x: %f y: %f z: %f", target_pose_B.position.x, target_pose_B.position.y, target_pose_B.position.z);
        
        RCLCPP_INFO(LOGGER, "\033[32m Press any key to plan to object \033[0m");
        waitForKeyPress();
        return plantoTarget_dualarm(target_pose_A, target_pose_B, State::MOVE_TO_OBJECT, "Planning to object succeeded!");
    }

    bool moveToObject() {
        //execute the planned trajectory
        return executeMovement_dualarm(State::OPEN_GRIPPER, "Successfully moved to object position", 
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
        //next state where we plan for grasp point
        // Reuse target_pose with new z pos
        target_pose_A.position.z -= 0.19;
        target_pose_B.position.z -= 0.19;
        
        RCLCPP_INFO(LOGGER, "\033[32m Press any key to plan to grasp\033[0m");
        waitForKeyPress();
        return plantoTarget_dualarm(target_pose_A, target_pose_B, State::MOVE_TO_GRASP, 
                          "Planning to grasp succeeded!");
    }

    bool moveToGrasp() {
        //state for executing the trajectory for moving to grasp point
        return executeMovement_dualarm(State::GRASP, "Successfully moved to grasp pose", 
                             "Press any key to grasp object");
    }

    bool Grasp() {

        // Now we attach the object
        attached_object.object.id = object_params_.object_id;
        //cannot link both end effectors so linking left arm alone
        attached_object.link_name = arm_move_group_A.getEndEffectorLink();
        
        // Define which links are allowed to touch the bin
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
        geometry_msgs::msg::PoseStamped current_pose1 = arm_move_group_A.getCurrentPose();
        geometry_msgs::msg::PoseStamped current_pose2 = arm_move_group_B.getCurrentPose();

        geometry_msgs::msg::Pose lift_pose1 = current_pose1.pose;
        geometry_msgs::msg::Pose lift_pose2 = current_pose2.pose;

        // Add to z position
        if (go_to_home) {
            lift_pose1.position.z += 0.2;  // Lift by 20cm
            lift_pose2.position.z += 0.2;  // Lift by 20cm
        } else {
            lift_pose1 = object_params_.left_grasp_pose;
            lift_pose2 = object_params_.right_grasp_pose;
            lift_pose1.position.z = current_pose1.pose.position.z + 0.35;
            lift_pose2.position.z = current_pose2.pose.position.z + 0.35;
        }
                
        RCLCPP_INFO(LOGGER, "\033[32m Press any key to plan to lift\033[0m");
        waitForKeyPress();
        return plantoTarget_dualarm(lift_pose1, lift_pose2, State::MOVE_TO_LIFT, 
                            "Planning to lift succeeded!");
    }

    bool moveToLift() {
        if (go_to_home) {
            return executeMovement_dualarm(State::PLAN_TO_HOME, "Successfully moved to lift position",
                                "Press any key to go to home");
        } else {
            return executeMovement_dualarm(State::PLAN_TO_ROTATE_BACK, "Successfully moved to lift position",
                                "Press any key to plan to rotation");
        }
    }

    bool planToRotateBack() {

        rotate(-M_PI/4, 0, 0); // Rotate 30 degrees around X axis
        return plantoTarget_dualarm(rotated_pose1, rotated_pose2, State::MOVE_TO_ROTATE, 
                        "Planning rotation succeeded!");
    }

    bool planToRotateFront() {

        rotate(M_PI/4, 0, 0); // Rotate 30 degrees around X axis
        return plantoTarget_dualarm(rotated_pose1, rotated_pose2, State::MOVE_TO_ROTATE, 
                        "Planning rotation succeeded!");
    }

    bool moveToRotate() {
        rotations++;
        RCLCPP_INFO(LOGGER, "Current rotation count: %d", rotations);
        if (rotations < 2) {
            return executeMovement_dualarm(State::PLAN_TO_ROTATE_BACK, "Successfully rotated",
                                "Press any key to plan to rotate back");
        } else if (rotations >= 2 && rotations < 6) {
            return executeMovement_dualarm(State::PLAN_TO_ROTATE_FRONT, "Successfully rotated",
                            "Press any key to plan to rotate front");
        } else if (rotations >= 6 && rotations < 8) {
            return executeMovement_dualarm(State::PLAN_TO_ROTATE_BACK, "Successfully rotated",
                "Press any key to plan to rotate back");
        } else {
            return executeMovement_dualarm(State::PLAN_TO_PLACE, "Successfully showed three sides",
                "Press any key to plan to place");
        }
        
    }

    bool planToPlace() {
        target_pose_A.position.z = 1.31;
        target_pose_B.position.z = 1.31;

        return plantoTarget_dualarm(target_pose_A, target_pose_B, State::MOVE_TO_PLACE, 
                             "Planning to place succeeded!");
    }

    bool moveToPlace() {
        return executeMovement_dualarm(State::PLACE, "Successfully moved to place position",
                             "Press any key to drop object");
    }

    bool Place() {

        // Open gripper
        gripper_move_group_dual.setNamedTarget("Open");
        gripper_move_group_dual.move();

        // Make sure to detach from planning scene interfaces
        attached_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        planning_scene_interface_dual.applyAttachedCollisionObject(attached_object);
        
        // Also try gripper detach
        gripper_move_group_A.detachObject(attached_object.object.id);
        gripper_move_group_B.detachObject(attached_object.object.id);
        
        RCLCPP_INFO(LOGGER, "Successfully dropped object");
        current_state_ = State::PLAN_TO_LIFT;
        go_to_home = true;
        
        return true;
    }

    bool planToHome() {

        // Close gripper
        gripper_move_group_dual.setNamedTarget("Close");
        gripper_move_group_dual.move();

        // Use named target instead of recorded position
        arm_move_group_dual.setNamedTarget("Home");
        arm_move_group_dual.setPlanningTime(5.0);
        bool success = (arm_move_group_dual.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            current_state_ = State::MOVE_TO_HOME;
        } else {
            current_state_ = State::FAILED;
        }
        
        return true;
    }

    bool moveToHome() {
        return executeMovement_dualarm(State::SUCCEEDED, "Successfully moved to home position");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("motion_planning_dual_arms", node_options);

    rclcpp::Parameter sim_time_param("use_sim_time", true);
    move_group_node->set_parameter(sim_time_param);

    // Select object type
    ObjectType object_type = ObjectType::CYLINDER_WITH_SPOKES;

    move_group_node->declare_parameter("robot_description_kinematics.left_arm.kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
    move_group_node->declare_parameter("robot_description_kinematics.right_arm.kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");

    move_group_node->declare_parameter("trajectory_execution.allowed_start_tolerance", 0.05);  // Increase from 0.01
    move_group_node->declare_parameter("trajectory_execution.allowed_execution_duration_scaling", 2.0);  // Default is 1.0
    move_group_node->declare_parameter("trajectory_execution.allowed_goal_duration_margin", 1.0);  // Default is 0.5

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
    while (fsm.execute()) {
        // Give time for things to process
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}