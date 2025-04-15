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
#include <tf2_eigen/tf2_eigen.h>

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
        const std::string& gripper_planning_group_dual_) 
        : node_(node),
        //initialising arm_group names
          arm_planning_group_A(arm_planning_group_A_),
          arm_planning_group_B(arm_planning_group_B_),
          arm_planning_group_dual(arm_planning_group_dual_),
        //initialising gripper group names
          gripper_planning_group_A(gripper_planning_group_A_),
          gripper_planning_group_B(gripper_planning_group_B_),
          gripper_planning_group_dual(gripper_planning_group_dual_),
        //initialising movegroups
          arm_move_group_A(node,arm_planning_group_A_),
          arm_move_group_B(node,arm_planning_group_B_),
          arm_move_group_dual(node,arm_planning_group_dual_),
        //gripper movegroup initialisation
          gripper_move_group_A(node, gripper_planning_group_A_),
          gripper_move_group_B(node, gripper_planning_group_B_),
          gripper_move_group_dual(node, gripper_planning_group_dual_),
          current_state_(State::HOME) {

        // Add this in constructor
        arm_move_group_A.setMaxVelocityScalingFactor(0.6); // Increase from default
        arm_move_group_B.setMaxVelocityScalingFactor(0.6);
        arm_move_group_dual.setMaxVelocityScalingFactor(0.5); // More conservative for dual-arm

        // For kinematic chain movements specifically (after grasping)
        arm_move_group_dual.setMaxVelocityScalingFactor(0.4); // Safe but still faster
        arm_move_group_dual.setMaxAccelerationScalingFactor(0.3);
        
        // Create subscription to the grasp pose topic
        grasp_pose_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
            "/arm_pose", 10, 
            std::bind(&MotionPlanningFSM::graspPoseCallback, this, std::placeholders::_1));

        grasp_pose_received_ = false;
        
        RCLCPP_INFO(LOGGER, "MotionPlanningFSM initialized");

        // Add object to the planning scene
        RCLCPP_INFO(LOGGER, "\033[32m Press any key to add bin to planning scene \033[0m");
        waitForKeyPress();

        // Create a bin using multiple collision objects for sides and bottom
        double bin_width = 0.38;   // X dimension (length)
        double bin_depth = 0.32;   // Y dimension (width)
        double bin_height = 0.266; // Z dimension (height)
        double wall_thickness = 0.01; // Wall thickness of the bin

        // Base position for the bin
        double bin_x = 0.0;
        double bin_y = 0.0;
        double bin_z = 1.0646; // height of table is 0.9316
        double bin_bottom_z = bin_z - (bin_height / 2) + (wall_thickness / 2);

        // Create a single collision object for the entire bin
        moveit_msgs::msg::CollisionObject bin_object;
        bin_object.id = "bin";
        bin_object.header.frame_id = "world";
        bin_object.operation = moveit_msgs::msg::CollisionObject::ADD;

        // Create a quaternion for orientation (not rotated)
        tf2::Quaternion bin_quat;
        bin_quat.setRPY(0, 0, 0);
        geometry_msgs::msg::Quaternion bin_orientation = tf2::toMsg(bin_quat);

        // 1. Bottom of the bin
        shape_msgs::msg::SolidPrimitive bottom_primitive;
        bottom_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        bottom_primitive.dimensions.resize(3);
        bottom_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = bin_width;
        bottom_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = bin_depth;
        bottom_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = wall_thickness;

        geometry_msgs::msg::Pose bottom_pose;
        bottom_pose.orientation = bin_orientation;
        bottom_pose.position.x = bin_x;
        bottom_pose.position.y = bin_y;
        bottom_pose.position.z = bin_bottom_z;

        bin_object.primitives.push_back(bottom_primitive);
        bin_object.primitive_poses.push_back(bottom_pose);

        // 2. Front wall of the bin
        shape_msgs::msg::SolidPrimitive front_primitive;
        front_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        front_primitive.dimensions.resize(3);
        front_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = bin_width;
        front_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = wall_thickness;
        front_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = bin_height;

        geometry_msgs::msg::Pose front_pose;
        front_pose.orientation = bin_orientation;
        front_pose.position.x = bin_x;
        front_pose.position.y = bin_y + (bin_depth / 2) - (wall_thickness / 2);
        front_pose.position.z = bin_z;

        bin_object.primitives.push_back(front_primitive);
        bin_object.primitive_poses.push_back(front_pose);

        // 3. Back wall of the bin
        shape_msgs::msg::SolidPrimitive back_primitive;
        back_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        back_primitive.dimensions.resize(3);
        back_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = bin_width;
        back_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = wall_thickness;
        back_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = bin_height;

        geometry_msgs::msg::Pose back_pose;
        back_pose.orientation = bin_orientation;
        back_pose.position.x = bin_x;
        back_pose.position.y = bin_y - (bin_depth / 2) + (wall_thickness / 2);
        back_pose.position.z = bin_z;

        bin_object.primitives.push_back(back_primitive);
        bin_object.primitive_poses.push_back(back_pose);

        // 4. Left wall of the bin
        shape_msgs::msg::SolidPrimitive left_primitive;
        left_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        left_primitive.dimensions.resize(3);
        left_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = wall_thickness;
        left_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = bin_depth;
        left_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = bin_height;

        geometry_msgs::msg::Pose left_pose;
        left_pose.orientation = bin_orientation;
        left_pose.position.x = bin_x - (bin_width / 2) + (wall_thickness / 2);
        left_pose.position.y = bin_y;
        left_pose.position.z = bin_z;

        bin_object.primitives.push_back(left_primitive);
        bin_object.primitive_poses.push_back(left_pose);

        // 5. Right wall of the bin
        shape_msgs::msg::SolidPrimitive right_primitive;
        right_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        right_primitive.dimensions.resize(3);
        right_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = wall_thickness;
        right_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = bin_depth;
        right_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = bin_height;

        geometry_msgs::msg::Pose right_pose;
        right_pose.orientation = bin_orientation;
        right_pose.position.x = bin_x + (bin_width / 2) - (wall_thickness / 2);
        right_pose.position.y = bin_y;
        right_pose.position.z = bin_z;

        bin_object.primitives.push_back(right_primitive);
        bin_object.primitive_poses.push_back(right_pose);

        // Add bin object to the planning scene
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.push_back(bin_object);
        //adding bin in all planning scenes
        planning_scene_interface_A.applyCollisionObjects(collision_objects);
        planning_scene_interface_B.applyCollisionObjects(collision_objects);
        planning_scene_interface_dual.applyCollisionObjects(collision_objects);

        RCLCPP_INFO(LOGGER, "Added bin to planning scene");
    }

    bool execute() {
        switch (current_state_) {
            case State::HOME:
                // arm_move_group_.setNamedTarget("Home");
                // arm_move_group_.move();

                // Wait for pose if needed
                if (!grasp_pose_received_ && retry_count < 3) {
                    RCLCPP_INFO_THROTTLE(LOGGER, *node_->get_clock(), 2000, 
                                        "Waiting for pose on /left_arm_pose...");
                    retry_count++;
                    return true; // Stay in HOME state until we get a message
                }
                current_state_ = State::PLAN_TO_OBJECT;
     
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
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_A;

    //moveit groups for B
    moveit::planning_interface::MoveGroupInterface arm_move_group_B;
    moveit::planning_interface::MoveGroupInterface gripper_move_group_B;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_B;

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

    moveit_msgs::msg::AttachedCollisionObject attached_bin;

    //creating subscription for grasp_pose
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr grasp_pose_subscription_;
    
    // Grasp pose storage for gripper
    geometry_msgs::msg::Pose target_grasp_pose_A;
    geometry_msgs::msg::Pose target_grasp_pose_B;
    bool grasp_pose_received_;
    
    
    // Callback for the grasp pose subscriber
    void graspPoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        //checking if both poses are present
        if (msg->poses.size() < 2) {
            RCLCPP_WARN(LOGGER, "Received fewer than 2 poses.");
          }
        //getting grasp poses
        target_grasp_pose_A = msg->poses[0];
        target_grasp_pose_B = msg->poses[1];
        grasp_pose_received_ = true;
        RCLCPP_INFO(LOGGER, "Received grasp pose left: x=%f, y=%f, z=%f", 
                    target_grasp_pose_A.position.x,
                    target_grasp_pose_A.position.y,
                    target_grasp_pose_A.position.z);

        RCLCPP_INFO(LOGGER, "Received grasp pose right: x=%f, y=%f, z=%f", 
                    target_grasp_pose_B.position.x,
                    target_grasp_pose_B.position.y,
                    target_grasp_pose_B.position.z);

        grasp_pose_subscription_.reset();
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
        const int max_plan_attempts = 3;

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

        // Get joint model
        const moveit::core::JointModelGroup* joint_model_group_left = current_state_left->getJointModelGroup("left_arm");
        const moveit::core::JointModelGroup* joint_model_group_right = current_state_right->getJointModelGroup("right_arm");
 
        //Compute IK
        RCLCPP_INFO(LOGGER, "Computing IK solutions...");
        bool ik_left = current_state_left->setFromIK(joint_model_group_left, pose1, arm_move_group_A.getEndEffectorLink(), 10.0);
        bool ik_right = current_state_right->setFromIK(joint_model_group_right, pose2, arm_move_group_B.getEndEffectorLink(), 10.0);
        
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
        } else {
            RCLCPP_WARN(LOGGER, "IK failed for one or both arms");
            if (!ik_left) RCLCPP_WARN(LOGGER, "Left arm IK failed");
            if (!ik_right) RCLCPP_WARN(LOGGER, "Right arm IK failed");
        }

        arm_move_group_dual.setJointValueTarget(combined_joint_positions);

        // Set planning parameters
        arm_move_group_dual.setPlanningTime(20.0 + (5.0 * plan_attempts));

        // Plan with the dual arm group directly
        bool success = (arm_move_group_dual.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

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
        bool success = (arm_move_group_dual.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (success) {
            RCLCPP_INFO(LOGGER, "%s", success_message.c_str());
            
            if (!prompt_message.empty()) {
                RCLCPP_INFO(LOGGER, "\033[32m %s\033[0m", prompt_message.c_str());
                char input = waitForKeyPress();
            }
            current_state_ = next_state;
        } else {
            RCLCPP_ERROR(LOGGER, "Failed to execute movement");
            current_state_ = State::FAILED;
        }
        
        return true;
    }


    /*//////////////////////////////////////////

    START OF FSM FUNCTIONS

    *//////////////////////////////////////////

    bool planToObject() {
        //plans to the object
        // Define orientation in quaternion
        tf2::Quaternion tf2_quat;
        tf2_quat.setRPY(0, -3.14, 0);
        geometry_msgs::msg::Quaternion quat_orient;
        tf2::convert(tf2_quat, quat_orient);
    
        target_pose_A.orientation = quat_orient;
        target_pose_B.orientation = quat_orient;
    
        if (grasp_pose_received_) {
            RCLCPP_INFO(LOGGER, "Using position from topic");
            target_pose_A.position.x = target_grasp_pose_A.position.x;
            target_pose_A.position.y = target_grasp_pose_A.position.y;
            target_pose_A.position.z = target_grasp_pose_A.position.z; 

            target_pose_B.position.x = target_grasp_pose_B.position.x;
            target_pose_B.position.y = target_grasp_pose_B.position.y;
            target_pose_B.position.z = target_grasp_pose_B.position.z; 
        } else {
            RCLCPP_INFO(LOGGER, "Using default position");
            target_pose_A.position.x = 0.1868;
            target_pose_A.position.y = 0.0;
            target_pose_A.position.z = 1.4; 

            target_pose_B.position.x = -0.1868;
            target_pose_B.position.y = 0.0;
            target_pose_B.position.z = 1.4;
        }
        
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
        gripper_move_group_A.setNamedTarget("Open");
        bool success1 = (gripper_move_group_A.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        gripper_move_group_B.setNamedTarget("Open");
        bool success2 = (gripper_move_group_B.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (success1 && success2) {
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
        target_pose_A.position.z = 1.33;
        target_pose_B.position.z = 1.33;
        
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
        // Now we attach the compound bin
        attached_bin.object.id = "bin";
        //cannot link both end effectors so linking left arm alone
        attached_bin.link_name = arm_move_group_A.getEndEffectorLink();
        
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

        attached_bin.touch_links = touch_links;
        
        // Define object as attached
        attached_bin.object.operation = moveit_msgs::msg::CollisionObject::ADD;
        // bool attach_success_left = (planning_scene_interface_A.applyAttachedCollisionObject(attached_bin) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        // bool attach_success_right = (planning_scene_interface_B.applyAttachedCollisionObject(attached_bin) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        bool attach_success_dual = (planning_scene_interface_dual.applyAttachedCollisionObject(attached_bin) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        // Close gripper
        gripper_move_group_A.setNamedTarget("Close");
        bool gripper_success_A = (gripper_move_group_A.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        gripper_move_group_B.setNamedTarget("Close");
        bool gripper_success_B = (gripper_move_group_B.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        // Check both operations for overall success
        if (attach_success_dual && gripper_success_A && gripper_success_B) {
            RCLCPP_INFO(LOGGER, "Successfully grasped object");
            current_state_ = State::PLAN_TO_LIFT;
        } else {
            if (!attach_success_dual) {
                RCLCPP_ERROR(LOGGER, "Failed to attach object");
            }
            if (!gripper_success_A || !gripper_success_B) {
                RCLCPP_ERROR(LOGGER, "Failed to close gripper");
            }
            current_state_ = State::FAILED;
        }
        
        return true;
    }

    bool planToLift() {
        geometry_msgs::msg::PoseStamped current_pose1 = arm_move_group_A.getCurrentPose();
        geometry_msgs::msg::PoseStamped current_pose2 = arm_move_group_B.getCurrentPose();

        geometry_msgs::msg::PoseStamped current_pose = arm_move_group_A.getCurrentPose();

        // Add to z position
        geometry_msgs::msg::Pose lift_pose1 = current_pose1.pose;
        lift_pose1.position.z += 0.2;  // Lift by 20cm
        geometry_msgs::msg::Pose lift_pose2 = current_pose2.pose;
        lift_pose2.position.z += 0.2;  // Lift by 20cm
        
        // Try planning with the modified joint positions
        RCLCPP_INFO(LOGGER, "\033[32m Press any key to plan to lift\033[0m");
        waitForKeyPress();
        return plantoTarget_dualarm(lift_pose1, lift_pose2, State::MOVE_TO_LIFT, 
                            "Planning to lift succeeded!");
    }

    bool moveToLift() {
        return executeMovement_dualarm(State::PLAN_TO_PLACE, "Successfully moved to lift position",
                                "Press any key to plan to place");
    }

    bool planToPlace() {
        RCLCPP_INFO(LOGGER, "\033[32m Press any key to plan to place\033[0m");
        waitForKeyPress();
        return plantoTarget_dualarm(target_pose_A, target_pose_B, State::MOVE_TO_PLACE, 
                             "Planning to place succeeded!");
    }

    bool moveToPlace() {
        return executeMovement_dualarm(State::PLACE, "Successfully moved to place position",
                             "Press any key to drop object");
    }

    bool Place() {

        arm_move_group_dual.clearPathConstraints();

        // Open gripper
        gripper_move_group_A.setNamedTarget("Open");
        gripper_move_group_A.move();
        gripper_move_group_B.setNamedTarget("Open");
        gripper_move_group_B.move();

        bool success1 = (gripper_move_group_A.detachObject(attached_bin.object.id) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        bool success2 = (gripper_move_group_B.detachObject(attached_bin.object.id) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        

        if (success1 && success2) {
            RCLCPP_INFO(LOGGER, "Successfully dropped object");
            current_state_ = State::PLAN_TO_HOME;
        } else {
            RCLCPP_ERROR(LOGGER, "Failed to drop object");
            current_state_ = State::FAILED;
        }
        
        return true;
    }

    bool planToHome() {
        RCLCPP_INFO(LOGGER, "\033[32m Press any key to plan to home\033[0m");
        // waitForKeyPress();
        // return plantoTarget_dualarm("Home", "Home", State::MOVE_TO_HOME, 
        //                      "Successfully planned to home");
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
    static const std::string GRIPPER_GROUP_dual = "both_arms_with_grippers";

    // Create and run FSM
    MotionPlanningFSM fsm(move_group_node, PLANNING_GROUP_A, GRIPPER_GROUP_A,PLANNING_GROUP_B,GRIPPER_GROUP_B,PLANNING_GROUP_dual,GRIPPER_GROUP_dual);
    
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