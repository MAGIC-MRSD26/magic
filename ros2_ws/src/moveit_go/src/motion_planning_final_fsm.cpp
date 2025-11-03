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

// Helpers
#include "dual_arm_planner.hpp"
#include "fsm_states.hpp"

// This is a finite state machine for Kinova Gen3 7DOF arm
static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_planning");

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

            case State::ROTATE_EE:
                return rotateEndEffectors();

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
    geometry_msgs::msg::Pose approach_pose_A;
    geometry_msgs::msg::Pose approach_pose_B;
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
        // Adjust Z for approach
        target_pose_A.position.z -= 0.1;
        target_pose_B.position.z -= 0.1;

        approach_pose_A = target_pose_A;
        pproach_pose_B = target_pose_B;

        RCLCPP_INFO(LOGGER, "Left arm target pose x: %f y: %f z: %f", target_pose_A.position.x, target_pose_A.position.y, target_pose_A.position.z);
        RCLCPP_INFO(LOGGER, "Right arm target pose x: %f y: %f z: %f", target_pose_B.position.x, target_pose_B.position.y, target_pose_B.position.z);
        
        RCLCPP_INFO(LOGGER, "\033[32m Press any key to plan to object \033[0m");
        waitForKeyPress();
        return dual_arm_planner_->plantoTarget_dualarm(target_pose_A, target_pose_B, current_state_, State::MOVE_TO_OBJECT, plan, "Planning to object succeeded!");
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
        //next state where we plan for grasp point
        // Reuse target_pose with new z pos
        target_pose_A = approach_pose_A;
        target_pose_B = approach_pose_B;

        target_pose_A.position.z -= 0.19;
        target_pose_B.position.z -= 0.19;
        
        RCLCPP_INFO(LOGGER, "\033[32m Press any key to plan to grasp\033[0m");
        waitForKeyPress();

        planning_scene_interface_dual.allowCollisions(
            object_params_.object_id,
            {arm_move_group_A.getEndEffectorLink(), 
             arm_move_group_B.getEndEffectorLink()},
            true
        );

        return dual_arm_planner_->plantoTarget_dualarm(target_pose_A, target_pose_B, current_state_, State::MOVE_TO_GRASP, plan,
                          "Planning to grasp succeeded!");
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

        // Straighten out the arms for 360 rotation
        dual_arm_planner_->rotate(0, 0, -M_PI/4, rotated_pose1, rotated_pose2);

        // Add to z position
        if (go_to_home) {
            rotated_pose1.position.z += 0.2;
            rotated_pose2.position.z += 0.2;
        } else {
            rotated_pose1.position.z += 0.35;
            rotated_pose2.position.z += 0.35;
        }
                
        RCLCPP_INFO(LOGGER, "\033[32m Press any key to plan to lift\033[0m");
        waitForKeyPress();
        return dual_arm_planner_->plantoTarget_dualarm(rotated_pose1, rotated_pose2, current_state_, State::MOVE_TO_LIFT, plan,
                            "Planning to lift succeeded!");
    }

    bool moveToLift() {
        if (go_to_home) {
            return dual_arm_planner_->executeMovement_dualarm(current_state_, State::PLAN_TO_HOME, plan, "Successfully moved to lift position",
                                "Press any key to go to home");
        } else {
            return dual_arm_planner_->executeMovement_dualarm(current_state_, State::ROTATE_EE, plan, "Successfully moved to lift position",
                                "Press any key to straighten lift");
        }
    }

    bool rotateEndEffectors() {
       RCLCPP_INFO(LOGGER, "\033[32m STEP 3: End Effector Rotation Demonstration \033[0m");
       RCLCPP_INFO(LOGGER, "Demonstrating independent end effector rotation - 360° without arm movement");
       RCLCPP_INFO(LOGGER, "Press any key to start end effector rotation");
       waitForKeyPress();
      
       std::vector<double> current_joints = arm_move_group_dual.getCurrentJointValues();
      
       double left_wrist_joint = 6;  
       double right_wrist_joint = 13; 
      
       double original_left_wrist = current_joints[left_wrist_joint];
       double original_right_wrist = current_joints[right_wrist_joint];
      
       const int steps = 8;
       const double rotation_per_step = 2 * M_PI / steps; // 45 degrees per step
      
       for (int i = 1; i <= steps; i++) {
           RCLCPP_INFO(LOGGER, "Rotation step %d/%d (%.1f degrees)",
                      i, steps, (i * 360.0 / steps));
          
           current_joints[left_wrist_joint] = original_left_wrist + (i * rotation_per_step);
           current_joints[right_wrist_joint] = original_right_wrist - (i * rotation_per_step);

           arm_move_group_dual.setJointValueTarget(current_joints);
           arm_move_group_dual.setPlanningTime(5.0);
          
           moveit::planning_interface::MoveGroupInterface::Plan rotation_plan;
           bool plan_success = (arm_move_group_dual.plan(rotation_plan) == moveit::core::MoveItErrorCode::SUCCESS);
          
           if (plan_success) {
               bool execute_success = (arm_move_group_dual.execute(rotation_plan) == moveit::core::MoveItErrorCode::SUCCESS);
               if (execute_success) {
                   RCLCPP_INFO(LOGGER, "Step %d completed - notice arm positions remain unchanged!", i);
                   rclcpp::sleep_for(std::chrono::milliseconds(500));
               } else {
                   RCLCPP_ERROR(LOGGER, "Failed to execute rotation step %d", i);
                   current_state_ = State::FAILED;
                   return true;
               }
           } else {
               RCLCPP_ERROR(LOGGER, "Failed to plan rotation step %d", i);
               current_state_ = State::FAILED;
               return true;
           }
       }
      
       RCLCPP_INFO(LOGGER, "\033[32m360° rotation completed! End effectors rotated independently.\033[0m");
       RCLCPP_INFO(LOGGER, "Press any key to return to home position");
       waitForKeyPress();
      
       current_state_ = State::PLAN_TO_PLACE;
       return true;
    }

    bool planToPlace() {
        target_pose_A.position.z = 1.31;
        target_pose_B.position.z = 1.31;

        return dual_arm_planner_->plantoTarget_dualarm(target_pose_A, target_pose_B, current_state_, State::MOVE_TO_PLACE, plan,
                             "Planning to place succeeded!");
    }

    bool moveToPlace() {
        return dual_arm_planner_->executeMovement_dualarm(current_state_, State::PLACE, plan, "Successfully moved to place position",
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
        return dual_arm_planner_->executeMovement_dualarm(current_state_, State::SUCCEEDED, plan, "Successfully moved to home position");
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