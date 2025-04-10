#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

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
        const std::string& arm_planning_group, 
        const std::string& gripper_planning_group) 
        : node_(node),
          arm_planning_group_(arm_planning_group),
          gripper_planning_group_(gripper_planning_group),
          arm_move_group_(node, arm_planning_group),
          gripper_move_group_(node, gripper_planning_group),
          current_state_(State::HOME) {

        
        // Create subscription to the grasp pose topic
        grasp_pose_subscription_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/left_arm_pose", 10, 
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
        planning_scene_interface_.applyCollisionObjects(collision_objects);

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
    int retry_count = 0;
    rclcpp::Node::SharedPtr node_;
    std::string arm_planning_group_;
    std::string gripper_planning_group_;
    moveit::planning_interface::MoveGroupInterface arm_move_group_;
    moveit::planning_interface::MoveGroupInterface gripper_move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    State current_state_;
    moveit::planning_interface::MoveGroupInterface::Plan current_plan_;

    geometry_msgs::msg::Pose target_pose;
    moveit_msgs::msg::AttachedCollisionObject attached_bin;

    // Grasp pose subscription and storage
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr grasp_pose_subscription_;
    geometry_msgs::msg::Pose target_grasp_pose_;
    bool grasp_pose_received_;

    // Callback for the grasp pose subscriber
    void graspPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        target_grasp_pose_ = *msg;
        grasp_pose_received_ = true;
        RCLCPP_INFO(LOGGER, "Received grasp pose: x=%f, y=%f, z=%f", 
                    target_grasp_pose_.position.x,
                    target_grasp_pose_.position.y,
                    target_grasp_pose_.position.z);

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

    bool plantoTarget(const std::variant<geometry_msgs::msg::Pose, std::string>& target, 
            State next_state, 
            const std::string& planning_message = "Planning succeeded!") {

        static int plan_attempts = 0;
        const int max_plan_attempts = 3;  // Maximum number of automatic retries
        
        // Set the target whether it is pose or named target
        if (std::holds_alternative<geometry_msgs::msg::Pose>(target)) {
            const auto& target_pose = std::get<geometry_msgs::msg::Pose>(target);
            arm_move_group_.setPoseTarget(target_pose);
        } else if (std::holds_alternative<std::string>(target)) {
            const auto& target_name = std::get<std::string>(target);
            arm_move_group_.setNamedTarget(target_name);
        }

        arm_move_group_.setPlanningTime(15.0 + (5.0 * plan_attempts));  // Increase planning time based on attempt number
        auto const success = static_cast<bool>(arm_move_group_.plan(current_plan_));
    
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
                
                // For named targets, we can adjust planning time and other parameters
                arm_move_group_.setPlanningTime(15.0 + (5.0 * plan_attempts));
                
                // Stay in current state to try again
                return true;
            } else {
                // After max attempts, ask the user what to do
                RCLCPP_ERROR(LOGGER, "Failed to plan after %d attempts", max_plan_attempts);
                current_state_ = State::FAILED;
            }
        }
    }

    bool executeMovement(State next_state, const std::string& success_message, const std::string& prompt_message = "") {
        bool success = (arm_move_group_.execute(current_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
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

    bool planToObject() {
        // Define orientation in quaternion
        tf2::Quaternion tf2_quat;
        tf2_quat.setRPY(0, -3.14, 0);
        geometry_msgs::msg::Quaternion quat_orient;
        tf2::convert(tf2_quat, quat_orient);
    
        target_pose.orientation = quat_orient;
    
        if (grasp_pose_received_) {
            RCLCPP_INFO(LOGGER, "Using position from topic");
            target_pose.position.x = target_grasp_pose_.position.x;
            target_pose.position.y = target_grasp_pose_.position.y;
            target_pose.position.z = target_grasp_pose_.position.z; // Slightly above the grasp position
        } else {
            RCLCPP_INFO(LOGGER, "Using default position");
            target_pose.position.x = 0.1868; // Default position
            target_pose.position.y = 0.0;
            target_pose.position.z = 1.4; // Slightly above the grasp position
        }
        
        RCLCPP_INFO(LOGGER, "\033[32m Press any key to plan to object \033[0m");
        waitForKeyPress();
        return plantoTarget(target_pose, State::MOVE_TO_OBJECT, "Planning to object succeeded!");
    }

    bool moveToObject() {
        return executeMovement(State::OPEN_GRIPPER, "Successfully moved to object position", 
                                    "Press any key to open gripper");
    }

    bool opengripper() {
        gripper_move_group_.setNamedTarget("Open");
        bool success = (gripper_move_group_.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
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
        
        // Reuse target_pose with new z pos
        target_pose.position.z = 1.33;
        
        RCLCPP_INFO(LOGGER, "\033[32m Press any key to plan to grasp\033[0m");
        waitForKeyPress();
        return plantoTarget(target_pose, State::MOVE_TO_GRASP, 
                          "Planning to grasp succeeded!");
    }

    bool moveToGrasp() {
        return executeMovement(State::GRASP, "Successfully moved to grasp pose", 
                             "Press any key to grasp object");
    }

    bool Grasp() {
        // Now we attach the compound bin
        attached_bin.object.id = "bin";
        attached_bin.link_name = arm_move_group_.getEndEffectorLink();
        
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
        attached_bin.touch_links = touch_links;
        
        // Define object as attached
        attached_bin.object.operation = moveit_msgs::msg::CollisionObject::ADD;
        bool attach_success = (planning_scene_interface_.applyAttachedCollisionObject(attached_bin) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        // Close gripper
        gripper_move_group_.setNamedTarget("Close");
        bool gripper_success = (gripper_move_group_.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        // Check both operations for overall success
        if (attach_success && gripper_success) {
            RCLCPP_INFO(LOGGER, "Successfully grasped object");
            current_state_ = State::PLAN_TO_LIFT;
        } else {
            if (!attach_success) {
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
        // Get current pose as starting point
        geometry_msgs::msg::PoseStamped current_pose = arm_move_group_.getCurrentPose();
        
        // Add to z position
        geometry_msgs::msg::Pose lift_pose = current_pose.pose;
        lift_pose.position.z += 0.2;  // Lift by 20cm
        
        RCLCPP_INFO(LOGGER, "\033[32m Press any key to plan to lift\033[0m");
        waitForKeyPress();
        return plantoTarget(lift_pose, State::MOVE_TO_LIFT, 
                          "Successfully planned lift movement");
    }

    bool moveToLift() {
        return executeMovement(State::PLAN_TO_PLACE, "Successfully moved to lift position",
                             "Press any key to plan to place");
    }

    bool planToPlace() {
        // Reuse target pose from grasp

        return plantoTarget(target_pose, State::MOVE_TO_PLACE, 
                          "Successfully planned to place position");
    }

    bool moveToPlace() {
        return executeMovement(State::PLACE, "Successfully moved to place position",
                             "Press any key to drop object");
    }

    bool Place() {

        // Open gripper
        gripper_move_group_.setNamedTarget("Open");
        gripper_move_group_.move();

        bool success = (arm_move_group_.detachObject(attached_bin.object.id) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success) {
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
        waitForKeyPress();
        return plantoTarget("Home", State::MOVE_TO_HOME, 
                             "Successfully planned to home");
    }

    bool moveToHome() {
        return executeMovement(State::SUCCEEDED, "Successfully moved to home position");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("motion_planning", node_options);

    rclcpp::Parameter sim_time_param("use_sim_time", true);
    move_group_node->set_parameter(sim_time_param);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread spinner([&executor]() { executor.spin(); });

    // Initialize MoveIt interfaces
    static const std::string PLANNING_GROUP = "left_arm";
    static const std::string GRIPPER_GROUP = "left_gripper";

    // Create and run FSM
    MotionPlanningFSM fsm(move_group_node, PLANNING_GROUP, GRIPPER_GROUP);
    
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