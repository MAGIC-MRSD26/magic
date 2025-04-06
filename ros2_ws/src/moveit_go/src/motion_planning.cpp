#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

// This is a finite state machine for Kinova Gen3 7DOF arm
static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_planning");

enum class State {
    HOME,
    PLAN_TO_OBJECT,
    MOVE_TO_OBJECT,
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
        const std::string& gripper_planning_group,
        moveit_visual_tools::MoveItVisualTools& visual_tools) 
        : node_(node),
          arm_planning_group_(arm_planning_group),
          gripper_planning_group_(gripper_planning_group),
          visual_tools_(visual_tools),
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
        visual_tools_.prompt("Press 'Next' to add bin to planning scene");

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
                visual_tools_.prompt("Press 'Next' in the RvizVisualToolsGui window to plan to object");
                return true;
     
            case State::PLAN_TO_OBJECT:
                return planToObject();

            case State::MOVE_TO_OBJECT:
                return moveToObject();

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
    moveit_visual_tools::MoveItVisualTools& visual_tools_;
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

    bool planToObject() {
        // End Effector target pose
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
        arm_move_group_.setPoseTarget(target_pose);
        arm_move_group_.setPlanningTime(15.0);  // Give the planner more time (15 seconds)
        auto const success = static_cast<bool>(arm_move_group_.plan(current_plan_));

        if (success) {
            // visual_tools_.prompt("Press 'Continue' if you want to replan, 'Next' to execute the plan");
            // int result = visual_tools_.getUserInput();
            
            // // Can replan if plan sucks
            // if (result == visual_tools_.CONTINUE) {
            //     return true; // Stay in same state for replanning
            // } else if (result == visual_tools_.NEXT) {
            //     current_state_ = State::MOVE_TO_OBJECT;
            //     return true;
            // }
            current_state_ = State::MOVE_TO_OBJECT;
            visual_tools_.prompt("Press 'Next' in the RvizVisualToolsGui window to execute the plan");
            return true;
        } else {
            RCLCPP_ERROR(LOGGER, "Failed to plan to object");
            current_state_ = State::FAILED;
            return true;
        }
        
        return true;
    }

    bool moveToObject() {
        bool success = (arm_move_group_.execute(current_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (success) {
            RCLCPP_INFO(LOGGER, "Successfully moved to object");
            current_state_ = State::PLAN_TO_GRASP;
        } else {
            RCLCPP_ERROR(LOGGER, "Failed to move to object");
            current_state_ = State::FAILED;
        }
        
        return true;
    }

    bool planToGrasp() {

        // Open gripper
        gripper_move_group_.setNamedTarget("Open");
        gripper_move_group_.move();
        
        // Plan to the grasp position with collision avoidance
        visual_tools_.prompt("Press 'Next' to plan to grasp pose");
        tf2::Quaternion q;
        q.setRPY(0, -3.14, 0);  // Adjusted orientation for top 
        geometry_msgs::msg::Pose grasp_pose;
        grasp_pose.orientation = tf2::toMsg(q);
        grasp_pose.position.x = 0.1905;
        grasp_pose.position.y = 0.0;
        grasp_pose.position.z = 1.33;
        
        arm_move_group_.setPoseTarget(grasp_pose);
        arm_move_group_.setPlanningTime(15.0);
        
        auto const success = static_cast<bool>(arm_move_group_.plan(current_plan_));
    
        if (success) {
            RCLCPP_INFO(LOGGER, "Planning to grasp");
            visual_tools_.prompt("Press 'Next' to move to grasp pose");
            current_state_ = State::MOVE_TO_GRASP;
        } else {
            RCLCPP_ERROR(LOGGER, "Failed to move to grasp pose");
            current_state_ = State::FAILED;
        }
        return true;
    }

    bool moveToGrasp() {

        bool success = (arm_move_group_.execute(current_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (success) {
            RCLCPP_INFO(LOGGER, "Successfully moved to grasp pose");
            visual_tools_.prompt("Press 'Next' to grasp object");
            current_state_ = State::GRASP;
        } else {
            RCLCPP_ERROR(LOGGER, "Failed to move to grasp pose");
            current_state_ = State::FAILED;
        }
        
        return true;
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
            visual_tools_.prompt("Press 'Next' to plan lifting the object");
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
        
        arm_move_group_.setPoseTarget(lift_pose);
        arm_move_group_.setPlanningTime(15.0);  // Give the planner more time
        
        auto const success = static_cast<bool>(arm_move_group_.plan(current_plan_));
        
        if (success) {
            RCLCPP_INFO(LOGGER, "Successfully planned to lift");
            visual_tools_.prompt("Press 'Next' to move to lift pose");
            current_state_ = State::MOVE_TO_LIFT;
        } else {
            RCLCPP_ERROR(LOGGER, "Failed to plan to lift");
            current_state_ = State::FAILED;
        }
        
        return true;
    }

    bool moveToLift() {

        bool success = (arm_move_group_.execute(current_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (success) {
            RCLCPP_INFO(LOGGER, "Successfully moved to lift pose");
            visual_tools_.prompt("Press 'Next' to plan to place");
            current_state_ = State::PLAN_TO_PLACE;
        } else {
            RCLCPP_ERROR(LOGGER, "Failed to move to lift pose");
            current_state_ = State::FAILED;
        }
        
        return true;
    }

    bool planToPlace() {
        target_pose.position.z = 1.33;
        arm_move_group_.setPoseTarget(target_pose);
        arm_move_group_.setPlanningTime(15.0);
        
        auto const success = static_cast<bool>(arm_move_group_.plan(current_plan_));
        
        if (success) {
            RCLCPP_INFO(LOGGER, "Successfully planned to place position");
            visual_tools_.prompt("Press 'Next' to execute placement motion");
            current_state_ = State::MOVE_TO_PLACE;
        } else {
            RCLCPP_ERROR(LOGGER, "Failed to plan to place position");
            current_state_ = State::FAILED;
        }
        
        return true;
    }

    bool moveToPlace() {

        bool success = (arm_move_group_.execute(current_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
        if (success) {
            RCLCPP_INFO(LOGGER, "Successfully moved to place position");
            visual_tools_.prompt("Press 'Next' to drop object");
            current_state_ = State::PLACE;
        } else {
            RCLCPP_ERROR(LOGGER, "Failed to move to drop position");
            current_state_ = State::FAILED;
        }
        
        return true;
    }

    bool Place() {

        // Open gripper
        gripper_move_group_.setNamedTarget("Open");
        gripper_move_group_.move();

        bool success = (arm_move_group_.detachObject(attached_bin.object.id) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(LOGGER, "Successfully dropped object");
            visual_tools_.prompt("Press 'Next' to plan to home");
            current_state_ = State::PLAN_TO_HOME;
        } else {
            RCLCPP_ERROR(LOGGER, "Failed to drop object");
            current_state_ = State::FAILED;
        }
        
        return true;
    }

    bool planToHome() {

        arm_move_group_.setNamedTarget("Home");
        arm_move_group_.setPlanningTime(15.0);
        auto const success = static_cast<bool>(arm_move_group_.plan(current_plan_));

        if (success) {
            current_state_ = State::MOVE_TO_HOME;
            visual_tools_.prompt("Press 'Next' in the RvizVisualToolsGui window to execute move to home");
            return true;
        } else {
            RCLCPP_ERROR(LOGGER, "Failed to plan to object");
            current_state_ = State::FAILED;
            return true;
        }
        
        return true;
    }

    bool moveToHome() {

        bool success = (arm_move_group_.execute(current_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (success) {
            RCLCPP_INFO(LOGGER, "Successfully moved to object");
            current_state_ = State::SUCCEEDED;
        } else {
            RCLCPP_ERROR(LOGGER, "Failed to move to object");
            current_state_ = State::FAILED;
        }
        
        return true;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("motion_planning", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread spinner([&executor]() { executor.spin(); });

    // Initialize MoveIt interfaces
    static const std::string PLANNING_GROUP = "left_arm";
    static const std::string GRIPPER_GROUP = "left_gripper";

    // Initialize visual tools
    moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "left_base_link");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    // Create and run FSM
    MotionPlanningFSM fsm(move_group_node, PLANNING_GROUP, GRIPPER_GROUP, visual_tools);
    
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