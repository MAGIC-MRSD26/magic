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
        
        RCLCPP_INFO(LOGGER, "MotionPlanningFSM initialized");
    }

    bool execute() {
        switch (current_state_) {
            case State::HOME:
                // arm_move_group_.setNamedTarget("Home");
                // arm_move_group_.move();
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
    rclcpp::Node::SharedPtr node_;
    std::string arm_planning_group_;
    std::string gripper_planning_group_;
    moveit_visual_tools::MoveItVisualTools& visual_tools_;
    moveit::planning_interface::MoveGroupInterface arm_move_group_;
    moveit::planning_interface::MoveGroupInterface gripper_move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    State current_state_;
    moveit::planning_interface::MoveGroupInterface::Plan current_plan_;

    bool planToObject() {
        // End Effector target pose
        // Define orientation in quaternion
        tf2::Quaternion tf2_quat;
        tf2_quat.setRPY(0, -3.14, 0);
        geometry_msgs::msg::Quaternion quat_orient;
        tf2::convert(tf2_quat, quat_orient);

        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation = quat_orient;
        target_pose.position.x = 0.1868;
        target_pose.position.y = 0;
        target_pose.position.z = 1.4;
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

        // Add object to the planning scene
        visual_tools_.prompt("Press 'Next' to add object to planning scene");
        moveit_msgs::msg::CollisionObject bin_object;
        bin_object.id = "bin";
        bin_object.header.frame_id = "world";
        
        // Define the object shape
        shape_msgs::msg::SolidPrimitive box_primitive;
        box_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        box_primitive.dimensions.resize(3);
        box_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.01;  // Adjust dimensions
        box_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.32;  // based on your bin
        box_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.266;
        
        // Define the object pose
        geometry_msgs::msg::Pose object_pose;
        tf2::Quaternion obj_quat;
        obj_quat.setRPY(0, 0, 0);  // Set proper rotation
        object_pose.orientation = tf2::toMsg(obj_quat);
        object_pose.position.x = 0.1905;
        object_pose.position.y = 0.0;
        object_pose.position.z = 1.0646; // height of table is 0.9316
        
        // Add the bin to the planning scene
        bin_object.primitives.push_back(box_primitive);
        bin_object.primitive_poses.push_back(object_pose);
        bin_object.operation = moveit_msgs::msg::CollisionObject::ADD;
        
        planning_scene_interface_.applyCollisionObject(bin_object);
        
        RCLCPP_INFO(LOGGER, "Added cylinder to planning scene");
        
        // Plan to the grasp position with collision avoidance
        visual_tools_.prompt("Press 'Next' to plan to grasp pose");
        tf2::Quaternion q;
        q.setRPY(0, M_PI/2, 0);  // Adjusted orientation for top 
        geometry_msgs::msg::Pose grasp_pose;
        grasp_pose.orientation = tf2::toMsg(q);
        grasp_pose.position.x = -0.1905;  // Bin position
        grasp_pose.position.y = 0.15875;
        grasp_pose.position.z = 1.0316;
        
        arm_move_group_.setPoseTarget(grasp_pose);
        arm_move_group_.setPlanningTime(15.0);
        
        auto const success = static_cast<bool>(arm_move_group_.plan(current_plan_));
    
        if (success) {
            RCLCPP_INFO(LOGGER, "Planning to grasp");
            visual_tools_.prompt("Press 'Next' to move to grasp pose");
            current_state_ = State::MOVE_TO_GRASP;
        } else {
            RCLCPP_ERROR(LOGGER, "Failed to move to object");
            current_state_ = State::FAILED;
        }
        return true;
    }

    bool moveToGrasp() {

        RCLCPP_INFO(LOGGER, "Moving to grasp");
        current_state_ = State::GRASP;
        return true;
    }

    bool Grasp() {

        // Close gripper
        gripper_move_group_.setNamedTarget("Close");
        gripper_move_group_.move();

        // Attach object to gripper


        current_state_ = State::PLAN_TO_LIFT;
        return true;
    }

    bool planToLift() {

        RCLCPP_INFO(LOGGER, "Planning to lift");
        current_state_ = State::MOVE_TO_LIFT;
        return true;
    }

    bool moveToLift() {

        RCLCPP_INFO(LOGGER, "Moving to lift");
        current_state_ = State::PLAN_TO_PLACE;
        return true;
    }

    bool planToPlace() {

        RCLCPP_INFO(LOGGER, "Planning to place");
        current_state_ = State::MOVE_TO_PLACE;
        return true;
    }

    bool moveToPlace() {

        RCLCPP_INFO(LOGGER, "Moving to place");
        current_state_ = State::PLAN_TO_HOME;
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