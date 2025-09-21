#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <future>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <tf2_eigen/tf2_eigen.h>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("dual_arm_demo");


enum class DemoState {
   HOME,
   PLAN_TO_POINT,
   MOVE_TO_POINT,
   ROTATE_END_EFFECTORS,
   PLAN_TO_HOME,
   MOVE_TO_HOME,
   SUCCEEDED,
   FAILED
};


class DualArmDemoFSM {
public:
   DualArmDemoFSM(
       const rclcpp::Node::SharedPtr& node,
       const std::string& arm_planning_group_A_,
       const std::string& arm_planning_group_B_,
       const std::string& arm_planning_group_dual_)
       : node_(node),
         arm_planning_group_A(arm_planning_group_A_),
         arm_planning_group_B(arm_planning_group_B_),
         arm_planning_group_dual(arm_planning_group_dual_),
         arm_move_group_A(node, arm_planning_group_A_),
         arm_move_group_B(node, arm_planning_group_B_),
         arm_move_group_dual(node, arm_planning_group_dual_),
         current_state_(DemoState::HOME) {


       arm_move_group_A.setMaxVelocityScalingFactor(0.3);
       arm_move_group_B.setMaxVelocityScalingFactor(0.3);
       arm_move_group_dual.setMaxVelocityScalingFactor(0.3);
      
       arm_move_group_A.setMaxAccelerationScalingFactor(0.3);
       arm_move_group_B.setMaxAccelerationScalingFactor(0.3);
       arm_move_group_dual.setMaxAccelerationScalingFactor(0.3);
      
       RCLCPP_INFO(LOGGER, "DualArmDemoFSM initialized");
       RCLCPP_INFO(LOGGER, "=== DUAL ARM DEMONSTRATION ===");
       RCLCPP_INFO(LOGGER, "This demo will showcase:");
       RCLCPP_INFO(LOGGER, "1. Joint locking - moving arms to point at each other using only top joints");
       RCLCPP_INFO(LOGGER, "2. Independent end effector rotation - 360° rotation without arm movement");
   }


   bool execute() {
       switch (current_state_) {
           case DemoState::HOME:
               return goToHome();


           case DemoState::PLAN_TO_POINT:
               return planToPointingPosition();


           case DemoState::MOVE_TO_POINT:
               return moveToPointingPosition();


           case DemoState::ROTATE_END_EFFECTORS:
               return rotateEndEffectors();


           case DemoState::PLAN_TO_HOME:
               return planToFinalHome();


           case DemoState::MOVE_TO_HOME:
               return moveToFinalHome();


           case DemoState::SUCCEEDED:
               RCLCPP_INFO(LOGGER, "=== DEMONSTRATION COMPLETED SUCCESSFULLY ===");
               return false;


           case DemoState::FAILED:
               RCLCPP_ERROR(LOGGER, "=== DEMONSTRATION FAILED ===");
               return false;
              
           default:
               RCLCPP_ERROR(LOGGER, "Unknown state.");
               current_state_ = DemoState::FAILED;
               return false;
       }
   }


private:
   rclcpp::Node::SharedPtr node_;
   std::string arm_planning_group_A;
   std::string arm_planning_group_B;
   std::string arm_planning_group_dual;
  
   moveit::planning_interface::MoveGroupInterface arm_move_group_A;
   moveit::planning_interface::MoveGroupInterface arm_move_group_B;
   moveit::planning_interface::MoveGroupInterface arm_move_group_dual;
   moveit::planning_interface::MoveGroupInterface::Plan plan;


   DemoState current_state_;
   std::vector<double> home_joint_values;


   char waitForKeyPress() {
       system("stty raw");
       char input = getchar();
       system("stty cooked");
       std::cout << std::endl;
       return input;
   }


   bool goToHome() {
       RCLCPP_INFO(LOGGER, "\033[32m STEP 1: Moving to Home Position \033[0m");
       RCLCPP_INFO(LOGGER, "Press any key to move arms to home position");
       waitForKeyPress();
      
       home_joint_values = arm_move_group_dual.getCurrentJointValues();

       arm_move_group_dual.setNamedTarget("Home");
       bool success = (arm_move_group_dual.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      
       if (success) {
           RCLCPP_INFO(LOGGER, "Successfully moved to home position");
           current_state_ = DemoState::PLAN_TO_POINT;
       } else {
           RCLCPP_ERROR(LOGGER, "Failed to move to home position");
           current_state_ = DemoState::FAILED;
       }
      
       return true;
   }


   bool planToPointingPosition() {
       RCLCPP_INFO(LOGGER, "\033[32m STEP 2: Planning Pointing Position \033[0m");
       RCLCPP_INFO(LOGGER, "Demonstrating joint locking - only moving specific joints to point arms at each other");
       RCLCPP_INFO(LOGGER, "Press any key to plan pointing position");
       waitForKeyPress();
      
       std::vector<double> joint_values = arm_move_group_dual.getCurrentJointValues();
      
      
       // Left arm 
       joint_values[4] = 0.5;    // base
       joint_values[5] = -1.0;   // shoulder
       joint_values[6] = 1.0;    // elbow 
      
       // Right arm 
       joint_values[11] = -0.5;   // base 
       joint_values[12] = -1.0;   // shoulder
       joint_values[13] = -1.0;   // elbow  
      
       RCLCPP_INFO(LOGGER, "Locking joints 4-7 for each arm, moving only joints 1-3");
      
       arm_move_group_dual.setJointValueTarget(joint_values);
       arm_move_group_dual.setPlanningTime(10.0);
      
       bool success = (arm_move_group_dual.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      
       if (success) {
           RCLCPP_INFO(LOGGER, "Planning to pointing position succeeded!");
           RCLCPP_INFO(LOGGER, "\033[32mPress 'r' to replan, or any other key to execute\033[0m");
           char input = waitForKeyPress();
          
           if (input == 'r' || input == 'R') {
               RCLCPP_INFO(LOGGER, "Replanning requested");
               return true;
           } else {
               current_state_ = DemoState::MOVE_TO_POINT;
               return true;
           }
       } else {
           RCLCPP_ERROR(LOGGER, "Failed to plan to pointing position");
           current_state_ = DemoState::FAILED;
           return true;
       }
   }


   bool moveToPointingPosition() {
       RCLCPP_INFO(LOGGER, "Executing movement to pointing position...");
      
       bool success = (arm_move_group_dual.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      
       if (success) {
           RCLCPP_INFO(LOGGER, "Successfully moved to pointing position!");
           RCLCPP_INFO(LOGGER, "Notice how only the top joints moved while others remained locked");
           RCLCPP_INFO(LOGGER, "\033[32mPress any key to proceed to end effector rotation\033[0m");
           waitForKeyPress();
           current_state_ = DemoState::ROTATE_END_EFFECTORS;
       } else {
           RCLCPP_ERROR(LOGGER, "Failed to execute pointing movement");
           current_state_ = DemoState::FAILED;
       }
      
       return true;
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
               bool execute_success = (arm_move_group_dual.execute(rotation_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
               if (execute_success) {
                   RCLCPP_INFO(LOGGER, "Step %d completed - notice arm positions remain unchanged!", i);
                   rclcpp::sleep_for(std::chrono::milliseconds(500));
               } else {
                   RCLCPP_ERROR(LOGGER, "Failed to execute rotation step %d", i);
                   current_state_ = DemoState::FAILED;
                   return true;
               }
           } else {
               RCLCPP_ERROR(LOGGER, "Failed to plan rotation step %d", i);
               current_state_ = DemoState::FAILED;
               return true;
           }
       }
      
       RCLCPP_INFO(LOGGER, "\033[32m360° rotation completed! End effectors rotated independently.\033[0m");
       RCLCPP_INFO(LOGGER, "Press any key to return to home position");
       waitForKeyPress();
      
       current_state_ = DemoState::PLAN_TO_HOME;
       return true;
   }


   bool planToFinalHome() {
       RCLCPP_INFO(LOGGER, "\033[32m STEP 4: Returning to Home \033[0m");
       RCLCPP_INFO(LOGGER, "Planning return to home position");
      
       arm_move_group_dual.setNamedTarget("Home");
       arm_move_group_dual.setPlanningTime(10.0);
      
       bool success = (arm_move_group_dual.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      
       if (success) {
           RCLCPP_INFO(LOGGER, "Planning to home succeeded!");
           current_state_ = DemoState::MOVE_TO_HOME;
       } else {
           RCLCPP_ERROR(LOGGER, "Failed to plan return to home");
           current_state_ = DemoState::FAILED;
       }
      
       return true;
   }


   bool moveToFinalHome() {
       RCLCPP_INFO(LOGGER, "Returning to home position...");
      
       bool success = (arm_move_group_dual.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      
       if (success) {
           RCLCPP_INFO(LOGGER, "Successfully returned to home position!");
           RCLCPP_INFO(LOGGER, "\033[32m=== DEMONSTRATION SUMMARY ===\033[0m");
           RCLCPP_INFO(LOGGER, "✓ Joint locking demonstrated - controlled specific joints while keeping others fixed");
           RCLCPP_INFO(LOGGER, "✓ Independent end effector rotation - 360° rotation without arm movement");
           RCLCPP_INFO(LOGGER, "✓ Precise robotic control achieved");
           current_state_ = DemoState::SUCCEEDED;
       } else {
           RCLCPP_ERROR(LOGGER, "Failed to return to home position");
           current_state_ = DemoState::FAILED;
       }
      
       return true;
   }
};


int main(int argc, char** argv) {
   rclcpp::init(argc, argv);
   rclcpp::NodeOptions node_options;
   node_options.automatically_declare_parameters_from_overrides(true);
   auto move_group_node = rclcpp::Node::make_shared("dual_arm_demo", node_options);


   rclcpp::Parameter sim_time_param("use_sim_time", true);
   move_group_node->set_parameter(sim_time_param);


   move_group_node->declare_parameter("robot_description_kinematics.left_arm.kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
   move_group_node->declare_parameter("robot_description_kinematics.right_arm.kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");


   rclcpp::executors::SingleThreadedExecutor executor;
   executor.add_node(move_group_node);
   std::thread spinner([&executor]() { executor.spin(); });


   static const std::string PLANNING_GROUP_A = "left_arm";
   static const std::string PLANNING_GROUP_B = "right_arm";
   static const std::string PLANNING_GROUP_DUAL = "both_arms";


   DualArmDemoFSM demo_fsm(move_group_node, PLANNING_GROUP_A, PLANNING_GROUP_B, PLANNING_GROUP_DUAL);

   while (demo_fsm.execute()) {
       rclcpp::sleep_for(std::chrono::milliseconds(100));
   }


   executor.cancel();
   spinner.join();
   rclcpp::shutdown();
   return 0;
}

