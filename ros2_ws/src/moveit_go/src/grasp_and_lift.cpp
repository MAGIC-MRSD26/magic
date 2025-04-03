#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/cost_terms.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("kinova_mtc_grasp");

using namespace moveit::task_constructor;

void setupDemoScene(std::shared_ptr<rclcpp::Node> node) {
    // Create planning scene interface
    moveit::planning_interface::PlanningSceneInterface psi;
    
    // Add a table
    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = "base_link";
    table.id = "table";
    
    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = 0.5;
    table_pose.position.y = 0.0;
    table_pose.position.z = -0.07;
    table_pose.orientation.w = 1.0;
    
    table.primitives.resize(1);
    table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    table.primitives[0].dimensions = {0.6, 1.0, 0.05};
    table.pose = table_pose;
    
    // Add an object to grasp
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = "base_link";
    object.id = "object";
    
    geometry_msgs::msg::Pose object_pose;
    object_pose.position.x = 0.5;
    object_pose.position.y = 0.0;
    object_pose.position.z = 0.0;
    object_pose.orientation.w = 1.0;
    
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = {0.15, 0.03}; // height, radius
    object.pose = object_pose;
    
    // Add objects to planning scene
    psi.applyCollisionObjects({table, object});
    
    RCLCPP_INFO(LOGGER, "Added table and object to planning scene");
}

Task createGraspTask(std::shared_ptr<rclcpp::Node> node) {
    Task task("kinova_grasp_task");
    task.stages()->setName("Kinova Grasp Pipeline");
    
    // Parameters (customize these for your Kinova robot)
    const std::string arm_group = "arm"; // Your arm group name (e.g., "arm", "manipulator", etc.)
    const std::string gripper_group = "gripper"; // Your gripper group name
    const std::string eef_link = "tool_frame"; // End effector link name
    
    // Set task properties
    task.loadRobotModel(node);
    
    // Planner used for most stages
    auto pipeline = std::make_shared<solvers::PipelinePlanner>(node);
    pipeline->setPlannerId("RRTConnect");
    
    // Cartesian planner for some stages
    auto cartesian = std::make_shared<solvers::CartesianPath>();
    cartesian->setMaxVelocityScaling(0.1);
    cartesian->setMaxAccelerationScaling(0.1);
    cartesian->setStepSize(0.01);
    
    // Set task properties
    task.setProperty("group", arm_group);
    task.setProperty("eef", gripper_group);
    task.setProperty("ik_frame", eef_link);
    
    /******************************************************
     *                      Task Stages                    *
     ******************************************************/
    
    // Current state stage
    Stage* current_state = nullptr;
    {
        auto stage = std::make_unique<stages::CurrentState>("current state");
        current_state = stage.get();
        task.add(std::move(stage));
    }
    
    // Open gripper stage
    Stage* open_gripper = nullptr;
    {
        auto stage = std::make_unique<stages::MoveTo>("open gripper", pipeline);
        stage->setGroup(gripper_group);
        stage->setGoal("open"); // Named joint target from SRDF
        open_gripper = stage.get();
        task.add(std::move(stage));
    }
    
    // Connect to pick stage
    stages::Connect::GroupPlannerVector planners = {{arm_group, pipeline}};
    auto connect = std::make_unique<stages::Connect>("connect", planners);
    connect->properties().configureInitFrom(Stage::PARENT);
    task.add(std::move(connect));
    
    // Pick stage (includes approach, grasp, lift)
    auto pick = std::make_unique<stages::SerialContainer>("pick object");
    pick->properties().configureInitFrom(Stage::PARENT, {"eef", "group"});
    task.add(std::move(pick));
    
    // Grasp generator stage
    {
        // Sample grasp poses based on object location
        auto stage = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
        stage->properties().configureInitFrom(Stage::PARENT);
        stage->properties().set("marker_ns", "grasp_poses");
        stage->setPreGraspPose("open");
        stage->setObject("object");
        stage->setAngleDelta(0.2);
        
        // Define transforms for grasp generation
        geometry_msgs::msg::PoseStamped grasp_frame_transform;
        grasp_frame_transform.header.frame_id = eef_link;
        grasp_frame_transform.pose.orientation.w = 1.0;
        
        // Set the grasp frame
        stage->setGraspFrame(grasp_frame_transform);
        
        // Set grasp pose
        stage->setGraspPose("open");
        
        pick->insert(std::move(stage));
    }
    
    // Approach object stage
    {
        auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian);
        stage->properties().set("marker_ns", "approach");
        stage->properties().set("link", eef_link);
        stage->properties().configureInitFrom(Stage::PARENT, {"group"});
        
        // Set direction: approach along eef z-axis
        geometry_msgs::msg::Vector3Stamped approach_direction;
        approach_direction.header.frame_id = eef_link;
        approach_direction.vector.z = 1.0;  // Assuming tool_frame z-axis points toward the object
        stage->setDirection(approach_direction);
        stage->setMinMaxDistance(0.1, 0.15);  // Move 10-15cm
        
        pick->insert(std::move(stage));
    }
    
    // Close gripper stage
    {
        auto stage = std::make_unique<stages::MoveTo>("close gripper", pipeline);
        stage->setGroup(gripper_group);
        stage->setGoal("closed");  // Named joint target from SRDF
        pick->insert(std::move(stage));
    }
    
    // Attach object stage
    {
        auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
        stage->attachObject("object", eef_link);
        pick->insert(std::move(stage));
    }
    
    // Lift object stage
    {
        auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian);
        stage->properties().configureInitFrom(Stage::PARENT, {"group"});
        stage->setMinMaxDistance(0.1, 0.15);  // Lift 10-15cm
        
        // Set direction: lift along world z-axis
        geometry_msgs::msg::Vector3Stamped lift_direction;
        lift_direction.header.frame_id = "base_link";  // World frame
        lift_direction.vector.z = 1.0;
        stage->setDirection(lift_direction);
        
        pick->insert(std::move(stage));
    }
    
    // Move to transport position
    {
        auto stage = std::make_unique<stages::MoveTo>("move to transport position", pipeline);
        stage->properties().configureInitFrom(Stage::PARENT, {"group"});
        stage->setGoal("transport");  // Named target from SRDF
        task.add(std::move(stage));
    }
    
    return task;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("kinova_mtc_grasp_node");
    
    // Setup demo scene (table and object)
    setupDemoScene(node);
    
    // Create MTC task
    auto task = createGraspTask(node);
    
    // Print task structure
    task.printState();
    
    // Plan and execute
    try {
        RCLCPP_INFO(LOGGER, "Planning and executing task...");
        task.plan();
        
        // Get solutions
        auto solutions = task.solutions();
        RCLCPP_INFO(LOGGER, "Found %ld solutions", solutions.size());
        
        if (!solutions.empty()) {
            // Visualize first solution
            RCLCPP_INFO(LOGGER, "Executing first solution");
            task.introspection().publishSolution(*solutions.front());
            
            // To actually execute the solution, uncomment this line:
            // solutions.front()->execute();
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "Task planning failed: %s", e.what());
        return 1;
    }
    
    // Keep the node running
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}