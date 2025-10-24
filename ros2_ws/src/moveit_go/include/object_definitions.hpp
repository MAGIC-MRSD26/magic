#ifndef OBJECT_DEFINITIONS_HPP
#define OBJECT_DEFINITIONS_HPP

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>

enum class ObjectType {
    BIN,
    CYLINDER_WITH_SPOKES
};

struct ObjectParameters {
    // Common dimensions
    double width = 0.0;       // X dimension (bin) or diameter (cylinder)
    double depth = 0.0;       // Y dimension (bin) or diameter (cylinder)
    double height = 0.0;      // Z dimension
    double wall_thickness = 0.0; // Wall thickness (bin) or spoke thickness (cylinder)
    double rotation_angle = 0.0; 


    // Position
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double bottom_z = 0.0;

    // Cylinder-specific
    double cylinder_radius = 0.0;
    double spoke_length = 0.0;
    double spoke_width = 0.0;
    double spoke_thickness = 0.0;

    // Grasp poses
    geometry_msgs::msg::Pose left_grasp_pose;
    geometry_msgs::msg::Pose right_grasp_pose;
    geometry_msgs::msg::Pose second_left_grasp_pose;
    geometry_msgs::msg::Pose second_right_grasp_pose;

    // Object ID
    std::string object_id;
};

class ObjectFactory {
public:
    // Create parameters for a bin
    static ObjectParameters createBinParameters(double x = 0.0, double y = 0.0);

    // Create parameters for a cylinder with spokes
    static ObjectParameters createCylinderParameters(double x = 0.0, double y = 0.0, double rotation_angle = 45.0);

    // Calculate grasp poses for an object
    static void calculateGraspPoses(ObjectType type, ObjectParameters& params);

    // Create a collision object for a bin
    static moveit_msgs::msg::CollisionObject createBin(const ObjectParameters& params);

    // Create a collision object for a cylinder with spokes
    static moveit_msgs::msg::CollisionObject createCylinderWithSpokes(const ObjectParameters& params);

    // General object creation
    static moveit_msgs::msg::CollisionObject createObject(ObjectType type, const ObjectParameters& params);

    // Logger for info/debug
    static const rclcpp::Logger LOGGER;
};

#endif