#include "moveit_go/object_definitions.hpp"

const rclcpp::Logger ObjectFactory::LOGGER = rclcpp::get_logger("object_factory");

ObjectParameters ObjectFactory::createBinParameters(double x, double y) {
    ObjectParameters params;
    
    // Bin dimensions
    params.width = 0.38;        // X dimension (length)
    params.depth = 0.32;        // Y dimension (width)
    params.height = 0.266;      // Z dimension (height)
    params.wall_thickness = 0.01; // Wall thickness
    
    // Position
    params.x = x;
    params.y = y;
    params.z = 1.0646;  // height of table is 0.9316
    params.bottom_z = params.z - (params.height / 2) + (params.wall_thickness / 2);
    
    // Object ID
    params.object_id = "bin";
    
    // Calculate grasp poses
    calculateGraspPoses(ObjectType::BIN, params);
    
    return params;
}

ObjectParameters ObjectFactory::createCylinderParameters(double x, double y) {
    ObjectParameters params;
    
    // Cylinder dimensions
    params.cylinder_radius = 0.1;  // 1cm radius
    params.height = 0.3;            // 30cm height
    params.spoke_length = 0.175;     // Spokes extend beyond cylinder
    params.spoke_width = 0.05;      // 5cm wide spokes
    params.spoke_thickness = 0.01;  // 1cm thick spokes
    
    // For compatibility with grasp calculations
    params.width = params.cylinder_radius * 2;  // Diameter for X
    params.depth = params.cylinder_radius * 2;  // Diameter for Y
    
    // Position
    params.x = x;
    params.y = y;
    params.z = 1.0646;
    params.bottom_z = params.z - (params.height / 2);
    
    // Object ID
    params.object_id = "cylinder_with_spokes";
    
    // Calculate grasp poses
    calculateGraspPoses(ObjectType::CYLINDER_WITH_SPOKES, params);
    
    return params;
}

void ObjectFactory::calculateGraspPoses(ObjectType type, ObjectParameters& params) {
    // Set orientation for both grippers (pointing down)
    tf2::Quaternion tf2_quat;
    tf2_quat.setRPY(0, -3.14, 0);
    geometry_msgs::msg::Quaternion quat_orient;
    tf2::convert(tf2_quat, quat_orient);
    
    params.left_grasp_pose.orientation = quat_orient;
    params.right_grasp_pose.orientation = quat_orient;
    
    // Calculate position based on object type
    if (type == ObjectType::BIN) {
        // Grasp the left and right walls of the bin
        params.left_grasp_pose.position.x = params.x + (params.width / 2) - (params.wall_thickness / 2);
        params.left_grasp_pose.position.y = params.y;
        params.left_grasp_pose.position.z = params.z + 0.034;  // Slightly above center
        
        params.right_grasp_pose.position.x = params.x - (params.width / 2) + (params.wall_thickness / 2);
        params.right_grasp_pose.position.y = params.y;
        params.right_grasp_pose.position.z = params.z + 0.034;
    } else if (type == ObjectType::CYLINDER_WITH_SPOKES) {
        // Grasp the ends of the horizontal spokes
        params.left_grasp_pose.position.x = params.x + (params.spoke_length / 2) - 0.05;
        params.left_grasp_pose.position.y = params.y;
        params.left_grasp_pose.position.z = params.z;
        
        params.right_grasp_pose.position.x = params.x - (params.spoke_length / 2) + 0.05;
        params.right_grasp_pose.position.y = params.y;
        params.right_grasp_pose.position.z = params.z;
    }
}

moveit_msgs::msg::CollisionObject ObjectFactory::createBin(const ObjectParameters& params) {
    moveit_msgs::msg::CollisionObject bin_object;
    bin_object.id = params.object_id;
    bin_object.header.frame_id = "world";
    bin_object.operation = moveit_msgs::msg::CollisionObject::ADD;
    
    // Create orientation
    tf2::Quaternion bin_quat;
    bin_quat.setRPY(0, 0, 0);
    geometry_msgs::msg::Quaternion bin_orientation = tf2::toMsg(bin_quat);
    
    // 1. Bottom of the bin
    shape_msgs::msg::SolidPrimitive bottom_primitive;
    bottom_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    bottom_primitive.dimensions.resize(3);
    bottom_primitive.dimensions[0] = params.width;
    bottom_primitive.dimensions[1] = params.depth;
    bottom_primitive.dimensions[2] = params.wall_thickness;
    
    geometry_msgs::msg::Pose bottom_pose;
    bottom_pose.orientation = bin_orientation;
    bottom_pose.position.x = params.x;
    bottom_pose.position.y = params.y;
    bottom_pose.position.z = params.bottom_z;
    
    bin_object.primitives.push_back(bottom_primitive);
    bin_object.primitive_poses.push_back(bottom_pose);
    
    // 2. Front wall
    shape_msgs::msg::SolidPrimitive front_primitive;
    front_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    front_primitive.dimensions.resize(3);
    front_primitive.dimensions[0] = params.width;
    front_primitive.dimensions[1] = params.wall_thickness;
    front_primitive.dimensions[2] = params.height;
    
    geometry_msgs::msg::Pose front_pose;
    front_pose.orientation = bin_orientation;
    front_pose.position.x = params.x;
    front_pose.position.y = params.y + (params.depth / 2) - (params.wall_thickness / 2);
    front_pose.position.z = params.z;
    
    bin_object.primitives.push_back(front_primitive);
    bin_object.primitive_poses.push_back(front_pose);
    
    // 3. Back wall
    shape_msgs::msg::SolidPrimitive back_primitive;
    back_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    back_primitive.dimensions.resize(3);
    back_primitive.dimensions[0] = params.width;
    back_primitive.dimensions[1] = params.wall_thickness;
    back_primitive.dimensions[2] = params.height;
    
    geometry_msgs::msg::Pose back_pose;
    back_pose.orientation = bin_orientation;
    back_pose.position.x = params.x;
    back_pose.position.y = params.y - (params.depth / 2) + (params.wall_thickness / 2);
    back_pose.position.z = params.z;
    
    bin_object.primitives.push_back(back_primitive);
    bin_object.primitive_poses.push_back(back_pose);
    
    // 4. Left wall
    shape_msgs::msg::SolidPrimitive left_primitive;
    left_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    left_primitive.dimensions.resize(3);
    left_primitive.dimensions[0] = params.wall_thickness;
    left_primitive.dimensions[1] = params.depth;
    left_primitive.dimensions[2] = params.height;
    
    geometry_msgs::msg::Pose left_pose;
    left_pose.orientation = bin_orientation;
    left_pose.position.x = params.x - (params.width / 2) + (params.wall_thickness / 2);
    left_pose.position.y = params.y;
    left_pose.position.z = params.z;
    
    bin_object.primitives.push_back(left_primitive);
    bin_object.primitive_poses.push_back(left_pose);
    
    // 5. Right wall
    shape_msgs::msg::SolidPrimitive right_primitive;
    right_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    right_primitive.dimensions.resize(3);
    right_primitive.dimensions[0] = params.wall_thickness;
    right_primitive.dimensions[1] = params.depth;
    right_primitive.dimensions[2] = params.height;
    
    geometry_msgs::msg::Pose right_pose;
    right_pose.orientation = bin_orientation;
    right_pose.position.x = params.x + (params.width / 2) - (params.wall_thickness / 2);
    right_pose.position.y = params.y;
    right_pose.position.z = params.z;
    
    bin_object.primitives.push_back(right_primitive);
    bin_object.primitive_poses.push_back(right_pose);
    
    RCLCPP_INFO(LOGGER, "Created bin collision object with 5 walls");
    return bin_object;
}

moveit_msgs::msg::CollisionObject ObjectFactory::createCylinderWithSpokes(const ObjectParameters& params) {
    moveit_msgs::msg::CollisionObject cylinder_object;
    cylinder_object.id = params.object_id;
    cylinder_object.header.frame_id = "world";
    cylinder_object.operation = moveit_msgs::msg::CollisionObject::ADD;
    
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, 0);
    geometry_msgs::msg::Quaternion quat_orientation = tf2::toMsg(orientation);
    
    // 1. Central cylinder
    shape_msgs::msg::SolidPrimitive cylinder_primitive;
    cylinder_primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cylinder_primitive.dimensions.resize(2);
    cylinder_primitive.dimensions[0] = params.height;  // height
    cylinder_primitive.dimensions[1] = params.cylinder_radius;  // radius
    
    geometry_msgs::msg::Pose cylinder_pose;
    cylinder_pose.orientation = quat_orientation;
    cylinder_pose.position.x = params.x;
    cylinder_pose.position.y = params.y;
    cylinder_pose.position.z = params.z;
    
    cylinder_object.primitives.push_back(cylinder_primitive);
    cylinder_object.primitive_poses.push_back(cylinder_pose);
    
    // 2. Four rectangular spokes (0°, 90°, 180°, 270°)
    for (int i = 0; i < 4; ++i) {
        shape_msgs::msg::SolidPrimitive spoke_primitive;
        spoke_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        spoke_primitive.dimensions.resize(3);
        
        geometry_msgs::msg::Pose spoke_pose;
        
        // Spokes extend radially from center
        if (i % 2 == 0) {  // 0° and 180° - along X axis
            spoke_primitive.dimensions[0] = params.spoke_length;
            spoke_primitive.dimensions[1] = params.spoke_thickness;
            spoke_primitive.dimensions[2] = params.spoke_width;
            
            spoke_pose.position.x = params.x + (i == 0 ? params.spoke_length/2 : -params.spoke_length/2);
            spoke_pose.position.y = params.y;
        } else {  // 90° and 270° - along Y axis
            spoke_primitive.dimensions[0] = params.spoke_thickness;
            spoke_primitive.dimensions[1] = params.spoke_length;
            spoke_primitive.dimensions[2] = params.spoke_width; 
            
            spoke_pose.position.x = params.x;
            spoke_pose.position.y = params.y + (i == 1 ? params.spoke_length/2 : -params.spoke_length/2);
        }
        
        spoke_pose.position.z = params.z;
        spoke_pose.orientation = quat_orientation;
        
        cylinder_object.primitives.push_back(spoke_primitive);
        cylinder_object.primitive_poses.push_back(spoke_pose);
    }
    
    RCLCPP_INFO(LOGGER, "Created cylinder with 4 spokes collision object");
    return cylinder_object;
}

moveit_msgs::msg::CollisionObject ObjectFactory::createObject(ObjectType type, const ObjectParameters& params) {
    switch (type) {
        case ObjectType::BIN:
            return createBin(params);
        case ObjectType::CYLINDER_WITH_SPOKES:
            return createCylinderWithSpokes(params);
        default:
            RCLCPP_ERROR(LOGGER, "Unknown object type, returning empty collision object");
            return moveit_msgs::msg::CollisionObject();
    }
}