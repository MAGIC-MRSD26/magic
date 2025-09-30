
#include "object_definitions.hpp"

const rclcpp::Logger ObjectFactory::LOGGER = rclcpp::get_logger("object_factory");

ObjectParameters ObjectFactory::createBinParameters(double x, double y) {
    ObjectParameters params;
    params.width = 0.38;
    params.depth = 0.32;
    params.height = 0.266;
    params.wall_thickness = 0.01;
    params.x = x;
    params.y = y;
    params.z = 1.0646;
    params.bottom_z = params.z - (params.height / 2) + (params.wall_thickness / 2);
    params.object_id = "bin";
    calculateGraspPoses(ObjectType::BIN, params);
    return params;
}

ObjectParameters ObjectFactory::createCylinderParameters(double x, double y) {
    ObjectParameters params;
    params.cylinder_radius = 0.1;
    params.height = 0.3;
    params.spoke_length = 0.175;
    params.spoke_width = 0.05;
    params.spoke_thickness = 0.01;
    params.width = params.cylinder_radius * 2;
    params.depth = params.cylinder_radius * 2;
    params.x = x;
    params.y = y;
    params.z = 1.09;
    params.object_id = "cylinder_with_spokes";
    
    // Calculate grasp poses
    calculateGraspPoses(ObjectType::CYLINDER_WITH_SPOKES, params);
    return params;
}

void ObjectFactory::calculateGraspPoses(ObjectType type, ObjectParameters& params) {
    
    // Calculate position based on object type
    if (type == ObjectType::BIN) {
        // Set orientation for both grippers (pointing down)
        tf2::Quaternion tf2_quat;
        tf2_quat.setRPY(0, -3.14, 0);
        geometry_msgs::msg::Quaternion quat_orient;
        tf2::convert(tf2_quat, quat_orient);
        
        params.left_grasp_pose.orientation = quat_orient;
        params.right_grasp_pose.orientation = quat_orient;

        // Grasp the left and right walls of the bin
        params.left_grasp_pose.position.x = params.x + (params.width / 2) - (params.wall_thickness / 2);
        params.left_grasp_pose.position.y = params.y;
        params.left_grasp_pose.position.z = params.z + 0.034;  // Slightly above center
        
        params.right_grasp_pose.position.x = params.x - (params.width / 2) + (params.wall_thickness / 2);
        params.right_grasp_pose.position.y = params.y;
        params.right_grasp_pose.position.z = params.z + 0.034;
    } else if (type == ObjectType::CYLINDER_WITH_SPOKES) {
        // Set orientation for both grippers (pointing in)
        // Left gripper - grasp the 45° spoke
        tf2::Quaternion base_left_quat;
        base_left_quat.setRPY(0.785, -1.57, 0);
        tf2::Vector3 rotation_axis(0, 0, 1);
        tf2::Quaternion finger_rotation_left(rotation_axis, 1.57);
        tf2::Quaternion final_left_quat = base_left_quat * finger_rotation_left;
        tf2::convert(final_left_quat, params.left_grasp_pose.orientation);

        // Right gripper - grasp the 225° spoke
        tf2::Quaternion base_right_quat;
        base_right_quat.setRPY(-0.785, 1.57, 0);
        tf2::Quaternion finger_rotation_right(rotation_axis, -1.57);
        tf2::Quaternion final_right_quat = base_right_quat * finger_rotation_right;
        tf2::convert(final_right_quat, params.right_grasp_pose.orientation);

        // Calculate grasp positions for diagonal spokes
        double grasp_distance = params.spoke_length + params.cylinder_radius + 0.02;

        // Left gripper position (45° spoke end)
        params.left_grasp_pose.position.x = params.x + cos(0.785) * grasp_distance;
        params.left_grasp_pose.position.y = params.y + sin(0.785) * grasp_distance;
        params.left_grasp_pose.position.z = params.z + params.height;

        // Right gripper position (225° spoke end)
        params.right_grasp_pose.position.x = params.x + cos(3.927) * grasp_distance;
        params.right_grasp_pose.position.y = params.y + sin(3.927) * grasp_distance;
        params.right_grasp_pose.position.z = params.z + params.height;
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
    RCLCPP_INFO(ObjectFactory::LOGGER, "Created bin collision object with 5 walls");
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
    
    // // 2. Four rectangular spokes (0°, 90°, 180°, 270°)
    // for (int i = 0; i < 4; ++i) {
    //     shape_msgs::msg::SolidPrimitive spoke_primitive;
    //     spoke_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    //     spoke_primitive.dimensions.resize(3);
        
    //     geometry_msgs::msg::Pose spoke_pose;
        
    //     // Spokes extend radially from center
    //     if (i % 2 == 0) {  // 0° and 180° - along X axis
    //         spoke_primitive.dimensions[0] = params.spoke_length;
    //         spoke_primitive.dimensions[1] = params.spoke_thickness;
    //         spoke_primitive.dimensions[2] = params.spoke_width;
            
    //         spoke_pose.position.x = params.x + (i == 0 ? params.spoke_length/2 : -params.spoke_length/2);
    //         spoke_pose.position.y = params.y;
    //     } else {  // 90° and 270° - along Y axis
    //         spoke_primitive.dimensions[0] = params.spoke_thickness;
    //         spoke_primitive.dimensions[1] = params.spoke_length;
    //         spoke_primitive.dimensions[2] = params.spoke_width; 
            
    //         spoke_pose.position.x = params.x;
    //         spoke_pose.position.y = params.y + (i == 1 ? params.spoke_length/2 : -params.spoke_length/2);
    //     }
        
    //     spoke_pose.position.z = params.z;
    //     spoke_pose.orientation = quat_orientation;
        
    //     cylinder_object.primitives.push_back(spoke_primitive);
    //     cylinder_object.primitive_poses.push_back(spoke_pose);
    // }

    // 2. Four rectangular spokes (45°, 135°, 225°, 315°)
    for (int i = 0; i < 4; ++i) {
        shape_msgs::msg::SolidPrimitive spoke_primitive;
        spoke_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        spoke_primitive.dimensions.resize(3);
        
        geometry_msgs::msg::Pose spoke_pose;
        
        // Calculate angle for each spoke (45°, 135°, 225°, 315°)
        double angle = (45 + i * 90) * M_PI / 180.0;  // Convert to radians
        
        // All spokes have the same dimensions
        spoke_primitive.dimensions[0] = params.spoke_length;
        spoke_primitive.dimensions[1] = params.spoke_thickness;
        spoke_primitive.dimensions[2] = params.spoke_width;
        
        // Position spokes radially at diagonal angles
        spoke_pose.position.x = params.x + cos(angle) * params.spoke_length/2;
        spoke_pose.position.y = params.y + sin(angle) * params.spoke_length/2;
        spoke_pose.position.z = params.z;
        
        // Rotate the spoke to align with the radial direction
        tf2::Quaternion spoke_quat;
        spoke_quat.setRPY(0, 0, angle);  // Rotate around Z-axis
        spoke_pose.orientation = tf2::toMsg(spoke_quat);
        
        cylinder_object.primitives.push_back(spoke_primitive);
        cylinder_object.primitive_poses.push_back(spoke_pose);
    }
    RCLCPP_INFO(ObjectFactory::LOGGER, "Created cylinder with 4 spokes collision object");
    return cylinder_object;
}

moveit_msgs::msg::CollisionObject ObjectFactory::createObject(ObjectType type, const ObjectParameters& params) {
    switch (type) {
        case ObjectType::BIN:
            return createBin(params);
        case ObjectType::CYLINDER_WITH_SPOKES:
            return createCylinderWithSpokes(params);
        default:
            RCLCPP_ERROR(ObjectFactory::LOGGER, "Unknown object type, returning empty collision object");
            return moveit_msgs::msg::CollisionObject();
    }
}