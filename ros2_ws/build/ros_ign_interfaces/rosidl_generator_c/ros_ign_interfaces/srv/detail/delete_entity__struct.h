// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_ign_interfaces:srv/DeleteEntity.idl
// generated code does not contain a copyright notice

#ifndef ROS_IGN_INTERFACES__SRV__DETAIL__DELETE_ENTITY__STRUCT_H_
#define ROS_IGN_INTERFACES__SRV__DETAIL__DELETE_ENTITY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'entity'
#include "ros_gz_interfaces/msg/detail/entity__struct.h"

/// Struct defined in srv/DeleteEntity in the package ros_ign_interfaces.
typedef struct ros_ign_interfaces__srv__DeleteEntity_Request
{
  /// Gazebo Sim entity to be deleted.
  ros_gz_interfaces__msg__Entity entity;
} ros_ign_interfaces__srv__DeleteEntity_Request;

// Struct for a sequence of ros_ign_interfaces__srv__DeleteEntity_Request.
typedef struct ros_ign_interfaces__srv__DeleteEntity_Request__Sequence
{
  ros_ign_interfaces__srv__DeleteEntity_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_ign_interfaces__srv__DeleteEntity_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/DeleteEntity in the package ros_ign_interfaces.
typedef struct ros_ign_interfaces__srv__DeleteEntity_Response
{
  /// Return true if deletion is successful.
  bool success;
} ros_ign_interfaces__srv__DeleteEntity_Response;

// Struct for a sequence of ros_ign_interfaces__srv__DeleteEntity_Response.
typedef struct ros_ign_interfaces__srv__DeleteEntity_Response__Sequence
{
  ros_ign_interfaces__srv__DeleteEntity_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_ign_interfaces__srv__DeleteEntity_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_IGN_INTERFACES__SRV__DETAIL__DELETE_ENTITY__STRUCT_H_
