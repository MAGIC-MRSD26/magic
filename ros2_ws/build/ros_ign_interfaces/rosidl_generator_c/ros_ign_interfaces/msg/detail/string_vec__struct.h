// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_ign_interfaces:msg/StringVec.idl
// generated code does not contain a copyright notice

#ifndef ROS_IGN_INTERFACES__MSG__DETAIL__STRING_VEC__STRUCT_H_
#define ROS_IGN_INTERFACES__MSG__DETAIL__STRING_VEC__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'data'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/StringVec in the package ros_ign_interfaces.
/**
  * A message for a vector of string data.
 */
typedef struct ros_ign_interfaces__msg__StringVec
{
  /// Optional header data.
  std_msgs__msg__Header header;
  /// The vector of strings.
  rosidl_runtime_c__String__Sequence data;
} ros_ign_interfaces__msg__StringVec;

// Struct for a sequence of ros_ign_interfaces__msg__StringVec.
typedef struct ros_ign_interfaces__msg__StringVec__Sequence
{
  ros_ign_interfaces__msg__StringVec * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_ign_interfaces__msg__StringVec__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_IGN_INTERFACES__MSG__DETAIL__STRING_VEC__STRUCT_H_
