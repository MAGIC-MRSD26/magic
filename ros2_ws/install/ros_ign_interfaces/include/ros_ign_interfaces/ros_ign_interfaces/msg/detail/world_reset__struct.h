// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_ign_interfaces:msg/WorldReset.idl
// generated code does not contain a copyright notice

#ifndef ROS_IGN_INTERFACES__MSG__DETAIL__WORLD_RESET__STRUCT_H_
#define ROS_IGN_INTERFACES__MSG__DETAIL__WORLD_RESET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/WorldReset in the package ros_ign_interfaces.
typedef struct ros_ign_interfaces__msg__WorldReset
{
  /// Reset time and model
  bool all;
  /// Reset time only
  bool time_only;
  /// Reset model only
  bool model_only;
} ros_ign_interfaces__msg__WorldReset;

// Struct for a sequence of ros_ign_interfaces__msg__WorldReset.
typedef struct ros_ign_interfaces__msg__WorldReset__Sequence
{
  ros_ign_interfaces__msg__WorldReset * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_ign_interfaces__msg__WorldReset__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_IGN_INTERFACES__MSG__DETAIL__WORLD_RESET__STRUCT_H_
