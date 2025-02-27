// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_ign_interfaces:srv/SetEntityPose.idl
// generated code does not contain a copyright notice

#ifndef ROS_IGN_INTERFACES__SRV__DETAIL__SET_ENTITY_POSE__BUILDER_HPP_
#define ROS_IGN_INTERFACES__SRV__DETAIL__SET_ENTITY_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_ign_interfaces/srv/detail/set_entity_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_ign_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetEntityPose_Request_pose
{
public:
  explicit Init_SetEntityPose_Request_pose(::ros_ign_interfaces::srv::SetEntityPose_Request & msg)
  : msg_(msg)
  {}
  ::ros_ign_interfaces::srv::SetEntityPose_Request pose(::ros_ign_interfaces::srv::SetEntityPose_Request::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_ign_interfaces::srv::SetEntityPose_Request msg_;
};

class Init_SetEntityPose_Request_entity
{
public:
  Init_SetEntityPose_Request_entity()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetEntityPose_Request_pose entity(::ros_ign_interfaces::srv::SetEntityPose_Request::_entity_type arg)
  {
    msg_.entity = std::move(arg);
    return Init_SetEntityPose_Request_pose(msg_);
  }

private:
  ::ros_ign_interfaces::srv::SetEntityPose_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_ign_interfaces::srv::SetEntityPose_Request>()
{
  return ros_ign_interfaces::srv::builder::Init_SetEntityPose_Request_entity();
}

}  // namespace ros_ign_interfaces


namespace ros_ign_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetEntityPose_Response_success
{
public:
  Init_SetEntityPose_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ros_ign_interfaces::srv::SetEntityPose_Response success(::ros_ign_interfaces::srv::SetEntityPose_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_ign_interfaces::srv::SetEntityPose_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_ign_interfaces::srv::SetEntityPose_Response>()
{
  return ros_ign_interfaces::srv::builder::Init_SetEntityPose_Response_success();
}

}  // namespace ros_ign_interfaces

#endif  // ROS_IGN_INTERFACES__SRV__DETAIL__SET_ENTITY_POSE__BUILDER_HPP_
