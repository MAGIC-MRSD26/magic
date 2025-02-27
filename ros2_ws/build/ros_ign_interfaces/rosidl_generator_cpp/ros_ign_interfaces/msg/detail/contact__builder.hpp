// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_ign_interfaces:msg/Contact.idl
// generated code does not contain a copyright notice

#ifndef ROS_IGN_INTERFACES__MSG__DETAIL__CONTACT__BUILDER_HPP_
#define ROS_IGN_INTERFACES__MSG__DETAIL__CONTACT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_ign_interfaces/msg/detail/contact__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_ign_interfaces
{

namespace msg
{

namespace builder
{

class Init_Contact_wrenches
{
public:
  explicit Init_Contact_wrenches(::ros_ign_interfaces::msg::Contact & msg)
  : msg_(msg)
  {}
  ::ros_ign_interfaces::msg::Contact wrenches(::ros_ign_interfaces::msg::Contact::_wrenches_type arg)
  {
    msg_.wrenches = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_ign_interfaces::msg::Contact msg_;
};

class Init_Contact_depths
{
public:
  explicit Init_Contact_depths(::ros_ign_interfaces::msg::Contact & msg)
  : msg_(msg)
  {}
  Init_Contact_wrenches depths(::ros_ign_interfaces::msg::Contact::_depths_type arg)
  {
    msg_.depths = std::move(arg);
    return Init_Contact_wrenches(msg_);
  }

private:
  ::ros_ign_interfaces::msg::Contact msg_;
};

class Init_Contact_normals
{
public:
  explicit Init_Contact_normals(::ros_ign_interfaces::msg::Contact & msg)
  : msg_(msg)
  {}
  Init_Contact_depths normals(::ros_ign_interfaces::msg::Contact::_normals_type arg)
  {
    msg_.normals = std::move(arg);
    return Init_Contact_depths(msg_);
  }

private:
  ::ros_ign_interfaces::msg::Contact msg_;
};

class Init_Contact_positions
{
public:
  explicit Init_Contact_positions(::ros_ign_interfaces::msg::Contact & msg)
  : msg_(msg)
  {}
  Init_Contact_normals positions(::ros_ign_interfaces::msg::Contact::_positions_type arg)
  {
    msg_.positions = std::move(arg);
    return Init_Contact_normals(msg_);
  }

private:
  ::ros_ign_interfaces::msg::Contact msg_;
};

class Init_Contact_collision2
{
public:
  explicit Init_Contact_collision2(::ros_ign_interfaces::msg::Contact & msg)
  : msg_(msg)
  {}
  Init_Contact_positions collision2(::ros_ign_interfaces::msg::Contact::_collision2_type arg)
  {
    msg_.collision2 = std::move(arg);
    return Init_Contact_positions(msg_);
  }

private:
  ::ros_ign_interfaces::msg::Contact msg_;
};

class Init_Contact_collision1
{
public:
  Init_Contact_collision1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Contact_collision2 collision1(::ros_ign_interfaces::msg::Contact::_collision1_type arg)
  {
    msg_.collision1 = std::move(arg);
    return Init_Contact_collision2(msg_);
  }

private:
  ::ros_ign_interfaces::msg::Contact msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_ign_interfaces::msg::Contact>()
{
  return ros_ign_interfaces::msg::builder::Init_Contact_collision1();
}

}  // namespace ros_ign_interfaces

#endif  // ROS_IGN_INTERFACES__MSG__DETAIL__CONTACT__BUILDER_HPP_
