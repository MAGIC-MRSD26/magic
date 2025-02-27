// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_ign_interfaces:msg/JointWrench.idl
// generated code does not contain a copyright notice

#ifndef ROS_IGN_INTERFACES__MSG__DETAIL__JOINT_WRENCH__BUILDER_HPP_
#define ROS_IGN_INTERFACES__MSG__DETAIL__JOINT_WRENCH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_ign_interfaces/msg/detail/joint_wrench__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_ign_interfaces
{

namespace msg
{

namespace builder
{

class Init_JointWrench_body_2_wrench
{
public:
  explicit Init_JointWrench_body_2_wrench(::ros_ign_interfaces::msg::JointWrench & msg)
  : msg_(msg)
  {}
  ::ros_ign_interfaces::msg::JointWrench body_2_wrench(::ros_ign_interfaces::msg::JointWrench::_body_2_wrench_type arg)
  {
    msg_.body_2_wrench = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_ign_interfaces::msg::JointWrench msg_;
};

class Init_JointWrench_body_1_wrench
{
public:
  explicit Init_JointWrench_body_1_wrench(::ros_ign_interfaces::msg::JointWrench & msg)
  : msg_(msg)
  {}
  Init_JointWrench_body_2_wrench body_1_wrench(::ros_ign_interfaces::msg::JointWrench::_body_1_wrench_type arg)
  {
    msg_.body_1_wrench = std::move(arg);
    return Init_JointWrench_body_2_wrench(msg_);
  }

private:
  ::ros_ign_interfaces::msg::JointWrench msg_;
};

class Init_JointWrench_body_2_id
{
public:
  explicit Init_JointWrench_body_2_id(::ros_ign_interfaces::msg::JointWrench & msg)
  : msg_(msg)
  {}
  Init_JointWrench_body_1_wrench body_2_id(::ros_ign_interfaces::msg::JointWrench::_body_2_id_type arg)
  {
    msg_.body_2_id = std::move(arg);
    return Init_JointWrench_body_1_wrench(msg_);
  }

private:
  ::ros_ign_interfaces::msg::JointWrench msg_;
};

class Init_JointWrench_body_2_name
{
public:
  explicit Init_JointWrench_body_2_name(::ros_ign_interfaces::msg::JointWrench & msg)
  : msg_(msg)
  {}
  Init_JointWrench_body_2_id body_2_name(::ros_ign_interfaces::msg::JointWrench::_body_2_name_type arg)
  {
    msg_.body_2_name = std::move(arg);
    return Init_JointWrench_body_2_id(msg_);
  }

private:
  ::ros_ign_interfaces::msg::JointWrench msg_;
};

class Init_JointWrench_body_1_id
{
public:
  explicit Init_JointWrench_body_1_id(::ros_ign_interfaces::msg::JointWrench & msg)
  : msg_(msg)
  {}
  Init_JointWrench_body_2_name body_1_id(::ros_ign_interfaces::msg::JointWrench::_body_1_id_type arg)
  {
    msg_.body_1_id = std::move(arg);
    return Init_JointWrench_body_2_name(msg_);
  }

private:
  ::ros_ign_interfaces::msg::JointWrench msg_;
};

class Init_JointWrench_body_1_name
{
public:
  explicit Init_JointWrench_body_1_name(::ros_ign_interfaces::msg::JointWrench & msg)
  : msg_(msg)
  {}
  Init_JointWrench_body_1_id body_1_name(::ros_ign_interfaces::msg::JointWrench::_body_1_name_type arg)
  {
    msg_.body_1_name = std::move(arg);
    return Init_JointWrench_body_1_id(msg_);
  }

private:
  ::ros_ign_interfaces::msg::JointWrench msg_;
};

class Init_JointWrench_header
{
public:
  Init_JointWrench_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JointWrench_body_1_name header(::ros_ign_interfaces::msg::JointWrench::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_JointWrench_body_1_name(msg_);
  }

private:
  ::ros_ign_interfaces::msg::JointWrench msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_ign_interfaces::msg::JointWrench>()
{
  return ros_ign_interfaces::msg::builder::Init_JointWrench_header();
}

}  // namespace ros_ign_interfaces

#endif  // ROS_IGN_INTERFACES__MSG__DETAIL__JOINT_WRENCH__BUILDER_HPP_
