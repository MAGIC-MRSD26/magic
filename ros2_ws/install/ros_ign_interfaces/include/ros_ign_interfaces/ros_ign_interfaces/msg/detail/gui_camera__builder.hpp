// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_ign_interfaces:msg/GuiCamera.idl
// generated code does not contain a copyright notice

#ifndef ROS_IGN_INTERFACES__MSG__DETAIL__GUI_CAMERA__BUILDER_HPP_
#define ROS_IGN_INTERFACES__MSG__DETAIL__GUI_CAMERA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_ign_interfaces/msg/detail/gui_camera__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_ign_interfaces
{

namespace msg
{

namespace builder
{

class Init_GuiCamera_projection_type
{
public:
  explicit Init_GuiCamera_projection_type(::ros_ign_interfaces::msg::GuiCamera & msg)
  : msg_(msg)
  {}
  ::ros_ign_interfaces::msg::GuiCamera projection_type(::ros_ign_interfaces::msg::GuiCamera::_projection_type_type arg)
  {
    msg_.projection_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_ign_interfaces::msg::GuiCamera msg_;
};

class Init_GuiCamera_track
{
public:
  explicit Init_GuiCamera_track(::ros_ign_interfaces::msg::GuiCamera & msg)
  : msg_(msg)
  {}
  Init_GuiCamera_projection_type track(::ros_ign_interfaces::msg::GuiCamera::_track_type arg)
  {
    msg_.track = std::move(arg);
    return Init_GuiCamera_projection_type(msg_);
  }

private:
  ::ros_ign_interfaces::msg::GuiCamera msg_;
};

class Init_GuiCamera_pose
{
public:
  explicit Init_GuiCamera_pose(::ros_ign_interfaces::msg::GuiCamera & msg)
  : msg_(msg)
  {}
  Init_GuiCamera_track pose(::ros_ign_interfaces::msg::GuiCamera::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_GuiCamera_track(msg_);
  }

private:
  ::ros_ign_interfaces::msg::GuiCamera msg_;
};

class Init_GuiCamera_view_controller
{
public:
  explicit Init_GuiCamera_view_controller(::ros_ign_interfaces::msg::GuiCamera & msg)
  : msg_(msg)
  {}
  Init_GuiCamera_pose view_controller(::ros_ign_interfaces::msg::GuiCamera::_view_controller_type arg)
  {
    msg_.view_controller = std::move(arg);
    return Init_GuiCamera_pose(msg_);
  }

private:
  ::ros_ign_interfaces::msg::GuiCamera msg_;
};

class Init_GuiCamera_name
{
public:
  explicit Init_GuiCamera_name(::ros_ign_interfaces::msg::GuiCamera & msg)
  : msg_(msg)
  {}
  Init_GuiCamera_view_controller name(::ros_ign_interfaces::msg::GuiCamera::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_GuiCamera_view_controller(msg_);
  }

private:
  ::ros_ign_interfaces::msg::GuiCamera msg_;
};

class Init_GuiCamera_header
{
public:
  Init_GuiCamera_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GuiCamera_name header(::ros_ign_interfaces::msg::GuiCamera::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GuiCamera_name(msg_);
  }

private:
  ::ros_ign_interfaces::msg::GuiCamera msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_ign_interfaces::msg::GuiCamera>()
{
  return ros_ign_interfaces::msg::builder::Init_GuiCamera_header();
}

}  // namespace ros_ign_interfaces

#endif  // ROS_IGN_INTERFACES__MSG__DETAIL__GUI_CAMERA__BUILDER_HPP_
