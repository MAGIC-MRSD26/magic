// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_ign_interfaces:msg/VideoRecord.idl
// generated code does not contain a copyright notice

#ifndef ROS_IGN_INTERFACES__MSG__DETAIL__VIDEO_RECORD__BUILDER_HPP_
#define ROS_IGN_INTERFACES__MSG__DETAIL__VIDEO_RECORD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_ign_interfaces/msg/detail/video_record__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_ign_interfaces
{

namespace msg
{

namespace builder
{

class Init_VideoRecord_save_filename
{
public:
  explicit Init_VideoRecord_save_filename(::ros_ign_interfaces::msg::VideoRecord & msg)
  : msg_(msg)
  {}
  ::ros_ign_interfaces::msg::VideoRecord save_filename(::ros_ign_interfaces::msg::VideoRecord::_save_filename_type arg)
  {
    msg_.save_filename = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_ign_interfaces::msg::VideoRecord msg_;
};

class Init_VideoRecord_format
{
public:
  explicit Init_VideoRecord_format(::ros_ign_interfaces::msg::VideoRecord & msg)
  : msg_(msg)
  {}
  Init_VideoRecord_save_filename format(::ros_ign_interfaces::msg::VideoRecord::_format_type arg)
  {
    msg_.format = std::move(arg);
    return Init_VideoRecord_save_filename(msg_);
  }

private:
  ::ros_ign_interfaces::msg::VideoRecord msg_;
};

class Init_VideoRecord_stop
{
public:
  explicit Init_VideoRecord_stop(::ros_ign_interfaces::msg::VideoRecord & msg)
  : msg_(msg)
  {}
  Init_VideoRecord_format stop(::ros_ign_interfaces::msg::VideoRecord::_stop_type arg)
  {
    msg_.stop = std::move(arg);
    return Init_VideoRecord_format(msg_);
  }

private:
  ::ros_ign_interfaces::msg::VideoRecord msg_;
};

class Init_VideoRecord_start
{
public:
  explicit Init_VideoRecord_start(::ros_ign_interfaces::msg::VideoRecord & msg)
  : msg_(msg)
  {}
  Init_VideoRecord_stop start(::ros_ign_interfaces::msg::VideoRecord::_start_type arg)
  {
    msg_.start = std::move(arg);
    return Init_VideoRecord_stop(msg_);
  }

private:
  ::ros_ign_interfaces::msg::VideoRecord msg_;
};

class Init_VideoRecord_header
{
public:
  Init_VideoRecord_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VideoRecord_start header(::ros_ign_interfaces::msg::VideoRecord::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_VideoRecord_start(msg_);
  }

private:
  ::ros_ign_interfaces::msg::VideoRecord msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_ign_interfaces::msg::VideoRecord>()
{
  return ros_ign_interfaces::msg::builder::Init_VideoRecord_header();
}

}  // namespace ros_ign_interfaces

#endif  // ROS_IGN_INTERFACES__MSG__DETAIL__VIDEO_RECORD__BUILDER_HPP_
