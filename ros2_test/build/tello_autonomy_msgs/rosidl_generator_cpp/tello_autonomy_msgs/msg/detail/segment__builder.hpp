// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tello_autonomy_msgs:msg/Segment.idl
// generated code does not contain a copyright notice

#ifndef TELLO_AUTONOMY_MSGS__MSG__DETAIL__SEGMENT__BUILDER_HPP_
#define TELLO_AUTONOMY_MSGS__MSG__DETAIL__SEGMENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tello_autonomy_msgs/msg/detail/segment__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tello_autonomy_msgs
{

namespace msg
{

namespace builder
{

class Init_Segment_poses
{
public:
  explicit Init_Segment_poses(::tello_autonomy_msgs::msg::Segment & msg)
  : msg_(msg)
  {}
  ::tello_autonomy_msgs::msg::Segment poses(::tello_autonomy_msgs::msg::Segment::_poses_type arg)
  {
    msg_.poses = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tello_autonomy_msgs::msg::Segment msg_;
};

class Init_Segment_header
{
public:
  Init_Segment_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Segment_poses header(::tello_autonomy_msgs::msg::Segment::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Segment_poses(msg_);
  }

private:
  ::tello_autonomy_msgs::msg::Segment msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tello_autonomy_msgs::msg::Segment>()
{
  return tello_autonomy_msgs::msg::builder::Init_Segment_header();
}

}  // namespace tello_autonomy_msgs

#endif  // TELLO_AUTONOMY_MSGS__MSG__DETAIL__SEGMENT__BUILDER_HPP_
