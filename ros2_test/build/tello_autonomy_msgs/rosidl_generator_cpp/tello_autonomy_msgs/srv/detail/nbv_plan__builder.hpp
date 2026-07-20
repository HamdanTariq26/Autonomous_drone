// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tello_autonomy_msgs:srv/NbvPlan.idl
// generated code does not contain a copyright notice

#ifndef TELLO_AUTONOMY_MSGS__SRV__DETAIL__NBV_PLAN__BUILDER_HPP_
#define TELLO_AUTONOMY_MSGS__SRV__DETAIL__NBV_PLAN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tello_autonomy_msgs/srv/detail/nbv_plan__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tello_autonomy_msgs
{

namespace srv
{

namespace builder
{

class Init_NbvPlan_Request_header
{
public:
  Init_NbvPlan_Request_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::tello_autonomy_msgs::srv::NbvPlan_Request header(::tello_autonomy_msgs::srv::NbvPlan_Request::_header_type arg)
  {
    msg_.header = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tello_autonomy_msgs::srv::NbvPlan_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::tello_autonomy_msgs::srv::NbvPlan_Request>()
{
  return tello_autonomy_msgs::srv::builder::Init_NbvPlan_Request_header();
}

}  // namespace tello_autonomy_msgs


namespace tello_autonomy_msgs
{

namespace srv
{

namespace builder
{

class Init_NbvPlan_Response_path
{
public:
  Init_NbvPlan_Response_path()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::tello_autonomy_msgs::srv::NbvPlan_Response path(::tello_autonomy_msgs::srv::NbvPlan_Response::_path_type arg)
  {
    msg_.path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tello_autonomy_msgs::srv::NbvPlan_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::tello_autonomy_msgs::srv::NbvPlan_Response>()
{
  return tello_autonomy_msgs::srv::builder::Init_NbvPlan_Response_path();
}

}  // namespace tello_autonomy_msgs

#endif  // TELLO_AUTONOMY_MSGS__SRV__DETAIL__NBV_PLAN__BUILDER_HPP_
