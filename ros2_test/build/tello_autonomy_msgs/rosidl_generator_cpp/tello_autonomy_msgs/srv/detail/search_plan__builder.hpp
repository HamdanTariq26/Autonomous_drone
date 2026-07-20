// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tello_autonomy_msgs:srv/SearchPlan.idl
// generated code does not contain a copyright notice

#ifndef TELLO_AUTONOMY_MSGS__SRV__DETAIL__SEARCH_PLAN__BUILDER_HPP_
#define TELLO_AUTONOMY_MSGS__SRV__DETAIL__SEARCH_PLAN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tello_autonomy_msgs/srv/detail/search_plan__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tello_autonomy_msgs
{

namespace srv
{

namespace builder
{

class Init_SearchPlan_Request_goal
{
public:
  explicit Init_SearchPlan_Request_goal(::tello_autonomy_msgs::srv::SearchPlan_Request & msg)
  : msg_(msg)
  {}
  ::tello_autonomy_msgs::srv::SearchPlan_Request goal(::tello_autonomy_msgs::srv::SearchPlan_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tello_autonomy_msgs::srv::SearchPlan_Request msg_;
};

class Init_SearchPlan_Request_start
{
public:
  Init_SearchPlan_Request_start()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SearchPlan_Request_goal start(::tello_autonomy_msgs::srv::SearchPlan_Request::_start_type arg)
  {
    msg_.start = std::move(arg);
    return Init_SearchPlan_Request_goal(msg_);
  }

private:
  ::tello_autonomy_msgs::srv::SearchPlan_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::tello_autonomy_msgs::srv::SearchPlan_Request>()
{
  return tello_autonomy_msgs::srv::builder::Init_SearchPlan_Request_start();
}

}  // namespace tello_autonomy_msgs


namespace tello_autonomy_msgs
{

namespace srv
{

namespace builder
{

class Init_SearchPlan_Response_path
{
public:
  explicit Init_SearchPlan_Response_path(::tello_autonomy_msgs::srv::SearchPlan_Response & msg)
  : msg_(msg)
  {}
  ::tello_autonomy_msgs::srv::SearchPlan_Response path(::tello_autonomy_msgs::srv::SearchPlan_Response::_path_type arg)
  {
    msg_.path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tello_autonomy_msgs::srv::SearchPlan_Response msg_;
};

class Init_SearchPlan_Response_success
{
public:
  Init_SearchPlan_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SearchPlan_Response_path success(::tello_autonomy_msgs::srv::SearchPlan_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SearchPlan_Response_path(msg_);
  }

private:
  ::tello_autonomy_msgs::srv::SearchPlan_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::tello_autonomy_msgs::srv::SearchPlan_Response>()
{
  return tello_autonomy_msgs::srv::builder::Init_SearchPlan_Response_success();
}

}  // namespace tello_autonomy_msgs

#endif  // TELLO_AUTONOMY_MSGS__SRV__DETAIL__SEARCH_PLAN__BUILDER_HPP_
