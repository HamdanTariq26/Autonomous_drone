// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tello_autonomy_msgs:srv/NbvPlan.idl
// generated code does not contain a copyright notice

#ifndef TELLO_AUTONOMY_MSGS__SRV__DETAIL__NBV_PLAN__TRAITS_HPP_
#define TELLO_AUTONOMY_MSGS__SRV__DETAIL__NBV_PLAN__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tello_autonomy_msgs/srv/detail/nbv_plan__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace tello_autonomy_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const NbvPlan_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NbvPlan_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NbvPlan_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace tello_autonomy_msgs

namespace rosidl_generator_traits
{

[[deprecated("use tello_autonomy_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tello_autonomy_msgs::srv::NbvPlan_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  tello_autonomy_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tello_autonomy_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const tello_autonomy_msgs::srv::NbvPlan_Request & msg)
{
  return tello_autonomy_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<tello_autonomy_msgs::srv::NbvPlan_Request>()
{
  return "tello_autonomy_msgs::srv::NbvPlan_Request";
}

template<>
inline const char * name<tello_autonomy_msgs::srv::NbvPlan_Request>()
{
  return "tello_autonomy_msgs/srv/NbvPlan_Request";
}

template<>
struct has_fixed_size<tello_autonomy_msgs::srv::NbvPlan_Request>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<tello_autonomy_msgs::srv::NbvPlan_Request>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<tello_autonomy_msgs::srv::NbvPlan_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'path'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace tello_autonomy_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const NbvPlan_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: path
  {
    if (msg.path.size() == 0) {
      out << "path: []";
    } else {
      out << "path: [";
      size_t pending_items = msg.path.size();
      for (auto item : msg.path) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NbvPlan_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.path.size() == 0) {
      out << "path: []\n";
    } else {
      out << "path:\n";
      for (auto item : msg.path) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NbvPlan_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace tello_autonomy_msgs

namespace rosidl_generator_traits
{

[[deprecated("use tello_autonomy_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tello_autonomy_msgs::srv::NbvPlan_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  tello_autonomy_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tello_autonomy_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const tello_autonomy_msgs::srv::NbvPlan_Response & msg)
{
  return tello_autonomy_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<tello_autonomy_msgs::srv::NbvPlan_Response>()
{
  return "tello_autonomy_msgs::srv::NbvPlan_Response";
}

template<>
inline const char * name<tello_autonomy_msgs::srv::NbvPlan_Response>()
{
  return "tello_autonomy_msgs/srv/NbvPlan_Response";
}

template<>
struct has_fixed_size<tello_autonomy_msgs::srv::NbvPlan_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tello_autonomy_msgs::srv::NbvPlan_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tello_autonomy_msgs::srv::NbvPlan_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<tello_autonomy_msgs::srv::NbvPlan>()
{
  return "tello_autonomy_msgs::srv::NbvPlan";
}

template<>
inline const char * name<tello_autonomy_msgs::srv::NbvPlan>()
{
  return "tello_autonomy_msgs/srv/NbvPlan";
}

template<>
struct has_fixed_size<tello_autonomy_msgs::srv::NbvPlan>
  : std::integral_constant<
    bool,
    has_fixed_size<tello_autonomy_msgs::srv::NbvPlan_Request>::value &&
    has_fixed_size<tello_autonomy_msgs::srv::NbvPlan_Response>::value
  >
{
};

template<>
struct has_bounded_size<tello_autonomy_msgs::srv::NbvPlan>
  : std::integral_constant<
    bool,
    has_bounded_size<tello_autonomy_msgs::srv::NbvPlan_Request>::value &&
    has_bounded_size<tello_autonomy_msgs::srv::NbvPlan_Response>::value
  >
{
};

template<>
struct is_service<tello_autonomy_msgs::srv::NbvPlan>
  : std::true_type
{
};

template<>
struct is_service_request<tello_autonomy_msgs::srv::NbvPlan_Request>
  : std::true_type
{
};

template<>
struct is_service_response<tello_autonomy_msgs::srv::NbvPlan_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TELLO_AUTONOMY_MSGS__SRV__DETAIL__NBV_PLAN__TRAITS_HPP_
