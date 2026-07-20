// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tello_autonomy_msgs:msg/Segment.idl
// generated code does not contain a copyright notice

#ifndef TELLO_AUTONOMY_MSGS__MSG__DETAIL__SEGMENT__TRAITS_HPP_
#define TELLO_AUTONOMY_MSGS__MSG__DETAIL__SEGMENT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tello_autonomy_msgs/msg/detail/segment__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'poses'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace tello_autonomy_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Segment & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: poses
  {
    if (msg.poses.size() == 0) {
      out << "poses: []";
    } else {
      out << "poses: [";
      size_t pending_items = msg.poses.size();
      for (auto item : msg.poses) {
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
  const Segment & msg,
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

  // member: poses
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.poses.size() == 0) {
      out << "poses: []\n";
    } else {
      out << "poses:\n";
      for (auto item : msg.poses) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Segment & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace tello_autonomy_msgs

namespace rosidl_generator_traits
{

[[deprecated("use tello_autonomy_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tello_autonomy_msgs::msg::Segment & msg,
  std::ostream & out, size_t indentation = 0)
{
  tello_autonomy_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tello_autonomy_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const tello_autonomy_msgs::msg::Segment & msg)
{
  return tello_autonomy_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tello_autonomy_msgs::msg::Segment>()
{
  return "tello_autonomy_msgs::msg::Segment";
}

template<>
inline const char * name<tello_autonomy_msgs::msg::Segment>()
{
  return "tello_autonomy_msgs/msg/Segment";
}

template<>
struct has_fixed_size<tello_autonomy_msgs::msg::Segment>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tello_autonomy_msgs::msg::Segment>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tello_autonomy_msgs::msg::Segment>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TELLO_AUTONOMY_MSGS__MSG__DETAIL__SEGMENT__TRAITS_HPP_
