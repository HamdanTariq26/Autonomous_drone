// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tello_autonomy_msgs:msg/Segment.idl
// generated code does not contain a copyright notice

#ifndef TELLO_AUTONOMY_MSGS__MSG__DETAIL__SEGMENT__STRUCT_HPP_
#define TELLO_AUTONOMY_MSGS__MSG__DETAIL__SEGMENT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'poses'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__tello_autonomy_msgs__msg__Segment __attribute__((deprecated))
#else
# define DEPRECATED__tello_autonomy_msgs__msg__Segment __declspec(deprecated)
#endif

namespace tello_autonomy_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Segment_
{
  using Type = Segment_<ContainerAllocator>;

  explicit Segment_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit Segment_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _poses_type =
    std::vector<geometry_msgs::msg::Pose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Pose_<ContainerAllocator>>>;
  _poses_type poses;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__poses(
    const std::vector<geometry_msgs::msg::Pose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Pose_<ContainerAllocator>>> & _arg)
  {
    this->poses = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tello_autonomy_msgs::msg::Segment_<ContainerAllocator> *;
  using ConstRawPtr =
    const tello_autonomy_msgs::msg::Segment_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tello_autonomy_msgs::msg::Segment_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tello_autonomy_msgs::msg::Segment_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tello_autonomy_msgs::msg::Segment_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tello_autonomy_msgs::msg::Segment_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tello_autonomy_msgs::msg::Segment_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tello_autonomy_msgs::msg::Segment_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tello_autonomy_msgs::msg::Segment_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tello_autonomy_msgs::msg::Segment_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tello_autonomy_msgs__msg__Segment
    std::shared_ptr<tello_autonomy_msgs::msg::Segment_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tello_autonomy_msgs__msg__Segment
    std::shared_ptr<tello_autonomy_msgs::msg::Segment_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Segment_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->poses != other.poses) {
      return false;
    }
    return true;
  }
  bool operator!=(const Segment_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Segment_

// alias to use template instance with default allocator
using Segment =
  tello_autonomy_msgs::msg::Segment_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace tello_autonomy_msgs

#endif  // TELLO_AUTONOMY_MSGS__MSG__DETAIL__SEGMENT__STRUCT_HPP_
