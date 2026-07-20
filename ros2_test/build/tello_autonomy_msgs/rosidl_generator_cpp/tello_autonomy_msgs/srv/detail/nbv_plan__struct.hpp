// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tello_autonomy_msgs:srv/NbvPlan.idl
// generated code does not contain a copyright notice

#ifndef TELLO_AUTONOMY_MSGS__SRV__DETAIL__NBV_PLAN__STRUCT_HPP_
#define TELLO_AUTONOMY_MSGS__SRV__DETAIL__NBV_PLAN__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__tello_autonomy_msgs__srv__NbvPlan_Request __attribute__((deprecated))
#else
# define DEPRECATED__tello_autonomy_msgs__srv__NbvPlan_Request __declspec(deprecated)
#endif

namespace tello_autonomy_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct NbvPlan_Request_
{
  using Type = NbvPlan_Request_<ContainerAllocator>;

  explicit NbvPlan_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit NbvPlan_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tello_autonomy_msgs::srv::NbvPlan_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const tello_autonomy_msgs::srv::NbvPlan_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tello_autonomy_msgs::srv::NbvPlan_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tello_autonomy_msgs::srv::NbvPlan_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tello_autonomy_msgs::srv::NbvPlan_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tello_autonomy_msgs::srv::NbvPlan_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tello_autonomy_msgs::srv::NbvPlan_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tello_autonomy_msgs::srv::NbvPlan_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tello_autonomy_msgs::srv::NbvPlan_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tello_autonomy_msgs::srv::NbvPlan_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tello_autonomy_msgs__srv__NbvPlan_Request
    std::shared_ptr<tello_autonomy_msgs::srv::NbvPlan_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tello_autonomy_msgs__srv__NbvPlan_Request
    std::shared_ptr<tello_autonomy_msgs::srv::NbvPlan_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NbvPlan_Request_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    return true;
  }
  bool operator!=(const NbvPlan_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NbvPlan_Request_

// alias to use template instance with default allocator
using NbvPlan_Request =
  tello_autonomy_msgs::srv::NbvPlan_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace tello_autonomy_msgs


// Include directives for member types
// Member 'path'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__tello_autonomy_msgs__srv__NbvPlan_Response __attribute__((deprecated))
#else
# define DEPRECATED__tello_autonomy_msgs__srv__NbvPlan_Response __declspec(deprecated)
#endif

namespace tello_autonomy_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct NbvPlan_Response_
{
  using Type = NbvPlan_Response_<ContainerAllocator>;

  explicit NbvPlan_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit NbvPlan_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _path_type =
    std::vector<geometry_msgs::msg::Pose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Pose_<ContainerAllocator>>>;
  _path_type path;

  // setters for named parameter idiom
  Type & set__path(
    const std::vector<geometry_msgs::msg::Pose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Pose_<ContainerAllocator>>> & _arg)
  {
    this->path = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tello_autonomy_msgs::srv::NbvPlan_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const tello_autonomy_msgs::srv::NbvPlan_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tello_autonomy_msgs::srv::NbvPlan_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tello_autonomy_msgs::srv::NbvPlan_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tello_autonomy_msgs::srv::NbvPlan_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tello_autonomy_msgs::srv::NbvPlan_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tello_autonomy_msgs::srv::NbvPlan_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tello_autonomy_msgs::srv::NbvPlan_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tello_autonomy_msgs::srv::NbvPlan_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tello_autonomy_msgs::srv::NbvPlan_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tello_autonomy_msgs__srv__NbvPlan_Response
    std::shared_ptr<tello_autonomy_msgs::srv::NbvPlan_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tello_autonomy_msgs__srv__NbvPlan_Response
    std::shared_ptr<tello_autonomy_msgs::srv::NbvPlan_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NbvPlan_Response_ & other) const
  {
    if (this->path != other.path) {
      return false;
    }
    return true;
  }
  bool operator!=(const NbvPlan_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NbvPlan_Response_

// alias to use template instance with default allocator
using NbvPlan_Response =
  tello_autonomy_msgs::srv::NbvPlan_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace tello_autonomy_msgs

namespace tello_autonomy_msgs
{

namespace srv
{

struct NbvPlan
{
  using Request = tello_autonomy_msgs::srv::NbvPlan_Request;
  using Response = tello_autonomy_msgs::srv::NbvPlan_Response;
};

}  // namespace srv

}  // namespace tello_autonomy_msgs

#endif  // TELLO_AUTONOMY_MSGS__SRV__DETAIL__NBV_PLAN__STRUCT_HPP_
