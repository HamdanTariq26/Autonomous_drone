// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tello_autonomy_msgs:srv/SearchPlan.idl
// generated code does not contain a copyright notice

#ifndef TELLO_AUTONOMY_MSGS__SRV__DETAIL__SEARCH_PLAN__STRUCT_HPP_
#define TELLO_AUTONOMY_MSGS__SRV__DETAIL__SEARCH_PLAN__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'start'
// Member 'goal'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__tello_autonomy_msgs__srv__SearchPlan_Request __attribute__((deprecated))
#else
# define DEPRECATED__tello_autonomy_msgs__srv__SearchPlan_Request __declspec(deprecated)
#endif

namespace tello_autonomy_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SearchPlan_Request_
{
  using Type = SearchPlan_Request_<ContainerAllocator>;

  explicit SearchPlan_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : start(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit SearchPlan_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : start(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _start_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _start_type start;
  using _goal_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__start(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->start = _arg;
    return *this;
  }
  Type & set__goal(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tello_autonomy_msgs::srv::SearchPlan_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const tello_autonomy_msgs::srv::SearchPlan_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tello_autonomy_msgs::srv::SearchPlan_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tello_autonomy_msgs::srv::SearchPlan_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tello_autonomy_msgs::srv::SearchPlan_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tello_autonomy_msgs::srv::SearchPlan_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tello_autonomy_msgs::srv::SearchPlan_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tello_autonomy_msgs::srv::SearchPlan_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tello_autonomy_msgs::srv::SearchPlan_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tello_autonomy_msgs::srv::SearchPlan_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tello_autonomy_msgs__srv__SearchPlan_Request
    std::shared_ptr<tello_autonomy_msgs::srv::SearchPlan_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tello_autonomy_msgs__srv__SearchPlan_Request
    std::shared_ptr<tello_autonomy_msgs::srv::SearchPlan_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SearchPlan_Request_ & other) const
  {
    if (this->start != other.start) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const SearchPlan_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SearchPlan_Request_

// alias to use template instance with default allocator
using SearchPlan_Request =
  tello_autonomy_msgs::srv::SearchPlan_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace tello_autonomy_msgs


// Include directives for member types
// Member 'path'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__tello_autonomy_msgs__srv__SearchPlan_Response __attribute__((deprecated))
#else
# define DEPRECATED__tello_autonomy_msgs__srv__SearchPlan_Response __declspec(deprecated)
#endif

namespace tello_autonomy_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SearchPlan_Response_
{
  using Type = SearchPlan_Response_<ContainerAllocator>;

  explicit SearchPlan_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit SearchPlan_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _path_type =
    std::vector<geometry_msgs::msg::PoseStamped_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::PoseStamped_<ContainerAllocator>>>;
  _path_type path;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__path(
    const std::vector<geometry_msgs::msg::PoseStamped_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::PoseStamped_<ContainerAllocator>>> & _arg)
  {
    this->path = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tello_autonomy_msgs::srv::SearchPlan_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const tello_autonomy_msgs::srv::SearchPlan_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tello_autonomy_msgs::srv::SearchPlan_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tello_autonomy_msgs::srv::SearchPlan_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tello_autonomy_msgs::srv::SearchPlan_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tello_autonomy_msgs::srv::SearchPlan_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tello_autonomy_msgs::srv::SearchPlan_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tello_autonomy_msgs::srv::SearchPlan_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tello_autonomy_msgs::srv::SearchPlan_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tello_autonomy_msgs::srv::SearchPlan_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tello_autonomy_msgs__srv__SearchPlan_Response
    std::shared_ptr<tello_autonomy_msgs::srv::SearchPlan_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tello_autonomy_msgs__srv__SearchPlan_Response
    std::shared_ptr<tello_autonomy_msgs::srv::SearchPlan_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SearchPlan_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->path != other.path) {
      return false;
    }
    return true;
  }
  bool operator!=(const SearchPlan_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SearchPlan_Response_

// alias to use template instance with default allocator
using SearchPlan_Response =
  tello_autonomy_msgs::srv::SearchPlan_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace tello_autonomy_msgs

namespace tello_autonomy_msgs
{

namespace srv
{

struct SearchPlan
{
  using Request = tello_autonomy_msgs::srv::SearchPlan_Request;
  using Response = tello_autonomy_msgs::srv::SearchPlan_Response;
};

}  // namespace srv

}  // namespace tello_autonomy_msgs

#endif  // TELLO_AUTONOMY_MSGS__SRV__DETAIL__SEARCH_PLAN__STRUCT_HPP_
