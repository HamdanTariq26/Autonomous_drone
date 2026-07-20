// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tello_autonomy_msgs:srv/SearchPlan.idl
// generated code does not contain a copyright notice

#ifndef TELLO_AUTONOMY_MSGS__SRV__DETAIL__SEARCH_PLAN__STRUCT_H_
#define TELLO_AUTONOMY_MSGS__SRV__DETAIL__SEARCH_PLAN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'start'
// Member 'goal'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in srv/SearchPlan in the package tello_autonomy_msgs.
typedef struct tello_autonomy_msgs__srv__SearchPlan_Request
{
  geometry_msgs__msg__PoseStamped start;
  geometry_msgs__msg__PoseStamped goal;
} tello_autonomy_msgs__srv__SearchPlan_Request;

// Struct for a sequence of tello_autonomy_msgs__srv__SearchPlan_Request.
typedef struct tello_autonomy_msgs__srv__SearchPlan_Request__Sequence
{
  tello_autonomy_msgs__srv__SearchPlan_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tello_autonomy_msgs__srv__SearchPlan_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'path'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in srv/SearchPlan in the package tello_autonomy_msgs.
typedef struct tello_autonomy_msgs__srv__SearchPlan_Response
{
  bool success;
  geometry_msgs__msg__PoseStamped__Sequence path;
} tello_autonomy_msgs__srv__SearchPlan_Response;

// Struct for a sequence of tello_autonomy_msgs__srv__SearchPlan_Response.
typedef struct tello_autonomy_msgs__srv__SearchPlan_Response__Sequence
{
  tello_autonomy_msgs__srv__SearchPlan_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tello_autonomy_msgs__srv__SearchPlan_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TELLO_AUTONOMY_MSGS__SRV__DETAIL__SEARCH_PLAN__STRUCT_H_
