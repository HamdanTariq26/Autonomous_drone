// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tello_autonomy_msgs:srv/NbvPlan.idl
// generated code does not contain a copyright notice

#ifndef TELLO_AUTONOMY_MSGS__SRV__DETAIL__NBV_PLAN__STRUCT_H_
#define TELLO_AUTONOMY_MSGS__SRV__DETAIL__NBV_PLAN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in srv/NbvPlan in the package tello_autonomy_msgs.
typedef struct tello_autonomy_msgs__srv__NbvPlan_Request
{
  std_msgs__msg__Header header;
} tello_autonomy_msgs__srv__NbvPlan_Request;

// Struct for a sequence of tello_autonomy_msgs__srv__NbvPlan_Request.
typedef struct tello_autonomy_msgs__srv__NbvPlan_Request__Sequence
{
  tello_autonomy_msgs__srv__NbvPlan_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tello_autonomy_msgs__srv__NbvPlan_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'path'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in srv/NbvPlan in the package tello_autonomy_msgs.
typedef struct tello_autonomy_msgs__srv__NbvPlan_Response
{
  geometry_msgs__msg__Pose__Sequence path;
} tello_autonomy_msgs__srv__NbvPlan_Response;

// Struct for a sequence of tello_autonomy_msgs__srv__NbvPlan_Response.
typedef struct tello_autonomy_msgs__srv__NbvPlan_Response__Sequence
{
  tello_autonomy_msgs__srv__NbvPlan_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tello_autonomy_msgs__srv__NbvPlan_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TELLO_AUTONOMY_MSGS__SRV__DETAIL__NBV_PLAN__STRUCT_H_
