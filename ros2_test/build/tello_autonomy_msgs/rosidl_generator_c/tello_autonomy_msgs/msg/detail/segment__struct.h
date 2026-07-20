// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tello_autonomy_msgs:msg/Segment.idl
// generated code does not contain a copyright notice

#ifndef TELLO_AUTONOMY_MSGS__MSG__DETAIL__SEGMENT__STRUCT_H_
#define TELLO_AUTONOMY_MSGS__MSG__DETAIL__SEGMENT__STRUCT_H_

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
// Member 'poses'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/Segment in the package tello_autonomy_msgs.
typedef struct tello_autonomy_msgs__msg__Segment
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Pose__Sequence poses;
} tello_autonomy_msgs__msg__Segment;

// Struct for a sequence of tello_autonomy_msgs__msg__Segment.
typedef struct tello_autonomy_msgs__msg__Segment__Sequence
{
  tello_autonomy_msgs__msg__Segment * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tello_autonomy_msgs__msg__Segment__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TELLO_AUTONOMY_MSGS__MSG__DETAIL__SEGMENT__STRUCT_H_
