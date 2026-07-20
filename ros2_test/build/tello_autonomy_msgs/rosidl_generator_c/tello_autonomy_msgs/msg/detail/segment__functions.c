// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tello_autonomy_msgs:msg/Segment.idl
// generated code does not contain a copyright notice
#include "tello_autonomy_msgs/msg/detail/segment__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `poses`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
tello_autonomy_msgs__msg__Segment__init(tello_autonomy_msgs__msg__Segment * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    tello_autonomy_msgs__msg__Segment__fini(msg);
    return false;
  }
  // poses
  if (!geometry_msgs__msg__Pose__Sequence__init(&msg->poses, 0)) {
    tello_autonomy_msgs__msg__Segment__fini(msg);
    return false;
  }
  return true;
}

void
tello_autonomy_msgs__msg__Segment__fini(tello_autonomy_msgs__msg__Segment * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // poses
  geometry_msgs__msg__Pose__Sequence__fini(&msg->poses);
}

bool
tello_autonomy_msgs__msg__Segment__are_equal(const tello_autonomy_msgs__msg__Segment * lhs, const tello_autonomy_msgs__msg__Segment * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // poses
  if (!geometry_msgs__msg__Pose__Sequence__are_equal(
      &(lhs->poses), &(rhs->poses)))
  {
    return false;
  }
  return true;
}

bool
tello_autonomy_msgs__msg__Segment__copy(
  const tello_autonomy_msgs__msg__Segment * input,
  tello_autonomy_msgs__msg__Segment * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // poses
  if (!geometry_msgs__msg__Pose__Sequence__copy(
      &(input->poses), &(output->poses)))
  {
    return false;
  }
  return true;
}

tello_autonomy_msgs__msg__Segment *
tello_autonomy_msgs__msg__Segment__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tello_autonomy_msgs__msg__Segment * msg = (tello_autonomy_msgs__msg__Segment *)allocator.allocate(sizeof(tello_autonomy_msgs__msg__Segment), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tello_autonomy_msgs__msg__Segment));
  bool success = tello_autonomy_msgs__msg__Segment__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tello_autonomy_msgs__msg__Segment__destroy(tello_autonomy_msgs__msg__Segment * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tello_autonomy_msgs__msg__Segment__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tello_autonomy_msgs__msg__Segment__Sequence__init(tello_autonomy_msgs__msg__Segment__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tello_autonomy_msgs__msg__Segment * data = NULL;

  if (size) {
    data = (tello_autonomy_msgs__msg__Segment *)allocator.zero_allocate(size, sizeof(tello_autonomy_msgs__msg__Segment), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tello_autonomy_msgs__msg__Segment__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tello_autonomy_msgs__msg__Segment__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
tello_autonomy_msgs__msg__Segment__Sequence__fini(tello_autonomy_msgs__msg__Segment__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      tello_autonomy_msgs__msg__Segment__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

tello_autonomy_msgs__msg__Segment__Sequence *
tello_autonomy_msgs__msg__Segment__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tello_autonomy_msgs__msg__Segment__Sequence * array = (tello_autonomy_msgs__msg__Segment__Sequence *)allocator.allocate(sizeof(tello_autonomy_msgs__msg__Segment__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tello_autonomy_msgs__msg__Segment__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tello_autonomy_msgs__msg__Segment__Sequence__destroy(tello_autonomy_msgs__msg__Segment__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tello_autonomy_msgs__msg__Segment__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tello_autonomy_msgs__msg__Segment__Sequence__are_equal(const tello_autonomy_msgs__msg__Segment__Sequence * lhs, const tello_autonomy_msgs__msg__Segment__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tello_autonomy_msgs__msg__Segment__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tello_autonomy_msgs__msg__Segment__Sequence__copy(
  const tello_autonomy_msgs__msg__Segment__Sequence * input,
  tello_autonomy_msgs__msg__Segment__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tello_autonomy_msgs__msg__Segment);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tello_autonomy_msgs__msg__Segment * data =
      (tello_autonomy_msgs__msg__Segment *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tello_autonomy_msgs__msg__Segment__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tello_autonomy_msgs__msg__Segment__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tello_autonomy_msgs__msg__Segment__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
