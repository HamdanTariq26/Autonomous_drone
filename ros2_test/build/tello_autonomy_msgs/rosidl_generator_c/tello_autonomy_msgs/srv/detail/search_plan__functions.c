// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tello_autonomy_msgs:srv/SearchPlan.idl
// generated code does not contain a copyright notice
#include "tello_autonomy_msgs/srv/detail/search_plan__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `start`
// Member `goal`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"

bool
tello_autonomy_msgs__srv__SearchPlan_Request__init(tello_autonomy_msgs__srv__SearchPlan_Request * msg)
{
  if (!msg) {
    return false;
  }
  // start
  if (!geometry_msgs__msg__PoseStamped__init(&msg->start)) {
    tello_autonomy_msgs__srv__SearchPlan_Request__fini(msg);
    return false;
  }
  // goal
  if (!geometry_msgs__msg__PoseStamped__init(&msg->goal)) {
    tello_autonomy_msgs__srv__SearchPlan_Request__fini(msg);
    return false;
  }
  return true;
}

void
tello_autonomy_msgs__srv__SearchPlan_Request__fini(tello_autonomy_msgs__srv__SearchPlan_Request * msg)
{
  if (!msg) {
    return;
  }
  // start
  geometry_msgs__msg__PoseStamped__fini(&msg->start);
  // goal
  geometry_msgs__msg__PoseStamped__fini(&msg->goal);
}

bool
tello_autonomy_msgs__srv__SearchPlan_Request__are_equal(const tello_autonomy_msgs__srv__SearchPlan_Request * lhs, const tello_autonomy_msgs__srv__SearchPlan_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // start
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->start), &(rhs->start)))
  {
    return false;
  }
  // goal
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
tello_autonomy_msgs__srv__SearchPlan_Request__copy(
  const tello_autonomy_msgs__srv__SearchPlan_Request * input,
  tello_autonomy_msgs__srv__SearchPlan_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // start
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->start), &(output->start)))
  {
    return false;
  }
  // goal
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

tello_autonomy_msgs__srv__SearchPlan_Request *
tello_autonomy_msgs__srv__SearchPlan_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tello_autonomy_msgs__srv__SearchPlan_Request * msg = (tello_autonomy_msgs__srv__SearchPlan_Request *)allocator.allocate(sizeof(tello_autonomy_msgs__srv__SearchPlan_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tello_autonomy_msgs__srv__SearchPlan_Request));
  bool success = tello_autonomy_msgs__srv__SearchPlan_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tello_autonomy_msgs__srv__SearchPlan_Request__destroy(tello_autonomy_msgs__srv__SearchPlan_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tello_autonomy_msgs__srv__SearchPlan_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tello_autonomy_msgs__srv__SearchPlan_Request__Sequence__init(tello_autonomy_msgs__srv__SearchPlan_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tello_autonomy_msgs__srv__SearchPlan_Request * data = NULL;

  if (size) {
    data = (tello_autonomy_msgs__srv__SearchPlan_Request *)allocator.zero_allocate(size, sizeof(tello_autonomy_msgs__srv__SearchPlan_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tello_autonomy_msgs__srv__SearchPlan_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tello_autonomy_msgs__srv__SearchPlan_Request__fini(&data[i - 1]);
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
tello_autonomy_msgs__srv__SearchPlan_Request__Sequence__fini(tello_autonomy_msgs__srv__SearchPlan_Request__Sequence * array)
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
      tello_autonomy_msgs__srv__SearchPlan_Request__fini(&array->data[i]);
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

tello_autonomy_msgs__srv__SearchPlan_Request__Sequence *
tello_autonomy_msgs__srv__SearchPlan_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tello_autonomy_msgs__srv__SearchPlan_Request__Sequence * array = (tello_autonomy_msgs__srv__SearchPlan_Request__Sequence *)allocator.allocate(sizeof(tello_autonomy_msgs__srv__SearchPlan_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tello_autonomy_msgs__srv__SearchPlan_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tello_autonomy_msgs__srv__SearchPlan_Request__Sequence__destroy(tello_autonomy_msgs__srv__SearchPlan_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tello_autonomy_msgs__srv__SearchPlan_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tello_autonomy_msgs__srv__SearchPlan_Request__Sequence__are_equal(const tello_autonomy_msgs__srv__SearchPlan_Request__Sequence * lhs, const tello_autonomy_msgs__srv__SearchPlan_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tello_autonomy_msgs__srv__SearchPlan_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tello_autonomy_msgs__srv__SearchPlan_Request__Sequence__copy(
  const tello_autonomy_msgs__srv__SearchPlan_Request__Sequence * input,
  tello_autonomy_msgs__srv__SearchPlan_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tello_autonomy_msgs__srv__SearchPlan_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tello_autonomy_msgs__srv__SearchPlan_Request * data =
      (tello_autonomy_msgs__srv__SearchPlan_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tello_autonomy_msgs__srv__SearchPlan_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tello_autonomy_msgs__srv__SearchPlan_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tello_autonomy_msgs__srv__SearchPlan_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `path`
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__functions.h"

bool
tello_autonomy_msgs__srv__SearchPlan_Response__init(tello_autonomy_msgs__srv__SearchPlan_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // path
  if (!geometry_msgs__msg__PoseStamped__Sequence__init(&msg->path, 0)) {
    tello_autonomy_msgs__srv__SearchPlan_Response__fini(msg);
    return false;
  }
  return true;
}

void
tello_autonomy_msgs__srv__SearchPlan_Response__fini(tello_autonomy_msgs__srv__SearchPlan_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // path
  geometry_msgs__msg__PoseStamped__Sequence__fini(&msg->path);
}

bool
tello_autonomy_msgs__srv__SearchPlan_Response__are_equal(const tello_autonomy_msgs__srv__SearchPlan_Response * lhs, const tello_autonomy_msgs__srv__SearchPlan_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // path
  if (!geometry_msgs__msg__PoseStamped__Sequence__are_equal(
      &(lhs->path), &(rhs->path)))
  {
    return false;
  }
  return true;
}

bool
tello_autonomy_msgs__srv__SearchPlan_Response__copy(
  const tello_autonomy_msgs__srv__SearchPlan_Response * input,
  tello_autonomy_msgs__srv__SearchPlan_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // path
  if (!geometry_msgs__msg__PoseStamped__Sequence__copy(
      &(input->path), &(output->path)))
  {
    return false;
  }
  return true;
}

tello_autonomy_msgs__srv__SearchPlan_Response *
tello_autonomy_msgs__srv__SearchPlan_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tello_autonomy_msgs__srv__SearchPlan_Response * msg = (tello_autonomy_msgs__srv__SearchPlan_Response *)allocator.allocate(sizeof(tello_autonomy_msgs__srv__SearchPlan_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tello_autonomy_msgs__srv__SearchPlan_Response));
  bool success = tello_autonomy_msgs__srv__SearchPlan_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tello_autonomy_msgs__srv__SearchPlan_Response__destroy(tello_autonomy_msgs__srv__SearchPlan_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tello_autonomy_msgs__srv__SearchPlan_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tello_autonomy_msgs__srv__SearchPlan_Response__Sequence__init(tello_autonomy_msgs__srv__SearchPlan_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tello_autonomy_msgs__srv__SearchPlan_Response * data = NULL;

  if (size) {
    data = (tello_autonomy_msgs__srv__SearchPlan_Response *)allocator.zero_allocate(size, sizeof(tello_autonomy_msgs__srv__SearchPlan_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tello_autonomy_msgs__srv__SearchPlan_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tello_autonomy_msgs__srv__SearchPlan_Response__fini(&data[i - 1]);
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
tello_autonomy_msgs__srv__SearchPlan_Response__Sequence__fini(tello_autonomy_msgs__srv__SearchPlan_Response__Sequence * array)
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
      tello_autonomy_msgs__srv__SearchPlan_Response__fini(&array->data[i]);
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

tello_autonomy_msgs__srv__SearchPlan_Response__Sequence *
tello_autonomy_msgs__srv__SearchPlan_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tello_autonomy_msgs__srv__SearchPlan_Response__Sequence * array = (tello_autonomy_msgs__srv__SearchPlan_Response__Sequence *)allocator.allocate(sizeof(tello_autonomy_msgs__srv__SearchPlan_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tello_autonomy_msgs__srv__SearchPlan_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tello_autonomy_msgs__srv__SearchPlan_Response__Sequence__destroy(tello_autonomy_msgs__srv__SearchPlan_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tello_autonomy_msgs__srv__SearchPlan_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tello_autonomy_msgs__srv__SearchPlan_Response__Sequence__are_equal(const tello_autonomy_msgs__srv__SearchPlan_Response__Sequence * lhs, const tello_autonomy_msgs__srv__SearchPlan_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tello_autonomy_msgs__srv__SearchPlan_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tello_autonomy_msgs__srv__SearchPlan_Response__Sequence__copy(
  const tello_autonomy_msgs__srv__SearchPlan_Response__Sequence * input,
  tello_autonomy_msgs__srv__SearchPlan_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tello_autonomy_msgs__srv__SearchPlan_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tello_autonomy_msgs__srv__SearchPlan_Response * data =
      (tello_autonomy_msgs__srv__SearchPlan_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tello_autonomy_msgs__srv__SearchPlan_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tello_autonomy_msgs__srv__SearchPlan_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tello_autonomy_msgs__srv__SearchPlan_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
