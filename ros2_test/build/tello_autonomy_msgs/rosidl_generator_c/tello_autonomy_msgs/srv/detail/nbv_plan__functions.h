// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from tello_autonomy_msgs:srv/NbvPlan.idl
// generated code does not contain a copyright notice

#ifndef TELLO_AUTONOMY_MSGS__SRV__DETAIL__NBV_PLAN__FUNCTIONS_H_
#define TELLO_AUTONOMY_MSGS__SRV__DETAIL__NBV_PLAN__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "tello_autonomy_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "tello_autonomy_msgs/srv/detail/nbv_plan__struct.h"

/// Initialize srv/NbvPlan message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * tello_autonomy_msgs__srv__NbvPlan_Request
 * )) before or use
 * tello_autonomy_msgs__srv__NbvPlan_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_autonomy_msgs
bool
tello_autonomy_msgs__srv__NbvPlan_Request__init(tello_autonomy_msgs__srv__NbvPlan_Request * msg);

/// Finalize srv/NbvPlan message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_autonomy_msgs
void
tello_autonomy_msgs__srv__NbvPlan_Request__fini(tello_autonomy_msgs__srv__NbvPlan_Request * msg);

/// Create srv/NbvPlan message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * tello_autonomy_msgs__srv__NbvPlan_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_autonomy_msgs
tello_autonomy_msgs__srv__NbvPlan_Request *
tello_autonomy_msgs__srv__NbvPlan_Request__create();

/// Destroy srv/NbvPlan message.
/**
 * It calls
 * tello_autonomy_msgs__srv__NbvPlan_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_autonomy_msgs
void
tello_autonomy_msgs__srv__NbvPlan_Request__destroy(tello_autonomy_msgs__srv__NbvPlan_Request * msg);

/// Check for srv/NbvPlan message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_autonomy_msgs
bool
tello_autonomy_msgs__srv__NbvPlan_Request__are_equal(const tello_autonomy_msgs__srv__NbvPlan_Request * lhs, const tello_autonomy_msgs__srv__NbvPlan_Request * rhs);

/// Copy a srv/NbvPlan message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_autonomy_msgs
bool
tello_autonomy_msgs__srv__NbvPlan_Request__copy(
  const tello_autonomy_msgs__srv__NbvPlan_Request * input,
  tello_autonomy_msgs__srv__NbvPlan_Request * output);

/// Initialize array of srv/NbvPlan messages.
/**
 * It allocates the memory for the number of elements and calls
 * tello_autonomy_msgs__srv__NbvPlan_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_autonomy_msgs
bool
tello_autonomy_msgs__srv__NbvPlan_Request__Sequence__init(tello_autonomy_msgs__srv__NbvPlan_Request__Sequence * array, size_t size);

/// Finalize array of srv/NbvPlan messages.
/**
 * It calls
 * tello_autonomy_msgs__srv__NbvPlan_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_autonomy_msgs
void
tello_autonomy_msgs__srv__NbvPlan_Request__Sequence__fini(tello_autonomy_msgs__srv__NbvPlan_Request__Sequence * array);

/// Create array of srv/NbvPlan messages.
/**
 * It allocates the memory for the array and calls
 * tello_autonomy_msgs__srv__NbvPlan_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_autonomy_msgs
tello_autonomy_msgs__srv__NbvPlan_Request__Sequence *
tello_autonomy_msgs__srv__NbvPlan_Request__Sequence__create(size_t size);

/// Destroy array of srv/NbvPlan messages.
/**
 * It calls
 * tello_autonomy_msgs__srv__NbvPlan_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_autonomy_msgs
void
tello_autonomy_msgs__srv__NbvPlan_Request__Sequence__destroy(tello_autonomy_msgs__srv__NbvPlan_Request__Sequence * array);

/// Check for srv/NbvPlan message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_autonomy_msgs
bool
tello_autonomy_msgs__srv__NbvPlan_Request__Sequence__are_equal(const tello_autonomy_msgs__srv__NbvPlan_Request__Sequence * lhs, const tello_autonomy_msgs__srv__NbvPlan_Request__Sequence * rhs);

/// Copy an array of srv/NbvPlan messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_autonomy_msgs
bool
tello_autonomy_msgs__srv__NbvPlan_Request__Sequence__copy(
  const tello_autonomy_msgs__srv__NbvPlan_Request__Sequence * input,
  tello_autonomy_msgs__srv__NbvPlan_Request__Sequence * output);

/// Initialize srv/NbvPlan message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * tello_autonomy_msgs__srv__NbvPlan_Response
 * )) before or use
 * tello_autonomy_msgs__srv__NbvPlan_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_autonomy_msgs
bool
tello_autonomy_msgs__srv__NbvPlan_Response__init(tello_autonomy_msgs__srv__NbvPlan_Response * msg);

/// Finalize srv/NbvPlan message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_autonomy_msgs
void
tello_autonomy_msgs__srv__NbvPlan_Response__fini(tello_autonomy_msgs__srv__NbvPlan_Response * msg);

/// Create srv/NbvPlan message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * tello_autonomy_msgs__srv__NbvPlan_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_autonomy_msgs
tello_autonomy_msgs__srv__NbvPlan_Response *
tello_autonomy_msgs__srv__NbvPlan_Response__create();

/// Destroy srv/NbvPlan message.
/**
 * It calls
 * tello_autonomy_msgs__srv__NbvPlan_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_autonomy_msgs
void
tello_autonomy_msgs__srv__NbvPlan_Response__destroy(tello_autonomy_msgs__srv__NbvPlan_Response * msg);

/// Check for srv/NbvPlan message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_autonomy_msgs
bool
tello_autonomy_msgs__srv__NbvPlan_Response__are_equal(const tello_autonomy_msgs__srv__NbvPlan_Response * lhs, const tello_autonomy_msgs__srv__NbvPlan_Response * rhs);

/// Copy a srv/NbvPlan message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_autonomy_msgs
bool
tello_autonomy_msgs__srv__NbvPlan_Response__copy(
  const tello_autonomy_msgs__srv__NbvPlan_Response * input,
  tello_autonomy_msgs__srv__NbvPlan_Response * output);

/// Initialize array of srv/NbvPlan messages.
/**
 * It allocates the memory for the number of elements and calls
 * tello_autonomy_msgs__srv__NbvPlan_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_autonomy_msgs
bool
tello_autonomy_msgs__srv__NbvPlan_Response__Sequence__init(tello_autonomy_msgs__srv__NbvPlan_Response__Sequence * array, size_t size);

/// Finalize array of srv/NbvPlan messages.
/**
 * It calls
 * tello_autonomy_msgs__srv__NbvPlan_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_autonomy_msgs
void
tello_autonomy_msgs__srv__NbvPlan_Response__Sequence__fini(tello_autonomy_msgs__srv__NbvPlan_Response__Sequence * array);

/// Create array of srv/NbvPlan messages.
/**
 * It allocates the memory for the array and calls
 * tello_autonomy_msgs__srv__NbvPlan_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_autonomy_msgs
tello_autonomy_msgs__srv__NbvPlan_Response__Sequence *
tello_autonomy_msgs__srv__NbvPlan_Response__Sequence__create(size_t size);

/// Destroy array of srv/NbvPlan messages.
/**
 * It calls
 * tello_autonomy_msgs__srv__NbvPlan_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_autonomy_msgs
void
tello_autonomy_msgs__srv__NbvPlan_Response__Sequence__destroy(tello_autonomy_msgs__srv__NbvPlan_Response__Sequence * array);

/// Check for srv/NbvPlan message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_autonomy_msgs
bool
tello_autonomy_msgs__srv__NbvPlan_Response__Sequence__are_equal(const tello_autonomy_msgs__srv__NbvPlan_Response__Sequence * lhs, const tello_autonomy_msgs__srv__NbvPlan_Response__Sequence * rhs);

/// Copy an array of srv/NbvPlan messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_autonomy_msgs
bool
tello_autonomy_msgs__srv__NbvPlan_Response__Sequence__copy(
  const tello_autonomy_msgs__srv__NbvPlan_Response__Sequence * input,
  tello_autonomy_msgs__srv__NbvPlan_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TELLO_AUTONOMY_MSGS__SRV__DETAIL__NBV_PLAN__FUNCTIONS_H_
