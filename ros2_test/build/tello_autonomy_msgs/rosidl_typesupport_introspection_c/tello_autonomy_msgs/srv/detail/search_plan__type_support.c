// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from tello_autonomy_msgs:srv/SearchPlan.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "tello_autonomy_msgs/srv/detail/search_plan__rosidl_typesupport_introspection_c.h"
#include "tello_autonomy_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "tello_autonomy_msgs/srv/detail/search_plan__functions.h"
#include "tello_autonomy_msgs/srv/detail/search_plan__struct.h"


// Include directives for member types
// Member `start`
// Member `goal`
#include "geometry_msgs/msg/pose_stamped.h"
// Member `start`
// Member `goal`
#include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void tello_autonomy_msgs__srv__SearchPlan_Request__rosidl_typesupport_introspection_c__SearchPlan_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  tello_autonomy_msgs__srv__SearchPlan_Request__init(message_memory);
}

void tello_autonomy_msgs__srv__SearchPlan_Request__rosidl_typesupport_introspection_c__SearchPlan_Request_fini_function(void * message_memory)
{
  tello_autonomy_msgs__srv__SearchPlan_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember tello_autonomy_msgs__srv__SearchPlan_Request__rosidl_typesupport_introspection_c__SearchPlan_Request_message_member_array[2] = {
  {
    "start",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tello_autonomy_msgs__srv__SearchPlan_Request, start),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tello_autonomy_msgs__srv__SearchPlan_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers tello_autonomy_msgs__srv__SearchPlan_Request__rosidl_typesupport_introspection_c__SearchPlan_Request_message_members = {
  "tello_autonomy_msgs__srv",  // message namespace
  "SearchPlan_Request",  // message name
  2,  // number of fields
  sizeof(tello_autonomy_msgs__srv__SearchPlan_Request),
  tello_autonomy_msgs__srv__SearchPlan_Request__rosidl_typesupport_introspection_c__SearchPlan_Request_message_member_array,  // message members
  tello_autonomy_msgs__srv__SearchPlan_Request__rosidl_typesupport_introspection_c__SearchPlan_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  tello_autonomy_msgs__srv__SearchPlan_Request__rosidl_typesupport_introspection_c__SearchPlan_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t tello_autonomy_msgs__srv__SearchPlan_Request__rosidl_typesupport_introspection_c__SearchPlan_Request_message_type_support_handle = {
  0,
  &tello_autonomy_msgs__srv__SearchPlan_Request__rosidl_typesupport_introspection_c__SearchPlan_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_tello_autonomy_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tello_autonomy_msgs, srv, SearchPlan_Request)() {
  tello_autonomy_msgs__srv__SearchPlan_Request__rosidl_typesupport_introspection_c__SearchPlan_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  tello_autonomy_msgs__srv__SearchPlan_Request__rosidl_typesupport_introspection_c__SearchPlan_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  if (!tello_autonomy_msgs__srv__SearchPlan_Request__rosidl_typesupport_introspection_c__SearchPlan_Request_message_type_support_handle.typesupport_identifier) {
    tello_autonomy_msgs__srv__SearchPlan_Request__rosidl_typesupport_introspection_c__SearchPlan_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &tello_autonomy_msgs__srv__SearchPlan_Request__rosidl_typesupport_introspection_c__SearchPlan_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "tello_autonomy_msgs/srv/detail/search_plan__rosidl_typesupport_introspection_c.h"
// already included above
// #include "tello_autonomy_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "tello_autonomy_msgs/srv/detail/search_plan__functions.h"
// already included above
// #include "tello_autonomy_msgs/srv/detail/search_plan__struct.h"


// Include directives for member types
// Member `path`
// already included above
// #include "geometry_msgs/msg/pose_stamped.h"
// Member `path`
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void tello_autonomy_msgs__srv__SearchPlan_Response__rosidl_typesupport_introspection_c__SearchPlan_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  tello_autonomy_msgs__srv__SearchPlan_Response__init(message_memory);
}

void tello_autonomy_msgs__srv__SearchPlan_Response__rosidl_typesupport_introspection_c__SearchPlan_Response_fini_function(void * message_memory)
{
  tello_autonomy_msgs__srv__SearchPlan_Response__fini(message_memory);
}

size_t tello_autonomy_msgs__srv__SearchPlan_Response__rosidl_typesupport_introspection_c__size_function__SearchPlan_Response__path(
  const void * untyped_member)
{
  const geometry_msgs__msg__PoseStamped__Sequence * member =
    (const geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  return member->size;
}

const void * tello_autonomy_msgs__srv__SearchPlan_Response__rosidl_typesupport_introspection_c__get_const_function__SearchPlan_Response__path(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__PoseStamped__Sequence * member =
    (const geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tello_autonomy_msgs__srv__SearchPlan_Response__rosidl_typesupport_introspection_c__get_function__SearchPlan_Response__path(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__PoseStamped__Sequence * member =
    (geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  return &member->data[index];
}

void tello_autonomy_msgs__srv__SearchPlan_Response__rosidl_typesupport_introspection_c__fetch_function__SearchPlan_Response__path(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__PoseStamped * item =
    ((const geometry_msgs__msg__PoseStamped *)
    tello_autonomy_msgs__srv__SearchPlan_Response__rosidl_typesupport_introspection_c__get_const_function__SearchPlan_Response__path(untyped_member, index));
  geometry_msgs__msg__PoseStamped * value =
    (geometry_msgs__msg__PoseStamped *)(untyped_value);
  *value = *item;
}

void tello_autonomy_msgs__srv__SearchPlan_Response__rosidl_typesupport_introspection_c__assign_function__SearchPlan_Response__path(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__PoseStamped * item =
    ((geometry_msgs__msg__PoseStamped *)
    tello_autonomy_msgs__srv__SearchPlan_Response__rosidl_typesupport_introspection_c__get_function__SearchPlan_Response__path(untyped_member, index));
  const geometry_msgs__msg__PoseStamped * value =
    (const geometry_msgs__msg__PoseStamped *)(untyped_value);
  *item = *value;
}

bool tello_autonomy_msgs__srv__SearchPlan_Response__rosidl_typesupport_introspection_c__resize_function__SearchPlan_Response__path(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__PoseStamped__Sequence * member =
    (geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  geometry_msgs__msg__PoseStamped__Sequence__fini(member);
  return geometry_msgs__msg__PoseStamped__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember tello_autonomy_msgs__srv__SearchPlan_Response__rosidl_typesupport_introspection_c__SearchPlan_Response_message_member_array[2] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tello_autonomy_msgs__srv__SearchPlan_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "path",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tello_autonomy_msgs__srv__SearchPlan_Response, path),  // bytes offset in struct
    NULL,  // default value
    tello_autonomy_msgs__srv__SearchPlan_Response__rosidl_typesupport_introspection_c__size_function__SearchPlan_Response__path,  // size() function pointer
    tello_autonomy_msgs__srv__SearchPlan_Response__rosidl_typesupport_introspection_c__get_const_function__SearchPlan_Response__path,  // get_const(index) function pointer
    tello_autonomy_msgs__srv__SearchPlan_Response__rosidl_typesupport_introspection_c__get_function__SearchPlan_Response__path,  // get(index) function pointer
    tello_autonomy_msgs__srv__SearchPlan_Response__rosidl_typesupport_introspection_c__fetch_function__SearchPlan_Response__path,  // fetch(index, &value) function pointer
    tello_autonomy_msgs__srv__SearchPlan_Response__rosidl_typesupport_introspection_c__assign_function__SearchPlan_Response__path,  // assign(index, value) function pointer
    tello_autonomy_msgs__srv__SearchPlan_Response__rosidl_typesupport_introspection_c__resize_function__SearchPlan_Response__path  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers tello_autonomy_msgs__srv__SearchPlan_Response__rosidl_typesupport_introspection_c__SearchPlan_Response_message_members = {
  "tello_autonomy_msgs__srv",  // message namespace
  "SearchPlan_Response",  // message name
  2,  // number of fields
  sizeof(tello_autonomy_msgs__srv__SearchPlan_Response),
  tello_autonomy_msgs__srv__SearchPlan_Response__rosidl_typesupport_introspection_c__SearchPlan_Response_message_member_array,  // message members
  tello_autonomy_msgs__srv__SearchPlan_Response__rosidl_typesupport_introspection_c__SearchPlan_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  tello_autonomy_msgs__srv__SearchPlan_Response__rosidl_typesupport_introspection_c__SearchPlan_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t tello_autonomy_msgs__srv__SearchPlan_Response__rosidl_typesupport_introspection_c__SearchPlan_Response_message_type_support_handle = {
  0,
  &tello_autonomy_msgs__srv__SearchPlan_Response__rosidl_typesupport_introspection_c__SearchPlan_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_tello_autonomy_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tello_autonomy_msgs, srv, SearchPlan_Response)() {
  tello_autonomy_msgs__srv__SearchPlan_Response__rosidl_typesupport_introspection_c__SearchPlan_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  if (!tello_autonomy_msgs__srv__SearchPlan_Response__rosidl_typesupport_introspection_c__SearchPlan_Response_message_type_support_handle.typesupport_identifier) {
    tello_autonomy_msgs__srv__SearchPlan_Response__rosidl_typesupport_introspection_c__SearchPlan_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &tello_autonomy_msgs__srv__SearchPlan_Response__rosidl_typesupport_introspection_c__SearchPlan_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "tello_autonomy_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "tello_autonomy_msgs/srv/detail/search_plan__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers tello_autonomy_msgs__srv__detail__search_plan__rosidl_typesupport_introspection_c__SearchPlan_service_members = {
  "tello_autonomy_msgs__srv",  // service namespace
  "SearchPlan",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // tello_autonomy_msgs__srv__detail__search_plan__rosidl_typesupport_introspection_c__SearchPlan_Request_message_type_support_handle,
  NULL  // response message
  // tello_autonomy_msgs__srv__detail__search_plan__rosidl_typesupport_introspection_c__SearchPlan_Response_message_type_support_handle
};

static rosidl_service_type_support_t tello_autonomy_msgs__srv__detail__search_plan__rosidl_typesupport_introspection_c__SearchPlan_service_type_support_handle = {
  0,
  &tello_autonomy_msgs__srv__detail__search_plan__rosidl_typesupport_introspection_c__SearchPlan_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tello_autonomy_msgs, srv, SearchPlan_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tello_autonomy_msgs, srv, SearchPlan_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_tello_autonomy_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tello_autonomy_msgs, srv, SearchPlan)() {
  if (!tello_autonomy_msgs__srv__detail__search_plan__rosidl_typesupport_introspection_c__SearchPlan_service_type_support_handle.typesupport_identifier) {
    tello_autonomy_msgs__srv__detail__search_plan__rosidl_typesupport_introspection_c__SearchPlan_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)tello_autonomy_msgs__srv__detail__search_plan__rosidl_typesupport_introspection_c__SearchPlan_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tello_autonomy_msgs, srv, SearchPlan_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tello_autonomy_msgs, srv, SearchPlan_Response)()->data;
  }

  return &tello_autonomy_msgs__srv__detail__search_plan__rosidl_typesupport_introspection_c__SearchPlan_service_type_support_handle;
}
