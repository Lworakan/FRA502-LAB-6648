// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from controller_interfaces:srv/SetMaxPizza.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "controller_interfaces/srv/detail/set_max_pizza__rosidl_typesupport_introspection_c.h"
#include "controller_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "controller_interfaces/srv/detail/set_max_pizza__functions.h"
#include "controller_interfaces/srv/detail/set_max_pizza__struct.h"


// Include directives for member types
// Member `max_pizza`
#include "std_msgs/msg/int64.h"
// Member `max_pizza`
#include "std_msgs/msg/detail/int64__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void controller_interfaces__srv__SetMaxPizza_Request__rosidl_typesupport_introspection_c__SetMaxPizza_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  controller_interfaces__srv__SetMaxPizza_Request__init(message_memory);
}

void controller_interfaces__srv__SetMaxPizza_Request__rosidl_typesupport_introspection_c__SetMaxPizza_Request_fini_function(void * message_memory)
{
  controller_interfaces__srv__SetMaxPizza_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember controller_interfaces__srv__SetMaxPizza_Request__rosidl_typesupport_introspection_c__SetMaxPizza_Request_message_member_array[1] = {
  {
    "max_pizza",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller_interfaces__srv__SetMaxPizza_Request, max_pizza),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers controller_interfaces__srv__SetMaxPizza_Request__rosidl_typesupport_introspection_c__SetMaxPizza_Request_message_members = {
  "controller_interfaces__srv",  // message namespace
  "SetMaxPizza_Request",  // message name
  1,  // number of fields
  sizeof(controller_interfaces__srv__SetMaxPizza_Request),
  controller_interfaces__srv__SetMaxPizza_Request__rosidl_typesupport_introspection_c__SetMaxPizza_Request_message_member_array,  // message members
  controller_interfaces__srv__SetMaxPizza_Request__rosidl_typesupport_introspection_c__SetMaxPizza_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  controller_interfaces__srv__SetMaxPizza_Request__rosidl_typesupport_introspection_c__SetMaxPizza_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t controller_interfaces__srv__SetMaxPizza_Request__rosidl_typesupport_introspection_c__SetMaxPizza_Request_message_type_support_handle = {
  0,
  &controller_interfaces__srv__SetMaxPizza_Request__rosidl_typesupport_introspection_c__SetMaxPizza_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_controller_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller_interfaces, srv, SetMaxPizza_Request)() {
  controller_interfaces__srv__SetMaxPizza_Request__rosidl_typesupport_introspection_c__SetMaxPizza_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Int64)();
  if (!controller_interfaces__srv__SetMaxPizza_Request__rosidl_typesupport_introspection_c__SetMaxPizza_Request_message_type_support_handle.typesupport_identifier) {
    controller_interfaces__srv__SetMaxPizza_Request__rosidl_typesupport_introspection_c__SetMaxPizza_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &controller_interfaces__srv__SetMaxPizza_Request__rosidl_typesupport_introspection_c__SetMaxPizza_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "controller_interfaces/srv/detail/set_max_pizza__rosidl_typesupport_introspection_c.h"
// already included above
// #include "controller_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "controller_interfaces/srv/detail/set_max_pizza__functions.h"
// already included above
// #include "controller_interfaces/srv/detail/set_max_pizza__struct.h"


// Include directives for member types
// Member `log`
#include "std_msgs/msg/string.h"
// Member `log`
#include "std_msgs/msg/detail/string__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void controller_interfaces__srv__SetMaxPizza_Response__rosidl_typesupport_introspection_c__SetMaxPizza_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  controller_interfaces__srv__SetMaxPizza_Response__init(message_memory);
}

void controller_interfaces__srv__SetMaxPizza_Response__rosidl_typesupport_introspection_c__SetMaxPizza_Response_fini_function(void * message_memory)
{
  controller_interfaces__srv__SetMaxPizza_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember controller_interfaces__srv__SetMaxPizza_Response__rosidl_typesupport_introspection_c__SetMaxPizza_Response_message_member_array[1] = {
  {
    "log",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller_interfaces__srv__SetMaxPizza_Response, log),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers controller_interfaces__srv__SetMaxPizza_Response__rosidl_typesupport_introspection_c__SetMaxPizza_Response_message_members = {
  "controller_interfaces__srv",  // message namespace
  "SetMaxPizza_Response",  // message name
  1,  // number of fields
  sizeof(controller_interfaces__srv__SetMaxPizza_Response),
  controller_interfaces__srv__SetMaxPizza_Response__rosidl_typesupport_introspection_c__SetMaxPizza_Response_message_member_array,  // message members
  controller_interfaces__srv__SetMaxPizza_Response__rosidl_typesupport_introspection_c__SetMaxPizza_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  controller_interfaces__srv__SetMaxPizza_Response__rosidl_typesupport_introspection_c__SetMaxPizza_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t controller_interfaces__srv__SetMaxPizza_Response__rosidl_typesupport_introspection_c__SetMaxPizza_Response_message_type_support_handle = {
  0,
  &controller_interfaces__srv__SetMaxPizza_Response__rosidl_typesupport_introspection_c__SetMaxPizza_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_controller_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller_interfaces, srv, SetMaxPizza_Response)() {
  controller_interfaces__srv__SetMaxPizza_Response__rosidl_typesupport_introspection_c__SetMaxPizza_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, String)();
  if (!controller_interfaces__srv__SetMaxPizza_Response__rosidl_typesupport_introspection_c__SetMaxPizza_Response_message_type_support_handle.typesupport_identifier) {
    controller_interfaces__srv__SetMaxPizza_Response__rosidl_typesupport_introspection_c__SetMaxPizza_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &controller_interfaces__srv__SetMaxPizza_Response__rosidl_typesupport_introspection_c__SetMaxPizza_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "controller_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "controller_interfaces/srv/detail/set_max_pizza__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers controller_interfaces__srv__detail__set_max_pizza__rosidl_typesupport_introspection_c__SetMaxPizza_service_members = {
  "controller_interfaces__srv",  // service namespace
  "SetMaxPizza",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // controller_interfaces__srv__detail__set_max_pizza__rosidl_typesupport_introspection_c__SetMaxPizza_Request_message_type_support_handle,
  NULL  // response message
  // controller_interfaces__srv__detail__set_max_pizza__rosidl_typesupport_introspection_c__SetMaxPizza_Response_message_type_support_handle
};

static rosidl_service_type_support_t controller_interfaces__srv__detail__set_max_pizza__rosidl_typesupport_introspection_c__SetMaxPizza_service_type_support_handle = {
  0,
  &controller_interfaces__srv__detail__set_max_pizza__rosidl_typesupport_introspection_c__SetMaxPizza_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller_interfaces, srv, SetMaxPizza_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller_interfaces, srv, SetMaxPizza_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_controller_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller_interfaces, srv, SetMaxPizza)() {
  if (!controller_interfaces__srv__detail__set_max_pizza__rosidl_typesupport_introspection_c__SetMaxPizza_service_type_support_handle.typesupport_identifier) {
    controller_interfaces__srv__detail__set_max_pizza__rosidl_typesupport_introspection_c__SetMaxPizza_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)controller_interfaces__srv__detail__set_max_pizza__rosidl_typesupport_introspection_c__SetMaxPizza_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller_interfaces, srv, SetMaxPizza_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller_interfaces, srv, SetMaxPizza_Response)()->data;
  }

  return &controller_interfaces__srv__detail__set_max_pizza__rosidl_typesupport_introspection_c__SetMaxPizza_service_type_support_handle;
}
