// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller_interfaces:srv/SetMaxPizza.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER_INTERFACES__SRV__DETAIL__SET_MAX_PIZZA__STRUCT_H_
#define CONTROLLER_INTERFACES__SRV__DETAIL__SET_MAX_PIZZA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'max_pizza'
#include "std_msgs/msg/detail/int64__struct.h"

/// Struct defined in srv/SetMaxPizza in the package controller_interfaces.
typedef struct controller_interfaces__srv__SetMaxPizza_Request
{
  std_msgs__msg__Int64 max_pizza;
} controller_interfaces__srv__SetMaxPizza_Request;

// Struct for a sequence of controller_interfaces__srv__SetMaxPizza_Request.
typedef struct controller_interfaces__srv__SetMaxPizza_Request__Sequence
{
  controller_interfaces__srv__SetMaxPizza_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller_interfaces__srv__SetMaxPizza_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'log'
#include "std_msgs/msg/detail/string__struct.h"

/// Struct defined in srv/SetMaxPizza in the package controller_interfaces.
typedef struct controller_interfaces__srv__SetMaxPizza_Response
{
  std_msgs__msg__String log;
} controller_interfaces__srv__SetMaxPizza_Response;

// Struct for a sequence of controller_interfaces__srv__SetMaxPizza_Response.
typedef struct controller_interfaces__srv__SetMaxPizza_Response__Sequence
{
  controller_interfaces__srv__SetMaxPizza_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller_interfaces__srv__SetMaxPizza_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER_INTERFACES__SRV__DETAIL__SET_MAX_PIZZA__STRUCT_H_
