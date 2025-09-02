// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller_interfaces:srv/SetMaxPizza.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER_INTERFACES__SRV__DETAIL__SET_MAX_PIZZA__TRAITS_HPP_
#define CONTROLLER_INTERFACES__SRV__DETAIL__SET_MAX_PIZZA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller_interfaces/srv/detail/set_max_pizza__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'max_pizza'
#include "std_msgs/msg/detail/int64__traits.hpp"

namespace controller_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetMaxPizza_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: max_pizza
  {
    out << "max_pizza: ";
    to_flow_style_yaml(msg.max_pizza, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetMaxPizza_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: max_pizza
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_pizza:\n";
    to_block_style_yaml(msg.max_pizza, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetMaxPizza_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace controller_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use controller_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const controller_interfaces::srv::SetMaxPizza_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller_interfaces::srv::SetMaxPizza_Request & msg)
{
  return controller_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller_interfaces::srv::SetMaxPizza_Request>()
{
  return "controller_interfaces::srv::SetMaxPizza_Request";
}

template<>
inline const char * name<controller_interfaces::srv::SetMaxPizza_Request>()
{
  return "controller_interfaces/srv/SetMaxPizza_Request";
}

template<>
struct has_fixed_size<controller_interfaces::srv::SetMaxPizza_Request>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Int64>::value> {};

template<>
struct has_bounded_size<controller_interfaces::srv::SetMaxPizza_Request>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Int64>::value> {};

template<>
struct is_message<controller_interfaces::srv::SetMaxPizza_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'log'
#include "std_msgs/msg/detail/string__traits.hpp"

namespace controller_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetMaxPizza_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: log
  {
    out << "log: ";
    to_flow_style_yaml(msg.log, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetMaxPizza_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: log
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "log:\n";
    to_block_style_yaml(msg.log, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetMaxPizza_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace controller_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use controller_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const controller_interfaces::srv::SetMaxPizza_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller_interfaces::srv::SetMaxPizza_Response & msg)
{
  return controller_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller_interfaces::srv::SetMaxPizza_Response>()
{
  return "controller_interfaces::srv::SetMaxPizza_Response";
}

template<>
inline const char * name<controller_interfaces::srv::SetMaxPizza_Response>()
{
  return "controller_interfaces/srv/SetMaxPizza_Response";
}

template<>
struct has_fixed_size<controller_interfaces::srv::SetMaxPizza_Response>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::String>::value> {};

template<>
struct has_bounded_size<controller_interfaces::srv::SetMaxPizza_Response>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::String>::value> {};

template<>
struct is_message<controller_interfaces::srv::SetMaxPizza_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<controller_interfaces::srv::SetMaxPizza>()
{
  return "controller_interfaces::srv::SetMaxPizza";
}

template<>
inline const char * name<controller_interfaces::srv::SetMaxPizza>()
{
  return "controller_interfaces/srv/SetMaxPizza";
}

template<>
struct has_fixed_size<controller_interfaces::srv::SetMaxPizza>
  : std::integral_constant<
    bool,
    has_fixed_size<controller_interfaces::srv::SetMaxPizza_Request>::value &&
    has_fixed_size<controller_interfaces::srv::SetMaxPizza_Response>::value
  >
{
};

template<>
struct has_bounded_size<controller_interfaces::srv::SetMaxPizza>
  : std::integral_constant<
    bool,
    has_bounded_size<controller_interfaces::srv::SetMaxPizza_Request>::value &&
    has_bounded_size<controller_interfaces::srv::SetMaxPizza_Response>::value
  >
{
};

template<>
struct is_service<controller_interfaces::srv::SetMaxPizza>
  : std::true_type
{
};

template<>
struct is_service_request<controller_interfaces::srv::SetMaxPizza_Request>
  : std::true_type
{
};

template<>
struct is_service_response<controller_interfaces::srv::SetMaxPizza_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER_INTERFACES__SRV__DETAIL__SET_MAX_PIZZA__TRAITS_HPP_
