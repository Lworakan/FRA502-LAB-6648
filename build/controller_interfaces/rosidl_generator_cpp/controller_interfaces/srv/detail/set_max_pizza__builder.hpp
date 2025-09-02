// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller_interfaces:srv/SetMaxPizza.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER_INTERFACES__SRV__DETAIL__SET_MAX_PIZZA__BUILDER_HPP_
#define CONTROLLER_INTERFACES__SRV__DETAIL__SET_MAX_PIZZA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller_interfaces/srv/detail/set_max_pizza__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetMaxPizza_Request_max_pizza
{
public:
  Init_SetMaxPizza_Request_max_pizza()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller_interfaces::srv::SetMaxPizza_Request max_pizza(::controller_interfaces::srv::SetMaxPizza_Request::_max_pizza_type arg)
  {
    msg_.max_pizza = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller_interfaces::srv::SetMaxPizza_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller_interfaces::srv::SetMaxPizza_Request>()
{
  return controller_interfaces::srv::builder::Init_SetMaxPizza_Request_max_pizza();
}

}  // namespace controller_interfaces


namespace controller_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetMaxPizza_Response_log
{
public:
  Init_SetMaxPizza_Response_log()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller_interfaces::srv::SetMaxPizza_Response log(::controller_interfaces::srv::SetMaxPizza_Response::_log_type arg)
  {
    msg_.log = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller_interfaces::srv::SetMaxPizza_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller_interfaces::srv::SetMaxPizza_Response>()
{
  return controller_interfaces::srv::builder::Init_SetMaxPizza_Response_log();
}

}  // namespace controller_interfaces

#endif  // CONTROLLER_INTERFACES__SRV__DETAIL__SET_MAX_PIZZA__BUILDER_HPP_
