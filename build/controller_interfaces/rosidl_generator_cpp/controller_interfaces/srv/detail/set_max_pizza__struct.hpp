// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller_interfaces:srv/SetMaxPizza.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER_INTERFACES__SRV__DETAIL__SET_MAX_PIZZA__STRUCT_HPP_
#define CONTROLLER_INTERFACES__SRV__DETAIL__SET_MAX_PIZZA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'max_pizza'
#include "std_msgs/msg/detail/int64__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__controller_interfaces__srv__SetMaxPizza_Request __attribute__((deprecated))
#else
# define DEPRECATED__controller_interfaces__srv__SetMaxPizza_Request __declspec(deprecated)
#endif

namespace controller_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetMaxPizza_Request_
{
  using Type = SetMaxPizza_Request_<ContainerAllocator>;

  explicit SetMaxPizza_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : max_pizza(_init)
  {
    (void)_init;
  }

  explicit SetMaxPizza_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : max_pizza(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _max_pizza_type =
    std_msgs::msg::Int64_<ContainerAllocator>;
  _max_pizza_type max_pizza;

  // setters for named parameter idiom
  Type & set__max_pizza(
    const std_msgs::msg::Int64_<ContainerAllocator> & _arg)
  {
    this->max_pizza = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller_interfaces::srv::SetMaxPizza_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller_interfaces::srv::SetMaxPizza_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller_interfaces::srv::SetMaxPizza_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller_interfaces::srv::SetMaxPizza_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller_interfaces::srv::SetMaxPizza_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller_interfaces::srv::SetMaxPizza_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller_interfaces::srv::SetMaxPizza_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller_interfaces::srv::SetMaxPizza_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller_interfaces::srv::SetMaxPizza_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller_interfaces::srv::SetMaxPizza_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller_interfaces__srv__SetMaxPizza_Request
    std::shared_ptr<controller_interfaces::srv::SetMaxPizza_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller_interfaces__srv__SetMaxPizza_Request
    std::shared_ptr<controller_interfaces::srv::SetMaxPizza_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetMaxPizza_Request_ & other) const
  {
    if (this->max_pizza != other.max_pizza) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetMaxPizza_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetMaxPizza_Request_

// alias to use template instance with default allocator
using SetMaxPizza_Request =
  controller_interfaces::srv::SetMaxPizza_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller_interfaces


// Include directives for member types
// Member 'log'
#include "std_msgs/msg/detail/string__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__controller_interfaces__srv__SetMaxPizza_Response __attribute__((deprecated))
#else
# define DEPRECATED__controller_interfaces__srv__SetMaxPizza_Response __declspec(deprecated)
#endif

namespace controller_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetMaxPizza_Response_
{
  using Type = SetMaxPizza_Response_<ContainerAllocator>;

  explicit SetMaxPizza_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : log(_init)
  {
    (void)_init;
  }

  explicit SetMaxPizza_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : log(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _log_type =
    std_msgs::msg::String_<ContainerAllocator>;
  _log_type log;

  // setters for named parameter idiom
  Type & set__log(
    const std_msgs::msg::String_<ContainerAllocator> & _arg)
  {
    this->log = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller_interfaces::srv::SetMaxPizza_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller_interfaces::srv::SetMaxPizza_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller_interfaces::srv::SetMaxPizza_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller_interfaces::srv::SetMaxPizza_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller_interfaces::srv::SetMaxPizza_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller_interfaces::srv::SetMaxPizza_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller_interfaces::srv::SetMaxPizza_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller_interfaces::srv::SetMaxPizza_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller_interfaces::srv::SetMaxPizza_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller_interfaces::srv::SetMaxPizza_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller_interfaces__srv__SetMaxPizza_Response
    std::shared_ptr<controller_interfaces::srv::SetMaxPizza_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller_interfaces__srv__SetMaxPizza_Response
    std::shared_ptr<controller_interfaces::srv::SetMaxPizza_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetMaxPizza_Response_ & other) const
  {
    if (this->log != other.log) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetMaxPizza_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetMaxPizza_Response_

// alias to use template instance with default allocator
using SetMaxPizza_Response =
  controller_interfaces::srv::SetMaxPizza_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller_interfaces

namespace controller_interfaces
{

namespace srv
{

struct SetMaxPizza
{
  using Request = controller_interfaces::srv::SetMaxPizza_Request;
  using Response = controller_interfaces::srv::SetMaxPizza_Response;
};

}  // namespace srv

}  // namespace controller_interfaces

#endif  // CONTROLLER_INTERFACES__SRV__DETAIL__SET_MAX_PIZZA__STRUCT_HPP_
