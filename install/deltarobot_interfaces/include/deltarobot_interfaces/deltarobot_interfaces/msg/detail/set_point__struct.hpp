// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from deltarobot_interfaces:msg/SetPoint.idl
// generated code does not contain a copyright notice

#ifndef DELTAROBOT_INTERFACES__MSG__DETAIL__SET_POINT__STRUCT_HPP_
#define DELTAROBOT_INTERFACES__MSG__DETAIL__SET_POINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__deltarobot_interfaces__msg__SetPoint __attribute__((deprecated))
#else
# define DEPRECATED__deltarobot_interfaces__msg__SetPoint __declspec(deprecated)
#endif

namespace deltarobot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SetPoint_
{
  using Type = SetPoint_<ContainerAllocator>;

  explicit SetPoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->z = 0.0f;
      this->delta_t = 0.0f;
    }
  }

  explicit SetPoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->z = 0.0f;
      this->delta_t = 0.0f;
    }
  }

  // field types and members
  using _x_type =
    float;
  _x_type x;
  using _z_type =
    float;
  _z_type z;
  using _delta_t_type =
    float;
  _delta_t_type delta_t;

  // setters for named parameter idiom
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__z(
    const float & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__delta_t(
    const float & _arg)
  {
    this->delta_t = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    deltarobot_interfaces::msg::SetPoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const deltarobot_interfaces::msg::SetPoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<deltarobot_interfaces::msg::SetPoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<deltarobot_interfaces::msg::SetPoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      deltarobot_interfaces::msg::SetPoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<deltarobot_interfaces::msg::SetPoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      deltarobot_interfaces::msg::SetPoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<deltarobot_interfaces::msg::SetPoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<deltarobot_interfaces::msg::SetPoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<deltarobot_interfaces::msg::SetPoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__deltarobot_interfaces__msg__SetPoint
    std::shared_ptr<deltarobot_interfaces::msg::SetPoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__deltarobot_interfaces__msg__SetPoint
    std::shared_ptr<deltarobot_interfaces::msg::SetPoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetPoint_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->delta_t != other.delta_t) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetPoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetPoint_

// alias to use template instance with default allocator
using SetPoint =
  deltarobot_interfaces::msg::SetPoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace deltarobot_interfaces

#endif  // DELTAROBOT_INTERFACES__MSG__DETAIL__SET_POINT__STRUCT_HPP_
