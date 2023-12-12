// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from deltarobot_interfaces:msg/SetPointsVector.idl
// generated code does not contain a copyright notice

#ifndef DELTAROBOT_INTERFACES__MSG__DETAIL__SET_POINTS_VECTOR__STRUCT_HPP_
#define DELTAROBOT_INTERFACES__MSG__DETAIL__SET_POINTS_VECTOR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'set_points'
#include "deltarobot_interfaces/msg/detail/set_point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__deltarobot_interfaces__msg__SetPointsVector __attribute__((deprecated))
#else
# define DEPRECATED__deltarobot_interfaces__msg__SetPointsVector __declspec(deprecated)
#endif

namespace deltarobot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SetPointsVector_
{
  using Type = SetPointsVector_<ContainerAllocator>;

  explicit SetPointsVector_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit SetPointsVector_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _set_points_type =
    std::vector<deltarobot_interfaces::msg::SetPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<deltarobot_interfaces::msg::SetPoint_<ContainerAllocator>>>;
  _set_points_type set_points;

  // setters for named parameter idiom
  Type & set__set_points(
    const std::vector<deltarobot_interfaces::msg::SetPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<deltarobot_interfaces::msg::SetPoint_<ContainerAllocator>>> & _arg)
  {
    this->set_points = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    deltarobot_interfaces::msg::SetPointsVector_<ContainerAllocator> *;
  using ConstRawPtr =
    const deltarobot_interfaces::msg::SetPointsVector_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<deltarobot_interfaces::msg::SetPointsVector_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<deltarobot_interfaces::msg::SetPointsVector_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      deltarobot_interfaces::msg::SetPointsVector_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<deltarobot_interfaces::msg::SetPointsVector_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      deltarobot_interfaces::msg::SetPointsVector_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<deltarobot_interfaces::msg::SetPointsVector_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<deltarobot_interfaces::msg::SetPointsVector_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<deltarobot_interfaces::msg::SetPointsVector_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__deltarobot_interfaces__msg__SetPointsVector
    std::shared_ptr<deltarobot_interfaces::msg::SetPointsVector_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__deltarobot_interfaces__msg__SetPointsVector
    std::shared_ptr<deltarobot_interfaces::msg::SetPointsVector_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetPointsVector_ & other) const
  {
    if (this->set_points != other.set_points) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetPointsVector_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetPointsVector_

// alias to use template instance with default allocator
using SetPointsVector =
  deltarobot_interfaces::msg::SetPointsVector_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace deltarobot_interfaces

#endif  // DELTAROBOT_INTERFACES__MSG__DETAIL__SET_POINTS_VECTOR__STRUCT_HPP_
