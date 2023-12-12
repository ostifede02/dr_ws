// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from deltarobot_interfaces:msg/TrajectoryTask.idl
// generated code does not contain a copyright notice

#ifndef DELTAROBOT_INTERFACES__MSG__DETAIL__TRAJECTORY_TASK__STRUCT_HPP_
#define DELTAROBOT_INTERFACES__MSG__DETAIL__TRAJECTORY_TASK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pos_start'
// Member 'pos_end'
#include "geometry_msgs/msg/detail/point32__struct.hpp"
// Member 'task_type'
#include "std_msgs/msg/detail/string__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__deltarobot_interfaces__msg__TrajectoryTask __attribute__((deprecated))
#else
# define DEPRECATED__deltarobot_interfaces__msg__TrajectoryTask __declspec(deprecated)
#endif

namespace deltarobot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrajectoryTask_
{
  using Type = TrajectoryTask_<ContainerAllocator>;

  explicit TrajectoryTask_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pos_start(_init),
    pos_end(_init),
    task_type(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->task_time = 0.0f;
    }
  }

  explicit TrajectoryTask_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pos_start(_alloc, _init),
    pos_end(_alloc, _init),
    task_type(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->task_time = 0.0f;
    }
  }

  // field types and members
  using _pos_start_type =
    geometry_msgs::msg::Point32_<ContainerAllocator>;
  _pos_start_type pos_start;
  using _pos_end_type =
    geometry_msgs::msg::Point32_<ContainerAllocator>;
  _pos_end_type pos_end;
  using _task_time_type =
    float;
  _task_time_type task_time;
  using _task_type_type =
    std_msgs::msg::String_<ContainerAllocator>;
  _task_type_type task_type;

  // setters for named parameter idiom
  Type & set__pos_start(
    const geometry_msgs::msg::Point32_<ContainerAllocator> & _arg)
  {
    this->pos_start = _arg;
    return *this;
  }
  Type & set__pos_end(
    const geometry_msgs::msg::Point32_<ContainerAllocator> & _arg)
  {
    this->pos_end = _arg;
    return *this;
  }
  Type & set__task_time(
    const float & _arg)
  {
    this->task_time = _arg;
    return *this;
  }
  Type & set__task_type(
    const std_msgs::msg::String_<ContainerAllocator> & _arg)
  {
    this->task_type = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    deltarobot_interfaces::msg::TrajectoryTask_<ContainerAllocator> *;
  using ConstRawPtr =
    const deltarobot_interfaces::msg::TrajectoryTask_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<deltarobot_interfaces::msg::TrajectoryTask_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<deltarobot_interfaces::msg::TrajectoryTask_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      deltarobot_interfaces::msg::TrajectoryTask_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<deltarobot_interfaces::msg::TrajectoryTask_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      deltarobot_interfaces::msg::TrajectoryTask_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<deltarobot_interfaces::msg::TrajectoryTask_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<deltarobot_interfaces::msg::TrajectoryTask_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<deltarobot_interfaces::msg::TrajectoryTask_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__deltarobot_interfaces__msg__TrajectoryTask
    std::shared_ptr<deltarobot_interfaces::msg::TrajectoryTask_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__deltarobot_interfaces__msg__TrajectoryTask
    std::shared_ptr<deltarobot_interfaces::msg::TrajectoryTask_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrajectoryTask_ & other) const
  {
    if (this->pos_start != other.pos_start) {
      return false;
    }
    if (this->pos_end != other.pos_end) {
      return false;
    }
    if (this->task_time != other.task_time) {
      return false;
    }
    if (this->task_type != other.task_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrajectoryTask_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrajectoryTask_

// alias to use template instance with default allocator
using TrajectoryTask =
  deltarobot_interfaces::msg::TrajectoryTask_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace deltarobot_interfaces

#endif  // DELTAROBOT_INTERFACES__MSG__DETAIL__TRAJECTORY_TASK__STRUCT_HPP_
