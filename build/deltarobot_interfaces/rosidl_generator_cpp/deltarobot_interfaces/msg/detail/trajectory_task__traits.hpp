// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from deltarobot_interfaces:msg/TrajectoryTask.idl
// generated code does not contain a copyright notice

#ifndef DELTAROBOT_INTERFACES__MSG__DETAIL__TRAJECTORY_TASK__TRAITS_HPP_
#define DELTAROBOT_INTERFACES__MSG__DETAIL__TRAJECTORY_TASK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "deltarobot_interfaces/msg/detail/trajectory_task__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pos_start'
// Member 'pos_end'
#include "geometry_msgs/msg/detail/point32__traits.hpp"
// Member 'task_type'
#include "std_msgs/msg/detail/string__traits.hpp"

namespace deltarobot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const TrajectoryTask & msg,
  std::ostream & out)
{
  out << "{";
  // member: pos_start
  {
    out << "pos_start: ";
    to_flow_style_yaml(msg.pos_start, out);
    out << ", ";
  }

  // member: pos_end
  {
    out << "pos_end: ";
    to_flow_style_yaml(msg.pos_end, out);
    out << ", ";
  }

  // member: task_time
  {
    out << "task_time: ";
    rosidl_generator_traits::value_to_yaml(msg.task_time, out);
    out << ", ";
  }

  // member: task_type
  {
    out << "task_type: ";
    to_flow_style_yaml(msg.task_type, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TrajectoryTask & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pos_start
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos_start:\n";
    to_block_style_yaml(msg.pos_start, out, indentation + 2);
  }

  // member: pos_end
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos_end:\n";
    to_block_style_yaml(msg.pos_end, out, indentation + 2);
  }

  // member: task_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "task_time: ";
    rosidl_generator_traits::value_to_yaml(msg.task_time, out);
    out << "\n";
  }

  // member: task_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "task_type:\n";
    to_block_style_yaml(msg.task_type, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TrajectoryTask & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace deltarobot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use deltarobot_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const deltarobot_interfaces::msg::TrajectoryTask & msg,
  std::ostream & out, size_t indentation = 0)
{
  deltarobot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use deltarobot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const deltarobot_interfaces::msg::TrajectoryTask & msg)
{
  return deltarobot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<deltarobot_interfaces::msg::TrajectoryTask>()
{
  return "deltarobot_interfaces::msg::TrajectoryTask";
}

template<>
inline const char * name<deltarobot_interfaces::msg::TrajectoryTask>()
{
  return "deltarobot_interfaces/msg/TrajectoryTask";
}

template<>
struct has_fixed_size<deltarobot_interfaces::msg::TrajectoryTask>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point32>::value && has_fixed_size<std_msgs::msg::String>::value> {};

template<>
struct has_bounded_size<deltarobot_interfaces::msg::TrajectoryTask>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point32>::value && has_bounded_size<std_msgs::msg::String>::value> {};

template<>
struct is_message<deltarobot_interfaces::msg::TrajectoryTask>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DELTAROBOT_INTERFACES__MSG__DETAIL__TRAJECTORY_TASK__TRAITS_HPP_
