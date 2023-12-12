// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from deltarobot_interfaces:msg/SetPointsVector.idl
// generated code does not contain a copyright notice

#ifndef DELTAROBOT_INTERFACES__MSG__DETAIL__SET_POINTS_VECTOR__TRAITS_HPP_
#define DELTAROBOT_INTERFACES__MSG__DETAIL__SET_POINTS_VECTOR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "deltarobot_interfaces/msg/detail/set_points_vector__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'set_points'
#include "deltarobot_interfaces/msg/detail/set_point__traits.hpp"

namespace deltarobot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const SetPointsVector & msg,
  std::ostream & out)
{
  out << "{";
  // member: set_points
  {
    if (msg.set_points.size() == 0) {
      out << "set_points: []";
    } else {
      out << "set_points: [";
      size_t pending_items = msg.set_points.size();
      for (auto item : msg.set_points) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetPointsVector & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: set_points
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.set_points.size() == 0) {
      out << "set_points: []\n";
    } else {
      out << "set_points:\n";
      for (auto item : msg.set_points) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetPointsVector & msg, bool use_flow_style = false)
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
  const deltarobot_interfaces::msg::SetPointsVector & msg,
  std::ostream & out, size_t indentation = 0)
{
  deltarobot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use deltarobot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const deltarobot_interfaces::msg::SetPointsVector & msg)
{
  return deltarobot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<deltarobot_interfaces::msg::SetPointsVector>()
{
  return "deltarobot_interfaces::msg::SetPointsVector";
}

template<>
inline const char * name<deltarobot_interfaces::msg::SetPointsVector>()
{
  return "deltarobot_interfaces/msg/SetPointsVector";
}

template<>
struct has_fixed_size<deltarobot_interfaces::msg::SetPointsVector>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<deltarobot_interfaces::msg::SetPointsVector>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<deltarobot_interfaces::msg::SetPointsVector>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DELTAROBOT_INTERFACES__MSG__DETAIL__SET_POINTS_VECTOR__TRAITS_HPP_
