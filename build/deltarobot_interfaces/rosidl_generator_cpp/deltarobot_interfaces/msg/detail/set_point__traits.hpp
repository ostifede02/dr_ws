// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from deltarobot_interfaces:msg/SetPoint.idl
// generated code does not contain a copyright notice

#ifndef DELTAROBOT_INTERFACES__MSG__DETAIL__SET_POINT__TRAITS_HPP_
#define DELTAROBOT_INTERFACES__MSG__DETAIL__SET_POINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "deltarobot_interfaces/msg/detail/set_point__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace deltarobot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const SetPoint & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << ", ";
  }

  // member: delta_t
  {
    out << "delta_t: ";
    rosidl_generator_traits::value_to_yaml(msg.delta_t, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetPoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }

  // member: delta_t
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "delta_t: ";
    rosidl_generator_traits::value_to_yaml(msg.delta_t, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetPoint & msg, bool use_flow_style = false)
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
  const deltarobot_interfaces::msg::SetPoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  deltarobot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use deltarobot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const deltarobot_interfaces::msg::SetPoint & msg)
{
  return deltarobot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<deltarobot_interfaces::msg::SetPoint>()
{
  return "deltarobot_interfaces::msg::SetPoint";
}

template<>
inline const char * name<deltarobot_interfaces::msg::SetPoint>()
{
  return "deltarobot_interfaces/msg/SetPoint";
}

template<>
struct has_fixed_size<deltarobot_interfaces::msg::SetPoint>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<deltarobot_interfaces::msg::SetPoint>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<deltarobot_interfaces::msg::SetPoint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DELTAROBOT_INTERFACES__MSG__DETAIL__SET_POINT__TRAITS_HPP_
