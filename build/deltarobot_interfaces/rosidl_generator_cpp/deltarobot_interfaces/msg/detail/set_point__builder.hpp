// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from deltarobot_interfaces:msg/SetPoint.idl
// generated code does not contain a copyright notice

#ifndef DELTAROBOT_INTERFACES__MSG__DETAIL__SET_POINT__BUILDER_HPP_
#define DELTAROBOT_INTERFACES__MSG__DETAIL__SET_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "deltarobot_interfaces/msg/detail/set_point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace deltarobot_interfaces
{

namespace msg
{

namespace builder
{

class Init_SetPoint_delta_t
{
public:
  explicit Init_SetPoint_delta_t(::deltarobot_interfaces::msg::SetPoint & msg)
  : msg_(msg)
  {}
  ::deltarobot_interfaces::msg::SetPoint delta_t(::deltarobot_interfaces::msg::SetPoint::_delta_t_type arg)
  {
    msg_.delta_t = std::move(arg);
    return std::move(msg_);
  }

private:
  ::deltarobot_interfaces::msg::SetPoint msg_;
};

class Init_SetPoint_z
{
public:
  explicit Init_SetPoint_z(::deltarobot_interfaces::msg::SetPoint & msg)
  : msg_(msg)
  {}
  Init_SetPoint_delta_t z(::deltarobot_interfaces::msg::SetPoint::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_SetPoint_delta_t(msg_);
  }

private:
  ::deltarobot_interfaces::msg::SetPoint msg_;
};

class Init_SetPoint_x
{
public:
  Init_SetPoint_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetPoint_z x(::deltarobot_interfaces::msg::SetPoint::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_SetPoint_z(msg_);
  }

private:
  ::deltarobot_interfaces::msg::SetPoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::deltarobot_interfaces::msg::SetPoint>()
{
  return deltarobot_interfaces::msg::builder::Init_SetPoint_x();
}

}  // namespace deltarobot_interfaces

#endif  // DELTAROBOT_INTERFACES__MSG__DETAIL__SET_POINT__BUILDER_HPP_
