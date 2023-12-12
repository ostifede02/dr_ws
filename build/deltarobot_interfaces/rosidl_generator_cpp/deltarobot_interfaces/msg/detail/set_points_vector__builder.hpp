// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from deltarobot_interfaces:msg/SetPointsVector.idl
// generated code does not contain a copyright notice

#ifndef DELTAROBOT_INTERFACES__MSG__DETAIL__SET_POINTS_VECTOR__BUILDER_HPP_
#define DELTAROBOT_INTERFACES__MSG__DETAIL__SET_POINTS_VECTOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "deltarobot_interfaces/msg/detail/set_points_vector__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace deltarobot_interfaces
{

namespace msg
{

namespace builder
{

class Init_SetPointsVector_set_points
{
public:
  Init_SetPointsVector_set_points()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::deltarobot_interfaces::msg::SetPointsVector set_points(::deltarobot_interfaces::msg::SetPointsVector::_set_points_type arg)
  {
    msg_.set_points = std::move(arg);
    return std::move(msg_);
  }

private:
  ::deltarobot_interfaces::msg::SetPointsVector msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::deltarobot_interfaces::msg::SetPointsVector>()
{
  return deltarobot_interfaces::msg::builder::Init_SetPointsVector_set_points();
}

}  // namespace deltarobot_interfaces

#endif  // DELTAROBOT_INTERFACES__MSG__DETAIL__SET_POINTS_VECTOR__BUILDER_HPP_
