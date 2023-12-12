// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from deltarobot_interfaces:msg/TrajectoryTask.idl
// generated code does not contain a copyright notice

#ifndef DELTAROBOT_INTERFACES__MSG__DETAIL__TRAJECTORY_TASK__BUILDER_HPP_
#define DELTAROBOT_INTERFACES__MSG__DETAIL__TRAJECTORY_TASK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "deltarobot_interfaces/msg/detail/trajectory_task__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace deltarobot_interfaces
{

namespace msg
{

namespace builder
{

class Init_TrajectoryTask_task_type
{
public:
  explicit Init_TrajectoryTask_task_type(::deltarobot_interfaces::msg::TrajectoryTask & msg)
  : msg_(msg)
  {}
  ::deltarobot_interfaces::msg::TrajectoryTask task_type(::deltarobot_interfaces::msg::TrajectoryTask::_task_type_type arg)
  {
    msg_.task_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::deltarobot_interfaces::msg::TrajectoryTask msg_;
};

class Init_TrajectoryTask_task_time
{
public:
  explicit Init_TrajectoryTask_task_time(::deltarobot_interfaces::msg::TrajectoryTask & msg)
  : msg_(msg)
  {}
  Init_TrajectoryTask_task_type task_time(::deltarobot_interfaces::msg::TrajectoryTask::_task_time_type arg)
  {
    msg_.task_time = std::move(arg);
    return Init_TrajectoryTask_task_type(msg_);
  }

private:
  ::deltarobot_interfaces::msg::TrajectoryTask msg_;
};

class Init_TrajectoryTask_pos_end
{
public:
  explicit Init_TrajectoryTask_pos_end(::deltarobot_interfaces::msg::TrajectoryTask & msg)
  : msg_(msg)
  {}
  Init_TrajectoryTask_task_time pos_end(::deltarobot_interfaces::msg::TrajectoryTask::_pos_end_type arg)
  {
    msg_.pos_end = std::move(arg);
    return Init_TrajectoryTask_task_time(msg_);
  }

private:
  ::deltarobot_interfaces::msg::TrajectoryTask msg_;
};

class Init_TrajectoryTask_pos_start
{
public:
  Init_TrajectoryTask_pos_start()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrajectoryTask_pos_end pos_start(::deltarobot_interfaces::msg::TrajectoryTask::_pos_start_type arg)
  {
    msg_.pos_start = std::move(arg);
    return Init_TrajectoryTask_pos_end(msg_);
  }

private:
  ::deltarobot_interfaces::msg::TrajectoryTask msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::deltarobot_interfaces::msg::TrajectoryTask>()
{
  return deltarobot_interfaces::msg::builder::Init_TrajectoryTask_pos_start();
}

}  // namespace deltarobot_interfaces

#endif  // DELTAROBOT_INTERFACES__MSG__DETAIL__TRAJECTORY_TASK__BUILDER_HPP_
