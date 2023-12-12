// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from deltarobot_interfaces:msg/TrajectoryTask.idl
// generated code does not contain a copyright notice

#ifndef DELTAROBOT_INTERFACES__MSG__DETAIL__TRAJECTORY_TASK__STRUCT_H_
#define DELTAROBOT_INTERFACES__MSG__DETAIL__TRAJECTORY_TASK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pos_start'
// Member 'pos_end'
#include "geometry_msgs/msg/detail/point32__struct.h"
// Member 'task_type'
#include "std_msgs/msg/detail/string__struct.h"

/// Struct defined in msg/TrajectoryTask in the package deltarobot_interfaces.
typedef struct deltarobot_interfaces__msg__TrajectoryTask
{
  geometry_msgs__msg__Point32 pos_start;
  geometry_msgs__msg__Point32 pos_end;
  float task_time;
  std_msgs__msg__String task_type;
} deltarobot_interfaces__msg__TrajectoryTask;

// Struct for a sequence of deltarobot_interfaces__msg__TrajectoryTask.
typedef struct deltarobot_interfaces__msg__TrajectoryTask__Sequence
{
  deltarobot_interfaces__msg__TrajectoryTask * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} deltarobot_interfaces__msg__TrajectoryTask__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DELTAROBOT_INTERFACES__MSG__DETAIL__TRAJECTORY_TASK__STRUCT_H_
