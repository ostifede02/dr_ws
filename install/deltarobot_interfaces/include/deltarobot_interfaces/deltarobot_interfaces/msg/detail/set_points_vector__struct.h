// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from deltarobot_interfaces:msg/SetPointsVector.idl
// generated code does not contain a copyright notice

#ifndef DELTAROBOT_INTERFACES__MSG__DETAIL__SET_POINTS_VECTOR__STRUCT_H_
#define DELTAROBOT_INTERFACES__MSG__DETAIL__SET_POINTS_VECTOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'set_points'
#include "deltarobot_interfaces/msg/detail/set_point__struct.h"

/// Struct defined in msg/SetPointsVector in the package deltarobot_interfaces.
typedef struct deltarobot_interfaces__msg__SetPointsVector
{
  deltarobot_interfaces__msg__SetPoint__Sequence set_points;
} deltarobot_interfaces__msg__SetPointsVector;

// Struct for a sequence of deltarobot_interfaces__msg__SetPointsVector.
typedef struct deltarobot_interfaces__msg__SetPointsVector__Sequence
{
  deltarobot_interfaces__msg__SetPointsVector * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} deltarobot_interfaces__msg__SetPointsVector__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DELTAROBOT_INTERFACES__MSG__DETAIL__SET_POINTS_VECTOR__STRUCT_H_
