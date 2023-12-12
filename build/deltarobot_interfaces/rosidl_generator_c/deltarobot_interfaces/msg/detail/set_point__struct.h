// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from deltarobot_interfaces:msg/SetPoint.idl
// generated code does not contain a copyright notice

#ifndef DELTAROBOT_INTERFACES__MSG__DETAIL__SET_POINT__STRUCT_H_
#define DELTAROBOT_INTERFACES__MSG__DETAIL__SET_POINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/SetPoint in the package deltarobot_interfaces.
typedef struct deltarobot_interfaces__msg__SetPoint
{
  float x;
  float z;
  float delta_t;
} deltarobot_interfaces__msg__SetPoint;

// Struct for a sequence of deltarobot_interfaces__msg__SetPoint.
typedef struct deltarobot_interfaces__msg__SetPoint__Sequence
{
  deltarobot_interfaces__msg__SetPoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} deltarobot_interfaces__msg__SetPoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DELTAROBOT_INTERFACES__MSG__DETAIL__SET_POINT__STRUCT_H_
