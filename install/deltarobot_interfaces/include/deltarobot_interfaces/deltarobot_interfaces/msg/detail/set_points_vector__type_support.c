// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from deltarobot_interfaces:msg/SetPointsVector.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "deltarobot_interfaces/msg/detail/set_points_vector__rosidl_typesupport_introspection_c.h"
#include "deltarobot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "deltarobot_interfaces/msg/detail/set_points_vector__functions.h"
#include "deltarobot_interfaces/msg/detail/set_points_vector__struct.h"


// Include directives for member types
// Member `set_points`
#include "deltarobot_interfaces/msg/set_point.h"
// Member `set_points`
#include "deltarobot_interfaces/msg/detail/set_point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void deltarobot_interfaces__msg__SetPointsVector__rosidl_typesupport_introspection_c__SetPointsVector_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  deltarobot_interfaces__msg__SetPointsVector__init(message_memory);
}

void deltarobot_interfaces__msg__SetPointsVector__rosidl_typesupport_introspection_c__SetPointsVector_fini_function(void * message_memory)
{
  deltarobot_interfaces__msg__SetPointsVector__fini(message_memory);
}

size_t deltarobot_interfaces__msg__SetPointsVector__rosidl_typesupport_introspection_c__size_function__SetPointsVector__set_points(
  const void * untyped_member)
{
  const deltarobot_interfaces__msg__SetPoint__Sequence * member =
    (const deltarobot_interfaces__msg__SetPoint__Sequence *)(untyped_member);
  return member->size;
}

const void * deltarobot_interfaces__msg__SetPointsVector__rosidl_typesupport_introspection_c__get_const_function__SetPointsVector__set_points(
  const void * untyped_member, size_t index)
{
  const deltarobot_interfaces__msg__SetPoint__Sequence * member =
    (const deltarobot_interfaces__msg__SetPoint__Sequence *)(untyped_member);
  return &member->data[index];
}

void * deltarobot_interfaces__msg__SetPointsVector__rosidl_typesupport_introspection_c__get_function__SetPointsVector__set_points(
  void * untyped_member, size_t index)
{
  deltarobot_interfaces__msg__SetPoint__Sequence * member =
    (deltarobot_interfaces__msg__SetPoint__Sequence *)(untyped_member);
  return &member->data[index];
}

void deltarobot_interfaces__msg__SetPointsVector__rosidl_typesupport_introspection_c__fetch_function__SetPointsVector__set_points(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const deltarobot_interfaces__msg__SetPoint * item =
    ((const deltarobot_interfaces__msg__SetPoint *)
    deltarobot_interfaces__msg__SetPointsVector__rosidl_typesupport_introspection_c__get_const_function__SetPointsVector__set_points(untyped_member, index));
  deltarobot_interfaces__msg__SetPoint * value =
    (deltarobot_interfaces__msg__SetPoint *)(untyped_value);
  *value = *item;
}

void deltarobot_interfaces__msg__SetPointsVector__rosidl_typesupport_introspection_c__assign_function__SetPointsVector__set_points(
  void * untyped_member, size_t index, const void * untyped_value)
{
  deltarobot_interfaces__msg__SetPoint * item =
    ((deltarobot_interfaces__msg__SetPoint *)
    deltarobot_interfaces__msg__SetPointsVector__rosidl_typesupport_introspection_c__get_function__SetPointsVector__set_points(untyped_member, index));
  const deltarobot_interfaces__msg__SetPoint * value =
    (const deltarobot_interfaces__msg__SetPoint *)(untyped_value);
  *item = *value;
}

bool deltarobot_interfaces__msg__SetPointsVector__rosidl_typesupport_introspection_c__resize_function__SetPointsVector__set_points(
  void * untyped_member, size_t size)
{
  deltarobot_interfaces__msg__SetPoint__Sequence * member =
    (deltarobot_interfaces__msg__SetPoint__Sequence *)(untyped_member);
  deltarobot_interfaces__msg__SetPoint__Sequence__fini(member);
  return deltarobot_interfaces__msg__SetPoint__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember deltarobot_interfaces__msg__SetPointsVector__rosidl_typesupport_introspection_c__SetPointsVector_message_member_array[1] = {
  {
    "set_points",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(deltarobot_interfaces__msg__SetPointsVector, set_points),  // bytes offset in struct
    NULL,  // default value
    deltarobot_interfaces__msg__SetPointsVector__rosidl_typesupport_introspection_c__size_function__SetPointsVector__set_points,  // size() function pointer
    deltarobot_interfaces__msg__SetPointsVector__rosidl_typesupport_introspection_c__get_const_function__SetPointsVector__set_points,  // get_const(index) function pointer
    deltarobot_interfaces__msg__SetPointsVector__rosidl_typesupport_introspection_c__get_function__SetPointsVector__set_points,  // get(index) function pointer
    deltarobot_interfaces__msg__SetPointsVector__rosidl_typesupport_introspection_c__fetch_function__SetPointsVector__set_points,  // fetch(index, &value) function pointer
    deltarobot_interfaces__msg__SetPointsVector__rosidl_typesupport_introspection_c__assign_function__SetPointsVector__set_points,  // assign(index, value) function pointer
    deltarobot_interfaces__msg__SetPointsVector__rosidl_typesupport_introspection_c__resize_function__SetPointsVector__set_points  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers deltarobot_interfaces__msg__SetPointsVector__rosidl_typesupport_introspection_c__SetPointsVector_message_members = {
  "deltarobot_interfaces__msg",  // message namespace
  "SetPointsVector",  // message name
  1,  // number of fields
  sizeof(deltarobot_interfaces__msg__SetPointsVector),
  deltarobot_interfaces__msg__SetPointsVector__rosidl_typesupport_introspection_c__SetPointsVector_message_member_array,  // message members
  deltarobot_interfaces__msg__SetPointsVector__rosidl_typesupport_introspection_c__SetPointsVector_init_function,  // function to initialize message memory (memory has to be allocated)
  deltarobot_interfaces__msg__SetPointsVector__rosidl_typesupport_introspection_c__SetPointsVector_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t deltarobot_interfaces__msg__SetPointsVector__rosidl_typesupport_introspection_c__SetPointsVector_message_type_support_handle = {
  0,
  &deltarobot_interfaces__msg__SetPointsVector__rosidl_typesupport_introspection_c__SetPointsVector_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_deltarobot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, deltarobot_interfaces, msg, SetPointsVector)() {
  deltarobot_interfaces__msg__SetPointsVector__rosidl_typesupport_introspection_c__SetPointsVector_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, deltarobot_interfaces, msg, SetPoint)();
  if (!deltarobot_interfaces__msg__SetPointsVector__rosidl_typesupport_introspection_c__SetPointsVector_message_type_support_handle.typesupport_identifier) {
    deltarobot_interfaces__msg__SetPointsVector__rosidl_typesupport_introspection_c__SetPointsVector_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &deltarobot_interfaces__msg__SetPointsVector__rosidl_typesupport_introspection_c__SetPointsVector_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
