// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from deltarobot_interfaces:msg/SetPoint.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "deltarobot_interfaces/msg/detail/set_point__rosidl_typesupport_introspection_c.h"
#include "deltarobot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "deltarobot_interfaces/msg/detail/set_point__functions.h"
#include "deltarobot_interfaces/msg/detail/set_point__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void deltarobot_interfaces__msg__SetPoint__rosidl_typesupport_introspection_c__SetPoint_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  deltarobot_interfaces__msg__SetPoint__init(message_memory);
}

void deltarobot_interfaces__msg__SetPoint__rosidl_typesupport_introspection_c__SetPoint_fini_function(void * message_memory)
{
  deltarobot_interfaces__msg__SetPoint__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember deltarobot_interfaces__msg__SetPoint__rosidl_typesupport_introspection_c__SetPoint_message_member_array[3] = {
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(deltarobot_interfaces__msg__SetPoint, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(deltarobot_interfaces__msg__SetPoint, z),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "delta_t",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(deltarobot_interfaces__msg__SetPoint, delta_t),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers deltarobot_interfaces__msg__SetPoint__rosidl_typesupport_introspection_c__SetPoint_message_members = {
  "deltarobot_interfaces__msg",  // message namespace
  "SetPoint",  // message name
  3,  // number of fields
  sizeof(deltarobot_interfaces__msg__SetPoint),
  deltarobot_interfaces__msg__SetPoint__rosidl_typesupport_introspection_c__SetPoint_message_member_array,  // message members
  deltarobot_interfaces__msg__SetPoint__rosidl_typesupport_introspection_c__SetPoint_init_function,  // function to initialize message memory (memory has to be allocated)
  deltarobot_interfaces__msg__SetPoint__rosidl_typesupport_introspection_c__SetPoint_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t deltarobot_interfaces__msg__SetPoint__rosidl_typesupport_introspection_c__SetPoint_message_type_support_handle = {
  0,
  &deltarobot_interfaces__msg__SetPoint__rosidl_typesupport_introspection_c__SetPoint_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_deltarobot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, deltarobot_interfaces, msg, SetPoint)() {
  if (!deltarobot_interfaces__msg__SetPoint__rosidl_typesupport_introspection_c__SetPoint_message_type_support_handle.typesupport_identifier) {
    deltarobot_interfaces__msg__SetPoint__rosidl_typesupport_introspection_c__SetPoint_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &deltarobot_interfaces__msg__SetPoint__rosidl_typesupport_introspection_c__SetPoint_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
