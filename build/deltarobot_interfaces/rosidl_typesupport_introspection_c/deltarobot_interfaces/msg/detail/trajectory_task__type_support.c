// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from deltarobot_interfaces:msg/TrajectoryTask.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "deltarobot_interfaces/msg/detail/trajectory_task__rosidl_typesupport_introspection_c.h"
#include "deltarobot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "deltarobot_interfaces/msg/detail/trajectory_task__functions.h"
#include "deltarobot_interfaces/msg/detail/trajectory_task__struct.h"


// Include directives for member types
// Member `pos_start`
// Member `pos_end`
#include "geometry_msgs/msg/point32.h"
// Member `pos_start`
// Member `pos_end`
#include "geometry_msgs/msg/detail/point32__rosidl_typesupport_introspection_c.h"
// Member `task_type`
#include "std_msgs/msg/string.h"
// Member `task_type`
#include "std_msgs/msg/detail/string__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void deltarobot_interfaces__msg__TrajectoryTask__rosidl_typesupport_introspection_c__TrajectoryTask_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  deltarobot_interfaces__msg__TrajectoryTask__init(message_memory);
}

void deltarobot_interfaces__msg__TrajectoryTask__rosidl_typesupport_introspection_c__TrajectoryTask_fini_function(void * message_memory)
{
  deltarobot_interfaces__msg__TrajectoryTask__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember deltarobot_interfaces__msg__TrajectoryTask__rosidl_typesupport_introspection_c__TrajectoryTask_message_member_array[4] = {
  {
    "pos_start",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(deltarobot_interfaces__msg__TrajectoryTask, pos_start),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pos_end",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(deltarobot_interfaces__msg__TrajectoryTask, pos_end),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "task_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(deltarobot_interfaces__msg__TrajectoryTask, task_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "task_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(deltarobot_interfaces__msg__TrajectoryTask, task_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers deltarobot_interfaces__msg__TrajectoryTask__rosidl_typesupport_introspection_c__TrajectoryTask_message_members = {
  "deltarobot_interfaces__msg",  // message namespace
  "TrajectoryTask",  // message name
  4,  // number of fields
  sizeof(deltarobot_interfaces__msg__TrajectoryTask),
  deltarobot_interfaces__msg__TrajectoryTask__rosidl_typesupport_introspection_c__TrajectoryTask_message_member_array,  // message members
  deltarobot_interfaces__msg__TrajectoryTask__rosidl_typesupport_introspection_c__TrajectoryTask_init_function,  // function to initialize message memory (memory has to be allocated)
  deltarobot_interfaces__msg__TrajectoryTask__rosidl_typesupport_introspection_c__TrajectoryTask_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t deltarobot_interfaces__msg__TrajectoryTask__rosidl_typesupport_introspection_c__TrajectoryTask_message_type_support_handle = {
  0,
  &deltarobot_interfaces__msg__TrajectoryTask__rosidl_typesupport_introspection_c__TrajectoryTask_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_deltarobot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, deltarobot_interfaces, msg, TrajectoryTask)() {
  deltarobot_interfaces__msg__TrajectoryTask__rosidl_typesupport_introspection_c__TrajectoryTask_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point32)();
  deltarobot_interfaces__msg__TrajectoryTask__rosidl_typesupport_introspection_c__TrajectoryTask_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point32)();
  deltarobot_interfaces__msg__TrajectoryTask__rosidl_typesupport_introspection_c__TrajectoryTask_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, String)();
  if (!deltarobot_interfaces__msg__TrajectoryTask__rosidl_typesupport_introspection_c__TrajectoryTask_message_type_support_handle.typesupport_identifier) {
    deltarobot_interfaces__msg__TrajectoryTask__rosidl_typesupport_introspection_c__TrajectoryTask_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &deltarobot_interfaces__msg__TrajectoryTask__rosidl_typesupport_introspection_c__TrajectoryTask_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
