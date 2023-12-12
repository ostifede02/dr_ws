// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from deltarobot_interfaces:msg/TrajectoryTask.idl
// generated code does not contain a copyright notice
#include "deltarobot_interfaces/msg/detail/trajectory_task__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `pos_start`
// Member `pos_end`
#include "geometry_msgs/msg/detail/point32__functions.h"
// Member `task_type`
#include "std_msgs/msg/detail/string__functions.h"

bool
deltarobot_interfaces__msg__TrajectoryTask__init(deltarobot_interfaces__msg__TrajectoryTask * msg)
{
  if (!msg) {
    return false;
  }
  // pos_start
  if (!geometry_msgs__msg__Point32__init(&msg->pos_start)) {
    deltarobot_interfaces__msg__TrajectoryTask__fini(msg);
    return false;
  }
  // pos_end
  if (!geometry_msgs__msg__Point32__init(&msg->pos_end)) {
    deltarobot_interfaces__msg__TrajectoryTask__fini(msg);
    return false;
  }
  // task_time
  // task_type
  if (!std_msgs__msg__String__init(&msg->task_type)) {
    deltarobot_interfaces__msg__TrajectoryTask__fini(msg);
    return false;
  }
  return true;
}

void
deltarobot_interfaces__msg__TrajectoryTask__fini(deltarobot_interfaces__msg__TrajectoryTask * msg)
{
  if (!msg) {
    return;
  }
  // pos_start
  geometry_msgs__msg__Point32__fini(&msg->pos_start);
  // pos_end
  geometry_msgs__msg__Point32__fini(&msg->pos_end);
  // task_time
  // task_type
  std_msgs__msg__String__fini(&msg->task_type);
}

bool
deltarobot_interfaces__msg__TrajectoryTask__are_equal(const deltarobot_interfaces__msg__TrajectoryTask * lhs, const deltarobot_interfaces__msg__TrajectoryTask * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pos_start
  if (!geometry_msgs__msg__Point32__are_equal(
      &(lhs->pos_start), &(rhs->pos_start)))
  {
    return false;
  }
  // pos_end
  if (!geometry_msgs__msg__Point32__are_equal(
      &(lhs->pos_end), &(rhs->pos_end)))
  {
    return false;
  }
  // task_time
  if (lhs->task_time != rhs->task_time) {
    return false;
  }
  // task_type
  if (!std_msgs__msg__String__are_equal(
      &(lhs->task_type), &(rhs->task_type)))
  {
    return false;
  }
  return true;
}

bool
deltarobot_interfaces__msg__TrajectoryTask__copy(
  const deltarobot_interfaces__msg__TrajectoryTask * input,
  deltarobot_interfaces__msg__TrajectoryTask * output)
{
  if (!input || !output) {
    return false;
  }
  // pos_start
  if (!geometry_msgs__msg__Point32__copy(
      &(input->pos_start), &(output->pos_start)))
  {
    return false;
  }
  // pos_end
  if (!geometry_msgs__msg__Point32__copy(
      &(input->pos_end), &(output->pos_end)))
  {
    return false;
  }
  // task_time
  output->task_time = input->task_time;
  // task_type
  if (!std_msgs__msg__String__copy(
      &(input->task_type), &(output->task_type)))
  {
    return false;
  }
  return true;
}

deltarobot_interfaces__msg__TrajectoryTask *
deltarobot_interfaces__msg__TrajectoryTask__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  deltarobot_interfaces__msg__TrajectoryTask * msg = (deltarobot_interfaces__msg__TrajectoryTask *)allocator.allocate(sizeof(deltarobot_interfaces__msg__TrajectoryTask), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(deltarobot_interfaces__msg__TrajectoryTask));
  bool success = deltarobot_interfaces__msg__TrajectoryTask__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
deltarobot_interfaces__msg__TrajectoryTask__destroy(deltarobot_interfaces__msg__TrajectoryTask * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    deltarobot_interfaces__msg__TrajectoryTask__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
deltarobot_interfaces__msg__TrajectoryTask__Sequence__init(deltarobot_interfaces__msg__TrajectoryTask__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  deltarobot_interfaces__msg__TrajectoryTask * data = NULL;

  if (size) {
    data = (deltarobot_interfaces__msg__TrajectoryTask *)allocator.zero_allocate(size, sizeof(deltarobot_interfaces__msg__TrajectoryTask), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = deltarobot_interfaces__msg__TrajectoryTask__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        deltarobot_interfaces__msg__TrajectoryTask__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
deltarobot_interfaces__msg__TrajectoryTask__Sequence__fini(deltarobot_interfaces__msg__TrajectoryTask__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      deltarobot_interfaces__msg__TrajectoryTask__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

deltarobot_interfaces__msg__TrajectoryTask__Sequence *
deltarobot_interfaces__msg__TrajectoryTask__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  deltarobot_interfaces__msg__TrajectoryTask__Sequence * array = (deltarobot_interfaces__msg__TrajectoryTask__Sequence *)allocator.allocate(sizeof(deltarobot_interfaces__msg__TrajectoryTask__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = deltarobot_interfaces__msg__TrajectoryTask__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
deltarobot_interfaces__msg__TrajectoryTask__Sequence__destroy(deltarobot_interfaces__msg__TrajectoryTask__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    deltarobot_interfaces__msg__TrajectoryTask__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
deltarobot_interfaces__msg__TrajectoryTask__Sequence__are_equal(const deltarobot_interfaces__msg__TrajectoryTask__Sequence * lhs, const deltarobot_interfaces__msg__TrajectoryTask__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!deltarobot_interfaces__msg__TrajectoryTask__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
deltarobot_interfaces__msg__TrajectoryTask__Sequence__copy(
  const deltarobot_interfaces__msg__TrajectoryTask__Sequence * input,
  deltarobot_interfaces__msg__TrajectoryTask__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(deltarobot_interfaces__msg__TrajectoryTask);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    deltarobot_interfaces__msg__TrajectoryTask * data =
      (deltarobot_interfaces__msg__TrajectoryTask *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!deltarobot_interfaces__msg__TrajectoryTask__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          deltarobot_interfaces__msg__TrajectoryTask__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!deltarobot_interfaces__msg__TrajectoryTask__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
