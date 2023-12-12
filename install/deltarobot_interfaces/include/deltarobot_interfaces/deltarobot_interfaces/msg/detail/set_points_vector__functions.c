// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from deltarobot_interfaces:msg/SetPointsVector.idl
// generated code does not contain a copyright notice
#include "deltarobot_interfaces/msg/detail/set_points_vector__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `set_points`
#include "deltarobot_interfaces/msg/detail/set_point__functions.h"

bool
deltarobot_interfaces__msg__SetPointsVector__init(deltarobot_interfaces__msg__SetPointsVector * msg)
{
  if (!msg) {
    return false;
  }
  // set_points
  if (!deltarobot_interfaces__msg__SetPoint__Sequence__init(&msg->set_points, 0)) {
    deltarobot_interfaces__msg__SetPointsVector__fini(msg);
    return false;
  }
  return true;
}

void
deltarobot_interfaces__msg__SetPointsVector__fini(deltarobot_interfaces__msg__SetPointsVector * msg)
{
  if (!msg) {
    return;
  }
  // set_points
  deltarobot_interfaces__msg__SetPoint__Sequence__fini(&msg->set_points);
}

bool
deltarobot_interfaces__msg__SetPointsVector__are_equal(const deltarobot_interfaces__msg__SetPointsVector * lhs, const deltarobot_interfaces__msg__SetPointsVector * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // set_points
  if (!deltarobot_interfaces__msg__SetPoint__Sequence__are_equal(
      &(lhs->set_points), &(rhs->set_points)))
  {
    return false;
  }
  return true;
}

bool
deltarobot_interfaces__msg__SetPointsVector__copy(
  const deltarobot_interfaces__msg__SetPointsVector * input,
  deltarobot_interfaces__msg__SetPointsVector * output)
{
  if (!input || !output) {
    return false;
  }
  // set_points
  if (!deltarobot_interfaces__msg__SetPoint__Sequence__copy(
      &(input->set_points), &(output->set_points)))
  {
    return false;
  }
  return true;
}

deltarobot_interfaces__msg__SetPointsVector *
deltarobot_interfaces__msg__SetPointsVector__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  deltarobot_interfaces__msg__SetPointsVector * msg = (deltarobot_interfaces__msg__SetPointsVector *)allocator.allocate(sizeof(deltarobot_interfaces__msg__SetPointsVector), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(deltarobot_interfaces__msg__SetPointsVector));
  bool success = deltarobot_interfaces__msg__SetPointsVector__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
deltarobot_interfaces__msg__SetPointsVector__destroy(deltarobot_interfaces__msg__SetPointsVector * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    deltarobot_interfaces__msg__SetPointsVector__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
deltarobot_interfaces__msg__SetPointsVector__Sequence__init(deltarobot_interfaces__msg__SetPointsVector__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  deltarobot_interfaces__msg__SetPointsVector * data = NULL;

  if (size) {
    data = (deltarobot_interfaces__msg__SetPointsVector *)allocator.zero_allocate(size, sizeof(deltarobot_interfaces__msg__SetPointsVector), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = deltarobot_interfaces__msg__SetPointsVector__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        deltarobot_interfaces__msg__SetPointsVector__fini(&data[i - 1]);
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
deltarobot_interfaces__msg__SetPointsVector__Sequence__fini(deltarobot_interfaces__msg__SetPointsVector__Sequence * array)
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
      deltarobot_interfaces__msg__SetPointsVector__fini(&array->data[i]);
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

deltarobot_interfaces__msg__SetPointsVector__Sequence *
deltarobot_interfaces__msg__SetPointsVector__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  deltarobot_interfaces__msg__SetPointsVector__Sequence * array = (deltarobot_interfaces__msg__SetPointsVector__Sequence *)allocator.allocate(sizeof(deltarobot_interfaces__msg__SetPointsVector__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = deltarobot_interfaces__msg__SetPointsVector__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
deltarobot_interfaces__msg__SetPointsVector__Sequence__destroy(deltarobot_interfaces__msg__SetPointsVector__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    deltarobot_interfaces__msg__SetPointsVector__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
deltarobot_interfaces__msg__SetPointsVector__Sequence__are_equal(const deltarobot_interfaces__msg__SetPointsVector__Sequence * lhs, const deltarobot_interfaces__msg__SetPointsVector__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!deltarobot_interfaces__msg__SetPointsVector__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
deltarobot_interfaces__msg__SetPointsVector__Sequence__copy(
  const deltarobot_interfaces__msg__SetPointsVector__Sequence * input,
  deltarobot_interfaces__msg__SetPointsVector__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(deltarobot_interfaces__msg__SetPointsVector);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    deltarobot_interfaces__msg__SetPointsVector * data =
      (deltarobot_interfaces__msg__SetPointsVector *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!deltarobot_interfaces__msg__SetPointsVector__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          deltarobot_interfaces__msg__SetPointsVector__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!deltarobot_interfaces__msg__SetPointsVector__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
