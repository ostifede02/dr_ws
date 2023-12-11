// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from deltarobot_interfaces:msg/SetPointsVector.idl
// generated code does not contain a copyright notice

#ifndef DELTAROBOT_INTERFACES__MSG__DETAIL__SET_POINTS_VECTOR__FUNCTIONS_H_
#define DELTAROBOT_INTERFACES__MSG__DETAIL__SET_POINTS_VECTOR__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "deltarobot_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "deltarobot_interfaces/msg/detail/set_points_vector__struct.h"

/// Initialize msg/SetPointsVector message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * deltarobot_interfaces__msg__SetPointsVector
 * )) before or use
 * deltarobot_interfaces__msg__SetPointsVector__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_deltarobot_interfaces
bool
deltarobot_interfaces__msg__SetPointsVector__init(deltarobot_interfaces__msg__SetPointsVector * msg);

/// Finalize msg/SetPointsVector message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_deltarobot_interfaces
void
deltarobot_interfaces__msg__SetPointsVector__fini(deltarobot_interfaces__msg__SetPointsVector * msg);

/// Create msg/SetPointsVector message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * deltarobot_interfaces__msg__SetPointsVector__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_deltarobot_interfaces
deltarobot_interfaces__msg__SetPointsVector *
deltarobot_interfaces__msg__SetPointsVector__create();

/// Destroy msg/SetPointsVector message.
/**
 * It calls
 * deltarobot_interfaces__msg__SetPointsVector__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_deltarobot_interfaces
void
deltarobot_interfaces__msg__SetPointsVector__destroy(deltarobot_interfaces__msg__SetPointsVector * msg);

/// Check for msg/SetPointsVector message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_deltarobot_interfaces
bool
deltarobot_interfaces__msg__SetPointsVector__are_equal(const deltarobot_interfaces__msg__SetPointsVector * lhs, const deltarobot_interfaces__msg__SetPointsVector * rhs);

/// Copy a msg/SetPointsVector message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_deltarobot_interfaces
bool
deltarobot_interfaces__msg__SetPointsVector__copy(
  const deltarobot_interfaces__msg__SetPointsVector * input,
  deltarobot_interfaces__msg__SetPointsVector * output);

/// Initialize array of msg/SetPointsVector messages.
/**
 * It allocates the memory for the number of elements and calls
 * deltarobot_interfaces__msg__SetPointsVector__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_deltarobot_interfaces
bool
deltarobot_interfaces__msg__SetPointsVector__Sequence__init(deltarobot_interfaces__msg__SetPointsVector__Sequence * array, size_t size);

/// Finalize array of msg/SetPointsVector messages.
/**
 * It calls
 * deltarobot_interfaces__msg__SetPointsVector__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_deltarobot_interfaces
void
deltarobot_interfaces__msg__SetPointsVector__Sequence__fini(deltarobot_interfaces__msg__SetPointsVector__Sequence * array);

/// Create array of msg/SetPointsVector messages.
/**
 * It allocates the memory for the array and calls
 * deltarobot_interfaces__msg__SetPointsVector__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_deltarobot_interfaces
deltarobot_interfaces__msg__SetPointsVector__Sequence *
deltarobot_interfaces__msg__SetPointsVector__Sequence__create(size_t size);

/// Destroy array of msg/SetPointsVector messages.
/**
 * It calls
 * deltarobot_interfaces__msg__SetPointsVector__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_deltarobot_interfaces
void
deltarobot_interfaces__msg__SetPointsVector__Sequence__destroy(deltarobot_interfaces__msg__SetPointsVector__Sequence * array);

/// Check for msg/SetPointsVector message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_deltarobot_interfaces
bool
deltarobot_interfaces__msg__SetPointsVector__Sequence__are_equal(const deltarobot_interfaces__msg__SetPointsVector__Sequence * lhs, const deltarobot_interfaces__msg__SetPointsVector__Sequence * rhs);

/// Copy an array of msg/SetPointsVector messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_deltarobot_interfaces
bool
deltarobot_interfaces__msg__SetPointsVector__Sequence__copy(
  const deltarobot_interfaces__msg__SetPointsVector__Sequence * input,
  deltarobot_interfaces__msg__SetPointsVector__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // DELTAROBOT_INTERFACES__MSG__DETAIL__SET_POINTS_VECTOR__FUNCTIONS_H_
