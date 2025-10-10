// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from livox_ros_driver2:msg/CustomMsg.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "livox_ros_driver2/msg/custom_msg.h"


#ifndef LIVOX_ROS_DRIVER2__MSG__DETAIL__CUSTOM_MSG__FUNCTIONS_H_
#define LIVOX_ROS_DRIVER2__MSG__DETAIL__CUSTOM_MSG__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "livox_ros_driver2/msg/rosidl_generator_c__visibility_control.h"

#include "livox_ros_driver2/msg/detail/custom_msg__struct.h"

/// Initialize msg/CustomMsg message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * livox_ros_driver2__msg__CustomMsg
 * )) before or use
 * livox_ros_driver2__msg__CustomMsg__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_livox_ros_driver2
bool
livox_ros_driver2__msg__CustomMsg__init(livox_ros_driver2__msg__CustomMsg * msg);

/// Finalize msg/CustomMsg message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_livox_ros_driver2
void
livox_ros_driver2__msg__CustomMsg__fini(livox_ros_driver2__msg__CustomMsg * msg);

/// Create msg/CustomMsg message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * livox_ros_driver2__msg__CustomMsg__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_livox_ros_driver2
livox_ros_driver2__msg__CustomMsg *
livox_ros_driver2__msg__CustomMsg__create(void);

/// Destroy msg/CustomMsg message.
/**
 * It calls
 * livox_ros_driver2__msg__CustomMsg__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_livox_ros_driver2
void
livox_ros_driver2__msg__CustomMsg__destroy(livox_ros_driver2__msg__CustomMsg * msg);

/// Check for msg/CustomMsg message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_livox_ros_driver2
bool
livox_ros_driver2__msg__CustomMsg__are_equal(const livox_ros_driver2__msg__CustomMsg * lhs, const livox_ros_driver2__msg__CustomMsg * rhs);

/// Copy a msg/CustomMsg message.
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
ROSIDL_GENERATOR_C_PUBLIC_livox_ros_driver2
bool
livox_ros_driver2__msg__CustomMsg__copy(
  const livox_ros_driver2__msg__CustomMsg * input,
  livox_ros_driver2__msg__CustomMsg * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_livox_ros_driver2
const rosidl_type_hash_t *
livox_ros_driver2__msg__CustomMsg__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_livox_ros_driver2
const rosidl_runtime_c__type_description__TypeDescription *
livox_ros_driver2__msg__CustomMsg__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_livox_ros_driver2
const rosidl_runtime_c__type_description__TypeSource *
livox_ros_driver2__msg__CustomMsg__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_livox_ros_driver2
const rosidl_runtime_c__type_description__TypeSource__Sequence *
livox_ros_driver2__msg__CustomMsg__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/CustomMsg messages.
/**
 * It allocates the memory for the number of elements and calls
 * livox_ros_driver2__msg__CustomMsg__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_livox_ros_driver2
bool
livox_ros_driver2__msg__CustomMsg__Sequence__init(livox_ros_driver2__msg__CustomMsg__Sequence * array, size_t size);

/// Finalize array of msg/CustomMsg messages.
/**
 * It calls
 * livox_ros_driver2__msg__CustomMsg__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_livox_ros_driver2
void
livox_ros_driver2__msg__CustomMsg__Sequence__fini(livox_ros_driver2__msg__CustomMsg__Sequence * array);

/// Create array of msg/CustomMsg messages.
/**
 * It allocates the memory for the array and calls
 * livox_ros_driver2__msg__CustomMsg__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_livox_ros_driver2
livox_ros_driver2__msg__CustomMsg__Sequence *
livox_ros_driver2__msg__CustomMsg__Sequence__create(size_t size);

/// Destroy array of msg/CustomMsg messages.
/**
 * It calls
 * livox_ros_driver2__msg__CustomMsg__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_livox_ros_driver2
void
livox_ros_driver2__msg__CustomMsg__Sequence__destroy(livox_ros_driver2__msg__CustomMsg__Sequence * array);

/// Check for msg/CustomMsg message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_livox_ros_driver2
bool
livox_ros_driver2__msg__CustomMsg__Sequence__are_equal(const livox_ros_driver2__msg__CustomMsg__Sequence * lhs, const livox_ros_driver2__msg__CustomMsg__Sequence * rhs);

/// Copy an array of msg/CustomMsg messages.
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
ROSIDL_GENERATOR_C_PUBLIC_livox_ros_driver2
bool
livox_ros_driver2__msg__CustomMsg__Sequence__copy(
  const livox_ros_driver2__msg__CustomMsg__Sequence * input,
  livox_ros_driver2__msg__CustomMsg__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // LIVOX_ROS_DRIVER2__MSG__DETAIL__CUSTOM_MSG__FUNCTIONS_H_
