// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from livox_ros_driver2:msg/CustomMsg.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "livox_ros_driver2/msg/custom_msg.h"


#ifndef LIVOX_ROS_DRIVER2__MSG__DETAIL__CUSTOM_MSG__STRUCT_H_
#define LIVOX_ROS_DRIVER2__MSG__DETAIL__CUSTOM_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'points'
#include "livox_ros_driver2/msg/detail/custom_point__struct.h"

/// Struct defined in msg/CustomMsg in the package livox_ros_driver2.
/**
  * Livox publish pointcloud msg format.
 */
typedef struct livox_ros_driver2__msg__CustomMsg
{
  /// ROS standard message header
  std_msgs__msg__Header header;
  /// The time of first point
  uint64_t timebase;
  /// Total number of pointclouds
  uint32_t point_num;
  /// Lidar device id number
  uint8_t lidar_id;
  /// Reserved use
  uint8_t rsvd[3];
  /// Pointcloud data
  livox_ros_driver2__msg__CustomPoint__Sequence points;
} livox_ros_driver2__msg__CustomMsg;

// Struct for a sequence of livox_ros_driver2__msg__CustomMsg.
typedef struct livox_ros_driver2__msg__CustomMsg__Sequence
{
  livox_ros_driver2__msg__CustomMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} livox_ros_driver2__msg__CustomMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LIVOX_ROS_DRIVER2__MSG__DETAIL__CUSTOM_MSG__STRUCT_H_
