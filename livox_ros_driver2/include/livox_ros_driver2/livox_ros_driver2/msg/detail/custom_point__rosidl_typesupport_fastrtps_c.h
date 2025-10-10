// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from livox_ros_driver2:msg/CustomPoint.idl
// generated code does not contain a copyright notice
#ifndef LIVOX_ROS_DRIVER2__MSG__DETAIL__CUSTOM_POINT__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define LIVOX_ROS_DRIVER2__MSG__DETAIL__CUSTOM_POINT__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "livox_ros_driver2/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "livox_ros_driver2/msg/detail/custom_point__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_livox_ros_driver2
bool cdr_serialize_livox_ros_driver2__msg__CustomPoint(
  const livox_ros_driver2__msg__CustomPoint * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_livox_ros_driver2
bool cdr_deserialize_livox_ros_driver2__msg__CustomPoint(
  eprosima::fastcdr::Cdr &,
  livox_ros_driver2__msg__CustomPoint * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_livox_ros_driver2
size_t get_serialized_size_livox_ros_driver2__msg__CustomPoint(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_livox_ros_driver2
size_t max_serialized_size_livox_ros_driver2__msg__CustomPoint(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_livox_ros_driver2
bool cdr_serialize_key_livox_ros_driver2__msg__CustomPoint(
  const livox_ros_driver2__msg__CustomPoint * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_livox_ros_driver2
size_t get_serialized_size_key_livox_ros_driver2__msg__CustomPoint(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_livox_ros_driver2
size_t max_serialized_size_key_livox_ros_driver2__msg__CustomPoint(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_livox_ros_driver2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, livox_ros_driver2, msg, CustomPoint)();

#ifdef __cplusplus
}
#endif

#endif  // LIVOX_ROS_DRIVER2__MSG__DETAIL__CUSTOM_POINT__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
