// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from livox_ros_driver2:msg/CustomPoint.idl
// generated code does not contain a copyright notice

#ifndef LIVOX_ROS_DRIVER2__MSG__DETAIL__CUSTOM_POINT__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define LIVOX_ROS_DRIVER2__MSG__DETAIL__CUSTOM_POINT__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "livox_ros_driver2/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "livox_ros_driver2/msg/detail/custom_point__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace livox_ros_driver2
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_livox_ros_driver2
cdr_serialize(
  const livox_ros_driver2::msg::CustomPoint & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_livox_ros_driver2
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  livox_ros_driver2::msg::CustomPoint & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_livox_ros_driver2
get_serialized_size(
  const livox_ros_driver2::msg::CustomPoint & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_livox_ros_driver2
max_serialized_size_CustomPoint(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_livox_ros_driver2
cdr_serialize_key(
  const livox_ros_driver2::msg::CustomPoint & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_livox_ros_driver2
get_serialized_size_key(
  const livox_ros_driver2::msg::CustomPoint & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_livox_ros_driver2
max_serialized_size_key_CustomPoint(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace livox_ros_driver2

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_livox_ros_driver2
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, livox_ros_driver2, msg, CustomPoint)();

#ifdef __cplusplus
}
#endif

#endif  // LIVOX_ROS_DRIVER2__MSG__DETAIL__CUSTOM_POINT__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
