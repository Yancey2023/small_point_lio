// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from livox_ros_driver2:msg/CustomMsg.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "livox_ros_driver2/msg/detail/custom_msg__functions.h"
#include "livox_ros_driver2/msg/detail/custom_msg__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace livox_ros_driver2
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void CustomMsg_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) livox_ros_driver2::msg::CustomMsg(_init);
}

void CustomMsg_fini_function(void * message_memory)
{
  auto typed_message = static_cast<livox_ros_driver2::msg::CustomMsg *>(message_memory);
  typed_message->~CustomMsg();
}

size_t size_function__CustomMsg__rsvd(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__CustomMsg__rsvd(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint8_t, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__CustomMsg__rsvd(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint8_t, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__CustomMsg__rsvd(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint8_t *>(
    get_const_function__CustomMsg__rsvd(untyped_member, index));
  auto & value = *reinterpret_cast<uint8_t *>(untyped_value);
  value = item;
}

void assign_function__CustomMsg__rsvd(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint8_t *>(
    get_function__CustomMsg__rsvd(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint8_t *>(untyped_value);
  item = value;
}

size_t size_function__CustomMsg__points(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<livox_ros_driver2::msg::CustomPoint> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CustomMsg__points(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<livox_ros_driver2::msg::CustomPoint> *>(untyped_member);
  return &member[index];
}

void * get_function__CustomMsg__points(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<livox_ros_driver2::msg::CustomPoint> *>(untyped_member);
  return &member[index];
}

void fetch_function__CustomMsg__points(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const livox_ros_driver2::msg::CustomPoint *>(
    get_const_function__CustomMsg__points(untyped_member, index));
  auto & value = *reinterpret_cast<livox_ros_driver2::msg::CustomPoint *>(untyped_value);
  value = item;
}

void assign_function__CustomMsg__points(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<livox_ros_driver2::msg::CustomPoint *>(
    get_function__CustomMsg__points(untyped_member, index));
  const auto & value = *reinterpret_cast<const livox_ros_driver2::msg::CustomPoint *>(untyped_value);
  item = value;
}

void resize_function__CustomMsg__points(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<livox_ros_driver2::msg::CustomPoint> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CustomMsg_message_member_array[6] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(livox_ros_driver2::msg::CustomMsg, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "timebase",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(livox_ros_driver2::msg::CustomMsg, timebase),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "point_num",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(livox_ros_driver2::msg::CustomMsg, point_num),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "lidar_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(livox_ros_driver2::msg::CustomMsg, lidar_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "rsvd",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(livox_ros_driver2::msg::CustomMsg, rsvd),  // bytes offset in struct
    nullptr,  // default value
    size_function__CustomMsg__rsvd,  // size() function pointer
    get_const_function__CustomMsg__rsvd,  // get_const(index) function pointer
    get_function__CustomMsg__rsvd,  // get(index) function pointer
    fetch_function__CustomMsg__rsvd,  // fetch(index, &value) function pointer
    assign_function__CustomMsg__rsvd,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "points",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<livox_ros_driver2::msg::CustomPoint>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(livox_ros_driver2::msg::CustomMsg, points),  // bytes offset in struct
    nullptr,  // default value
    size_function__CustomMsg__points,  // size() function pointer
    get_const_function__CustomMsg__points,  // get_const(index) function pointer
    get_function__CustomMsg__points,  // get(index) function pointer
    fetch_function__CustomMsg__points,  // fetch(index, &value) function pointer
    assign_function__CustomMsg__points,  // assign(index, value) function pointer
    resize_function__CustomMsg__points  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CustomMsg_message_members = {
  "livox_ros_driver2::msg",  // message namespace
  "CustomMsg",  // message name
  6,  // number of fields
  sizeof(livox_ros_driver2::msg::CustomMsg),
  false,  // has_any_key_member_
  CustomMsg_message_member_array,  // message members
  CustomMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  CustomMsg_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CustomMsg_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CustomMsg_message_members,
  get_message_typesupport_handle_function,
  &livox_ros_driver2__msg__CustomMsg__get_type_hash,
  &livox_ros_driver2__msg__CustomMsg__get_type_description,
  &livox_ros_driver2__msg__CustomMsg__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace livox_ros_driver2


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<livox_ros_driver2::msg::CustomMsg>()
{
  return &::livox_ros_driver2::msg::rosidl_typesupport_introspection_cpp::CustomMsg_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, livox_ros_driver2, msg, CustomMsg)() {
  return &::livox_ros_driver2::msg::rosidl_typesupport_introspection_cpp::CustomMsg_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
