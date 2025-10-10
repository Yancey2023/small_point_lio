// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from livox_ros_driver2:msg/CustomMsg.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "livox_ros_driver2/msg/custom_msg.hpp"


#ifndef LIVOX_ROS_DRIVER2__MSG__DETAIL__CUSTOM_MSG__BUILDER_HPP_
#define LIVOX_ROS_DRIVER2__MSG__DETAIL__CUSTOM_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "livox_ros_driver2/msg/detail/custom_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace livox_ros_driver2
{

namespace msg
{

namespace builder
{

class Init_CustomMsg_points
{
public:
  explicit Init_CustomMsg_points(::livox_ros_driver2::msg::CustomMsg & msg)
  : msg_(msg)
  {}
  ::livox_ros_driver2::msg::CustomMsg points(::livox_ros_driver2::msg::CustomMsg::_points_type arg)
  {
    msg_.points = std::move(arg);
    return std::move(msg_);
  }

private:
  ::livox_ros_driver2::msg::CustomMsg msg_;
};

class Init_CustomMsg_rsvd
{
public:
  explicit Init_CustomMsg_rsvd(::livox_ros_driver2::msg::CustomMsg & msg)
  : msg_(msg)
  {}
  Init_CustomMsg_points rsvd(::livox_ros_driver2::msg::CustomMsg::_rsvd_type arg)
  {
    msg_.rsvd = std::move(arg);
    return Init_CustomMsg_points(msg_);
  }

private:
  ::livox_ros_driver2::msg::CustomMsg msg_;
};

class Init_CustomMsg_lidar_id
{
public:
  explicit Init_CustomMsg_lidar_id(::livox_ros_driver2::msg::CustomMsg & msg)
  : msg_(msg)
  {}
  Init_CustomMsg_rsvd lidar_id(::livox_ros_driver2::msg::CustomMsg::_lidar_id_type arg)
  {
    msg_.lidar_id = std::move(arg);
    return Init_CustomMsg_rsvd(msg_);
  }

private:
  ::livox_ros_driver2::msg::CustomMsg msg_;
};

class Init_CustomMsg_point_num
{
public:
  explicit Init_CustomMsg_point_num(::livox_ros_driver2::msg::CustomMsg & msg)
  : msg_(msg)
  {}
  Init_CustomMsg_lidar_id point_num(::livox_ros_driver2::msg::CustomMsg::_point_num_type arg)
  {
    msg_.point_num = std::move(arg);
    return Init_CustomMsg_lidar_id(msg_);
  }

private:
  ::livox_ros_driver2::msg::CustomMsg msg_;
};

class Init_CustomMsg_timebase
{
public:
  explicit Init_CustomMsg_timebase(::livox_ros_driver2::msg::CustomMsg & msg)
  : msg_(msg)
  {}
  Init_CustomMsg_point_num timebase(::livox_ros_driver2::msg::CustomMsg::_timebase_type arg)
  {
    msg_.timebase = std::move(arg);
    return Init_CustomMsg_point_num(msg_);
  }

private:
  ::livox_ros_driver2::msg::CustomMsg msg_;
};

class Init_CustomMsg_header
{
public:
  Init_CustomMsg_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CustomMsg_timebase header(::livox_ros_driver2::msg::CustomMsg::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_CustomMsg_timebase(msg_);
  }

private:
  ::livox_ros_driver2::msg::CustomMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::livox_ros_driver2::msg::CustomMsg>()
{
  return livox_ros_driver2::msg::builder::Init_CustomMsg_header();
}

}  // namespace livox_ros_driver2

#endif  // LIVOX_ROS_DRIVER2__MSG__DETAIL__CUSTOM_MSG__BUILDER_HPP_
