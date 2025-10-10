// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from livox_ros_driver2:msg/CustomPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "livox_ros_driver2/msg/custom_point.hpp"


#ifndef LIVOX_ROS_DRIVER2__MSG__DETAIL__CUSTOM_POINT__BUILDER_HPP_
#define LIVOX_ROS_DRIVER2__MSG__DETAIL__CUSTOM_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "livox_ros_driver2/msg/detail/custom_point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace livox_ros_driver2
{

namespace msg
{

namespace builder
{

class Init_CustomPoint_line
{
public:
  explicit Init_CustomPoint_line(::livox_ros_driver2::msg::CustomPoint & msg)
  : msg_(msg)
  {}
  ::livox_ros_driver2::msg::CustomPoint line(::livox_ros_driver2::msg::CustomPoint::_line_type arg)
  {
    msg_.line = std::move(arg);
    return std::move(msg_);
  }

private:
  ::livox_ros_driver2::msg::CustomPoint msg_;
};

class Init_CustomPoint_tag
{
public:
  explicit Init_CustomPoint_tag(::livox_ros_driver2::msg::CustomPoint & msg)
  : msg_(msg)
  {}
  Init_CustomPoint_line tag(::livox_ros_driver2::msg::CustomPoint::_tag_type arg)
  {
    msg_.tag = std::move(arg);
    return Init_CustomPoint_line(msg_);
  }

private:
  ::livox_ros_driver2::msg::CustomPoint msg_;
};

class Init_CustomPoint_reflectivity
{
public:
  explicit Init_CustomPoint_reflectivity(::livox_ros_driver2::msg::CustomPoint & msg)
  : msg_(msg)
  {}
  Init_CustomPoint_tag reflectivity(::livox_ros_driver2::msg::CustomPoint::_reflectivity_type arg)
  {
    msg_.reflectivity = std::move(arg);
    return Init_CustomPoint_tag(msg_);
  }

private:
  ::livox_ros_driver2::msg::CustomPoint msg_;
};

class Init_CustomPoint_z
{
public:
  explicit Init_CustomPoint_z(::livox_ros_driver2::msg::CustomPoint & msg)
  : msg_(msg)
  {}
  Init_CustomPoint_reflectivity z(::livox_ros_driver2::msg::CustomPoint::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_CustomPoint_reflectivity(msg_);
  }

private:
  ::livox_ros_driver2::msg::CustomPoint msg_;
};

class Init_CustomPoint_y
{
public:
  explicit Init_CustomPoint_y(::livox_ros_driver2::msg::CustomPoint & msg)
  : msg_(msg)
  {}
  Init_CustomPoint_z y(::livox_ros_driver2::msg::CustomPoint::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_CustomPoint_z(msg_);
  }

private:
  ::livox_ros_driver2::msg::CustomPoint msg_;
};

class Init_CustomPoint_x
{
public:
  explicit Init_CustomPoint_x(::livox_ros_driver2::msg::CustomPoint & msg)
  : msg_(msg)
  {}
  Init_CustomPoint_y x(::livox_ros_driver2::msg::CustomPoint::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_CustomPoint_y(msg_);
  }

private:
  ::livox_ros_driver2::msg::CustomPoint msg_;
};

class Init_CustomPoint_offset_time
{
public:
  Init_CustomPoint_offset_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CustomPoint_x offset_time(::livox_ros_driver2::msg::CustomPoint::_offset_time_type arg)
  {
    msg_.offset_time = std::move(arg);
    return Init_CustomPoint_x(msg_);
  }

private:
  ::livox_ros_driver2::msg::CustomPoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::livox_ros_driver2::msg::CustomPoint>()
{
  return livox_ros_driver2::msg::builder::Init_CustomPoint_offset_time();
}

}  // namespace livox_ros_driver2

#endif  // LIVOX_ROS_DRIVER2__MSG__DETAIL__CUSTOM_POINT__BUILDER_HPP_
