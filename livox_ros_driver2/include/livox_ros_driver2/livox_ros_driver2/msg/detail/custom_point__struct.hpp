// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from livox_ros_driver2:msg/CustomPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "livox_ros_driver2/msg/custom_point.hpp"


#ifndef LIVOX_ROS_DRIVER2__MSG__DETAIL__CUSTOM_POINT__STRUCT_HPP_
#define LIVOX_ROS_DRIVER2__MSG__DETAIL__CUSTOM_POINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__livox_ros_driver2__msg__CustomPoint __attribute__((deprecated))
#else
# define DEPRECATED__livox_ros_driver2__msg__CustomPoint __declspec(deprecated)
#endif

namespace livox_ros_driver2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CustomPoint_
{
  using Type = CustomPoint_<ContainerAllocator>;

  explicit CustomPoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->offset_time = 0ul;
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->reflectivity = 0;
      this->tag = 0;
      this->line = 0;
    }
  }

  explicit CustomPoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->offset_time = 0ul;
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->reflectivity = 0;
      this->tag = 0;
      this->line = 0;
    }
  }

  // field types and members
  using _offset_time_type =
    uint32_t;
  _offset_time_type offset_time;
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _z_type =
    float;
  _z_type z;
  using _reflectivity_type =
    uint8_t;
  _reflectivity_type reflectivity;
  using _tag_type =
    uint8_t;
  _tag_type tag;
  using _line_type =
    uint8_t;
  _line_type line;

  // setters for named parameter idiom
  Type & set__offset_time(
    const uint32_t & _arg)
  {
    this->offset_time = _arg;
    return *this;
  }
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const float & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__reflectivity(
    const uint8_t & _arg)
  {
    this->reflectivity = _arg;
    return *this;
  }
  Type & set__tag(
    const uint8_t & _arg)
  {
    this->tag = _arg;
    return *this;
  }
  Type & set__line(
    const uint8_t & _arg)
  {
    this->line = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    livox_ros_driver2::msg::CustomPoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const livox_ros_driver2::msg::CustomPoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<livox_ros_driver2::msg::CustomPoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<livox_ros_driver2::msg::CustomPoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      livox_ros_driver2::msg::CustomPoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<livox_ros_driver2::msg::CustomPoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      livox_ros_driver2::msg::CustomPoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<livox_ros_driver2::msg::CustomPoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<livox_ros_driver2::msg::CustomPoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<livox_ros_driver2::msg::CustomPoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__livox_ros_driver2__msg__CustomPoint
    std::shared_ptr<livox_ros_driver2::msg::CustomPoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__livox_ros_driver2__msg__CustomPoint
    std::shared_ptr<livox_ros_driver2::msg::CustomPoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CustomPoint_ & other) const
  {
    if (this->offset_time != other.offset_time) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->reflectivity != other.reflectivity) {
      return false;
    }
    if (this->tag != other.tag) {
      return false;
    }
    if (this->line != other.line) {
      return false;
    }
    return true;
  }
  bool operator!=(const CustomPoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CustomPoint_

// alias to use template instance with default allocator
using CustomPoint =
  livox_ros_driver2::msg::CustomPoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace livox_ros_driver2

#endif  // LIVOX_ROS_DRIVER2__MSG__DETAIL__CUSTOM_POINT__STRUCT_HPP_
