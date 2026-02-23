// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from deadwheel_msgs:msg/DeadwheelTicks.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "deadwheel_msgs/msg/deadwheel_ticks.hpp"


#ifndef DEADWHEEL_MSGS__MSG__DETAIL__DEADWHEEL_TICKS__STRUCT_HPP_
#define DEADWHEEL_MSGS__MSG__DETAIL__DEADWHEEL_TICKS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__deadwheel_msgs__msg__DeadwheelTicks __attribute__((deprecated))
#else
# define DEPRECATED__deadwheel_msgs__msg__DeadwheelTicks __declspec(deprecated)
#endif

namespace deadwheel_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DeadwheelTicks_
{
  using Type = DeadwheelTicks_<ContainerAllocator>;

  explicit DeadwheelTicks_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->t0 = 0l;
      this->t1 = 0l;
      this->t2 = 0l;
    }
  }

  explicit DeadwheelTicks_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->t0 = 0l;
      this->t1 = 0l;
      this->t2 = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _t0_type =
    int32_t;
  _t0_type t0;
  using _t1_type =
    int32_t;
  _t1_type t1;
  using _t2_type =
    int32_t;
  _t2_type t2;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__t0(
    const int32_t & _arg)
  {
    this->t0 = _arg;
    return *this;
  }
  Type & set__t1(
    const int32_t & _arg)
  {
    this->t1 = _arg;
    return *this;
  }
  Type & set__t2(
    const int32_t & _arg)
  {
    this->t2 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    deadwheel_msgs::msg::DeadwheelTicks_<ContainerAllocator> *;
  using ConstRawPtr =
    const deadwheel_msgs::msg::DeadwheelTicks_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<deadwheel_msgs::msg::DeadwheelTicks_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<deadwheel_msgs::msg::DeadwheelTicks_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      deadwheel_msgs::msg::DeadwheelTicks_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<deadwheel_msgs::msg::DeadwheelTicks_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      deadwheel_msgs::msg::DeadwheelTicks_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<deadwheel_msgs::msg::DeadwheelTicks_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<deadwheel_msgs::msg::DeadwheelTicks_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<deadwheel_msgs::msg::DeadwheelTicks_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__deadwheel_msgs__msg__DeadwheelTicks
    std::shared_ptr<deadwheel_msgs::msg::DeadwheelTicks_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__deadwheel_msgs__msg__DeadwheelTicks
    std::shared_ptr<deadwheel_msgs::msg::DeadwheelTicks_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DeadwheelTicks_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->t0 != other.t0) {
      return false;
    }
    if (this->t1 != other.t1) {
      return false;
    }
    if (this->t2 != other.t2) {
      return false;
    }
    return true;
  }
  bool operator!=(const DeadwheelTicks_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DeadwheelTicks_

// alias to use template instance with default allocator
using DeadwheelTicks =
  deadwheel_msgs::msg::DeadwheelTicks_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace deadwheel_msgs

#endif  // DEADWHEEL_MSGS__MSG__DETAIL__DEADWHEEL_TICKS__STRUCT_HPP_
