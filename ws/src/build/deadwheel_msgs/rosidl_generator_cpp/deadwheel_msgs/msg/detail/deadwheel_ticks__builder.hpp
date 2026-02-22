// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from deadwheel_msgs:msg/DeadwheelTicks.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "deadwheel_msgs/msg/deadwheel_ticks.hpp"


#ifndef DEADWHEEL_MSGS__MSG__DETAIL__DEADWHEEL_TICKS__BUILDER_HPP_
#define DEADWHEEL_MSGS__MSG__DETAIL__DEADWHEEL_TICKS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "deadwheel_msgs/msg/detail/deadwheel_ticks__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace deadwheel_msgs
{

namespace msg
{

namespace builder
{

class Init_DeadwheelTicks_t2
{
public:
  explicit Init_DeadwheelTicks_t2(::deadwheel_msgs::msg::DeadwheelTicks & msg)
  : msg_(msg)
  {}
  ::deadwheel_msgs::msg::DeadwheelTicks t2(::deadwheel_msgs::msg::DeadwheelTicks::_t2_type arg)
  {
    msg_.t2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::deadwheel_msgs::msg::DeadwheelTicks msg_;
};

class Init_DeadwheelTicks_t1
{
public:
  explicit Init_DeadwheelTicks_t1(::deadwheel_msgs::msg::DeadwheelTicks & msg)
  : msg_(msg)
  {}
  Init_DeadwheelTicks_t2 t1(::deadwheel_msgs::msg::DeadwheelTicks::_t1_type arg)
  {
    msg_.t1 = std::move(arg);
    return Init_DeadwheelTicks_t2(msg_);
  }

private:
  ::deadwheel_msgs::msg::DeadwheelTicks msg_;
};

class Init_DeadwheelTicks_t0
{
public:
  explicit Init_DeadwheelTicks_t0(::deadwheel_msgs::msg::DeadwheelTicks & msg)
  : msg_(msg)
  {}
  Init_DeadwheelTicks_t1 t0(::deadwheel_msgs::msg::DeadwheelTicks::_t0_type arg)
  {
    msg_.t0 = std::move(arg);
    return Init_DeadwheelTicks_t1(msg_);
  }

private:
  ::deadwheel_msgs::msg::DeadwheelTicks msg_;
};

class Init_DeadwheelTicks_header
{
public:
  Init_DeadwheelTicks_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DeadwheelTicks_t0 header(::deadwheel_msgs::msg::DeadwheelTicks::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DeadwheelTicks_t0(msg_);
  }

private:
  ::deadwheel_msgs::msg::DeadwheelTicks msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::deadwheel_msgs::msg::DeadwheelTicks>()
{
  return deadwheel_msgs::msg::builder::Init_DeadwheelTicks_header();
}

}  // namespace deadwheel_msgs

#endif  // DEADWHEEL_MSGS__MSG__DETAIL__DEADWHEEL_TICKS__BUILDER_HPP_
