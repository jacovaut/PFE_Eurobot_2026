// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from deadwheel_msgs:msg/DeadwheelTicks.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "deadwheel_msgs/msg/deadwheel_ticks.hpp"


#ifndef DEADWHEEL_MSGS__MSG__DETAIL__DEADWHEEL_TICKS__TRAITS_HPP_
#define DEADWHEEL_MSGS__MSG__DETAIL__DEADWHEEL_TICKS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "deadwheel_msgs/msg/detail/deadwheel_ticks__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace deadwheel_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DeadwheelTicks & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: t0
  {
    out << "t0: ";
    rosidl_generator_traits::value_to_yaml(msg.t0, out);
    out << ", ";
  }

  // member: t1
  {
    out << "t1: ";
    rosidl_generator_traits::value_to_yaml(msg.t1, out);
    out << ", ";
  }

  // member: t2
  {
    out << "t2: ";
    rosidl_generator_traits::value_to_yaml(msg.t2, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DeadwheelTicks & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: t0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "t0: ";
    rosidl_generator_traits::value_to_yaml(msg.t0, out);
    out << "\n";
  }

  // member: t1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "t1: ";
    rosidl_generator_traits::value_to_yaml(msg.t1, out);
    out << "\n";
  }

  // member: t2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "t2: ";
    rosidl_generator_traits::value_to_yaml(msg.t2, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DeadwheelTicks & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace deadwheel_msgs

namespace rosidl_generator_traits
{

[[deprecated("use deadwheel_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const deadwheel_msgs::msg::DeadwheelTicks & msg,
  std::ostream & out, size_t indentation = 0)
{
  deadwheel_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use deadwheel_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const deadwheel_msgs::msg::DeadwheelTicks & msg)
{
  return deadwheel_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<deadwheel_msgs::msg::DeadwheelTicks>()
{
  return "deadwheel_msgs::msg::DeadwheelTicks";
}

template<>
inline const char * name<deadwheel_msgs::msg::DeadwheelTicks>()
{
  return "deadwheel_msgs/msg/DeadwheelTicks";
}

template<>
struct has_fixed_size<deadwheel_msgs::msg::DeadwheelTicks>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<deadwheel_msgs::msg::DeadwheelTicks>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<deadwheel_msgs::msg::DeadwheelTicks>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DEADWHEEL_MSGS__MSG__DETAIL__DEADWHEEL_TICKS__TRAITS_HPP_
