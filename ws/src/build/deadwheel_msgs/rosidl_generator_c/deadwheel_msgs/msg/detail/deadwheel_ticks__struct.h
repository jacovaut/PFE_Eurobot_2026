// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from deadwheel_msgs:msg/DeadwheelTicks.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "deadwheel_msgs/msg/deadwheel_ticks.h"


#ifndef DEADWHEEL_MSGS__MSG__DETAIL__DEADWHEEL_TICKS__STRUCT_H_
#define DEADWHEEL_MSGS__MSG__DETAIL__DEADWHEEL_TICKS__STRUCT_H_

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

/// Struct defined in msg/DeadwheelTicks in the package deadwheel_msgs.
typedef struct deadwheel_msgs__msg__DeadwheelTicks
{
  std_msgs__msg__Header header;
  int32_t t0;
  int32_t t1;
  int32_t t2;
} deadwheel_msgs__msg__DeadwheelTicks;

// Struct for a sequence of deadwheel_msgs__msg__DeadwheelTicks.
typedef struct deadwheel_msgs__msg__DeadwheelTicks__Sequence
{
  deadwheel_msgs__msg__DeadwheelTicks * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} deadwheel_msgs__msg__DeadwheelTicks__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DEADWHEEL_MSGS__MSG__DETAIL__DEADWHEEL_TICKS__STRUCT_H_
