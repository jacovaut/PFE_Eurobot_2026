// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from deadwheel_msgs:msg/DeadwheelTicks.idl
// generated code does not contain a copyright notice
#ifndef DEADWHEEL_MSGS__MSG__DETAIL__DEADWHEEL_TICKS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define DEADWHEEL_MSGS__MSG__DETAIL__DEADWHEEL_TICKS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "deadwheel_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "deadwheel_msgs/msg/detail/deadwheel_ticks__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_deadwheel_msgs
bool cdr_serialize_deadwheel_msgs__msg__DeadwheelTicks(
  const deadwheel_msgs__msg__DeadwheelTicks * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_deadwheel_msgs
bool cdr_deserialize_deadwheel_msgs__msg__DeadwheelTicks(
  eprosima::fastcdr::Cdr &,
  deadwheel_msgs__msg__DeadwheelTicks * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_deadwheel_msgs
size_t get_serialized_size_deadwheel_msgs__msg__DeadwheelTicks(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_deadwheel_msgs
size_t max_serialized_size_deadwheel_msgs__msg__DeadwheelTicks(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_deadwheel_msgs
bool cdr_serialize_key_deadwheel_msgs__msg__DeadwheelTicks(
  const deadwheel_msgs__msg__DeadwheelTicks * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_deadwheel_msgs
size_t get_serialized_size_key_deadwheel_msgs__msg__DeadwheelTicks(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_deadwheel_msgs
size_t max_serialized_size_key_deadwheel_msgs__msg__DeadwheelTicks(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_deadwheel_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, deadwheel_msgs, msg, DeadwheelTicks)();

#ifdef __cplusplus
}
#endif

#endif  // DEADWHEEL_MSGS__MSG__DETAIL__DEADWHEEL_TICKS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
