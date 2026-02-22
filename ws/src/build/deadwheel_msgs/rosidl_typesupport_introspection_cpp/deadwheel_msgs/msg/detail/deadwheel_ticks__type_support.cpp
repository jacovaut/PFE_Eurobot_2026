// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from deadwheel_msgs:msg/DeadwheelTicks.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "deadwheel_msgs/msg/detail/deadwheel_ticks__functions.h"
#include "deadwheel_msgs/msg/detail/deadwheel_ticks__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace deadwheel_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void DeadwheelTicks_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) deadwheel_msgs::msg::DeadwheelTicks(_init);
}

void DeadwheelTicks_fini_function(void * message_memory)
{
  auto typed_message = static_cast<deadwheel_msgs::msg::DeadwheelTicks *>(message_memory);
  typed_message->~DeadwheelTicks();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember DeadwheelTicks_message_member_array[4] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(deadwheel_msgs::msg::DeadwheelTicks, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "t0",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(deadwheel_msgs::msg::DeadwheelTicks, t0),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "t1",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(deadwheel_msgs::msg::DeadwheelTicks, t1),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "t2",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(deadwheel_msgs::msg::DeadwheelTicks, t2),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers DeadwheelTicks_message_members = {
  "deadwheel_msgs::msg",  // message namespace
  "DeadwheelTicks",  // message name
  4,  // number of fields
  sizeof(deadwheel_msgs::msg::DeadwheelTicks),
  false,  // has_any_key_member_
  DeadwheelTicks_message_member_array,  // message members
  DeadwheelTicks_init_function,  // function to initialize message memory (memory has to be allocated)
  DeadwheelTicks_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t DeadwheelTicks_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &DeadwheelTicks_message_members,
  get_message_typesupport_handle_function,
  &deadwheel_msgs__msg__DeadwheelTicks__get_type_hash,
  &deadwheel_msgs__msg__DeadwheelTicks__get_type_description,
  &deadwheel_msgs__msg__DeadwheelTicks__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace deadwheel_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<deadwheel_msgs::msg::DeadwheelTicks>()
{
  return &::deadwheel_msgs::msg::rosidl_typesupport_introspection_cpp::DeadwheelTicks_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, deadwheel_msgs, msg, DeadwheelTicks)() {
  return &::deadwheel_msgs::msg::rosidl_typesupport_introspection_cpp::DeadwheelTicks_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
