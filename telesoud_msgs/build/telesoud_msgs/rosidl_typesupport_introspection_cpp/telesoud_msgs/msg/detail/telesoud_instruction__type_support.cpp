// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from telesoud_msgs:msg/TelesoudInstruction.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "telesoud_msgs/msg/detail/telesoud_instruction__functions.h"
#include "telesoud_msgs/msg/detail/telesoud_instruction__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace telesoud_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void TelesoudInstruction_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) telesoud_msgs::msg::TelesoudInstruction(_init);
}

void TelesoudInstruction_fini_function(void * message_memory)
{
  auto typed_message = static_cast<telesoud_msgs::msg::TelesoudInstruction *>(message_memory);
  typed_message->~TelesoudInstruction();
}

size_t size_function__TelesoudInstruction__pose1(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__TelesoudInstruction__pose1(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__TelesoudInstruction__pose1(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 6> *>(untyped_member);
  return &member[index];
}

void fetch_function__TelesoudInstruction__pose1(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TelesoudInstruction__pose1(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TelesoudInstruction__pose1(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TelesoudInstruction__pose1(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__TelesoudInstruction__speed_vector(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__TelesoudInstruction__speed_vector(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__TelesoudInstruction__speed_vector(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 6> *>(untyped_member);
  return &member[index];
}

void fetch_function__TelesoudInstruction__speed_vector(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TelesoudInstruction__speed_vector(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TelesoudInstruction__speed_vector(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TelesoudInstruction__speed_vector(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TelesoudInstruction_message_member_array[5] = {
  {
    "instruction_code",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(telesoud_msgs::msg::TelesoudInstruction, instruction_code),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pose1",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(telesoud_msgs::msg::TelesoudInstruction, pose1),  // bytes offset in struct
    nullptr,  // default value
    size_function__TelesoudInstruction__pose1,  // size() function pointer
    get_const_function__TelesoudInstruction__pose1,  // get_const(index) function pointer
    get_function__TelesoudInstruction__pose1,  // get(index) function pointer
    fetch_function__TelesoudInstruction__pose1,  // fetch(index, &value) function pointer
    assign_function__TelesoudInstruction__pose1,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "speed_vector",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(telesoud_msgs::msg::TelesoudInstruction, speed_vector),  // bytes offset in struct
    nullptr,  // default value
    size_function__TelesoudInstruction__speed_vector,  // size() function pointer
    get_const_function__TelesoudInstruction__speed_vector,  // get_const(index) function pointer
    get_function__TelesoudInstruction__speed_vector,  // get(index) function pointer
    fetch_function__TelesoudInstruction__speed_vector,  // fetch(index, &value) function pointer
    assign_function__TelesoudInstruction__speed_vector,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "speed",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(telesoud_msgs::msg::TelesoudInstruction, speed),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "free_drive_btn_status",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(telesoud_msgs::msg::TelesoudInstruction, free_drive_btn_status),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TelesoudInstruction_message_members = {
  "telesoud_msgs::msg",  // message namespace
  "TelesoudInstruction",  // message name
  5,  // number of fields
  sizeof(telesoud_msgs::msg::TelesoudInstruction),
  TelesoudInstruction_message_member_array,  // message members
  TelesoudInstruction_init_function,  // function to initialize message memory (memory has to be allocated)
  TelesoudInstruction_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TelesoudInstruction_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TelesoudInstruction_message_members,
  get_message_typesupport_handle_function,
  &telesoud_msgs__msg__TelesoudInstruction__get_type_hash,
  &telesoud_msgs__msg__TelesoudInstruction__get_type_description,
  &telesoud_msgs__msg__TelesoudInstruction__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace telesoud_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<telesoud_msgs::msg::TelesoudInstruction>()
{
  return &::telesoud_msgs::msg::rosidl_typesupport_introspection_cpp::TelesoudInstruction_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, telesoud_msgs, msg, TelesoudInstruction)() {
  return &::telesoud_msgs::msg::rosidl_typesupport_introspection_cpp::TelesoudInstruction_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
