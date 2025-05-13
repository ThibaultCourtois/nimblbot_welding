// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from telesoud_msgs:msg/TelesoudInstruction.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "telesoud_msgs/msg/detail/telesoud_instruction__rosidl_typesupport_introspection_c.h"
#include "telesoud_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "telesoud_msgs/msg/detail/telesoud_instruction__functions.h"
#include "telesoud_msgs/msg/detail/telesoud_instruction__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__TelesoudInstruction_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  telesoud_msgs__msg__TelesoudInstruction__init(message_memory);
}

void telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__TelesoudInstruction_fini_function(void * message_memory)
{
  telesoud_msgs__msg__TelesoudInstruction__fini(message_memory);
}

size_t telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__size_function__TelesoudInstruction__pose1(
  const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__get_const_function__TelesoudInstruction__pose1(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__get_function__TelesoudInstruction__pose1(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__fetch_function__TelesoudInstruction__pose1(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__get_const_function__TelesoudInstruction__pose1(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__assign_function__TelesoudInstruction__pose1(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__get_function__TelesoudInstruction__pose1(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__size_function__TelesoudInstruction__speed_vector(
  const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__get_const_function__TelesoudInstruction__speed_vector(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__get_function__TelesoudInstruction__speed_vector(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__fetch_function__TelesoudInstruction__speed_vector(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__get_const_function__TelesoudInstruction__speed_vector(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__assign_function__TelesoudInstruction__speed_vector(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__get_function__TelesoudInstruction__speed_vector(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__TelesoudInstruction_message_member_array[5] = {
  {
    "instruction_code",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(telesoud_msgs__msg__TelesoudInstruction, instruction_code),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(telesoud_msgs__msg__TelesoudInstruction, pose1),  // bytes offset in struct
    NULL,  // default value
    telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__size_function__TelesoudInstruction__pose1,  // size() function pointer
    telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__get_const_function__TelesoudInstruction__pose1,  // get_const(index) function pointer
    telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__get_function__TelesoudInstruction__pose1,  // get(index) function pointer
    telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__fetch_function__TelesoudInstruction__pose1,  // fetch(index, &value) function pointer
    telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__assign_function__TelesoudInstruction__pose1,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "speed_vector",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(telesoud_msgs__msg__TelesoudInstruction, speed_vector),  // bytes offset in struct
    NULL,  // default value
    telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__size_function__TelesoudInstruction__speed_vector,  // size() function pointer
    telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__get_const_function__TelesoudInstruction__speed_vector,  // get_const(index) function pointer
    telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__get_function__TelesoudInstruction__speed_vector,  // get(index) function pointer
    telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__fetch_function__TelesoudInstruction__speed_vector,  // fetch(index, &value) function pointer
    telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__assign_function__TelesoudInstruction__speed_vector,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(telesoud_msgs__msg__TelesoudInstruction, speed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "free_drive_btn_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(telesoud_msgs__msg__TelesoudInstruction, free_drive_btn_status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__TelesoudInstruction_message_members = {
  "telesoud_msgs__msg",  // message namespace
  "TelesoudInstruction",  // message name
  5,  // number of fields
  sizeof(telesoud_msgs__msg__TelesoudInstruction),
  telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__TelesoudInstruction_message_member_array,  // message members
  telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__TelesoudInstruction_init_function,  // function to initialize message memory (memory has to be allocated)
  telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__TelesoudInstruction_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__TelesoudInstruction_message_type_support_handle = {
  0,
  &telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__TelesoudInstruction_message_members,
  get_message_typesupport_handle_function,
  &telesoud_msgs__msg__TelesoudInstruction__get_type_hash,
  &telesoud_msgs__msg__TelesoudInstruction__get_type_description,
  &telesoud_msgs__msg__TelesoudInstruction__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_telesoud_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, telesoud_msgs, msg, TelesoudInstruction)() {
  if (!telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__TelesoudInstruction_message_type_support_handle.typesupport_identifier) {
    telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__TelesoudInstruction_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &telesoud_msgs__msg__TelesoudInstruction__rosidl_typesupport_introspection_c__TelesoudInstruction_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
