// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from telesoud_msgs:msg/TelesoudInstruction.idl
// generated code does not contain a copyright notice
#include "telesoud_msgs/msg/detail/telesoud_instruction__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "telesoud_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "telesoud_msgs/msg/detail/telesoud_instruction__struct.h"
#include "telesoud_msgs/msg/detail/telesoud_instruction__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _TelesoudInstruction__ros_msg_type = telesoud_msgs__msg__TelesoudInstruction;

static bool _TelesoudInstruction__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _TelesoudInstruction__ros_msg_type * ros_message = static_cast<const _TelesoudInstruction__ros_msg_type *>(untyped_ros_message);
  // Field name: instruction_code
  {
    cdr << ros_message->instruction_code;
  }

  // Field name: pose1
  {
    size_t size = 6;
    auto array_ptr = ros_message->pose1;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: speed_vector
  {
    size_t size = 6;
    auto array_ptr = ros_message->speed_vector;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: speed
  {
    cdr << ros_message->speed;
  }

  // Field name: free_drive_btn_status
  {
    cdr << (ros_message->free_drive_btn_status ? true : false);
  }

  return true;
}

static bool _TelesoudInstruction__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _TelesoudInstruction__ros_msg_type * ros_message = static_cast<_TelesoudInstruction__ros_msg_type *>(untyped_ros_message);
  // Field name: instruction_code
  {
    cdr >> ros_message->instruction_code;
  }

  // Field name: pose1
  {
    size_t size = 6;
    auto array_ptr = ros_message->pose1;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: speed_vector
  {
    size_t size = 6;
    auto array_ptr = ros_message->speed_vector;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: speed
  {
    cdr >> ros_message->speed;
  }

  // Field name: free_drive_btn_status
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->free_drive_btn_status = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_telesoud_msgs
size_t get_serialized_size_telesoud_msgs__msg__TelesoudInstruction(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TelesoudInstruction__ros_msg_type * ros_message = static_cast<const _TelesoudInstruction__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name instruction_code
  {
    size_t item_size = sizeof(ros_message->instruction_code);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pose1
  {
    size_t array_size = 6;
    auto array_ptr = ros_message->pose1;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name speed_vector
  {
    size_t array_size = 6;
    auto array_ptr = ros_message->speed_vector;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name speed
  {
    size_t item_size = sizeof(ros_message->speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name free_drive_btn_status
  {
    size_t item_size = sizeof(ros_message->free_drive_btn_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _TelesoudInstruction__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_telesoud_msgs__msg__TelesoudInstruction(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_telesoud_msgs
size_t max_serialized_size_telesoud_msgs__msg__TelesoudInstruction(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: instruction_code
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: pose1
  {
    size_t array_size = 6;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: speed_vector
  {
    size_t array_size = 6;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: speed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: free_drive_btn_status
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = telesoud_msgs__msg__TelesoudInstruction;
    is_plain =
      (
      offsetof(DataType, free_drive_btn_status) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _TelesoudInstruction__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_telesoud_msgs__msg__TelesoudInstruction(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_TelesoudInstruction = {
  "telesoud_msgs::msg",
  "TelesoudInstruction",
  _TelesoudInstruction__cdr_serialize,
  _TelesoudInstruction__cdr_deserialize,
  _TelesoudInstruction__get_serialized_size,
  _TelesoudInstruction__max_serialized_size
};

static rosidl_message_type_support_t _TelesoudInstruction__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_TelesoudInstruction,
  get_message_typesupport_handle_function,
  &telesoud_msgs__msg__TelesoudInstruction__get_type_hash,
  &telesoud_msgs__msg__TelesoudInstruction__get_type_description,
  &telesoud_msgs__msg__TelesoudInstruction__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, telesoud_msgs, msg, TelesoudInstruction)() {
  return &_TelesoudInstruction__type_support;
}

#if defined(__cplusplus)
}
#endif
