// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from telesoud_msgs:msg/TelesoudInstruction.idl
// generated code does not contain a copyright notice
#include "telesoud_msgs/msg/detail/telesoud_instruction__rosidl_typesupport_fastrtps_cpp.hpp"
#include "telesoud_msgs/msg/detail/telesoud_instruction__functions.h"
#include "telesoud_msgs/msg/detail/telesoud_instruction__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace telesoud_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_telesoud_msgs
cdr_serialize(
  const telesoud_msgs::msg::TelesoudInstruction & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: instruction_code
  cdr << ros_message.instruction_code;
  // Member: pose1
  {
    cdr << ros_message.pose1;
  }
  // Member: speed_vector
  {
    cdr << ros_message.speed_vector;
  }
  // Member: speed
  cdr << ros_message.speed;
  // Member: free_drive_btn_status
  cdr << (ros_message.free_drive_btn_status ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_telesoud_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  telesoud_msgs::msg::TelesoudInstruction & ros_message)
{
  // Member: instruction_code
  cdr >> ros_message.instruction_code;

  // Member: pose1
  {
    cdr >> ros_message.pose1;
  }

  // Member: speed_vector
  {
    cdr >> ros_message.speed_vector;
  }

  // Member: speed
  cdr >> ros_message.speed;

  // Member: free_drive_btn_status
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.free_drive_btn_status = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_telesoud_msgs
get_serialized_size(
  const telesoud_msgs::msg::TelesoudInstruction & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: instruction_code
  {
    size_t item_size = sizeof(ros_message.instruction_code);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pose1
  {
    size_t array_size = 6;
    size_t item_size = sizeof(ros_message.pose1[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: speed_vector
  {
    size_t array_size = 6;
    size_t item_size = sizeof(ros_message.speed_vector[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: speed
  {
    size_t item_size = sizeof(ros_message.speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: free_drive_btn_status
  {
    size_t item_size = sizeof(ros_message.free_drive_btn_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_telesoud_msgs
max_serialized_size_TelesoudInstruction(
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


  // Member: instruction_code
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: pose1
  {
    size_t array_size = 6;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: speed_vector
  {
    size_t array_size = 6;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: speed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: free_drive_btn_status
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
    using DataType = telesoud_msgs::msg::TelesoudInstruction;
    is_plain =
      (
      offsetof(DataType, free_drive_btn_status) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _TelesoudInstruction__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const telesoud_msgs::msg::TelesoudInstruction *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _TelesoudInstruction__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<telesoud_msgs::msg::TelesoudInstruction *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _TelesoudInstruction__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const telesoud_msgs::msg::TelesoudInstruction *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _TelesoudInstruction__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_TelesoudInstruction(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _TelesoudInstruction__callbacks = {
  "telesoud_msgs::msg",
  "TelesoudInstruction",
  _TelesoudInstruction__cdr_serialize,
  _TelesoudInstruction__cdr_deserialize,
  _TelesoudInstruction__get_serialized_size,
  _TelesoudInstruction__max_serialized_size
};

static rosidl_message_type_support_t _TelesoudInstruction__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_TelesoudInstruction__callbacks,
  get_message_typesupport_handle_function,
  &telesoud_msgs__msg__TelesoudInstruction__get_type_hash,
  &telesoud_msgs__msg__TelesoudInstruction__get_type_description,
  &telesoud_msgs__msg__TelesoudInstruction__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace telesoud_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_telesoud_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<telesoud_msgs::msg::TelesoudInstruction>()
{
  return &telesoud_msgs::msg::typesupport_fastrtps_cpp::_TelesoudInstruction__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, telesoud_msgs, msg, TelesoudInstruction)() {
  return &telesoud_msgs::msg::typesupport_fastrtps_cpp::_TelesoudInstruction__handle;
}

#ifdef __cplusplus
}
#endif
