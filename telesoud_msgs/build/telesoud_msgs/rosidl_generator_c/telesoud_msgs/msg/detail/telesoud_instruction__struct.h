// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from telesoud_msgs:msg/TelesoudInstruction.idl
// generated code does not contain a copyright notice

#ifndef TELESOUD_MSGS__MSG__DETAIL__TELESOUD_INSTRUCTION__STRUCT_H_
#define TELESOUD_MSGS__MSG__DETAIL__TELESOUD_INSTRUCTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/TelesoudInstruction in the package telesoud_msgs.
typedef struct telesoud_msgs__msg__TelesoudInstruction
{
  uint16_t instruction_code;
  double pose1[6];
  double speed_vector[6];
  double speed;
  bool free_drive_btn_status;
} telesoud_msgs__msg__TelesoudInstruction;

// Struct for a sequence of telesoud_msgs__msg__TelesoudInstruction.
typedef struct telesoud_msgs__msg__TelesoudInstruction__Sequence
{
  telesoud_msgs__msg__TelesoudInstruction * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} telesoud_msgs__msg__TelesoudInstruction__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TELESOUD_MSGS__MSG__DETAIL__TELESOUD_INSTRUCTION__STRUCT_H_
