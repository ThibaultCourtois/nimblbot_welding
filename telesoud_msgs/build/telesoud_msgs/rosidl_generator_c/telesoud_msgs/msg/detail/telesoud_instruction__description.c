// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from telesoud_msgs:msg/TelesoudInstruction.idl
// generated code does not contain a copyright notice

#include "telesoud_msgs/msg/detail/telesoud_instruction__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_telesoud_msgs
const rosidl_type_hash_t *
telesoud_msgs__msg__TelesoudInstruction__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x18, 0x5b, 0x89, 0xa3, 0x7e, 0x1b, 0xce, 0x09,
      0x4e, 0x16, 0x2b, 0x40, 0xdf, 0xc8, 0x2f, 0x26,
      0x0c, 0xf8, 0x9f, 0x2b, 0x87, 0x92, 0x9b, 0x62,
      0xf5, 0x2c, 0x1d, 0xcb, 0x63, 0x21, 0x0a, 0x31,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char telesoud_msgs__msg__TelesoudInstruction__TYPE_NAME[] = "telesoud_msgs/msg/TelesoudInstruction";

// Define type names, field names, and default values
static char telesoud_msgs__msg__TelesoudInstruction__FIELD_NAME__instruction_code[] = "instruction_code";
static char telesoud_msgs__msg__TelesoudInstruction__FIELD_NAME__pose1[] = "pose1";
static char telesoud_msgs__msg__TelesoudInstruction__FIELD_NAME__speed_vector[] = "speed_vector";
static char telesoud_msgs__msg__TelesoudInstruction__FIELD_NAME__speed[] = "speed";
static char telesoud_msgs__msg__TelesoudInstruction__FIELD_NAME__free_drive_btn_status[] = "free_drive_btn_status";

static rosidl_runtime_c__type_description__Field telesoud_msgs__msg__TelesoudInstruction__FIELDS[] = {
  {
    {telesoud_msgs__msg__TelesoudInstruction__FIELD_NAME__instruction_code, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {telesoud_msgs__msg__TelesoudInstruction__FIELD_NAME__pose1, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      6,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {telesoud_msgs__msg__TelesoudInstruction__FIELD_NAME__speed_vector, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      6,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {telesoud_msgs__msg__TelesoudInstruction__FIELD_NAME__speed, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {telesoud_msgs__msg__TelesoudInstruction__FIELD_NAME__free_drive_btn_status, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
telesoud_msgs__msg__TelesoudInstruction__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {telesoud_msgs__msg__TelesoudInstruction__TYPE_NAME, 37, 37},
      {telesoud_msgs__msg__TelesoudInstruction__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint16 instruction_code\n"
  "float64[6] pose1\n"
  "float64[6] speed_vector\n"
  "float64 speed\n"
  "bool free_drive_btn_status";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
telesoud_msgs__msg__TelesoudInstruction__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {telesoud_msgs__msg__TelesoudInstruction__TYPE_NAME, 37, 37},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 106, 106},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
telesoud_msgs__msg__TelesoudInstruction__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *telesoud_msgs__msg__TelesoudInstruction__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
