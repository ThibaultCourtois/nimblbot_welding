// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from telesoud_msgs:msg/TelesoudInstruction.idl
// generated code does not contain a copyright notice

#ifndef TELESOUD_MSGS__MSG__DETAIL__TELESOUD_INSTRUCTION__TRAITS_HPP_
#define TELESOUD_MSGS__MSG__DETAIL__TELESOUD_INSTRUCTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "telesoud_msgs/msg/detail/telesoud_instruction__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace telesoud_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const TelesoudInstruction & msg,
  std::ostream & out)
{
  out << "{";
  // member: instruction_code
  {
    out << "instruction_code: ";
    rosidl_generator_traits::value_to_yaml(msg.instruction_code, out);
    out << ", ";
  }

  // member: pose1
  {
    if (msg.pose1.size() == 0) {
      out << "pose1: []";
    } else {
      out << "pose1: [";
      size_t pending_items = msg.pose1.size();
      for (auto item : msg.pose1) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: speed_vector
  {
    if (msg.speed_vector.size() == 0) {
      out << "speed_vector: []";
    } else {
      out << "speed_vector: [";
      size_t pending_items = msg.speed_vector.size();
      for (auto item : msg.speed_vector) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: speed
  {
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << ", ";
  }

  // member: free_drive_btn_status
  {
    out << "free_drive_btn_status: ";
    rosidl_generator_traits::value_to_yaml(msg.free_drive_btn_status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TelesoudInstruction & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: instruction_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "instruction_code: ";
    rosidl_generator_traits::value_to_yaml(msg.instruction_code, out);
    out << "\n";
  }

  // member: pose1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pose1.size() == 0) {
      out << "pose1: []\n";
    } else {
      out << "pose1:\n";
      for (auto item : msg.pose1) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: speed_vector
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.speed_vector.size() == 0) {
      out << "speed_vector: []\n";
    } else {
      out << "speed_vector:\n";
      for (auto item : msg.speed_vector) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << "\n";
  }

  // member: free_drive_btn_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "free_drive_btn_status: ";
    rosidl_generator_traits::value_to_yaml(msg.free_drive_btn_status, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TelesoudInstruction & msg, bool use_flow_style = false)
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

}  // namespace telesoud_msgs

namespace rosidl_generator_traits
{

[[deprecated("use telesoud_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const telesoud_msgs::msg::TelesoudInstruction & msg,
  std::ostream & out, size_t indentation = 0)
{
  telesoud_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use telesoud_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const telesoud_msgs::msg::TelesoudInstruction & msg)
{
  return telesoud_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<telesoud_msgs::msg::TelesoudInstruction>()
{
  return "telesoud_msgs::msg::TelesoudInstruction";
}

template<>
inline const char * name<telesoud_msgs::msg::TelesoudInstruction>()
{
  return "telesoud_msgs/msg/TelesoudInstruction";
}

template<>
struct has_fixed_size<telesoud_msgs::msg::TelesoudInstruction>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<telesoud_msgs::msg::TelesoudInstruction>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<telesoud_msgs::msg::TelesoudInstruction>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TELESOUD_MSGS__MSG__DETAIL__TELESOUD_INSTRUCTION__TRAITS_HPP_
