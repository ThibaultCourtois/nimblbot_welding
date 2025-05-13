// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from telesoud_msgs:msg/TelesoudInstruction.idl
// generated code does not contain a copyright notice

#ifndef TELESOUD_MSGS__MSG__DETAIL__TELESOUD_INSTRUCTION__BUILDER_HPP_
#define TELESOUD_MSGS__MSG__DETAIL__TELESOUD_INSTRUCTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "telesoud_msgs/msg/detail/telesoud_instruction__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace telesoud_msgs
{

namespace msg
{

namespace builder
{

class Init_TelesoudInstruction_free_drive_btn_status
{
public:
  explicit Init_TelesoudInstruction_free_drive_btn_status(::telesoud_msgs::msg::TelesoudInstruction & msg)
  : msg_(msg)
  {}
  ::telesoud_msgs::msg::TelesoudInstruction free_drive_btn_status(::telesoud_msgs::msg::TelesoudInstruction::_free_drive_btn_status_type arg)
  {
    msg_.free_drive_btn_status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::telesoud_msgs::msg::TelesoudInstruction msg_;
};

class Init_TelesoudInstruction_speed
{
public:
  explicit Init_TelesoudInstruction_speed(::telesoud_msgs::msg::TelesoudInstruction & msg)
  : msg_(msg)
  {}
  Init_TelesoudInstruction_free_drive_btn_status speed(::telesoud_msgs::msg::TelesoudInstruction::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_TelesoudInstruction_free_drive_btn_status(msg_);
  }

private:
  ::telesoud_msgs::msg::TelesoudInstruction msg_;
};

class Init_TelesoudInstruction_speed_vector
{
public:
  explicit Init_TelesoudInstruction_speed_vector(::telesoud_msgs::msg::TelesoudInstruction & msg)
  : msg_(msg)
  {}
  Init_TelesoudInstruction_speed speed_vector(::telesoud_msgs::msg::TelesoudInstruction::_speed_vector_type arg)
  {
    msg_.speed_vector = std::move(arg);
    return Init_TelesoudInstruction_speed(msg_);
  }

private:
  ::telesoud_msgs::msg::TelesoudInstruction msg_;
};

class Init_TelesoudInstruction_pose1
{
public:
  explicit Init_TelesoudInstruction_pose1(::telesoud_msgs::msg::TelesoudInstruction & msg)
  : msg_(msg)
  {}
  Init_TelesoudInstruction_speed_vector pose1(::telesoud_msgs::msg::TelesoudInstruction::_pose1_type arg)
  {
    msg_.pose1 = std::move(arg);
    return Init_TelesoudInstruction_speed_vector(msg_);
  }

private:
  ::telesoud_msgs::msg::TelesoudInstruction msg_;
};

class Init_TelesoudInstruction_instruction_code
{
public:
  Init_TelesoudInstruction_instruction_code()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TelesoudInstruction_pose1 instruction_code(::telesoud_msgs::msg::TelesoudInstruction::_instruction_code_type arg)
  {
    msg_.instruction_code = std::move(arg);
    return Init_TelesoudInstruction_pose1(msg_);
  }

private:
  ::telesoud_msgs::msg::TelesoudInstruction msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::telesoud_msgs::msg::TelesoudInstruction>()
{
  return telesoud_msgs::msg::builder::Init_TelesoudInstruction_instruction_code();
}

}  // namespace telesoud_msgs

#endif  // TELESOUD_MSGS__MSG__DETAIL__TELESOUD_INSTRUCTION__BUILDER_HPP_
