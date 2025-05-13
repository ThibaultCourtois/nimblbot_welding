// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from telesoud_msgs:msg/TelesoudInstruction.idl
// generated code does not contain a copyright notice

#ifndef TELESOUD_MSGS__MSG__DETAIL__TELESOUD_INSTRUCTION__STRUCT_HPP_
#define TELESOUD_MSGS__MSG__DETAIL__TELESOUD_INSTRUCTION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__telesoud_msgs__msg__TelesoudInstruction __attribute__((deprecated))
#else
# define DEPRECATED__telesoud_msgs__msg__TelesoudInstruction __declspec(deprecated)
#endif

namespace telesoud_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TelesoudInstruction_
{
  using Type = TelesoudInstruction_<ContainerAllocator>;

  explicit TelesoudInstruction_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->instruction_code = 0;
      std::fill<typename std::array<double, 6>::iterator, double>(this->pose1.begin(), this->pose1.end(), 0.0);
      std::fill<typename std::array<double, 6>::iterator, double>(this->speed_vector.begin(), this->speed_vector.end(), 0.0);
      this->speed = 0.0;
      this->free_drive_btn_status = false;
    }
  }

  explicit TelesoudInstruction_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose1(_alloc),
    speed_vector(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->instruction_code = 0;
      std::fill<typename std::array<double, 6>::iterator, double>(this->pose1.begin(), this->pose1.end(), 0.0);
      std::fill<typename std::array<double, 6>::iterator, double>(this->speed_vector.begin(), this->speed_vector.end(), 0.0);
      this->speed = 0.0;
      this->free_drive_btn_status = false;
    }
  }

  // field types and members
  using _instruction_code_type =
    uint16_t;
  _instruction_code_type instruction_code;
  using _pose1_type =
    std::array<double, 6>;
  _pose1_type pose1;
  using _speed_vector_type =
    std::array<double, 6>;
  _speed_vector_type speed_vector;
  using _speed_type =
    double;
  _speed_type speed;
  using _free_drive_btn_status_type =
    bool;
  _free_drive_btn_status_type free_drive_btn_status;

  // setters for named parameter idiom
  Type & set__instruction_code(
    const uint16_t & _arg)
  {
    this->instruction_code = _arg;
    return *this;
  }
  Type & set__pose1(
    const std::array<double, 6> & _arg)
  {
    this->pose1 = _arg;
    return *this;
  }
  Type & set__speed_vector(
    const std::array<double, 6> & _arg)
  {
    this->speed_vector = _arg;
    return *this;
  }
  Type & set__speed(
    const double & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__free_drive_btn_status(
    const bool & _arg)
  {
    this->free_drive_btn_status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    telesoud_msgs::msg::TelesoudInstruction_<ContainerAllocator> *;
  using ConstRawPtr =
    const telesoud_msgs::msg::TelesoudInstruction_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<telesoud_msgs::msg::TelesoudInstruction_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<telesoud_msgs::msg::TelesoudInstruction_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      telesoud_msgs::msg::TelesoudInstruction_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<telesoud_msgs::msg::TelesoudInstruction_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      telesoud_msgs::msg::TelesoudInstruction_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<telesoud_msgs::msg::TelesoudInstruction_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<telesoud_msgs::msg::TelesoudInstruction_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<telesoud_msgs::msg::TelesoudInstruction_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__telesoud_msgs__msg__TelesoudInstruction
    std::shared_ptr<telesoud_msgs::msg::TelesoudInstruction_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__telesoud_msgs__msg__TelesoudInstruction
    std::shared_ptr<telesoud_msgs::msg::TelesoudInstruction_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TelesoudInstruction_ & other) const
  {
    if (this->instruction_code != other.instruction_code) {
      return false;
    }
    if (this->pose1 != other.pose1) {
      return false;
    }
    if (this->speed_vector != other.speed_vector) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    if (this->free_drive_btn_status != other.free_drive_btn_status) {
      return false;
    }
    return true;
  }
  bool operator!=(const TelesoudInstruction_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TelesoudInstruction_

// alias to use template instance with default allocator
using TelesoudInstruction =
  telesoud_msgs::msg::TelesoudInstruction_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace telesoud_msgs

#endif  // TELESOUD_MSGS__MSG__DETAIL__TELESOUD_INSTRUCTION__STRUCT_HPP_
