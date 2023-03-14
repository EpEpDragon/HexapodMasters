// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/CartObs.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__CART_OBS__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__CART_OBS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__CartObs __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__CartObs __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CartObs_
{
  using Type = CartObs_<ContainerAllocator>;

  explicit CartObs_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->position = 0.0f;
      this->angle = 0.0f;
      this->velocity = 0.0f;
      this->angular_velocity = 0.0f;
    }
  }

  explicit CartObs_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->position = 0.0f;
      this->angle = 0.0f;
      this->velocity = 0.0f;
      this->angular_velocity = 0.0f;
    }
  }

  // field types and members
  using _position_type =
    float;
  _position_type position;
  using _angle_type =
    float;
  _angle_type angle;
  using _velocity_type =
    float;
  _velocity_type velocity;
  using _angular_velocity_type =
    float;
  _angular_velocity_type angular_velocity;

  // setters for named parameter idiom
  Type & set__position(
    const float & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__angle(
    const float & _arg)
  {
    this->angle = _arg;
    return *this;
  }
  Type & set__velocity(
    const float & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__angular_velocity(
    const float & _arg)
  {
    this->angular_velocity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::CartObs_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::CartObs_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::CartObs_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::CartObs_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::CartObs_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::CartObs_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::CartObs_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::CartObs_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::CartObs_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::CartObs_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__CartObs
    std::shared_ptr<interfaces::msg::CartObs_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__CartObs
    std::shared_ptr<interfaces::msg::CartObs_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CartObs_ & other) const
  {
    if (this->position != other.position) {
      return false;
    }
    if (this->angle != other.angle) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->angular_velocity != other.angular_velocity) {
      return false;
    }
    return true;
  }
  bool operator!=(const CartObs_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CartObs_

// alias to use template instance with default allocator
using CartObs =
  interfaces::msg::CartObs_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__CART_OBS__STRUCT_HPP_
