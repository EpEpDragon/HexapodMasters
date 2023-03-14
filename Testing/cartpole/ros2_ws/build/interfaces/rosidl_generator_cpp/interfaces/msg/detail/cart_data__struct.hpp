// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/CartData.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__CART_DATA__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__CART_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__CartData __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__CartData __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CartData_
{
  using Type = CartData_<ContainerAllocator>;

  explicit CartData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->velocity = 0.0f;
      this->angle = 0.0f;
    }
  }

  explicit CartData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->velocity = 0.0f;
      this->angle = 0.0f;
    }
  }

  // field types and members
  using _velocity_type =
    float;
  _velocity_type velocity;
  using _angle_type =
    float;
  _angle_type angle;

  // setters for named parameter idiom
  Type & set__velocity(
    const float & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__angle(
    const float & _arg)
  {
    this->angle = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::CartData_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::CartData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::CartData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::CartData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::CartData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::CartData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::CartData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::CartData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::CartData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::CartData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__CartData
    std::shared_ptr<interfaces::msg::CartData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__CartData
    std::shared_ptr<interfaces::msg::CartData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CartData_ & other) const
  {
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->angle != other.angle) {
      return false;
    }
    return true;
  }
  bool operator!=(const CartData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CartData_

// alias to use template instance with default allocator
using CartData =
  interfaces::msg::CartData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__CART_DATA__STRUCT_HPP_
