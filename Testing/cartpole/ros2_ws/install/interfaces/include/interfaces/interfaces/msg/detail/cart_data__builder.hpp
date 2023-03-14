// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/CartData.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__CART_DATA__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__CART_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/cart_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_CartData_angle
{
public:
  explicit Init_CartData_angle(::interfaces::msg::CartData & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::CartData angle(::interfaces::msg::CartData::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::CartData msg_;
};

class Init_CartData_velocity
{
public:
  Init_CartData_velocity()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CartData_angle velocity(::interfaces::msg::CartData::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_CartData_angle(msg_);
  }

private:
  ::interfaces::msg::CartData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::CartData>()
{
  return interfaces::msg::builder::Init_CartData_velocity();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__CART_DATA__BUILDER_HPP_
