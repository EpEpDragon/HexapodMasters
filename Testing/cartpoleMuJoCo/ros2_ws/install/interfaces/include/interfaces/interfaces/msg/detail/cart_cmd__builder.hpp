// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/CartCmd.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__CART_CMD__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__CART_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/cart_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_CartCmd_force
{
public:
  Init_CartCmd_force()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interfaces::msg::CartCmd force(::interfaces::msg::CartCmd::_force_type arg)
  {
    msg_.force = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::CartCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::CartCmd>()
{
  return interfaces::msg::builder::Init_CartCmd_force();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__CART_CMD__BUILDER_HPP_
