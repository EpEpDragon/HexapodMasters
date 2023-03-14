// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/CartObs.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__CART_OBS__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__CART_OBS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/cart_obs__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_CartObs_angular_velocity
{
public:
  explicit Init_CartObs_angular_velocity(::interfaces::msg::CartObs & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::CartObs angular_velocity(::interfaces::msg::CartObs::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::CartObs msg_;
};

class Init_CartObs_velocity
{
public:
  explicit Init_CartObs_velocity(::interfaces::msg::CartObs & msg)
  : msg_(msg)
  {}
  Init_CartObs_angular_velocity velocity(::interfaces::msg::CartObs::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_CartObs_angular_velocity(msg_);
  }

private:
  ::interfaces::msg::CartObs msg_;
};

class Init_CartObs_angle
{
public:
  explicit Init_CartObs_angle(::interfaces::msg::CartObs & msg)
  : msg_(msg)
  {}
  Init_CartObs_velocity angle(::interfaces::msg::CartObs::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return Init_CartObs_velocity(msg_);
  }

private:
  ::interfaces::msg::CartObs msg_;
};

class Init_CartObs_position
{
public:
  Init_CartObs_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CartObs_angle position(::interfaces::msg::CartObs::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_CartObs_angle(msg_);
  }

private:
  ::interfaces::msg::CartObs msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::CartObs>()
{
  return interfaces::msg::builder::Init_CartObs_position();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__CART_OBS__BUILDER_HPP_
