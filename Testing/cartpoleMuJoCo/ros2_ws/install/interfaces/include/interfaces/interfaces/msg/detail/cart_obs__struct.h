// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/CartObs.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__CART_OBS__STRUCT_H_
#define INTERFACES__MSG__DETAIL__CART_OBS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/CartObs in the package interfaces.
typedef struct interfaces__msg__CartObs
{
  float position;
  float angle;
  float velocity;
  float angular_velocity;
} interfaces__msg__CartObs;

// Struct for a sequence of interfaces__msg__CartObs.
typedef struct interfaces__msg__CartObs__Sequence
{
  interfaces__msg__CartObs * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__CartObs__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__CART_OBS__STRUCT_H_
