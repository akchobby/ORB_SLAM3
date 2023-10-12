
#pragma once
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "rcl_interfaces/srv/list_parameters.hpp"

template<typename NodeT>
void declare_parameter_if_not_declared(
  NodeT node,
  const std::string & param_name,
  const rclcpp::ParameterValue & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
  rcl_interfaces::msg::ParameterDescriptor())
{
  if (!node->has_parameter(param_name)) {
    node->declare_parameter(param_name, default_value, parameter_descriptor);
  }
}


double stamp_to_sec(builtin_interfaces::msg::Time stamp)
{
    double seconds = stamp.sec + (stamp.nanosec * pow(10,-9));
    return seconds;
}