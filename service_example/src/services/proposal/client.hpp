#pragma once

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>
#include <string>

namespace proposal
{

class ClientNode : public rclcpp::Node
{
  using AddTwoInts = example_interfaces::srv::AddTwoInts;

  const rclcpp::Client<AddTwoInts>::SharedPtr service_client_;

public:
  explicit ClientNode(const rclcpp::NodeOptions & options);

  // Callback for service responce
  void serviceResponseCallback(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future);
};

}  // namespace proposal
