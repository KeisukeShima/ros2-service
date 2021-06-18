#pragma once

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "std_msgs/msg/string.hpp"

#include <memory>
#include <string>

namespace proposal
{

class ClientNode : public rclcpp::Node
{
  using AddTwoInts = example_interfaces::srv::AddTwoInts;

  const rclcpp::Client<AddTwoInts>::SharedPtr service_client_;
  // Added timer to see if it works with client
  const rclcpp::TimerBase::SharedPtr timer_;
  // Added publisher to see if it works with client
  const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  // Added boolean to publish responce status
  bool response_received_;

public:
  explicit ClientNode(const rclcpp::NodeOptions & options);

  // Callback for service responce
  void serviceResponseCallback(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future);

  // Callback for timer
  void timerCallback();
};

}  // namespace proposal
