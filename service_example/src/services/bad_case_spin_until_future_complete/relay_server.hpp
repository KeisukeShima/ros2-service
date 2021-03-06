#pragma once

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>
#include <string>

namespace spin_until_future_complete
{

class RelayServerNode : public rclcpp::Node
{
  using AddTwoInts = example_interfaces::srv::AddTwoInts;

  rclcpp::CallbackGroup::SharedPtr callback_group_services_;
  const rclcpp::Client<AddTwoInts>::SharedPtr service_client_;
  const rclcpp::Service<AddTwoInts>::SharedPtr service_server_;
  int64_t response_value_;

public:
  explicit RelayServerNode(const rclcpp::NodeOptions & options);

  void requestReceivedCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response);

  // Callback for service responce
  void serviceResponseCallback(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future);
};

}  // namespace spin_until_future_complete
