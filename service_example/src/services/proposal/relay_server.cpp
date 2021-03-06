#include "relay_server.hpp"

namespace proposal
{

RelayServerNode::RelayServerNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("proposal_relay_node", options)
  , callback_group_services_
    {
      create_callback_group(rclcpp::CallbackGroupType::Reentrant)
    }
  , service_client_
    {
      create_client<AddTwoInts>("add_two_ints", rmw_qos_profile_default, callback_group_services_)
    }
  , service_server_
    {
      create_service<AddTwoInts>(
        "relay",
        std::bind(&RelayServerNode::requestReceivedCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3))
    }
{
}

void RelayServerNode::requestReceivedCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
  std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
{
  (void)request_header;
  RCLCPP_INFO(get_logger(), "request received");

  rclcpp::WallRate loop_rate(std::chrono::seconds(1));
  while (!service_client_->service_is_ready()) {
    RCLCPP_INFO(get_logger(), "waiting service");
    if (!rclcpp::ok()) {
      return;
    }
    loop_rate.sleep();
  }

  // Send service request.
  auto result {
    service_client_->async_send_request(
      request,
      [](rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture){})
  };

  std::future_status status = result.wait_for(std::chrono::seconds(0));
  while (status != std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "waiting response");
    if (!rclcpp::ok()) {
      return;
    }
    status = result.wait_for(std::chrono::seconds(1));
  }

  RCLCPP_INFO(get_logger(), "response: '%ld'", result.get()->sum);
  response->sum = result.get()->sum;
  RCLCPP_INFO(get_logger(), "send response");
}

}  // namespace proposal

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(proposal::RelayServerNode)
