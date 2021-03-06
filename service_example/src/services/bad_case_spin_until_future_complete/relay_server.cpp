#include "relay_server.hpp"

namespace spin_until_future_complete
{

RelayServerNode::RelayServerNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("spin_until_future_complete_relay_node", options)
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

  response_value_ = 0;

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
      std::bind(&RelayServerNode::serviceResponseCallback, this, std::placeholders::_1))
  };

  if (rclcpp::spin_until_future_complete(shared_from_this(), result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(get_logger(), "response: '%ld'", result.get()->sum);
  }

  response->sum = response_value_;
  RCLCPP_INFO(get_logger(), "send response");
}

void RelayServerNode::serviceResponseCallback(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
{
  RCLCPP_INFO(get_logger(), "response: '%ld'", future.get()->sum);
  response_value_ = future.get()->sum;
}

}  // namespace spin_until_future_complete

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(spin_until_future_complete::RelayServerNode)
