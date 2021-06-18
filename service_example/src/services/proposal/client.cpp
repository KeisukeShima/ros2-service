#include "client.hpp"

namespace proposal
{

ClientNode::ClientNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("proposal_client_node", options)
  , service_client_
    {
      create_client<AddTwoInts>("relay")
    }
{
  auto request {
    std::make_shared<AddTwoInts::Request>()
  };

  request->a = 1;
  request->b = 2;

  rclcpp::WallRate loop_rate(std::chrono::seconds(1));
  while (!service_client_->service_is_ready()) {
    RCLCPP_INFO(get_logger(), "waiting service");
    if (!rclcpp::ok()) {
      return;
    }
    loop_rate.sleep();
  }

  // Send service request only once.
  auto result {
    service_client_->async_send_request(
      request,
      std::bind(&ClientNode::serviceResponseCallback, this, std::placeholders::_1))
  };
  RCLCPP_INFO(get_logger(), "send request");
}

void ClientNode::serviceResponseCallback(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
{
  RCLCPP_INFO(get_logger(), "response: '%ld'", future.get()->sum);
}

}  // namespace proposal

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(proposal::ClientNode)
