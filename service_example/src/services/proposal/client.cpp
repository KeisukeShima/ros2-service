#include "client.hpp"

namespace proposal
{

ClientNode::ClientNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("proposal_client_node", options)
  , service_client_
    {
      create_client<AddTwoInts>("add_two_ints")
    }
  , timer_
    {
      create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ClientNode::timerCallback, this))
    }
  , publisher_
    {
      create_publisher<std_msgs::msg::String>("/chatter", rclcpp::QoS(1))
    }
  , response_received_{false}
{
  auto request {
    std::make_shared<AddTwoInts::Request>()
  };

  request->a = 1;
  request->b = 2;

  // Send service request only once.
  auto result {
    service_client_->async_send_request(
      request,
      std::bind(&ClientNode::serviceResponseCallback, this, std::placeholders::_1))
  };
}

void ClientNode::serviceResponseCallback(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
{
  RCLCPP_INFO(get_logger(), "response: '%ld'", future.get()->sum);
  response_received_ = true;
}

void ClientNode::timerCallback()
{
  RCLCPP_INFO(get_logger(), "timer callback");
  std_msgs::msg::String msg;
  if (response_received_) {
    msg.data = "Service responce received!";
  } else {
    msg.data = "Waiting for service responce...";
  }
  publisher_->publish(msg);
}

}  // namespace proposal

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(proposal::ClientNode)
