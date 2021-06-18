#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

#include <memory>
#include <string>

class ClientNode
  : public rclcpp::Node
{
  using AddTwoInts = example_interfaces::srv::AddTwoInts;

  const rclcpp::Client<AddTwoInts>::SharedPtr service_client;

public:
  explicit ClientNode(const rclcpp::NodeOptions & options)
    : rclcpp::Node("client_node", options)
    , service_client
      {
        create_client<AddTwoInts>("add_two_ints")
      }
  {
    auto request {
      std::make_shared<AddTwoInts::Request>()
    };

    request->a = 1;
    request->b = 2;

    auto result {
      service_client->async_send_request(
        request,
        std::bind(&ClientNode::callback, this, std::placeholders::_1))
    };
  }

  void callback(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
  {
    RCLCPP_INFO(get_logger(), "response: '%ld'", future.get()->sum);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor {};

  rclcpp::NodeOptions options {};

  auto node {
    std::make_shared<ClientNode>(options)
  };

  executor.add_node(node);

  executor.spin();

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
