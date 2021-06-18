#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

#include <memory>
#include <string>

class ClientNode
  : public rclcpp::Node
{
  using AddTwoInts = example_interfaces::srv::AddTwoInts;

  const rclcpp::Client<AddTwoInts>::SharedPtr service_client;

  const rclcpp::TimerBase::SharedPtr timer;

public:
  explicit ClientNode(const rclcpp::NodeOptions & options)
    : rclcpp::Node("client_node", options)
    , service_client
      {
        create_client<AddTwoInts>("add_two_ints")
      }
    , timer
      {
        create_wall_timer(
          std::chrono::seconds(1),
          [&]()
          {
            auto request {
              std::make_shared<AddTwoInts::Request>()
            };

            request->a = 1;
            request->b = 2;

            auto result {
              service_client->async_send_request(request)
            };

            if (rclcpp::spin_until_future_complete(shared_from_this(), result) == rclcpp::FutureReturnCode::SUCCESS)
            {
              RCLCPP_INFO(get_logger(), "response: '%ld'", result.get()->sum);
            }
          })
      }
  {
    while (not service_client->wait_for_service(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for service.");
        rclcpp::shutdown();
      }
      else
      {
        RCLCPP_INFO(get_logger(), "Waiting for service...");
      }
    }
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
