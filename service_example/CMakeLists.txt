cmake_minimum_required(VERSION 3.5)

project(service_example)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Tutorials of Request/Response with Services
ament_auto_add_executable(add_two_ints_client
  src/services/add_two_ints_client.cpp)

ament_auto_add_executable(bad_client
  src/services/bad_client.cpp)

ament_auto_add_executable(good_client_no_timer
  src/services/good_client_no_timer.cpp)

ament_auto_add_library(services_library SHARED
  src/services/add_two_ints_server.cpp
  src/services/add_two_ints_client.cpp
  src/services/add_two_ints_client_async.cpp
  src/services/proposal/client.cpp)

rclcpp_components_register_node(services_library
  PLUGIN "service_example::ServerNode"
  EXECUTABLE add_two_ints_server)

rclcpp_components_register_node(services_library
  PLUGIN "service_example::ClientNode"
  EXECUTABLE add_two_ints_client_async)

rclcpp_components_register_node(services_library
  PLUGIN "proposal::ClientNode"
  EXECUTABLE proposal_client)

# rclcpp_components_register_node() uses SingleThreadedExecutor and cannot be used.
# Intead, use MultiThreadedExecutor. See relay_server_mani.cpp
# https://qiita.com/NeK/items/4ed4c46c1087d1089b57
ament_auto_add_executable(proposal_relay_server
  src/services/proposal/relay_server.cpp
  src/services/proposal/relay_server_main.cpp)

# Install launch files.
ament_auto_package(INSTALL_TO_SHARE launch)
