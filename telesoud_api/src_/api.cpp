#include "../include/api_node.hpp"

int main(int argc, char *argv[]) {
  // Initialize ROS api_node
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor::SharedPtr exec = rclcpp::executors::MultiThreadedExecutor::make_shared();

  auto apiNode = std::make_shared<ApiNode>();

  exec->add_node(apiNode);
  while (rclcpp::ok()) {
    exec->spin();
  }
  exec->remove_node(apiNode);
  rclcpp::shutdown();
  return 0;
}
