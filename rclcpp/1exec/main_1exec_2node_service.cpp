#include <rclcpp/rclcpp.hpp>

#include "../common/tw_node_options.hpp"
#include "../common/tw_utils.hpp"
#include "../common/two_ways_service_node.hpp"

int main(int argc, char *argv[])
{
  std::cout << "Press C-c to quit" << std::endl;
  rclcpp::init(argc, argv);

  TwoWaysNodeOptions tw_options;
  auto exec = tw_options.get_executor();

  rclcpp::NodeOptions node_options;
  tw_options.set_node_options(node_options);

  auto client_node = std::make_shared<TwoWaysServiceNode>(tw_options.node_name_pub, tw_options, node_options);
  auto service_node = std::make_shared<TwoWaysServiceNode>(tw_options.node_name_sub, tw_options, node_options);
  if (!service_node->setup_ping_service()) {
     std::cerr << "cannot setup service" << std::endl;
  }
  if (!client_node->setup_ping_client()) {
    std::cerr << "cannot setup client" << std::endl;
  }

  exec->add_node(service_node);
  exec->add_node(client_node);
  exec->spin();
  exec->remove_node(client_node);
  exec->remove_node(service_node);
}