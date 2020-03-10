#include <chrono>
#include <memory>
#include <time.h>
#include <vector>
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "twmsgs/msg/data.hpp"
#include "../common/SubNode.hpp"
#include "../common/tw_utils.hpp"

int main(int argc, char *argv[])
{
  std::cout << "Press C-c to quit" << std::endl;

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  TwoWaysNodeOptions tw_options;
  rclcpp::NodeOptions node_options;
  // TODO: node_options.allocator = ...;
  // TODO: node_options.use_intra_process_comms = ...;
  auto nsub = std::make_shared<SubNode>(tw_options, node_options);

  exec.add_node(nsub);
  exec.spin();
  exec.remove_node(nsub);

  print_result("recv_jitters", nsub->ping_sub_jitters_, nsub->ping_sub_count_, tw_options.num_bin);

  rclcpp::shutdown();
}