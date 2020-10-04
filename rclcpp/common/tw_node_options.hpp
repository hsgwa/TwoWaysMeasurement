#ifndef SETTING_H_
#define SETTING_H_

#include <sched.h>
#include <string>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include "thread_options.hpp"

struct JitterReportOptions
{
  int bin;
  int round_ns;
  int num_skip;
};

// TODO unify option parser. See TODO in SCHED_POLICY.
enum RunType {
  E1N1,         // 1 executor, 1 node
  E1N2,         // 1 executor, 2 nodes
  E2_PING,      // 2 executor, ping
  E2_PONG,      // 2 executor, pong
  T2N2,         // 2 executor, 2 nodes 2 thread
};

class TwoWaysNodeOptions {
public:
  TwoWaysNodeOptions();

  TwoWaysNodeOptions(int argc, char *argv[]);

  /// Init NodeOptions
  void set_node_options(rclcpp::NodeOptions & node_options) const;

  /// Get Executor
  rclcpp::executor::Executor::SharedPtr get_executor();

  // run type
  RunType run_type;

  // scheduler
  int real_time_policy_;

  ThreadOptions main_thread_;
  ThreadOptions sig_handler_thread_;
  ThreadOptions dds_thread_;
  ThreadOptions ping_thread_;
  ThreadOptions pong_thread_;

  // executor
  int use_static_executor;
  int use_loaning_;

  // prefault
  size_t prefault_dynamic_size;

  // node, topic, qos
  std::string node_name;
  std::string node_name_pub;
  std::string node_name_sub;
  std::string namespace_;
  std::string topic_name;
  std::string topic_name_pong;
  std::string service_name;
  rclcpp::QoS qos;

  // wake up period[ns]
  const int period_ns;
  JitterReportOptions common_report_option;

  /////// test conditions
  // number of loops
  const int num_loops_;

  int array_size_;
  // Options for rclcpp::Node
  int use_intra_process_comms; // 0: false, 1: true

private:
  void init_report_option(int bin, int round_ns, int num_skip);
  RunType parse_run_type(const std::string &type);
};

#endif  /* SETTING_H_ */
