#include "tw_node_options.hpp"
#include <getopt.h>
#include <sched.h>
#include <unistd.h>
#include <iostream>

#define TRUE 1
#define FALSE 0

const int REPORT_BIN_DEFAULT = 600;
const int REPORT_ROUND_NS_DEFAULT = 1000;
const int REPORT_NUM_SKIP_DEFAULT = 10;

TwoWaysNodeOptions::TwoWaysNodeOptions(int argc, char *argv[])
    : TwoWaysNodeOptions()
{
  int c = 0;
  int _optind = optind, _opterr = opterr, _optopt = optopt;

  optind = 1;
  opterr = 0;

  // prevent permutation of argv
  const std::string optstring = "-";
  const struct option longopts[] = {
      {"ipm", no_argument, &use_intra_process_comms, TRUE},
      {"loan", no_argument, &use_loaning_, TRUE},
      {"static-executor", no_argument, &use_static_executor, TRUE},
      {"round-ns", required_argument, 0, 'i'},
      {"num-skip", required_argument, 0, 'r'},
      {"run-type", required_argument, 0, 't'},
      {"main-core", required_argument, 0, 'n'},
      {"dds-core", required_argument, 0, 'm'},
      {"ping-core", required_argument, 0, 'o'},
      {"pong-core", required_argument, 0, 'p'},
      {"main-priority", required_argument, 0, 'q'},
      {"dds-priority", required_argument, 0, 's'},
      {"ping-priority", required_argument, 0, 'u'},
      {"pong-priority", required_argument, 0, 'v'},
      {"sig-handler-core", required_argument, 0, 'w'},
      {"sig-handler-priority", required_argument, 0, 'x'},
      {"array-size", required_argument, 0, 'y'},
      {"ros-args", required_argument, 0, 'z'},
      {"bin-size", required_argument, 0, 'b'},
      {0, 0, 0, 0},
  };

  int longindex = 0;
  int tmp = -1;
  int round_ns = REPORT_ROUND_NS_DEFAULT;
  int num_skip = REPORT_NUM_SKIP_DEFAULT;
  int bin_size = REPORT_BIN_DEFAULT;
  bool needs_reinit = false;
  while ((c=getopt_long(argc, argv, optstring.c_str(), longopts, &longindex)) != -1) {
    if (c==0){
      continue;
    }
    std::string arg_str = argv[optind-2];
    if ( c == 'z' ) { // ros-args
      break;
    }

    switch (c) {
    case ('b'): // bin-size
      tmp = std::stoi(optarg);
      if (tmp > 0) {
        bin_size = tmp;
        needs_reinit = true;
      }
      break;
    case ('i'):  // round-ns
      tmp = std::stoi(optarg);
      if (tmp > 0) {
        round_ns = tmp;
        needs_reinit = true;
      }
      break;
    case ('r'): // num-skip
      tmp = std::stoi(optarg);
      if (tmp > 0) {
        num_skip = tmp;
        needs_reinit = true;
      }
      break;
    case ('t'): { // run-type
      run_type = parse_run_type(std::string(optarg));
      break;
    }
    case ('n'): { // main-core
      main_thread_.affinity_ = std::stoi(optarg);
      break;
    }
    case ('m'): { // dds-core
      dds_thread_.affinity_ = std::stoi(optarg);
      break;
    }
    case ('o'): { // ping-core
      if (run_type != T2N2) {
        std::cerr << "warn: unused option " << arg_str << std::endl;
      }
      ping_thread_.affinity_ = std::stoi(optarg);
      break;
    }
    case ('p'): { // pong-core
      if (run_type != T2N2) {
        std::cerr << "warn: unused option " << arg_str << std::endl;
      }
      pong_thread_.affinity_ = std::stoi(optarg);
      break;
    }
    case ('q'): { // main-priority
      main_thread_.priority_ = std::stoi(optarg);
      main_thread_.policy_ = real_time_policy_;
      break;
    }
    case ('s'): { // dds-priority
      dds_thread_.priority_ = std::stoi(optarg);
      dds_thread_.policy_ = real_time_policy_;
      break;
    }
    case ('u'): { // ping-priority
      if (run_type != T2N2) {
        std::cerr << "warn: unused option " << arg_str << std::endl;
      }
      ping_thread_.priority_ = std::stoi(optarg);
      ping_thread_.policy_ = real_time_policy_;
      break;
    }
    case ('v'): { // pong-priority
      if (run_type != T2N2) {
        std::cerr << "warn: unused option " << arg_str << std::endl;
      }
      pong_thread_.priority_ = std::stoi(optarg);
      pong_thread_.policy_ = real_time_policy_;
      break;
    }
    case ('w'): { // sig-handler-core
      sig_handler_thread_.affinity_ = std::stoi(optarg);
      break;
    }
    case ('x'): { // sig-handler-priority
      sig_handler_thread_.priority_ = std::stoi(optarg);
      sig_handler_thread_.policy_ = real_time_policy_;

      break;
    }
    case ('y'): { // array-size
      array_size_ = std::stoi(optarg);
      break;
    }
    default:
      std::cerr << "unknown option " << arg_str << std::endl;
      break;
    }
  }

  if(needs_reinit) {
    std::cout << "num_skip: " << num_skip << std::endl;
    init_report_option(bin_size, round_ns, num_skip);
  }

  optind = _optind;
  opterr = _opterr;
  optopt = _optopt;
}

TwoWaysNodeOptions::TwoWaysNodeOptions()
    : run_type(E1N1), real_time_policy_(SCHED_RR), main_thread_("main"),
      sig_handler_thread_("signal handler"),
      dds_thread_("dds"),
      ping_thread_("ping"),
      pong_thread_("pong"),
      use_static_executor(FALSE),
      use_loaning_(false), prefault_dynamic_size(209715200UL), // 200MB
      node_name("node"), node_name_pub("node_pub"), node_name_sub("node_sub"),
      namespace_("ns"), topic_name("ping"), topic_name_pong("pong"),
      topic_name_busy_loop("busy_loop"),
      service_name("ping"), qos(rclcpp::QoS(1).best_effort()),
      period_ns(10 * 1000 * 1000), num_loops_(10000),
      array_size_(-1),
      use_intra_process_comms(FALSE) {
  init_report_option(REPORT_BIN_DEFAULT, REPORT_ROUND_NS_DEFAULT, REPORT_NUM_SKIP_DEFAULT);
}

void TwoWaysNodeOptions::set_node_options(rclcpp::NodeOptions & node_options) const
{
  std::cout << "use_intra_process_comms = " << (use_intra_process_comms == TRUE ? "true" : "false") << std::endl;
  std::cout << "use_loaning = " << (use_loaning_ == TRUE ? "true" : "false") << std::endl;

  node_options.use_intra_process_comms(use_intra_process_comms == TRUE);
}

rclcpp::executor::Executor::SharedPtr TwoWaysNodeOptions::get_executor() {
  rclcpp::ExecutorOptions args;

  if (use_static_executor) {
    std::cout << "use StaticSingleThreadedExecutor" << std::endl;
    return std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>(args);
  }

  return std::make_shared<rclcpp::executors::SingleThreadedExecutor>(args);
}

void TwoWaysNodeOptions::init_report_option(int bin, int round_ns, int num_skip)
{
  common_report_option.bin = bin;
  common_report_option.round_ns = round_ns;
  common_report_option.num_skip = num_skip;
}

RunType TwoWaysNodeOptions::parse_run_type(const std::string &type)
{
  if(type == "1e1n") {
    return E1N1;
  } else if(type == "1e2n") {
    return E1N2;
  } else if(type == "2e_ping") {
    return E2_PING;
  } else if(type == "2e_pong") {
    return E2_PONG;
  } else if (type == "2t2n") {
    return T2N2;
  } else {
    // TODO: add 2t2n to comment
    // TODO: mod thread name to 1e1n etc.
    throw std::invalid_argument("unknown run-type: 1e1n, 1e2n, 2e_ping, 2e_pong, 2t2n");
  }
}
