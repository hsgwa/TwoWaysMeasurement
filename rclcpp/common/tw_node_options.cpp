#include <unistd.h>
#include <getopt.h>

#include <rclcpp/strategies/allocator_memory_strategy.hpp>
#include "tw_node_options.hpp"

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
      {"sched-rrts", no_argument, &sched_rrts, TRUE},
      {"sched-rrrr", no_argument, &sched_rrrr, TRUE},
      {"ipm", no_argument, &use_intra_process_comms, TRUE},
      {"loan", no_argument, &use_loaning_, TRUE},
      {"round-ns", required_argument, 0, 'i'},
      {"num-skip", required_argument, 0, 'r'},
      {"static-executor", no_argument, &use_static_executor, TRUE},
      {"main-sched", required_argument, 0, 'm'},
      {"child-sched", required_argument, 0, 'c'},
      {"run-type", required_argument, 0, 't'},
      {"default-memory-strategy", no_argument, 0, 's'},
      {0, 0, 0, 0},
  };

  int longindex = 0;
  int tmp = -1;
  int round_ns = REPORT_ROUND_NS_DEFAULT;
  int num_skip = REPORT_NUM_SKIP_DEFAULT;
  bool needs_reinit = false;
  while ((c=getopt_long(argc, argv, optstring.c_str(), longopts, &longindex)) != -1) {
    switch(c)
    {
      case('i'):
        tmp = std::stoi(optarg);
        if(tmp > 0) {
          round_ns = tmp;
          needs_reinit = true;
        }
        break;
      case('r'):
        tmp = std::stoi(optarg);
        if(tmp > 0) {
          num_skip = tmp;
          needs_reinit = true;
        }
        break;
      case('m'): {
        main_sched = get_schedule_policy(std::string(optarg));
        break;
      }
      case('c'): {
        child_sched = get_schedule_policy(std::string(optarg));
        break;
      }
      case('t'): {
        run_type = parse_run_type(std::string(optarg));
        break;
      }
      default:
        std::cerr << "unknown option " << c << std::endl;
        break;
    }
  }

  if(needs_reinit) {
    std::cout << "num_skip: " << num_skip << std::endl;
    init_report_option(REPORT_BIN_DEFAULT, round_ns, num_skip);
  }

  optind = _optind;
  opterr = _opterr;
  optopt = _optopt;
}

TwoWaysNodeOptions::TwoWaysNodeOptions():
    sched_rrts(0),
    sched_rrrr(0),
    main_sched(SCHED_POLICY::TS),
    child_sched(SCHED_POLICY::TS),
    sched_priority(98),
    sched_policy(SCHED_RR),
    run_type(E1N1),
    use_static_executor(FALSE),
    prefault_dynamic_size(209715200UL),  // 200MB
    node_name("node"),
    node_name_pub("node_pub"),
    node_name_sub("node_sub"),
    namespace_("ns"),
    topic_name("ping"),
    topic_name_pong("pong"),
    service_name("ping"),
    qos(rclcpp::QoS(1).best_effort()),
    period_ns(10 * 1000 * 1000),
    num_loops_(10000),
    use_loaning_(false),
    use_intra_process_comms(FALSE)
{
  init_report_option(REPORT_BIN_DEFAULT, REPORT_ROUND_NS_DEFAULT, REPORT_NUM_SKIP_DEFAULT);
}

void TwoWaysNodeOptions::set_node_options(rclcpp::NodeOptions & node_options) const
{
  std::cout << "use_intra_process_comms = " << (use_intra_process_comms == TRUE ? "true" : "false") << std::endl;

  std::cout << "use_loaning = "
            << (use_loaning_ == TRUE ? "true" : "false")
            << std::endl;
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

void TwoWaysNodeOptions::get_sched(SCHED_POLICY sp, size_t &priority, int &policy)
{

  switch(sp)
  {
    case(TS): {
      priority = 0;
      policy = SCHED_OTHER;
      break;
    }
    case(RR98): {
      priority = 98;
      policy = SCHED_RR;
      break;
    }
    case(RR97): {
      priority = 97;
      policy = SCHED_RR;
      break;
    }
  }
}

void TwoWaysNodeOptions::init_report_option(int bin, int round_ns, int num_skip)
{
  common_report_option.bin = bin;
  common_report_option.round_ns = round_ns;
  common_report_option.num_skip = num_skip;
}

SCHED_POLICY TwoWaysNodeOptions::get_schedule_policy(const std::string &opt)
{
  if(opt == "RR98") {
    return SCHED_POLICY::RR98;
  } else if(opt == "RR97") {
    return SCHED_POLICY::RR97;
  } else if(opt == "TS") {
    return SCHED_POLICY::TS;
  } else {
    throw std::invalid_argument("unknown policy: use RR98, RR97, TS");
  }
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
    throw std::invalid_argument("unknown run-type: 1e1n, 1e2n, 2e_ping, 2e_pong");
  }
}
