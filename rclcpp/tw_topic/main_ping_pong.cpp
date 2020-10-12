#include <chrono>
#include <memory>
#include <time.h>
#include <vector>
#include <numeric>
#include <thread>
#include <sched.h>
#include <sys/resource.h>

#include "../common/tw_utils.hpp"
#include "../common/two_ways_node.hpp"
#include "rclcpp/rclcpp.hpp"

const char * node_name = "one_node_ping_pong";

class Runner
{
public:
  Runner(rclcpp::executor::Executor::SharedPtr e):
      exec_(e) {}

  virtual void setup(const TwoWaysNodeOptions &tw_options) = 0;
  virtual void cleanup() = 0;
  virtual void report() = 0;

protected:
  rclcpp::executor::Executor::SharedPtr exec_;
};

class Runner_1e1n : public Runner {
public:
  Runner_1e1n(rclcpp::executor::Executor::SharedPtr e)
      : Runner(e) {
    std::cout << "runner = 1e1n" << std::endl;
  }

  void setup(const TwoWaysNodeOptions &tw_options) override {
    rclcpp::NodeOptions node_options;
    tw_options.set_node_options(node_options);
    n_ = std::make_shared<TwoWaysNode>("node_1e1n", "ns", tw_options, node_options);
    n_->setup_ping_publisher();
    n_->setup_ping_subscriber(true);
    n_->setup_pong_subscriber();

    exec_->add_node(n_);
  }

  void cleanup() override {
    exec_->remove_node(n_);
  }

  void report() override {
    n_->print_ping_wakeup_report();
    n_->print_diff_wakeup_report();
    n_->print_ping_sub_report();
    n_->print_pong_sub_report();
    n_->print_ping_pong_report();
    n_->print_timer_callback_process_time_report();
    n_->print_ping_callback_process_time_report();
    n_->print_pong_callback_process_time_report();
  }

private:
  std::shared_ptr<TwoWaysNode> n_;
};

class Runner_1e2n : public Runner {
public:
  Runner_1e2n(rclcpp::executor::Executor::SharedPtr e)
      : Runner(e) {
    std::cout << "runner = 1e2n" << std::endl;
  }

  void setup(const TwoWaysNodeOptions &tw_options) override {
    rclcpp::NodeOptions node_options;
    tw_options.set_node_options(node_options);
    npub_ = std::make_shared<TwoWaysNode>("pub", "ns", tw_options, node_options);
    nsub_ = std::make_shared<TwoWaysNode>("sub", "ns", tw_options, node_options);

    npub_->setup_ping_publisher();
    npub_->setup_pong_subscriber();
    nsub_->setup_ping_subscriber(true);

    exec_->add_node(npub_);
    exec_->add_node(nsub_);
  }

  void cleanup() override {
    exec_->remove_node(nsub_);
    exec_->remove_node(npub_);
  }

  void report() override {
     npub_->print_ping_wakeup_report();
     npub_->print_diff_wakeup_report();
     nsub_->print_ping_sub_report();
     npub_->print_pong_sub_report();
     npub_->print_ping_pong_report();
     npub_->print_timer_callback_process_time_report();
     nsub_->print_ping_callback_process_time_report();
     npub_->print_pong_callback_process_time_report();
  }

private:
  std::shared_ptr<TwoWaysNode> npub_, nsub_;
};

class Runner_2e_ping : public Runner {
public:
  Runner_2e_ping(rclcpp::executor::Executor::SharedPtr e)
      : Runner(e) {
    std::cout << "runner = 1e2n_ping" << std::endl;
  }

  void setup(const TwoWaysNodeOptions &tw_options) override {
    rclcpp::NodeOptions node_options;
    tw_options.set_node_options(node_options);
    npub_ = std::make_shared<TwoWaysNode>("main_pub_ping_pong", "ns", tw_options, node_options);

    npub_->setup_ping_publisher();
    npub_->setup_pong_subscriber();
    exec_->add_node(npub_);
  }

  void cleanup() override {
    exec_->remove_node(npub_);
  }

  void report() override {
    npub_->print_ping_wakeup_report();
    npub_->print_diff_wakeup_report();
    npub_->print_pong_sub_report();
    npub_->print_ping_pong_report();
    npub_->print_timer_callback_process_time_report();
    npub_->print_pong_callback_process_time_report();
  }

private:
  std::shared_ptr<TwoWaysNode> npub_;
};

class Runner_2e_pong : public Runner
{
public:
  Runner_2e_pong(rclcpp::executor::Executor::SharedPtr e):
      Runner(e) {
      std::cout << "runner = 1e2n_pong" << std::endl;
  }

  void setup(const TwoWaysNodeOptions &tw_options) override {
    rclcpp::NodeOptions node_options;
    tw_options.set_node_options(node_options);
    nsub_ = std::make_shared<TwoWaysNode>("main_sub_ping_pong", "ns", tw_options, node_options);

    nsub_->setup_ping_subscriber(true);
    exec_->add_node(nsub_);
  }

  void cleanup() override {
    exec_->remove_node(nsub_);
  }

  void report() override {
    nsub_->print_ping_sub_report();
    nsub_->print_ping_callback_process_time_report();
  }

private:
  std::shared_ptr<TwoWaysNode> nsub_;
};

std::unique_ptr<Runner> make_runner(RunType type,
                                    rclcpp::executor::Executor::SharedPtr e) {
  std::unique_ptr<Runner> p(nullptr);
  switch(type) {
    case(E1N1): {
      p.reset(new Runner_1e1n(e));
      break;
    }
    case(E1N2): {
      p.reset(new Runner_1e2n(e));
      break;
    }
    case(E2_PING): {
      p.reset(new Runner_2e_ping(e));
      break;
    }
    case(E2_PONG): {
      p.reset(new Runner_2e_pong(e));
      break;
    }
    default: {
      // TODO: add T2N2 comment
      break;
    }
  }

  return p;
}

class RusageCounter {
 public:
  void start() {
    update(&start_);
  }
  void end() {
    update(&end_);
  }
  void report() {
    std::cout << "major pagefault : " << end_.ru_majflt - start_.ru_majflt << std::endl;
    std::cout << "minor pagefault : " << end_.ru_minflt - start_.ru_minflt << std::endl;
    std::cout << "swap : " << end_.ru_nswap - start_.ru_nswap << std::endl;
    std::cout << "voluntary context switch : " << end_.ru_nvcsw - start_.ru_nvcsw << std::endl;
    std::cout << "involuntary context switch : " << end_.ru_nivcsw - start_.ru_nivcsw << std::endl;
  }

 private:
  void update(rusage *usage) {
    int result = getrusage(RUSAGE_SELF, usage);
    if (result != 0) {
      std::cerr << "failed to set affinity" << std::endl;
    }
  }
  struct rusage start_;
  struct rusage end_;
};

class BusyLoopNode : public rclcpp::Node
{
 public:
  BusyLoopNode(std::string namespace_)
      :Node(namespace_, rclcpp::NodeOptions().use_intra_process_comms(true))
            ,qos(rclcpp::QoS(1).best_effort())
  {
    std::string topic_name_busy_loop = "busy_loop";
    auto callback = [](std_msgs::msg::UInt64::UniquePtr msg) {
      uint64_t i;
      for (i = 0; i < msg->data; i++)
        ;
    };

    rclcpp::NodeOptions node_options;
    node_options.use_intra_process_comms(true);
    busy_sub_ = create_subscription<std_msgs::msg::UInt64>(topic_name_busy_loop,
                                                           qos, callback);
  }

 private:
   rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr busy_sub_;
   rclcpp::QoS qos;
};

int main(int argc, char *argv[])
{
  std::cout << "Press C-c to quit" << std::endl;

  TwoWaysNodeOptions tw_options(argc, argv);
  tw_options.main_thread_.report();
  tw_options.sig_handler_thread_.report();
  tw_options.dds_thread_.report();
  if (tw_options.run_type == T2N2 ) {
    tw_options.ping_thread_.report();
    tw_options.pong_thread_.report();
  }

  if(lock_and_prefault_dynamic(tw_options.prefault_dynamic_size) < 0) {
    std::cerr << "lock_and_prefault_dynamic failed" << std::endl;
  }

  tw_options.sig_handler_thread_.setup(pthread_self());
  rclcpp::init(argc, argv);

  RusageCounter rusageCounter;

  tw_options.busy_thread_.setup(pthread_self());
  auto busy_exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  busy_exec->add_node(std::make_shared<BusyLoopNode>("ns"));
  std::shared_ptr<std::thread> busy_thread;
  if (tw_options.run_type != E2_PONG) {
    busy_thread = std::make_shared<std::thread>(&rclcpp::Executor::spin, busy_exec);
  }

  if(tw_options.run_type != T2N2) {
    // setup dds process scheule

    tw_options.dds_thread_.setup(pthread_self());
    auto exec = tw_options.get_executor();
    auto runner = make_runner(tw_options.run_type, exec);
    runner->setup(tw_options);

    // setup main process scheule
    tw_options.main_thread_.setup(pthread_self());

    rusageCounter.start();
    exec->spin();
    rusageCounter.end();

    runner->cleanup();
    runner->report();
    rusageCounter.report();

  } else {
    tw_options.dds_thread_.setup(pthread_self());

    tw_options.run_type = E2_PING;
    auto exec_ping = tw_options.get_executor();
    auto runner_ping = make_runner(tw_options.run_type, exec_ping);
    runner_ping->setup(tw_options);

    tw_options.run_type = E2_PONG;
    auto exec_pong = tw_options.get_executor();
    auto runner_pong = make_runner(tw_options.run_type, exec_pong);
    runner_pong->setup(tw_options);

    tw_options.main_thread_.setup(pthread_self());

    std::thread th_ping(&rclcpp::Executor::spin, exec_ping);
    std::thread th_pong(&rclcpp::Executor::spin, exec_pong);
    // setup executor thread's schedule>

    tw_options.ping_thread_.setup(th_ping.native_handle());
    tw_options.pong_thread_.setup(th_pong.native_handle());

    rusageCounter.start();

    th_ping.join();
    th_pong.join();

    rusageCounter.end();

    runner_ping->cleanup();
    runner_ping->report();
    runner_pong->cleanup();
    runner_pong->report();

    rusageCounter.report();
  }
  if (busy_thread) {
    busy_thread->join();
  }

  rclcpp::shutdown();
}
