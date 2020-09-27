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
#include "osrf_testing_tools_cpp/memory_tools/memory_tools.hpp"
#include "osrf_testing_tools_cpp/scope_exit.hpp"
#include "rclcpp/rclcpp.hpp"
#include <dlfcn.h>

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

class Runner_1e1n : public Runner
{
public:
  Runner_1e1n(rclcpp::executor::Executor::SharedPtr e):
      Runner(e) {
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

class Runner_1e2n : public Runner
{
public:
  Runner_1e2n(rclcpp::executor::Executor::SharedPtr e):
      Runner(e) {
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

class Runner_2e_ping : public Runner
{
public:
  Runner_2e_ping(rclcpp::executor::Executor::SharedPtr e):
      Runner(e) {
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

std::unique_ptr<Runner>
make_runner(RunType type, rclcpp::executor::Executor::SharedPtr e)
{
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

void set_affinity(const pthread_t &thread, int core_num) {
  cpu_set_t cpu_set;

  CPU_ZERO(&cpu_set);
  CPU_SET(core_num, &cpu_set);

  int result = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpu_set);
  if (result != 0) {
    std::cout << "failed to set affinity" << std::endl;
  }
}

int main(int argc, char *argv[])
{
  std::cout << "Press C-c to quit" << std::endl;

  TwoWaysNodeOptions tw_options(argc, argv);

  if(lock_and_prefault_dynamic(tw_options.prefault_dynamic_size) < 0) {
    std::cerr << "lock_and_prefault_dynamic failed" << std::endl;
  }

  // setup child process scheule
  size_t priority = 0;
  int policy = 0;
  tw_options.get_child_thread_policy(priority, policy);
  set_sched_priority("child", priority, policy);

  set_affinity(pthread_self(), 1);

  rclcpp::init(argc, argv);


  osrf_testing_tools_cpp::memory_tools::initialize();
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT({
    osrf_testing_tools_cpp::memory_tools::uninitialize();
  });

  osrf_testing_tools_cpp::memory_tools::enable_monitoring();
  // TODO: add assert
  // ASSERT_TRUE(osrf_testing_tools_cpp::memory_tools::is_working());

  auto on_unexpected_memory =
    [](osrf_testing_tools_cpp::memory_tools::MemoryToolsService & service) {
      // ADD_FAILURE() << "unexpected malloc";
      service.print_backtrace();
    };

  osrf_testing_tools_cpp::memory_tools::on_unexpected_calloc(on_unexpected_memory);
  osrf_testing_tools_cpp::memory_tools::on_unexpected_free(on_unexpected_memory);
  osrf_testing_tools_cpp::memory_tools::on_unexpected_malloc(on_unexpected_memory);
  osrf_testing_tools_cpp::memory_tools::on_unexpected_realloc(on_unexpected_memory);

  RusageCounter rusageCounter;

  if(tw_options.run_type != T2N2) {
    auto exec = tw_options.get_executor();

    auto runner = make_runner(tw_options.run_type, exec);
    runner->setup(tw_options);

    // setup child process scheule
    tw_options.get_main_thread_policy(priority, policy);
    set_sched_priority("main", priority, policy);

    rusageCounter.start();
    osrf_testing_tools_cpp::memory_tools::expect_no_malloc_begin();
    exec->spin();
    osrf_testing_tools_cpp::memory_tools:: expect_no_malloc_end();
    rusageCounter.end();

    runner->cleanup();
    runner->report();
    rusageCounter.report();

  } else {
    tw_options.run_type = E2_PING;
    auto exec_ping = tw_options.get_executor();
    auto runner_ping = make_runner(tw_options.run_type, exec_ping);
    runner_ping->setup(tw_options);

    tw_options.run_type = E2_PONG;
    auto exec_pong = tw_options.get_executor();
    auto runner_pong = make_runner(tw_options.run_type, exec_pong);
    runner_pong->setup(tw_options);

    // setup child process scheule
    tw_options.get_main_thread_policy(priority, policy);
    set_sched_priority("main", priority, policy);

    std::thread th_ping(&rclcpp::Executor::spin, exec_ping);
    std::thread th_pong(&rclcpp::Executor::spin, exec_pong);
    set_affinity(th_ping.native_handle(), 2);
    set_affinity(th_pong.native_handle(), 2);
    rusageCounter.start();

    osrf_testing_tools_cpp::memory_tools::expect_no_malloc_begin();
    th_ping.join();
    th_pong.join();
    osrf_testing_tools_cpp::memory_tools::expect_no_malloc_end();

    rusageCounter.end();

    runner_ping->cleanup();
    runner_ping->report();
    runner_pong->cleanup();
    runner_pong->report();

    rusageCounter.report();
  }

  rclcpp::shutdown();
}
