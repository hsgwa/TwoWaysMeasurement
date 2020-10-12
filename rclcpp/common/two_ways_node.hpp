#ifndef TWO_WAYS_NODE_HPP_
#define TWO_WAYS_NODE_HPP_

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>
#include "twmsgs/msg/data.hpp"
#include "tw_node_options.hpp"
#include "tw_utils.hpp"

class TwoWaysNode : public rclcpp::Node
{
public:
  // explicit TwoWaysNode(
  //     const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  explicit TwoWaysNode(
      const std::string name,
      const std::string namespace_,
      const TwoWaysNodeOptions & tw_options = TwoWaysNodeOptions(),
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  virtual ~TwoWaysNode()
  {}

  void print_ping_wakeup_report() {
    ping_wakeup_report_.print("ping_wakeup");
    std::cout << "ping_deadline_count: " << ping_deadline_count << std::endl << std::endl;
  }
  void print_diff_wakeup_report() {
    diff_wakeup_report_.print("diff_wakeup");
  }
  void print_ping_sub_report() {
    ping_sub_report_.print("ping_sub");
    std::cout << "ping_drop: " << ping_drop << std::endl;
    std::cout << "ping_drop_gap_: " << ping_drop_gap_ << std::endl;
    std::cout << "ping_argdrop: < " << ping_argdrop_ << std::endl;
    std::cout << "ping_late: " << ping_late << std::endl;
    std::cout << "ping_sub_count_: " << ping_sub_count_ << std::endl;
    std::cout << "ping_argmax: " << ping_argmax_ << std::endl << std::endl;
  }
  void print_pong_sub_report() {
    pong_sub_report_.print("pong_sub");
    std::cout << "pong_drop: " << pong_drop << std::endl;
    std::cout << "pong_drop_gap_: " << pong_drop_gap_ << std::endl;
    std::cout << "pong_argdrop: " << pong_argdrop_ << std::endl;
    std::cout << "pong_late: " << pong_late << std::endl;
    std::cout << "pong_sub_count_: " << pong_sub_count_ << std::endl;
    std::cout << "pong_argmax: " << pong_argmax_ << std::endl << std::endl;
    std::cout << "pong_deadline_count: " << pong_deadline_count << std::endl << std::endl;
  }
  void print_ping_pong_report() {
    ping_pong_report_.print("ping_pong");
  }
  void print_timer_callback_process_time_report() {
    timer_callback_process_time_report_.print("timer_callback");
  }
  void print_ping_callback_process_time_report() {
    ping_callback_process_time_report_.print("ping_callback");
  }
  void print_pong_callback_process_time_report(){
    pong_callback_process_time_report_.print("pong_callback");
  }

  void setup_ping_publisher();
  void setup_ping_subscriber(bool send_pong=false);
  void setup_pong_subscriber();

protected:
  const TwoWaysNodeOptions & tw_options_;

private:
  // absolute time of timer wake up.
  int64_t time_next_wake_;

  // number of ping publish
  int ping_pub_count_;
  // number of ping subscribe
  int ping_sub_count_;
  // number of pong publish
  int pong_pub_count_;
  // number of pong subscribe
  int pong_sub_count_;

  twmsgs::msg::Data msg_;

  struct timespec epoch_ts_;
  struct timespec period_ts_;
  struct timespec expect_ts_;
  struct timespec last_wake_ts_;

  rclcpp::TimerBase::SharedPtr ping_timer_;
  rclcpp::Publisher<twmsgs::msg::Data>::SharedPtr ping_pub_;
  rclcpp::Subscription<twmsgs::msg::Data>::SharedPtr ping_sub_;
  rclcpp::Publisher<twmsgs::msg::Data>::SharedPtr pong_pub_;
  rclcpp::Subscription<twmsgs::msg::Data>::SharedPtr pong_sub_;

  bool send_pong_;

  // how many times ping drop
  uint64_t ping_drop;
  uint64_t ping_drop_gap_;
  uint64_t ping_argmax_;
  uint64_t ping_argdrop_;
  // how many times ping late
  uint64_t ping_late;
  // how many times pong drop
  uint64_t pong_drop;
  uint64_t pong_drop_gap_;
  uint64_t pong_argmax_;
  uint64_t pong_argdrop_;
  // how many times pong late
  uint64_t pong_late;

  uint64_t ping_deadline_count; // ping publisher deadline
  uint64_t pong_deadline_count; // pong subscriber deadline

  // wakeup jitter report
  JitterReportWithSkip ping_wakeup_report_;
  // wakeup jitter from last wakeup
  JitterReportWithSkip diff_wakeup_report_;
  // sub jitter report
  JitterReportWithSkip ping_sub_report_;
  // pong sub jitter report
  JitterReportWithSkip pong_sub_report_;
  // ping-pong jitter report
  JitterReportWithSkip ping_pong_report_;
  // timer callback process time report
  JitterReportWithSkip timer_callback_process_time_report_;
  // ping callback process time report
  JitterReportWithSkip ping_callback_process_time_report_;
  // pong callback process time report
  JitterReportWithSkip pong_callback_process_time_report_;

  int64_t get_now_int64();
};

#endif  // TWO_WAYS_NODE_HPP_
