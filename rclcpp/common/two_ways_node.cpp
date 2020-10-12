#include "two_ways_node.hpp"
#include <rclcpp/strategies/message_pool_memory_strategy.hpp>
#include <rttest/utils.hpp>

using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;


const std::string PERIOD_NS = "period_ns";
const std::string DL_PERIOD_NS = "dl_period_ns";
const std::string NUM_LOOPS = "num_loops";
const std::string DEBUG_PRINT = "debug_print";
const std::string BUSY_LOOPS = "busy_loops";

TwoWaysNode::TwoWaysNode(
    const std::string name,
    const std::string namespace_,
    const TwoWaysNodeOptions & tw_options,
    const rclcpp::NodeOptions & options)
    : Node(name, namespace_, options),
      tw_options_(tw_options),
      time_next_wake_(0),
      ping_pub_count_(0), ping_sub_count_(0),
      pong_pub_count_(0), pong_sub_count_(0),
      send_pong_(false),
      ping_drop(0), ping_drop_gap_(0), ping_argdrop_(0), ping_late(0),
      pong_drop(0), pong_drop_gap_(0), pong_argdrop_(0), pong_late(0),
      ping_deadline_count(0), pong_deadline_count(0)
{
  declare_parameter(PERIOD_NS, 10 * 1000 * 1000);
  declare_parameter(DL_PERIOD_NS, 10 * 1000 * 1000);
  declare_parameter(NUM_LOOPS, 10000);
  declare_parameter(DEBUG_PRINT, false);
  declare_parameter(BUSY_LOOPS, 0);

  // setup reports
  JitterReportWithSkip* reports[] = {
      &ping_wakeup_report_,
      &ping_sub_report_,
      &pong_sub_report_,
      &ping_pong_report_,
      &timer_callback_process_time_report_,
      &ping_callback_process_time_report_,
      &pong_callback_process_time_report_,
  };
  for(auto r : reports) {
    r->init(tw_options.common_report_option.bin,
            tw_options.common_report_option.round_ns,
            tw_options.common_report_option.num_skip);
  }
  diff_wakeup_report_.init(
      tw_options.common_report_option.bin,
      tw_options.common_report_option.round_ns,
      tw_options.common_report_option.num_skip,
      - tw_options.common_report_option.bin/2 * tw_options.common_report_option.round_ns);
}

void TwoWaysNode::setup_ping_publisher()
{
  auto topic_name = this->tw_options_.topic_name;
  auto qos = this->tw_options_.qos;
  auto topic_name_busy_loop = tw_options_.topic_name_busy_loop;
  auto period_ns = get_parameter(PERIOD_NS).get_value<int>();
  auto dl_period_ns = get_parameter(DL_PERIOD_NS).get_value<int>();
  auto num_loops = get_parameter(NUM_LOOPS).get_value<int>();
  auto debug_print = get_parameter(DEBUG_PRINT).get_value<bool>();
  auto busy_loops = get_parameter(BUSY_LOOPS).get_value<int>();

  std::cout << PERIOD_NS   << ": " << period_ns << std::endl;
  std::cout << DL_PERIOD_NS << ": " << dl_period_ns << std::endl;
  std::cout << NUM_LOOPS << ": " << num_loops << std::endl;
  std::cout << DEBUG_PRINT << ": " << (debug_print ? "true" : "false") << std::endl;

  rclcpp::PublisherOptions options;
  qos.deadline(rclcpp::Duration(std::chrono::nanoseconds(dl_period_ns)));
  options.event_callbacks.deadline_callback =
      [this](rclcpp::QOSDeadlineOfferedInfo &event) {
        (void) event;
        this->ping_deadline_count++;
      };

  ping_pub_ = create_publisher<twmsgs::msg::Data>(topic_name, qos, options);
  busy_pub_ = create_publisher<std_msgs::msg::UInt64>(topic_name_busy_loop, qos);

  auto callback_timer =
      [this, period_ns, num_loops, debug_print, busy_loops]() -> void
      {
        struct timespec time_wake_ts;
        getnow(&time_wake_ts);

        int64_t wakeup_;
        // calc wakeup jitter
        if (rcl_timer_get_now(&(*this->ping_timer_->get_timer_handle()),
                              &wakeup_) != RCL_RET_OK) {
          return;
        }

        auto latency = wakeup_ - time_next_wake_;
        if (ping_pub_count_ > 0) { // ignore first wakeup because time_next_call_ is zero initialized.
          ping_wakeup_report_.add(latency);
        }

        // calc difference from last callback
        struct timespec diff_from_last_wakeup_ts;
        subtract_timespecs(&time_wake_ts, &last_wake_ts_, &diff_from_last_wakeup_ts);
        // minus period because distribution mean is period_ns.
        subtract_timespecs(&diff_from_last_wakeup_ts, &period_ts_, &diff_from_last_wakeup_ts);
        diff_wakeup_report_.add(_timespec_to_uint64(&diff_from_last_wakeup_ts));

        // prepere to next
        add_timespecs(&epoch_ts_, &period_ts_, &expect_ts_);
        ping_pub_count_++;
        last_wake_ts_ = time_wake_ts;


        auto msg_busy = std::make_unique<std_msgs::msg::UInt64>();

        if (this->tw_options_.use_loaning_) {
          auto msg = ping_pub_->borrow_loaned_message();
          msg.get().data = ping_pub_count_;
          msg.get().time_sent_ns = get_now_int64();
          this->ping_pub_->publish(std::move(msg));
        } else {
          auto msg = std::make_unique<twmsgs::msg::Data>();
          msg->data = ping_pub_count_;

#ifdef USE_BOUNDED
      if (this->tw_options_.array_size_ > 0) {
        msg->image.resize(this->tw_options_.array_size_); // TODO move to initial phaze
      }
#endif
          msg->time_sent_ns = get_now_int64();
          this->ping_pub_->publish(std::move(msg));
        }

        if(debug_print) {
          struct timespec time_print;
          getnow(&time_print);
          std::cout << "time_print.tv_sec: " << time_print.tv_sec << " "
                    << "time_print.tv_nsec: " << time_print.tv_nsec << std::endl;
          std::cout << "sent ping id = " << ping_pub_count_
                    << " @" << _timespec_to_uint64(&time_print)
                    << " wake-up @ " << latency
                    << std::endl;
        }

        struct timespec time_exit;
        getnow(&time_exit);
        subtract_timespecs(&time_exit, &time_wake_ts, &time_exit);
        timer_callback_process_time_report_.add(_timespec_to_uint64(&time_exit));

        if (rcl_timer_get_time_next_call(&(*this->ping_timer_->get_timer_handle()), &time_next_wake_) !=
            RCL_RET_OK) {
          std::cerr << "failed to get next call time" << std::endl;
        };

        if(ping_pub_count_ == num_loops) {
          std::raise(SIGINT);
        }
    if (busy_loops > 0) {
      msg_busy->data = busy_loops;
      this->busy_pub_->publish(std::move(msg_busy));
    }
  };

  // set timer
  getnow(&epoch_ts_);
  period_ts_.tv_sec = 0;
  period_ts_.tv_nsec = tw_options_.period_ns;
  add_timespecs(&epoch_ts_, &period_ts_, &expect_ts_);
  last_wake_ts_ = epoch_ts_;
  this->ping_timer_ = this->create_wall_timer(
      std::chrono::nanoseconds(period_ns), callback_timer);

}

void TwoWaysNode::setup_ping_subscriber(bool send_pong)
{
  auto topic_name = tw_options_.topic_name;
  auto topic_name_pong = tw_options_.topic_name_pong;
  auto qos = tw_options_.qos;
  auto debug_print = get_parameter(DEBUG_PRINT).get_value<bool>();
  auto dl_period_ns = get_parameter(DL_PERIOD_NS).get_value<int>();

  // pong publisher
  send_pong_ = send_pong;
  if (send_pong) {
    rclcpp::PublisherOptions options;
    qos.deadline(rclcpp::Duration(std::chrono::nanoseconds(dl_period_ns)));
    options.event_callbacks.deadline_callback =
        [](rclcpp::QOSDeadlineOfferedInfo &event) {(void)event;};
    pong_pub_ = create_publisher<twmsgs::msg::Data>(topic_name_pong, qos, options);
  }

  // subscriber callback
  auto callback_sub =
  [this, debug_print](twmsgs::msg::Data::UniquePtr msg) -> void
  { struct timespec now_ts;
  getnow(&now_ts);
  auto now_ns = _timespec_to_uint64(&now_ts);
  if (ping_sub_report_.add(now_ns - msg->time_sent_ns)) {
    ping_argmax_ = msg->data;
        }

        if (msg->data == ping_sub_count_ + 1) {
          ping_sub_count_ += 1;
        } else if (msg->data > ping_sub_count_ + 1) {  // drop occur
          ping_drop += 1;
          ping_drop_gap_ += msg->data - (ping_sub_count_ + 1);
          ping_sub_count_ = msg->data;
          ping_argdrop_ = msg->data;
        } else {  // msg->data < ping_sub_count_ + 1, late delivery
          ping_late += 1;
        }

        // if(debug_print) {
        //   struct timespec time_print;
        //   getnow(&time_print);
        //   std::cout << "recv ping id = " << msg->data
        //             << " @" << now_ns
        //             << std::endl;
        // }

        if (!send_pong_) {
          return;
        }

        // pos-neg inversion
        // for(size_t i=0; i< msg_.image.size(); i++) {
        //   pong.image[i] = 255 - msg->image[i];
        // }
        msg->time_sent_pong_ns = get_now_int64();
        pong_pub_->publish(std::move(msg));
        pong_pub_count_++;

        struct timespec time_exit;
        getnow(&time_exit);
        subtract_timespecs(&time_exit, &now_ts, &time_exit);
        ping_callback_process_time_report_.add(_timespec_to_uint64(&time_exit));
      };

  rclcpp::SubscriptionOptions subscription_options;
  qos.deadline(rclcpp::Duration(std::chrono::nanoseconds(dl_period_ns)));
  subscription_options.event_callbacks.deadline_callback =
      [](rclcpp::QOSDeadlineRequestedInfo &event) {(void)event;};

  ping_sub_ = create_subscription<twmsgs::msg::Data>(
      topic_name, qos, callback_sub, subscription_options);
}

void TwoWaysNode::setup_pong_subscriber()
{
  auto debug_print = get_parameter(DEBUG_PRINT).get_value<bool>();
  auto dl_period_ns = get_parameter(DL_PERIOD_NS).get_value<int>();
  auto topic_name_pong = tw_options_.topic_name_pong;
  auto qos = tw_options_.qos;

  auto callback_pong_sub =
      [this, debug_print](const twmsgs::msg::Data::UniquePtr msg) -> void
      {
        struct timespec now;
        getnow(&now);

        auto now_ns = _timespec_to_uint64(&now);
        if (ping_pong_report_.add(now_ns - msg->time_sent_ns)) {
          pong_argmax_ = msg->data;
        }
        pong_sub_report_.add(now_ns - msg->time_sent_pong_ns);
        if (msg->data == pong_sub_count_ + 1) {
          pong_sub_count_ += 1;
        } else if (msg->data > pong_sub_count_ + 1) {  // drop occur
          pong_drop += 1;
          pong_drop_gap_ += msg->data - (pong_sub_count_ + 1);
          pong_sub_count_ = msg->data;
          pong_argdrop_ = msg->data;
        } else {  // msg->data < pong_sub_count_ + 1, late delivery
          pong_late += 1;
        }

        struct timespec time_exit;
        getnow(&time_exit);
        subtract_timespecs(&time_exit, &now, &time_exit);
        pong_callback_process_time_report_.add(_timespec_to_uint64(&time_exit));

        if (debug_print) {
          struct timespec time_print;
          getnow(&time_print);
          std::cout << "time_print.tv_sec: " << time_print.tv_sec << " "
                    << "time_print.tv_nsec: " << time_print.tv_nsec
                    << std::endl;
          std::cout << "recv pong id = " << pong_sub_count_ << " @"
                    << _timespec_to_uint64(&time_print) << " ping-pong @ "
                    << (now_ns - msg->time_sent_ns) << std::endl;
        }
      };


  rclcpp::SubscriptionOptions subscription_options;
  qos.deadline(rclcpp::Duration(std::chrono::nanoseconds(dl_period_ns)));
  subscription_options.event_callbacks.deadline_callback =
      [this](rclcpp::QOSDeadlineRequestedInfo &event) mutable {
        (void)event;
        this->pong_deadline_count++;
      };
  pong_sub_ = create_subscription<twmsgs::msg::Data>(
      topic_name_pong, qos, callback_pong_sub, subscription_options);
}

int64_t TwoWaysNode::get_now_int64()
{
  struct timespec now_ts;
  getnow(&now_ts);
  return _timespec_to_uint64(&now_ts);
}
