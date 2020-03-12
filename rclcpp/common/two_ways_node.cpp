#include <rosidl_generator_cpp/traits.hpp>
#include <rclcpp/strategies/message_pool_memory_strategy.hpp>
#include "two_ways_node.hpp"

using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;

void TwoWaysNode::setup_ping_publisher()
{
  auto topic_name = this->tw_options_.topic_name;
  auto qos = this->tw_options_.qos;
  auto period_ns = this->tw_options_.period_ns;

  // pub
  this->ping_pub_ = this->create_publisher<twmsgs::msg::Data>(topic_name, qos);

  auto callback_pub =
      [this, period_ns]() -> void
      {
        // calc wakeup jitter
        TIME_POINT now = _SC::now();
        auto expect = this->ping_epoch_ + std::chrono::nanoseconds(period_ns) * (this->ping_pub_count_ + 1);
        auto diff = std::chrono::duration_cast<std::chrono::nanoseconds>(now - expect);
        ping_wakeup_report_.add(diff.count());

        // pub
        twmsgs::msg::Data msg;
        msg.data = this->ping_pub_count_;
        // msg.data = "HelloWorld" + std::to_string(this->ping_pub_count_);
        msg.time_sent_ns = std::chrono::nanoseconds(_SC::now().time_since_epoch()).count();
        this->ping_pub_->publish(msg);

        this->ping_pub_count_++;
      };

  // set timer
  this->ping_epoch_ = _SC::now();
  this->ping_timer_ = this->create_wall_timer(std::chrono::nanoseconds(period_ns), callback_pub);
}

void TwoWaysNode::setup_ping_subscriber()
{
  auto topic_name = this->tw_options_.topic_name;
  auto qos = this->tw_options_.qos;
  rclcpp::SubscriptionOptions subscription_options;

  // sub
  auto callback_sub =
      [this](const twmsgs::msg::Data::SharedPtr msg) -> void
      {
        int now_ns = std::chrono::nanoseconds(_SC::now().time_since_epoch()).count();
        ping_sub_report_.add(now_ns - msg->time_sent_ns);
        this->ping_sub_count_++;
      };
  ping_sub_ =
      this->create_subscription<twmsgs::msg::Data>(topic_name,
                                                   qos,
                                                   callback_sub,
                                                   subscription_options);

  if (tw_options_.use_message_pool_memory_strategy) {
    auto data_msg_strategy =
        std::make_shared<MessagePoolMemoryStrategy<twmsgs::msg::Data, 1>>();
    ping_sub_->set_message_memory_strategy(data_msg_strategy);
  }

}
