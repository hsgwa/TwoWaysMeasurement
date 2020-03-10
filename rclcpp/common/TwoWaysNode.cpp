#include "TwoWaysNode.hpp"

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
        auto num_bin = this->tw_options_.num_bin;

        // calc wakeup jitter
        TIME_POINT now = _SC::now();
        auto expect = this->ping_epoch_ + std::chrono::nanoseconds(period_ns) * (this->ping_pub_count_ + 1);
        auto diff = std::chrono::duration_cast<std::chrono::nanoseconds>(now - expect);
        this->ping_wakeup_jitters_[this->ping_pub_count_ % num_bin] = diff.count();

        // pub
        twmsgs::msg::Data msg;
        msg.data = "HelloWorld" + std::to_string(this->ping_pub_count_);
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
  auto num_bin = this->tw_options_.num_bin;

  // sub
  auto callback_sub =
      [this, num_bin](const twmsgs::msg::Data::SharedPtr msg) -> void
      {
        int now_ns = std::chrono::nanoseconds(_SC::now().time_since_epoch()).count();
        this->ping_sub_jitters_[this->ping_sub_count_ % num_bin] = now_ns - msg->time_sent_ns;
        this->ping_sub_count_++;
      };
  this->ping_sub_ = this->create_subscription<twmsgs::msg::Data>(topic_name, qos, callback_sub);
}