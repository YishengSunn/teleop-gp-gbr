#include <algorithm>
#include <chrono>
#include <deque>
#include <memory>
#include <string>
#include <utility>

#include "geo_gp_interfaces/msg/tdpa_cartesian_state.hpp"
#include "geo_gp_interfaces/msg/tdpa_joint_state.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_value.hpp"

namespace {
using Clock = std::chrono::steady_clock;

double readDelayMs(const rclcpp::Node& node) {
  const auto parameter = node.get_parameter("delay_ms");
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return static_cast<double>(parameter.as_int());
  }
  return parameter.as_double();
}

template <typename MessageT>
class DelayRelay {
 public:
  explicit DelayRelay(rclcpp::Node& node)
      : node_(node),
        delay_(std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double, std::milli>(readDelayMs(node_)))) {
    const auto input_topic = node_.get_parameter("input_topic").as_string();
    const auto output_topic = node_.get_parameter("output_topic").as_string();
    const auto queue_depth =
        static_cast<size_t>(std::max<int64_t>(1, node_.get_parameter("queue_depth").as_int()));
    const auto timer_period_ms =
        std::max<int64_t>(1, node_.get_parameter("timer_period_ms").as_int());

    auto qos = rclcpp::QoS(rclcpp::KeepLast(queue_depth)).best_effort().durability_volatile();
    publisher_ = node_.create_publisher<MessageT>(output_topic, qos);
    subscription_ = node_.create_subscription<MessageT>(
        input_topic, qos,
        [this](typename MessageT::SharedPtr msg) { this->onMessage(std::move(msg)); });
    timer_ = node_.create_wall_timer(
        std::chrono::milliseconds(timer_period_ms), [this]() { this->flushReady(); });

    RCLCPP_INFO(
        node_.get_logger(),
        "TDPA delay relay started | input=%s | output=%s | delay_ms=%.3f | queue_depth=%zu",
        input_topic.c_str(), output_topic.c_str(), readDelayMs(node_), queue_depth);
  }

 private:
  void onMessage(typename MessageT::SharedPtr msg) {
    if (delay_.count() <= 0.0) {
      publisher_->publish(*msg);
      return;
    }

    queue_.emplace_back(Clock::now() + delay_, std::move(msg));
  }

  void flushReady() {
    const auto now = Clock::now();
    while (!queue_.empty() && queue_.front().first <= now) {
      publisher_->publish(*queue_.front().second);
      queue_.pop_front();
    }
  }

  rclcpp::Node& node_;
  Clock::duration delay_;
  std::deque<std::pair<Clock::time_point, typename MessageT::SharedPtr>> queue_;
  typename rclcpp::Publisher<MessageT>::SharedPtr publisher_;
  typename rclcpp::Subscription<MessageT>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("tdpa_delay_relay");

  node->declare_parameter<std::string>("message_type", "cartesian");
  node->declare_parameter<std::string>("input_topic", "");
  node->declare_parameter<std::string>("output_topic", "");
  rcl_interfaces::msg::ParameterDescriptor delay_descriptor;
  delay_descriptor.dynamic_typing = true;
  node->declare_parameter("delay_ms", rclcpp::ParameterValue(0.0), delay_descriptor);
  node->declare_parameter<int64_t>("queue_depth", 1000);
  node->declare_parameter<int64_t>("timer_period_ms", 1);

  const auto message_type = node->get_parameter("message_type").as_string();
  const auto input_topic = node->get_parameter("input_topic").as_string();
  const auto output_topic = node->get_parameter("output_topic").as_string();
  if (input_topic.empty() || output_topic.empty()) {
    RCLCPP_FATAL(node->get_logger(), "input_topic and output_topic must be set");
    rclcpp::shutdown();
    return 1;
  }

  if (message_type == "cartesian") {
    DelayRelay<geo_gp_interfaces::msg::TDPACartesianState> relay(*node);
    rclcpp::spin(node);
  } else if (message_type == "joint") {
    DelayRelay<geo_gp_interfaces::msg::TDPAJointState> relay(*node);
    rclcpp::spin(node);
  } else {
    RCLCPP_FATAL(
        node->get_logger(), "Unsupported message_type '%s' (expected 'cartesian' or 'joint')",
        message_type.c_str());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
