#pragma once

#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "franka_semantic_components/franka_robot_state.hpp"

namespace franka_torque_broadcaster {

class FrankaTorqueBroadcaster : public controller_interface::ControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

protected:
  static constexpr size_t kNumJoints = 7;

  std::string arm_id_;
  std::string state_interface_name_{"robot_state"};

  double frequency_{30.0};
  rclcpp::Time last_pub_;

  // Publishers (one per torque signal)
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> pub_tau_ext_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> pub_tau_j_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> pub_tau_j_d_;

  // RT publishers
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> rt_tau_ext_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> rt_tau_j_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> rt_tau_j_d_;

  std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state_;
};

}  // namespace franka_torque_broadcaster
