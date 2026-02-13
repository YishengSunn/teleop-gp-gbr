#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include "franka_example_controllers/comless/motion_generator.hpp"

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace teleop_controllers {

class GravityCompensationWithJointTorqueFeedbackController
  : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  [[nodiscard]] controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  enum class Mode { MOVE_TO_START, FEEDBACK_GRAVITY };
  Mode mode_{Mode::MOVE_TO_START};

  // --- robot state (from state interfaces)
  static constexpr int kNumJoints = 7;
  std::string arm_id_{"fr3"};
  Vector7d q_{Vector7d::Zero()};
  Vector7d q_goal_{Vector7d::Zero()};
  Vector7d dq_{Vector7d::Zero()};
  Vector7d dq_filtered_{Vector7d::Zero()};
  Vector7d k_gains_{Vector7d::Zero()};
  Vector7d d_gains_{Vector7d::Zero()};
  rclcpp::Time start_time_;
  std::unique_ptr<MotionGenerator> motion_generator_;

  // --- behavior params
  bool move_to_start_{true};

  // --- enable/disable feedback
  bool enable_feedback_{true};

  // --- single torque feedback topic (Float64MultiArray size 7)
  std::string torque_feedback_topic_{"/filtered_external_tau"};

  // Bias removal
  bool subtract_first_bias_{true};
  bool bias_initialized_{false};
  Vector7d tau_bias_{Vector7d::Zero()};

  // --- scaling / safety clamp
  double feedback_scale_{1.0};
  double feedback_max_abs_tau_{30.0};
  bool feedback_additive_{false};

  // realtime transport of tau feedback
  realtime_tools::RealtimeBuffer<Vector7d> tau_feedback_rt_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tau_sub_;

  void updateJointStates();
  void onTauArray(const std_msgs::msg::Float64MultiArray& msg);
};

}  // namespace teleop_controllers
