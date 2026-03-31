#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include <franka_example_controllers/comless/motion_generator.hpp>

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace geo_gp_controllers {

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

  // Robot state (from state interfaces)
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

  // Behavior params
  bool move_to_start_{true};

  // Enable/disable feedback
  bool enable_feedback_{true};

  // Feedback source selection:
  // - commanded: use commanded torque topic (tau_j_d)
  // - measured: use measured/estimated external torque topic (tau_ext_hat_filtered)
  std::string feedback_source_{"measured"};

  // Single torque feedback topic (Float64MultiArray size 7).
  // If non-empty, this overrides feedback_source_ auto-selection.
  std::string torque_feedback_topic_{""};

  // Bias removal
  bool subtract_first_bias_{true};
  bool bias_initialized_{false};
  Vector7d tau_bias_{Vector7d::Zero()};

  // Scaling / safety clamp
  double feedback_scale_{1.0};
  double feedback_max_abs_tau_{30.0};
  bool feedback_additive_{false};

  // While trajectory_executor reports running=true on execution_running_topic, scale
  // feedback by execution_feedback_scale (0 = off). Ends immediately when running=false.
  bool suppress_feedback_during_execution_{true};
  std::string execution_running_topic_{"/execution/running"};
  double execution_feedback_scale_{0.0};

  std::atomic<bool> execution_running_{false};

  // Realtime transport of tau feedback
  realtime_tools::RealtimeBuffer<Vector7d> tau_feedback_rt_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tau_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr execution_running_sub_;

  void updateJointStates();
  void onTauArray(const std_msgs::msg::Float64MultiArray& msg);
  void onExecutionRunning(const std_msgs::msg::Bool& msg);
  [[nodiscard]] double effectiveFeedbackScale() const;
};

}  // namespace geo_gp_controllers
