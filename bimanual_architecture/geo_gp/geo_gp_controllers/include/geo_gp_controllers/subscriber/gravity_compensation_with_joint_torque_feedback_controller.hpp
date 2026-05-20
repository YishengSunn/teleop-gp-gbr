#pragma once

#include "geo_gp_controllers/comless/motion_generator.hpp"
#include "geo_gp_controllers/tdpa/joint.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include <controller_interface/controller_interface.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>
#include <franka_semantic_components/franka_robot_state.hpp>
#include <geo_gp_interfaces/msg/tdpa_joint_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace geo_gp_controllers {

class GravityCompensationWithJointTorqueFeedbackController
  : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Matrix7d = Eigen::Matrix<double, 7, 7>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Vector3d = Eigen::Matrix<double, 3, 1>;

  ~GravityCompensationWithJointTorqueFeedbackController() override = default;

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
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  enum class Mode { MOVE_TO_START, FEEDBACK_GRAVITY };
  Mode mode_{Mode::MOVE_TO_START};

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

  bool move_to_start_{true};
  bool enable_feedback_{true};
  std::string feedback_source_{"measured"};
  std::string torque_feedback_topic_{""};
  bool subtract_first_bias_{true};
  bool bias_initialized_{false};
  Vector7d tau_bias_{Vector7d::Zero()};
  double feedback_scale_{1.0};
  double feedback_max_abs_tau_{30.0};
  bool feedback_additive_{false};
  bool suppress_feedback_during_execution_{true};
  std::string execution_running_topic_{"/execution/running"};
  std::string blend_running_topic_{"/execution/blend_to_leader_running"};
  double execution_feedback_scale_{0.0};

  bool TDPA_active_{false};
  bool tau_ext_feedback_{true};
  double eta_{0.0};
  std::string remote_state_topic_{"follower/tdpa_joint_state"};
  std::string local_state_topic_{"leader/tdpa_joint_state"};

  double pandatime_{0.0};
  double remote_pandatime_{0.0};

  Vector7d q_local_delta_{Vector7d::Zero()};
  Vector7d dq_local_{Vector7d::Zero()};
  Vector7d tau_ext_{Vector7d::Zero()};
  Vector7d tau_remote_{Vector7d::Zero()};
  Vector7d q_remote_delta_{Vector7d::Zero()};
  Vector7d dq_remote_{Vector7d::Zero()};
  Vector7d tau_diff_{Vector7d::Zero()};
  Vector7d tau_c_{Vector7d::Zero()};
  Vector7d q_remote_delta_intgl_{Vector7d::Zero()};
  Vector7d position_error_{Vector7d::Zero()};
  Vector6d f_local_{Vector6d::Zero()};
  Vector6d f_remote_{Vector6d::Zero()};
  Vector3d position_{Vector3d::Zero()};

  Vector7d last_tau_cmd_{Vector7d::Zero()};
  bool last_tau_cmd_initialized_{false};

  double E_F_in_delayed_{0.0};
  double E_L_in_{0.0};
  double E_L_out_{0.0};
  double E_L_diss_{0.0};
  double alpha_{0.0};
  double shortage_{0.0};

  uint64_t local_seq_{0};
  uint64_t last_received_remote_seq_{0};
  int64_t last_received_remote_tx_time_ns_{0};

  std::atomic<bool> execution_running_{false};
  std::atomic<bool> blend_running_{false};
  std::atomic<bool> tdpa_remote_ready_{false};

  realtime_tools::RealtimeBuffer<Vector7d> tau_feedback_rt_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tau_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr execution_running_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr blend_running_sub_;
  rclcpp::Subscription<geo_gp_interfaces::msg::TDPAJointState>::SharedPtr remote_state_sub_;
  rclcpp::Publisher<geo_gp_interfaces::msg::TDPAJointState>::SharedPtr local_state_pub_raw_;
  std::unique_ptr<
      realtime_tools::RealtimePublisher<geo_gp_interfaces::msg::TDPAJointState>>
      local_state_pub_;

  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state_;

  TDPA_Leader leaderPC_;

  inline void tdpaReset_() {
    E_F_in_delayed_ = 0.0;
    E_L_in_ = 0.0;
    E_L_out_ = 0.0;
    E_L_diss_ = 0.0;
    alpha_ = 0.0;
    shortage_ = 0.0;

    tau_diff_.setZero();
    tau_c_.setZero();
    q_remote_delta_intgl_.setZero();
    position_error_.setZero();
    tdpa_remote_ready_.store(false, std::memory_order_release);

    leaderPC_.init();
  }

  void updateJointStates();
  void onTauArray(const std_msgs::msg::Float64MultiArray& msg);
  void onExecutionRunning(const std_msgs::msg::Bool& msg);
  void onBlendRunning(const std_msgs::msg::Bool& msg);
  void remoteStateCallback(const geo_gp_interfaces::msg::TDPAJointState& msg);
  [[nodiscard]] double effectiveFeedbackScale() const;
  [[nodiscard]] Vector7d computeTdpaFeedbackTorque_(double dt);
};

}  // namespace geo_gp_controllers
