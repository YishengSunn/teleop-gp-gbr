#pragma once

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <controller_interface/controller_interface.hpp>
#include <franka_msgs/msg/franka_state.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>
#include <franka_semantic_components/franka_robot_state.hpp>
#include <geo_gp_interfaces/msg/tdpa_cartesian_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "geo_gp_controllers/comless/motion_generator.hpp"
#include "geo_gp_controllers/tdpa/cartesian.h"

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace geo_gp_controllers {

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Matrix4d = Eigen::Matrix<double, 4, 4>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix6x7d = Eigen::Matrix<double, 6, 7>;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using Vector3d = Eigen::Matrix<double, 3, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;

class HybridPositionForceController : public controller_interface::ControllerInterface {
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
      const rclcpp::Time& time,
      const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

private:
  static constexpr int kNumJoints = 7;
  static constexpr int kCartDims = 6;

  using CartesianArray = std::array<double, kCartDims>;

  enum class Mode : uint8_t { MOVE_TO_START = 0, HYBRID = 1, BLEND_TO_LEADER = 2 };

  struct DesiredPoseRT {
    double px{0.0}, py{0.0}, pz{0.0};
    double qw{1.0}, qx{0.0}, qy{0.0}, qz{0.0};
  };

  struct DesiredForceRT {
    double value{0.0};
  };

  static bool forceAxisFromString(const std::string& axis, int* axis_index);
  static Quaterniond quatFromDesiredPose(const DesiredPoseRT& p);
  static void desiredPoseFromQuaternion(const Quaterniond& q, DesiredPoseRT* out);

  static Vector3d quatErrorToRotvec(const Quaterniond& current, const Quaterniond& desired);
  static Matrix3d skewMatrix(const Vector3d& w);
  static Matrix3d projectToSO3(const Matrix3d& R);
  static Matrix3d integrateRotationWorld(
      const Matrix3d& R,
      const Vector3d& omega_world,
      double dt);
  static Vector6d wrenchFromTorque(
      const Matrix6x7d& jacobian,
      const Matrix7d& mass,
      const Vector7d& tau_ext);

  controller_interface::return_type updateWithoutTdpa_(const rclcpp::Duration& period);
  controller_interface::return_type updateWithTdpa_(const rclcpp::Duration& period);

  std::string arm_id_{"panda"};
  std::string leader_robot_state_topic_{"/leader/franka_robot_state_broadcaster/robot_state"};
  std::string execution_pose_topic_{"/execution/desired_pose"};
  std::string desired_force_topic_{"/execution/desired_force"};
  std::string execution_running_topic_{"/execution/running"};
  std::string blend_running_topic_{"/execution/blend_to_leader_running"};
  bool blend_to_leader_enabled_{true};
  double blend_seconds_per_meter_{2.0};
  double blend_seconds_per_rad_{1.2};
  double blend_duration_min_{0.25};
  double blend_duration_max_{8.0};
  double blend_running_hold_sec_{0.5};

  std::string force_axis_name_{"z"};
  int force_axis_{2};

  double pos_stiff_{100.0};
  double rot_stiff_{10.0};
  double n_stiffness_{10.0};
  double translational_damping_{20.0};
  double rotational_damping_{5.0};

  double force_kp_{1.0};
  double force_ki_{0.0};
  double force_damping_{5.0};
  double force_feedforward_scale_{1.0};
  double force_integral_limit_{10.0};
  double force_command_max_abs_{30.0};
  double measured_force_filter_alpha_{0.95};
  double initial_desired_force_{0.0};

  bool move_to_start_{false};
  Vector7d q_start_{
      (Vector7d() << 0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4).finished()};
  Vector7d k_start_{Vector7d::Zero()};
  Vector7d d_start_{Vector7d::Zero()};
  Vector7d dq_filtered_{Vector7d::Zero()};

  bool TDPA_active_{false};
  bool tau_ext_feedback_{false};
  double K_drift_{0.0};
  double eta_{0.0};
  std::string remote_state_topic_{"leader/tdpa_cartesian_state"};
  std::string local_state_topic_{"follower/tdpa_cartesian_state"};

  double pandatime_{0.0};

  Vector6d x_local_delta_{Vector6d::Zero()};
  Vector6d x_remote_delta_{Vector6d::Zero()};
  Vector6d x_remote_delta_intgl_{Vector6d::Zero()};
  Vector6d xdot_local_{Vector6d::Zero()};
  Vector6d xdot_remote_{Vector6d::Zero()};
  Vector6d wrench_ext_{Vector6d::Zero()};
  Vector6d wrench_applied_{Vector6d::Zero()};
  bool wrench_applied_initialized_{false};
  Vector6d position_error_tdpa_{Vector6d::Zero()};

  Vector3d position_start_{Vector3d::Zero()};
  Quaterniond orientation_start_{Quaterniond::Identity()};
  Vector3d desired_position_tdpa_{Vector3d::Zero()};
  Quaterniond desired_orientation_tdpa_{Quaterniond::Identity()};
  Matrix3d desired_rotation_tdpa_{Matrix3d::Identity()};

  double E_L_in_delayed_linear_{0.0};
  double E_L_in_delayed_rotational_{0.0};
  double E_F_in_{0.0};
  double E_F_in_linear_{0.0};
  double E_F_in_rotational_{0.0};
  double shortage_linear_{0.0};
  double shortage_rotational_{0.0};

  uint64_t local_seq_{0};
  uint64_t last_received_remote_seq_{0};
  int64_t last_received_remote_tx_time_ns_{0};

  realtime_tools::RealtimeBuffer<CartesianArray> xdot_cmd_buffer_;

  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state_;
  Vector7d desired_qn_{Vector7d::Zero()};

  std::unique_ptr<MotionGenerator> motion_generator_;
  rclcpp::Time start_time_;
  std::atomic<uint8_t> mode_{static_cast<uint8_t>(Mode::HYBRID)};
  std::atomic<bool> accept_desired_{true};

  realtime_tools::RealtimeBuffer<DesiredPoseRT> desired_pose_buffer_;
  DesiredPoseRT desired_pose_rt_;
  realtime_tools::RealtimeBuffer<DesiredPoseRT> leader_pose_cache_;
  std::atomic<uint64_t> desired_pose_seq_{0};
  uint64_t last_desired_pose_seq_{0};

  std::atomic<uint64_t> tdpa_cmd_seq_{0};
  uint64_t last_tdpa_cmd_seq_{0};

  realtime_tools::RealtimeBuffer<DesiredForceRT> desired_force_buffer_;
  DesiredForceRT desired_force_rt_;
  std::atomic<uint64_t> desired_force_seq_{0};
  uint64_t last_desired_force_seq_{0};

  std::atomic<bool> execution_running_{false};
  bool prev_execution_running_{false};
  std::atomic<bool> pending_blend_to_leader_{false};
  std::atomic<bool> blending_to_leader_{false};

  DesiredPoseRT blend_pose_start_{};
  DesiredPoseRT blend_pose_goal_{};
  rclcpp::Time blend_t0_{};
  rclcpp::Time blend_running_hold_until_{};
  double blend_duration_sec_{1.0};

  double force_error_integral_{0.0};
  double filtered_force_measurement_{0.0};
  bool force_filter_initialized_{false};
  bool last_execution_running_for_z_axis_{false};

  void leaderRobotStateCallback(const franka_msgs::msg::FrankaState& msg);
  void executionDesiredPoseCallback(const std_msgs::msg::Float64MultiArray& msg);
  void executionDesiredForceCallback(const std_msgs::msg::Float64& msg);
  void executionRunningCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void remoteStateCallback(const geo_gp_interfaces::msg::TDPACartesianState& msg);

  rclcpp::Subscription<franka_msgs::msg::FrankaState>::SharedPtr sub_leader_robot_state_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_execution_pose_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_desired_force_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_execution_running_;
  rclcpp::Subscription<geo_gp_interfaces::msg::TDPACartesianState>::SharedPtr remote_state_sub_;
  rclcpp::Publisher<geo_gp_interfaces::msg::TDPACartesianState>::SharedPtr local_state_pub_raw_;
  std::unique_ptr<
      realtime_tools::RealtimePublisher<geo_gp_interfaces::msg::TDPACartesianState>>
      local_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_blend_running_;
  bool last_blend_running_published_{false};

  CartesianTDPAFollower followerPC_linear_;
  CartesianTDPAFollower followerPC_rotational_;

  inline void tdpaReset_() {
    E_L_in_delayed_linear_ = 0.0;
    E_L_in_delayed_rotational_ = 0.0;
    E_F_in_ = 0.0;
    E_F_in_linear_ = 0.0;
    E_F_in_rotational_ = 0.0;
    shortage_linear_ = 0.0;
    shortage_rotational_ = 0.0;
    x_remote_delta_intgl_.setZero();
    position_error_tdpa_.setZero();
    followerPC_linear_.init();
    followerPC_rotational_.init();
  }
};

}  // namespace geo_gp_controllers
