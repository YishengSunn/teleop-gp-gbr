#pragma once

#include "geo_gp_controllers/comless/motion_generator.hpp"
#include "geo_gp_controllers/tdpa/cartesian.h"

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Eigen>

#include <controller_interface/controller_interface.hpp>
#include <franka_msgs/msg/franka_state.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>
#include <franka_semantic_components/franka_robot_state.hpp>
#include <geo_gp_interfaces/msg/tdpa_cartesian_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace geo_gp_controllers {

using Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix<double, 4, 4>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix7d = Eigen::Matrix<double, 7, 7>;

using Vector3d = Eigen::Matrix<double, 3, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;

using Eigen::Quaterniond;

class CartesianImpedanceController : public controller_interface::ControllerInterface {
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

private:
  static constexpr int num_joints = 7;
  static constexpr int cart_dims = 6;

  using Matrix6x7d = Eigen::Matrix<double, cart_dims, num_joints>;
  using CartesianArray = std::array<double, cart_dims>;

  enum class Mode : uint8_t { MOVE_TO_START = 0, CARTESIAN = 1, BLEND_TO_LEADER = 2 };

  struct DesiredPoseRT {
    double px{0.0}, py{0.0}, pz{0.0};
    double qw{1.0}, qx{0.0}, qy{0.0}, qz{0.0};
  };

  controller_interface::return_type updateTdpa_(const rclcpp::Time& time,
                                                const rclcpp::Duration& period);

  static Vector3d quatErrorToRotvec(const Quaterniond& current, const Quaterniond& desired);
  static Matrix3d skewMatrix(const Vector3d& w);
  static Matrix3d projectToSO3(const Matrix3d& R);
  static Matrix3d integrateRotationWorld(const Matrix3d& R,
                                         const Vector3d& omega_world,
                                         double dt);
  static Vector6d wrenchFromTorque(const Matrix6x7d& jacobian,
                                   const Matrix7d& mass,
                                   const Vector7d& tau_ext);

  void updateJointStates();
  void remoteStateCallback(const geo_gp_interfaces::msg::TDPACartesianState& msg);

  inline void tdpaReset_() {
    E_L_in_delayed_ = 0.0;
    E_L_in_delayed_linear_ = 0.0;
    E_L_in_delayed_rotational_ = 0.0;

    E_F_in_ = 0.0;
    E_F_in_linear_ = 0.0;
    E_F_in_rotational_ = 0.0;
    shortage_linear_ = 0.0;
    shortage_rotational_ = 0.0;

    x_local_delta_.setZero();
    x_remote_delta_.setZero();
    x_remote_delta_intgl_.setZero();
    xdot_local_.setZero();
    xdot_remote_.setZero();
    wrench_ext_.setZero();
    wrench_c_.setZero();
    wrench_applied_.setZero();
    wrench_applied_initialized_ = false;
    position_error_.setZero();

    tau_ext_.setZero();

    followerPC_linear_.init();
    followerPC_rotational_.init();
  }

  // Parameters
  std::string arm_id_;
  std::string leader_robot_state_topic_{"/leader/franka_robot_state_broadcaster/robot_state"};
  std::string execution_pose_topic_{"/execution/desired_pose"};
  std::string execution_running_topic_{"/execution/running"};
  std::string blend_running_topic_{"/execution/blend_to_leader_running"};
  std::string remote_state_topic_{"leader/tdpa_cartesian_state"};
  std::string local_state_topic_{"follower/tdpa_cartesian_state"};
  bool blend_to_leader_enabled_{true};
  double blend_seconds_per_meter_{2.0};
  double blend_seconds_per_rad_{1.2};
  double blend_duration_min_{0.25};
  double blend_duration_max_{8.0};
  double blend_running_hold_sec_{0.5};

  double pos_stiff_{100.0};
  double rot_stiff_{10.0};
  double n_stiffness_{10.0};

  bool TDPA_active_{false};
  bool tau_ext_feedback_{false};
  double K_drift_{0.0};
  double eta_{0.0};

  double pandatime_{0.0};
  double remote_pandatime_{0.0};

  // Start motion parameters/state
  bool move_to_start_{false};
  Vector7d q_start_{(Vector7d() << 0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4).finished()};

  Vector7d k_start_{Vector7d::Zero()};
  Vector7d d_start_{Vector7d::Zero()};
  Vector7d dq_filtered_{Vector7d::Zero()};

  Vector7d q_{Vector7d::Zero()};
  Vector7d dq_{Vector7d::Zero()};

  // Franka model / state
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state_;

  // Nullspace target
  Vector7d desired_qn_{Vector7d::Zero()};

  // Cartesian gains
  Matrix6d stiffness_{Matrix6d::Identity()};
  Matrix6d damping_{Matrix6d::Identity()};

  // TDPA cartesian state
  Vector6d x_local_delta_{Vector6d::Zero()};
  Vector6d x_remote_delta_{Vector6d::Zero()};
  Vector6d x_remote_delta_intgl_{Vector6d::Zero()};
  Vector6d xdot_local_{Vector6d::Zero()};
  Vector6d xdot_remote_{Vector6d::Zero()};
  Vector6d wrench_ext_{Vector6d::Zero()};
  Vector6d wrench_c_{Vector6d::Zero()};
  Vector6d wrench_applied_{Vector6d::Zero()};
  bool wrench_applied_initialized_{false};
  Vector6d position_error_{Vector6d::Zero()};
  Vector7d tau_ext_{Vector7d::Zero()};
  Vector7d last_tau_cmd_{Vector7d::Zero()};
  bool last_tau_cmd_initialized_{false};

  Vector3d position_start_{Vector3d::Zero()};
  Quaterniond orientation_start_{Quaterniond::Identity()};
  Vector3d desired_position_{Vector3d::Zero()};
  Quaterniond desired_orientation_{Quaterniond::Identity()};
  Matrix3d desired_rotation_{Matrix3d::Identity()};

  double E_L_in_delayed_{0.0};
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

  CartesianTDPAFollower followerPC_linear_;
  CartesianTDPAFollower followerPC_rotational_;

  realtime_tools::RealtimeBuffer<CartesianArray> xdot_cmd_buffer_;
  std::atomic<uint64_t> desired_seq_{0};
  uint64_t last_desired_seq_{0};

  rclcpp::Subscription<geo_gp_interfaces::msg::TDPACartesianState>::SharedPtr remote_state_sub_;
  rclcpp::Publisher<geo_gp_interfaces::msg::TDPACartesianState>::SharedPtr local_state_pub_raw_;
  std::unique_ptr<
      realtime_tools::RealtimePublisher<geo_gp_interfaces::msg::TDPACartesianState>>
      local_state_pub_;

  // Move-to-start runtime
  std::unique_ptr<MotionGenerator> motion_generator_;
  rclcpp::Time start_time_;
  std::atomic<uint8_t> mode_{static_cast<uint8_t>(Mode::CARTESIAN)};
  std::atomic<bool> accept_desired_{true};

  // Desired pose handoff (callback -> update)
  realtime_tools::RealtimeBuffer<DesiredPoseRT> desired_pose_buffer_;
  DesiredPoseRT desired_pose_rt_;

  // Leader pose cache for blend-to-leader
  realtime_tools::RealtimeBuffer<DesiredPoseRT> leader_pose_cache_;

  // Sequence gating to avoid applying stale desired after move_to_start
  std::atomic<uint64_t> desired_pose_seq_{0};
  uint64_t last_desired_pose_seq_{0};

  std::atomic<bool> execution_running_{false};
  bool prev_execution_running_{false};
  std::atomic<bool> pending_blend_to_leader_{false};
  std::atomic<bool> blending_to_leader_{false};

  DesiredPoseRT blend_pose_start_{};
  DesiredPoseRT blend_pose_goal_{};
  rclcpp::Time blend_t0_{};
  rclcpp::Time blend_running_hold_until_{};
  double blend_duration_sec_{1.0};

  void leaderRobotStateCallback(const franka_msgs::msg::FrankaState& msg);
  void executionDesiredPoseCallback(const std_msgs::msg::Float64MultiArray& msg);
  void executionRunningCallback(const std_msgs::msg::Bool::SharedPtr msg);

  static Quaterniond quatFromDesiredPose(const DesiredPoseRT& p);
  static void desiredPoseFromQuaternion(const Quaterniond& q, DesiredPoseRT* out);
  rclcpp::Subscription<franka_msgs::msg::FrankaState>::SharedPtr sub_leader_robot_state_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_execution_pose_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_execution_running_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_blend_running_;
  bool last_blend_running_published_{false};
};

}  // namespace geo_gp_controllers
