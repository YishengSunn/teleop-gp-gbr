#pragma once

#include "geo_gp_controllers/comless/motion_generator.hpp"
#include "geo_gp_controllers/tdpa/joint.h"

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Eigen>

#include <controller_interface/controller_interface.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>
#include <franka_semantic_components/franka_robot_state.hpp>
#include <geo_gp_interfaces/msg/tdpa_joint_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace geo_gp_controllers {

class JointImpedanceController : public controller_interface::ControllerInterface {
public:
  JointImpedanceController() = default;
  ~JointImpedanceController() override = default;

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
  using Vector7d = Eigen::Matrix<double, num_joints, 1>;
  using Matrix7d = Eigen::Matrix<double, num_joints, num_joints>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Vector3d = Eigen::Matrix<double, 3, 1>;
  using JointsArray = std::array<double, num_joints>;

  enum class Mode : uint8_t { MOVE_TO_START = 0, IMPEDANCE = 1 };

  void updateJointStates();
  void desiredJointCallback(const sensor_msgs::msg::JointState& msg);
  void remoteStateCallback(const geo_gp_interfaces::msg::TDPAJointState& msg);

  std::string command_arm_id_{"panda"};
  std::string source_arm_id_{"panda"};
  std::string remote_state_topic_{"leader/tdpa_joint_state"};
  std::string local_state_topic_{"follower/tdpa_joint_state"};

  double pandatime_{0.0};
  double remote_pandatime_{0.0};

  Vector7d q_{Vector7d::Zero()};
  Vector7d dq_{Vector7d::Zero()};
  Vector7d dq_filtered_{Vector7d::Zero()};

  Vector7d k_gains_{Vector7d::Zero()};
  Vector7d d_gains_{Vector7d::Zero()};
  Vector7d start_k_gains_{Vector7d::Zero()};
  Vector7d start_d_gains_{Vector7d::Zero()};

  Vector7d q_d_rt_{Vector7d::Zero()};
  Vector7d q_start_{
      (Vector7d() << 0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4).finished()};

  Vector7d q_local_delta_{Vector7d::Zero()};
  Vector7d q_remote_delta_{Vector7d::Zero()};
  Vector7d q_remote_delta_intgl_{Vector7d::Zero()};
  Vector7d dq_local_{Vector7d::Zero()};
  Vector7d dq_remote_{Vector7d::Zero()};
  Vector7d tau_remote_{Vector7d::Zero()};
  Vector7d tau_ext_{Vector7d::Zero()};
  Vector7d tau_c_{Vector7d::Zero()};
  Vector7d tau_c_no_mod_dq_{Vector7d::Zero()};
  Vector7d tau_diff_{Vector7d::Zero()};
  Vector7d q_des_{Vector7d::Zero()};
  Vector7d position_error_{Vector7d::Zero()};

  Vector7d last_tau_cmd_{Vector7d::Zero()};
  bool last_tau_cmd_initialized_{false};

  double E_L_in_delayed_{0.0};
  double E_F_in_{0.0};
  double E_F_out_{0.0};
  double E_F_diss_{0.0};
  double beta_{0.0};
  double shortage_{0.0};

  bool move_to_start_{false};
  bool TDPA_active_{false};
  bool tau_ext_feedback_{false};
  double K_drift_{0.0};
  double eta_{0.0};

  uint64_t local_seq_{0};
  uint64_t last_received_remote_seq_{0};
  int64_t last_received_remote_tx_time_ns_{0};

  realtime_tools::RealtimeBuffer<JointsArray> qd_buffer_;
  realtime_tools::RealtimeBuffer<JointsArray> qd_cmd_buffer_;

  std::atomic<uint64_t> desired_seq_{0};
  uint64_t last_desired_seq_{0};

  std::atomic<bool> accept_desired_{true};
  std::unique_ptr<MotionGenerator> motion_generator_;
  rclcpp::Time start_time_;
  std::atomic<uint8_t> mode_{static_cast<uint8_t>(Mode::IMPEDANCE)};

  std::vector<std::string> expected_joint_names_;
  std::unordered_map<std::string, size_t> expected_name_to_index_;
  bool warned_bad_jointstate_{false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_desired_joint_;
  rclcpp::Subscription<geo_gp_interfaces::msg::TDPAJointState>::SharedPtr remote_state_sub_;
  rclcpp::Publisher<geo_gp_interfaces::msg::TDPAJointState>::SharedPtr local_state_pub_raw_;
  std::unique_ptr<
      realtime_tools::RealtimePublisher<geo_gp_interfaces::msg::TDPAJointState>>
      local_state_pub_;

  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state_;

  TDPA_Follower followerPC_;

  inline void tdpaReset_() {
    E_L_in_delayed_ = 0.0;
    E_F_in_ = 0.0;
    E_F_out_ = 0.0;
    E_F_diss_ = 0.0;
    beta_ = 0.0;
    shortage_ = 0.0;

    q_remote_delta_intgl_.setZero();
    position_error_.setZero();
    tau_c_.setZero();
    tau_c_no_mod_dq_.setZero();
    tau_diff_.setZero();

    followerPC_.init();
  }
};

}  // namespace geo_gp_controllers
