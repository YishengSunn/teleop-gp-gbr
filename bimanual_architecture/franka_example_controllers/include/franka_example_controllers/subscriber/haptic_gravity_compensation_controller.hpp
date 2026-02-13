#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_publisher.h>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <franka/model.h>
#include "franka_semantic_components/franka_robot_model.hpp"

namespace franka_example_controllers {

class HapticGravityCompensationController : public controller_interface::ControllerInterface {
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  // --- ControllerInterface overrides ---
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override;

  controller_interface::return_type update(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  // ---------------- Types & Constants ----------------
  static constexpr int num_joints = 7;
  using Vector7d  = Eigen::Matrix<double, 7, 1>;
  using Matrix7d  = Eigen::Matrix<double, 7, 7>;
  using Vector6d  = Eigen::Matrix<double, 6, 1>;

  // ---------------- Parameters / Topics ----------------
  std::string arm_id_{"panda"};

  // Follower overlay input
  std::string follower_input_mode_{"torque_topic"};  // "torque_topic" or "joint_states"
  std::string follower_torque_topic_{"/haptic/follower_tau"};
  std::string joint_state_topic_{"/follower/joint_states"};
  double lpf_alpha_{0.1};
  Eigen::Matrix<double, 7, 1> torque_clamp_{Eigen::Matrix<double,7,1>::Constant(5.0)};

  // Desired publishers (always publish; no bool flags)
  std::string desired_q_topic_{"/joint_impedance/joints_desired"};
  std::string desired_xd_topic_{"/cartesian_impedance/pose_desired"};
  std::string ee_frame_{"panda_link8"};  // or "tool"

  // Impedance gains (used in joint_states follower mode)
  Eigen::Matrix<double, 7, 1> k_gains_{Eigen::Matrix<double,7,1>::Zero()};
  Eigen::Matrix<double, 7, 1> d_gains_{Eigen::Matrix<double,7,1>::Zero()};

  // ---------------- Publishers ----------------
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> rt_pub_qd_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> rt_pub_xd_;

  // ---------------- Subscriptions ----------------
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_follower_tau_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;

  // ---------------- Franka model (FK for Cartesian desired) ----------------
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  bool model_ready_{false};

  // ---------------- Runtime state ----------------
  Vector7d q_{Vector7d::Zero()};
  Vector7d dq_{Vector7d::Zero()};

  Vector7d q_follower_{Vector7d::Zero()};
  Vector7d dq_follower_{Vector7d::Zero()};
  bool have_follower_state_{false};

  Vector7d tau_raw_{Vector7d::Zero()};
  Vector7d tau_filtered_{Vector7d::Zero()};
  Vector7d tau_cmd_{Vector7d::Zero()};
  bool have_first_msg_{false};

  // Protect shared data written from callbacks
  std::mutex data_mutex_;

  // ---------------- Helpers ----------------
  inline static double clamp(double x, double lo, double hi) {
    return (x < lo) ? lo : ((x > hi) ? hi : x);
  }

  void readLeaderJointStates();

  void followerTorqueCallback(const std_msgs::msg::Float64MultiArray& msg);
  void jointStateCallback(const sensor_msgs::msg::JointState& msg);

  bool fill_cartesian_row_major(std_msgs::msg::Float64MultiArray& m);
};

}  // namespace franka_example_controllers
