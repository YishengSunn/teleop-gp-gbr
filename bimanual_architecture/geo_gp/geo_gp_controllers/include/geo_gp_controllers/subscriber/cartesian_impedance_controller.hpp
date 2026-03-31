#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <franka_msgs/msg/franka_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <realtime_tools/realtime_buffer.hpp>

#include <franka_semantic_components/franka_robot_model.hpp>

#include <geo_gp_controllers/comless/motion_generator.hpp>

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

  enum class Mode : uint8_t { MOVE_TO_START = 0, CARTESIAN = 1, BLEND_TO_LEADER = 2 };

  struct DesiredPoseRT {
    double px{0.0}, py{0.0}, pz{0.0};
    double qw{1.0}, qx{0.0}, qy{0.0}, qz{0.0};
  };

  // Parameters
  std::string arm_id_;
  std::string leader_robot_state_topic_{"/leader/franka_robot_state_broadcaster/robot_state"};
  std::string execution_pose_topic_{"/execution/desired_pose"};
  std::string execution_running_topic_{"/execution/running"};
  std::string blend_running_topic_{"/execution/blend_to_leader_running"};
  bool blend_to_leader_enabled_{true};
  double blend_seconds_per_meter_{2.0};
  double blend_seconds_per_rad_{1.2};
  double blend_duration_min_{0.25};
  double blend_duration_max_{8.0};
  double blend_running_hold_sec_{0.5};

  double pos_stiff_{100.0};
  double rot_stiff_{10.0};
  double n_stiffness_{10.0};

  // Start motion parameters/state
  bool move_to_start_{false};
  Vector7d q_start_{(Vector7d() << 0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4).finished()};

  Vector7d k_start_{Vector7d::Zero()};
  Vector7d d_start_{Vector7d::Zero()};
  Vector7d dq_filtered_{Vector7d::Zero()};

  // Franka model
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

  // Nullspace target
  Vector7d desired_qn_{Vector7d::Zero()};

  // Cartesian gains
  Matrix6d stiffness_{Matrix6d::Identity()};
  Matrix6d damping_{Matrix6d::Identity()};

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
