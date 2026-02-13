#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <controller_interface/controller_interface.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <realtime_tools/realtime_buffer.hpp>

#include <teleop_controllers/comless/motion_generator.hpp>

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace teleop_controllers {

using Eigen::Matrix3d;
using Eigen::Quaterniond;

using Matrix4d = Eigen::Matrix<double, 4, 4>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix7d = Eigen::Matrix<double, 7, 7>;

using Vector3d = Eigen::Matrix<double, 3, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;

class CartesianImpedanceDeltaController : public controller_interface::ControllerInterface {
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

  enum class Mode : uint8_t { MOVE_TO_START = 0, DELTA_CARTESIAN = 1 };

  struct DeltaCmdRT {
    bool valid{false};
    double dx{0.0}, dy{0.0}, dz{0.0};
    double R[9]{
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    };
  };

  std::string arm_id_;

  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

  // Desired pose only touched by update thread
  Quaterniond desired_orientation_{Quaterniond::Identity()};
  Vector3d desired_position_{Vector3d::Zero()};
  Vector7d desired_qn_{Vector7d::Zero()};

  // Cartesian gains
  Matrix6d stiffness_{Matrix6d::Identity()};
  Matrix6d damping_{Matrix6d::Identity()};
  double pos_stiff_{100.0};
  double rot_stiff_{10.0};
  double n_stiffness_{10.0};

  // Delta subscription + filtering
  std::string cartesian_delta_topic_;
  double delta_deadzone_{1e-4};
  double delta_max_step_{0.05};

  // Nullspace damping (DLS) for RT-friendly pinv
  double nullspace_damping_lambda_{0.2};

  // Move-to-start
  bool move_to_start_{false};
  Vector7d q_start_{(Vector7d() << 0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4).finished()};
  Vector7d k_start_{Vector7d::Zero()};
  Vector7d d_start_{Vector7d::Zero()};
  Vector7d dq_filtered_{Vector7d::Zero()};

  std::unique_ptr<MotionGenerator> motion_generator_;

  // IMPORTANT: start_time_ must match the same time source as update()'s `time`.
  // We latch it from the first update tick when we are in MOVE_TO_START.
  rclcpp::Time start_time_;
  bool start_time_set_{false};

  std::atomic<uint8_t> mode_{static_cast<uint8_t>(Mode::DELTA_CARTESIAN)};
  std::atomic<bool> accept_delta_{true};

  // Callback -> update handoff (latest-wins) + sequence gating
  realtime_tools::RealtimeBuffer<DeltaCmdRT> delta_buffer_;
  std::atomic<uint64_t> delta_seq_{0};   // incremented after writes (callback thread)
  uint64_t last_delta_seq_{0};           // used in update thread only

  void desiredCartesianDeltaCallback(const std_msgs::msg::Float64MultiArray& msg);
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_desired_cartesian_delta_;
};

}  // namespace teleop_controllers
