#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Eigen>

#include <controller_interface/controller_interface.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <realtime_tools/realtime_buffer.hpp>

#include <teleop_controllers/comless/motion_generator.hpp>

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace teleop_controllers {

class JointImpedanceController : public controller_interface::ControllerInterface {
public:
  JointImpedanceController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

private:
  static constexpr int num_joints = 7;
  using Vector7d = Eigen::Matrix<double, num_joints, 1>;
  using JointsArray = std::array<double, num_joints>;

  enum class Mode : uint8_t { MOVE_TO_START = 0, IMPEDANCE = 1 };

  void updateJointStates();
  void desiredJointCallback(const sensor_msgs::msg::JointState& msg);

  // Commanded robot vs source robot
  std::string command_arm_id_{"panda"};
  std::string source_arm_id_{"panda"};

  // Current commanded state
  Vector7d q_{Vector7d::Zero()};
  Vector7d dq_{Vector7d::Zero()};
  Vector7d dq_filtered_{Vector7d::Zero()};

  // Gains used in IMPEDANCE mode
  Vector7d k_gains_{Vector7d::Zero()};
  Vector7d d_gains_{Vector7d::Zero()};

  // Gains used only during MOVE_TO_START
  Vector7d start_k_gains_{Vector7d::Zero()};
  Vector7d start_d_gains_{Vector7d::Zero()};

  // Desired used by RT thread only
  Vector7d q_d_rt_{Vector7d::Zero()};

  // Desired handoff (callback -> update), lock-free
  realtime_tools::RealtimeBuffer<JointsArray> qd_buffer_;

  // Sequence gating: callback increments desired_seq_ after writing the buffer.
  // update() only applies buffer values when seq changes.
  std::atomic<uint64_t> desired_seq_{0};
  uint64_t last_desired_seq_{0};

  // While moving to start we ignore incoming desired messages
  std::atomic<bool> accept_desired_{true};

  // Move-to-start parameters/state
  bool move_to_start_{false};
  Vector7d q_start_{(Vector7d() << 0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4).finished()};
  std::unique_ptr<MotionGenerator> motion_generator_;
  rclcpp::Time start_time_;
  std::atomic<uint8_t> mode_{static_cast<uint8_t>(Mode::IMPEDANCE)};

  // Name mapping for JointState.name[] messages
  std::vector<std::string> expected_joint_names_;
  std::unordered_map<std::string, size_t> expected_name_to_index_;
  bool warned_bad_jointstate_{false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_desired_joint_;

  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
};

}  // namespace teleop_controllers
