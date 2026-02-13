#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>

namespace garmi_controllers {

class DualCartesianImpedanceController : public controller_interface::ControllerInterface {
public:
  DualCartesianImpedanceController() = default;
  ~DualCartesianImpedanceController() override = default;

  // Interfaces
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // Lifecycle
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& prev_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& prev_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev_state) override;
  controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& prev_state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  static constexpr int num_joints = 7;

  // Eigen typedefs for readability (matching your single-arm style)
  using Matrix4d = Eigen::Matrix<double,4,4>;
  using Matrix6d = Eigen::Matrix<double,6,6>;
  using Matrix7d = Eigen::Matrix<double,7,7>;
  using Vector6d = Eigen::Matrix<double,6,1>;
  using Vector7d = Eigen::Matrix<double,7,1>;
  using Vector3d = Eigen::Matrix<double,3,1>;
  using Quaterniond = Eigen::Quaterniond;

  struct Arm {
    std::string arm_id;                 // e.g., "left" / "right"
    std::string delta_topic;            // teleop delta topic
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_delta;

    // Franka model (same class you used)
    std::unique_ptr<franka_semantic_components::FrankaRobotModel> model;

    // Desired EE setpoint and nullspace target
    Vector3d     desired_position{Vector3d::Zero()};
    Quaterniond  desired_orientation{Quaterniond::Identity()};
    Vector7d     desired_qn{Vector7d::Zero()};

    // Gains
    Matrix6d stiffness{Matrix6d::Identity()};
    Matrix6d damping{Matrix6d::Identity()};
    double   n_stiffness{10.0};

    // Teleop delta clamping
    double max_delta_lin{0.03};  // [m] per message
    double max_delta_rot{0.15};  // [rad] per message
  };

  std::map<std::string, Arm> arms_;     // key = arm_id
  std::vector<std::string> arm_order_;  // to preserve command order

  rclcpp::Time start_time_;

  // Parameters
  std::string arm_id_1_{"left"};
  std::string arm_id_2_{"right"};
  std::string delta_topic_1_{"/teleop/left/ee_delta"};
  std::string delta_topic_2_{"/teleop/right/ee_delta"};
  double pos_stiff_{400.0};
  double rot_stiff_{20.0};
  double nullspace_stiff_{10.0};
  double max_delta_lin_{0.03};
  double max_delta_rot_{0.15};

  // Helpers
  static void pseudoInverse(const Eigen::MatrixXd& M, Eigen::MatrixXd& M_pinv, bool damped = true);
  static Eigen::Quaterniond smallAngleQuat(const Eigen::Vector3d& dtheta);
  static void integrate_delta(Arm& a, const geometry_msgs::msg::Twist& d, double max_lin, double max_rot);
};

} // namespace garmi_controllers
