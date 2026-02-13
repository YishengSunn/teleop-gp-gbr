#include <teleop_controllers/state_broadcaster/franka_torque_broadcaster.hpp>

#include <memory>
#include <string>

#include "rclcpp/qos.hpp"
#include "rcutils/logging_macros.h"
#include "pluginlib/class_list_macros.hpp"

namespace franka_torque_broadcaster {

controller_interface::CallbackReturn FrankaTorqueBroadcaster::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<int>("frequency", 30);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage: %s\n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaTorqueBroadcaster::on_configure(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  try {
    arm_id_ = get_node()->get_parameter("arm_id").as_string();
    frequency_ = static_cast<double>(get_node()->get_parameter("frequency").as_int());
    last_pub_ = get_node()->now();

    franka_robot_state_ = std::make_unique<franka_semantic_components::FrankaRobotState>(
      franka_semantic_components::FrankaRobotState(arm_id_ + "/" + state_interface_name_, arm_id_));

    // Create publishers
    pub_tau_ext_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/tau_ext_hat_filtered", rclcpp::SystemDefaultsQoS());

    pub_tau_j_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/tau_j", rclcpp::SystemDefaultsQoS());

    pub_tau_j_d_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/tau_j_d", rclcpp::SystemDefaultsQoS());

    // Create realtime publishers
    rt_tau_ext_ =
      std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(pub_tau_ext_);
    rt_tau_j_ =
      std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(pub_tau_j_);
    rt_tau_j_d_ =
      std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(pub_tau_j_d_);

    // Pre-size messages once (7 joints)
    rt_tau_ext_->msg_.data.resize(kNumJoints, 0.0);
    rt_tau_j_->msg_.data.resize(kNumJoints, 0.0);
    rt_tau_j_d_->msg_.data.resize(kNumJoints, 0.0);

  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during configure stage: %s\n", e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "%s franka_torque_broadcaster configured (topics: %s, %s, %s)",
    arm_id_.c_str(),
    (get_node()->get_name() + std::string("/tau_ext_hat_filtered")).c_str(),
    (get_node()->get_name() + std::string("/tau_j")).c_str(),
    (get_node()->get_name() + std::string("/tau_j_d")).c_str());

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaTorqueBroadcaster::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_state_->assign_loaned_state_interfaces(state_interfaces_);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaTorqueBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_state_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
FrankaTorqueBroadcaster::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
FrankaTorqueBroadcaster::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names = franka_robot_state_->get_state_interface_names();
  return cfg;
}

controller_interface::return_type FrankaTorqueBroadcaster::update(
  const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {

  if ((time.nanoseconds() - last_pub_.nanoseconds()) < static_cast<int64_t>(1e9 / frequency_)) {
    return controller_interface::return_type::OK;
  }

  if (!franka_robot_state_ || !rt_tau_ext_ || !rt_tau_j_ || !rt_tau_j_d_) {
    return controller_interface::return_type::ERROR;
  }

  auto* rs = franka_robot_state_->get_robot_state_ptr();
  if (rs == nullptr) {
    return controller_interface::return_type::ERROR;
  }

  // Trylock each independently: publish what we can without blocking
  if (rt_tau_ext_->trylock()) {
    for (size_t i = 0; i < kNumJoints; ++i) {
      rt_tau_ext_->msg_.data[i] = rs->tau_ext_hat_filtered[i];
    }
    rt_tau_ext_->unlockAndPublish();
  }

  if (rt_tau_j_->trylock()) {
    for (size_t i = 0; i < kNumJoints; ++i) {
      rt_tau_j_->msg_.data[i] = rs->tau_J[i];
    }
    rt_tau_j_->unlockAndPublish();
  }

  if (rt_tau_j_d_->trylock()) {
    for (size_t i = 0; i < kNumJoints; ++i) {
      rt_tau_j_d_->msg_.data[i] = rs->tau_J_d[i];
    }
    rt_tau_j_d_->unlockAndPublish();
  }

  last_pub_ = time;
  return controller_interface::return_type::OK;
}

}  // namespace franka_torque_broadcaster

// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(
  franka_torque_broadcaster::FrankaTorqueBroadcaster,
  controller_interface::ControllerInterface)
