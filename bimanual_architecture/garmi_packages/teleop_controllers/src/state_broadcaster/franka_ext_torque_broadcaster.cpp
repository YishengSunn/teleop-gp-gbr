#include <teleop_controllers/state_broadcaster/franka_ext_torque_broadcaster.hpp>

#include <memory>
#include <string>

#include "rclcpp/qos.hpp"
#include "rcutils/logging_macros.h"

#include "pluginlib/class_list_macros.hpp"

namespace franka_ext_torque_broadcaster {

controller_interface::CallbackReturn FrankaExtTorqueBroadcaster::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<int>("frequency", 30);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage: %s\n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaExtTorqueBroadcaster::on_configure(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  try {
    arm_id_ = get_node()->get_parameter("arm_id").as_string();
    frequency_ = static_cast<double>(get_node()->get_parameter("frequency").as_int());
    last_pub_ = get_node()->now();

    franka_robot_state_ = std::make_unique<franka_semantic_components::FrankaRobotState>(
      franka_semantic_components::FrankaRobotState(arm_id_ + "/" + state_interface_name_, arm_id_));

    pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/tau_ext_hat_filtered", rclcpp::SystemDefaultsQoS());

    rt_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(pub_);

    // Pre-size message once (7 joints)
    rt_pub_->msg_.data.resize(7, 0.0);

  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during configure stage: %s\n", e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "%s franka_ext_torque_broadcaster configured (topic: %s)",
    arm_id_.c_str(),
    (get_node()->get_name() + std::string("/tau_ext_hat_filtered")).c_str());

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaExtTorqueBroadcaster::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_state_->assign_loaned_state_interfaces(state_interfaces_);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaExtTorqueBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_state_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
FrankaExtTorqueBroadcaster::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
FrankaExtTorqueBroadcaster::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names = franka_robot_state_->get_state_interface_names();
  return cfg;
}

controller_interface::return_type FrankaExtTorqueBroadcaster::update(
  const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
  if ((time.nanoseconds() - last_pub_.nanoseconds()) < static_cast<int64_t>(1e9 / frequency_)) {
    return controller_interface::return_type::OK;
  }

  if (!rt_pub_ || !franka_robot_state_) {
    return controller_interface::return_type::ERROR;
  }

  if (!rt_pub_->trylock()) {
    return controller_interface::return_type::OK;  // don't hard-error on contention
  }

  auto* rs = franka_robot_state_->get_robot_state_ptr();
  if (rs == nullptr) {
    rt_pub_->unlock();
    return controller_interface::return_type::ERROR;
  }

  // Publish external torques (filtered) = tau_ext_hat_filtered
  for (size_t i = 0; i < 7; ++i) {
    rt_pub_->msg_.data[i] = rs->tau_ext_hat_filtered[i];
  }

  rt_pub_->unlockAndPublish();
  last_pub_ = time;
  return controller_interface::return_type::OK;
}

}  // namespace franka_ext_torque_broadcaster

// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(
  franka_ext_torque_broadcaster::FrankaExtTorqueBroadcaster,
  controller_interface::ControllerInterface)
