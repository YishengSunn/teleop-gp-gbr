#include "garmi_controllers/dual_joint_impedance_controller.hpp"

#include <cassert>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <sensor_msgs/msg/joint_state.hpp>

namespace garmi_controllers
{

// Helper: read arm IDs directly from parameters (so interface claiming works
// even before arms_ is populated in on_configure()).
static std::vector<std::string> get_arm_ids_from_params(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
{
  std::vector<std::string> ids;
  for (int i = 1; i <= 8; ++i) {  // up to 8 arms if ever needed
    const std::string pname = "arm_" + std::to_string(i) + ".arm_id";
    if (!node->has_parameter(pname)) continue;
    auto p = node->get_parameter(pname);
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      const auto v = p.as_string();
      if (!v.empty()) ids.push_back(v);
    }
  }
  return ids;
}

/*========================  Interface Declarations  ========================*/

controller_interface::InterfaceConfiguration
 DualJointImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Prefer arm IDs from params (works before on_configure), else fall back to arms_ (if already filled).
  auto ids = get_arm_ids_from_params(get_node());
  if (ids.empty()) {
    for (const auto& kv : arms_) ids.push_back(kv.first);
  }

  for (const auto& arm_id : ids) {
    for (int j = 1; j <= num_joints; ++j) {
      cfg.names.push_back(arm_id + "_joint" + std::to_string(j) + "/effort");
    }
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
DualJointImpedanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  auto ids = get_arm_ids_from_params(get_node());
  if (ids.empty()) {
    for (const auto& kv : arms_) ids.push_back(kv.first);
  }

  for (const auto& arm_id : ids) {
    for (int j = 1; j <= num_joints; ++j) {
      cfg.names.push_back(arm_id + "_joint" + std::to_string(j) + "/position");
      cfg.names.push_back(arm_id + "_joint" + std::to_string(j) + "/velocity");
    }
  }
  return cfg;
}

/*=============================  Lifecycle  ================================*/

CallbackReturn DualJointImpedanceController::on_init()
{
  try {
    // Arm IDs (YAML can override)
    auto_declare<std::string>("arm_1.arm_id", "left");
    auto_declare<std::string>("arm_2.arm_id", "right");

    // Per-arm impedance gains (element-wise)
    auto_declare<std::vector<double>>("arm_1.k_gains", std::vector<double>(7, 50.0));
    auto_declare<std::vector<double>>("arm_1.d_gains", std::vector<double>(7, 5.0));
    auto_declare<std::vector<double>>("arm_2.k_gains", std::vector<double>(7, 50.0));
    auto_declare<std::vector<double>>("arm_2.d_gains", std::vector<double>(7, 5.0));
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception during init: %s\n", e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Initialized DualJointImpedanceController");
  return CallbackReturn::SUCCESS;
}

CallbackReturn DualJointImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  const auto arm_id_1 = this->get_node()->get_parameter("arm_1.arm_id").as_string();
  const auto arm_id_2 = this->get_node()->get_parameter("arm_2.arm_id").as_string();
  const auto source_ns = auto_declare<std::string>("source_namespace", "parti");
  
  if (arm_id_1.empty() || arm_id_2.empty() || arm_id_1 == arm_id_2) {
    RCLCPP_FATAL(this->get_node()->get_logger(),
                 "Invalid or non-unique arm IDs: '%s' vs '%s'",
                 arm_id_1.c_str(), arm_id_2.c_str());
    return CallbackReturn::ERROR;
  }

  arms_.clear();
  arms_.insert({arm_id_1, ArmContainer()});
  arms_.insert({arm_id_2, ArmContainer()});

  // Copy gains from parameters and initialize vectors
  int idx = 1;
  for (auto& kv : arms_) {
    auto& arm_id = kv.first;
    auto& arm    = kv.second;

    arm.arm_id_ = arm_id;

    const auto k_vec =
        this->get_node()->get_parameter("arm_" + std::to_string(idx) + ".k_gains").as_double_array();
    const auto d_vec =
        this->get_node()->get_parameter("arm_" + std::to_string(idx) + ".d_gains").as_double_array();

    if (k_vec.size() != static_cast<size_t>(num_joints) ||
        d_vec.size() != static_cast<size_t>(num_joints)) {
      RCLCPP_FATAL(get_node()->get_logger(),
                   "arm_%d gains must have %d elements; got K:%zu, D:%zu",
                   idx, num_joints, k_vec.size(), d_vec.size());
      return CallbackReturn::ERROR;
    }

    for (int j = 0; j < num_joints; ++j) {
      arm.k_gains_(j) = k_vec[j];
      arm.d_gains_(j) = d_vec[j];
    }

    arm.q_.setZero();
    arm.dq_.setZero();
    arm.dq_filtered_.setZero();
    arm.q_des_.setZero();
    arm.initial_q_.setZero();

    ++idx;
  }

  RCLCPP_INFO(get_node()->get_logger(),
              "Configured DualJointImpedanceController for arms '%s' and '%s'",
              arm_id_1.c_str(), arm_id_2.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn DualJointImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  updateJointStates();
  for (auto& kv : arms_) {
    kv.second.initial_q_ = kv.second.q_;
  }
  start_time_ = this->get_node()->now();

  // Subscribe to desired joint positions (unordered names)
  sub_desired_joint_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      "/parti/joint_states",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&DualJointImpedanceController::desiredJointCallback, this, std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

CallbackReturn DualJointImpedanceController::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_ERROR(this->get_node()->get_logger(), "DualJointImpedanceController encountered an error");
  return CallbackReturn::ERROR;
}

/*===========================  Control Loop  ===============================*/

controller_interface::return_type DualJointImpedanceController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  // Read current joint states
  updateJointStates();

  // Impedance torque: tau = K ∘ (q_des - q) + D ∘ ( - dq_filtered )
  constexpr double alpha = 0.99;  // EMA smoothing for dq

  size_t k_cmd = 0;
  for (auto& kv : arms_) {
    auto& arm = kv.second;

    arm.dq_filtered_ = alpha * arm.dq_filtered_ + (1.0 - alpha) * arm.dq_;

    Eigen::Matrix<double, 7, 1> tau =
        arm.k_gains_.cwiseProduct(arm.q_des_ - arm.q_) +
        arm.d_gains_.cwiseProduct(-arm.dq_filtered_);

    for (int j = 0; j < num_joints; ++j, ++k_cmd) {
      command_interfaces_[k_cmd].set_value(tau(j));
    }
  }

  return controller_interface::return_type::OK;
}

/*===========================  Helpers & I/O  ==============================*/

// Safe pair-stepping over state interfaces: [pos0, vel0, pos1, vel1, ...]
void DualJointImpedanceController::updateJointStates()
{
  for (auto& kv : arms_) {
    const std::string& arm_id = kv.first;
    auto& arm = kv.second;

    int k = 0;
    for (size_t i = 0; i + 1 < state_interfaces_.size(); i += 2) {
      const auto& pos = state_interfaces_.at(i);
      const auto& vel = state_interfaces_.at(i + 1);

      if (pos.get_prefix_name().find(arm_id) == std::string::npos ||
          vel.get_prefix_name().find(arm_id) == std::string::npos) {
        continue;
      }

      // Expect exact pair position/velocity
      if (pos.get_interface_name() != "position" || vel.get_interface_name() != "velocity") {
        continue;
      }

      if (k >= num_joints) break;

      arm.q_(k)  = pos.get_value();
      arm.dq_(k) = vel.get_value();
      ++k;
    }
  }
}

// IMPORTANT: joint states are received in unstructured order; For this to work, it expects "<arm_id>_jointN"
void DualJointImpedanceController::desiredJointCallback(const sensor_msgs::msg::JointState& msg)
{
  if (msg.name.size() != msg.position.size()) return;

  for (size_t i = 0; i < msg.name.size(); ++i) {
    const std::string& full = msg.name[i];     // e.g., "left_joint3"
    const auto pos = full.rfind("_joint");
    if (pos == std::string::npos) continue;

    const std::string arm_id = full.substr(0, pos);      // "left"
    const char* idx_str = full.c_str() + pos + 6;        // points to "3"
    int j = std::atoi(idx_str) - 1;                      // "3" -> 2

    auto it = arms_.find(arm_id);
    if (it == arms_.end() || j < 0 || j >= num_joints) continue;

    it->second.q_des_(j) = msg.position[i];
  }
}

}  // namespace garmi_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(garmi_controllers::DualJointImpedanceController,
                       controller_interface::ControllerInterface)
