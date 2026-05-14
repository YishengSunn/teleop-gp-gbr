#include "teleop_controllers/subscriber/gravity_compensation_with_joint_torque_feedback_controller.hpp"

#include <cassert>
#include <cmath>
#include <exception>
#include <limits>

#include <pluginlib/class_list_macros.hpp>

namespace teleop_controllers {

namespace {
inline bool isFinite(double x) { return std::isfinite(x); }

inline double clampAbs(double x, double max_abs) {
  if (!isFinite(x)) return 0.0;
  if (x >  max_abs) return  max_abs;
  if (x < -max_abs) return -max_abs;
  return x;
}
}  // namespace

controller_interface::InterfaceConfiguration
GravityCompensationWithJointTorqueFeedbackController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
GravityCompensationWithJointTorqueFeedbackController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

CallbackReturn GravityCompensationWithJointTorqueFeedbackController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<bool>("move_to_start", true);

    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    auto_declare<std::vector<double>>(
      "start_joint_configuration",
      {0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4});

    // master enable for torque feedback
    auto_declare<bool>("enable_feedback", true);

    // single feedback topic (can be tau_ext_hat_filtered OR tau_j_d etc.)
    auto_declare<std::string>("torque_feedback_topic", "/filtered_external_tau");

    // Bias removal
    auto_declare<bool>("subtract_first_bias", true);

    // Scaling / clamping
    auto_declare<double>("feedback_scale", 1.0);
    auto_declare<double>("feedback_max_abs_tau", 30.0);

    // If true, add feedback on top during move-to-start
    auto_declare<bool>("feedback_additive", false);

  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn GravityCompensationWithJointTorqueFeedbackController::on_configure(
  const rclcpp_lifecycle::State& /*previous_state*/) {

  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  move_to_start_ = get_node()->get_parameter("move_to_start").as_bool();

  enable_feedback_ = get_node()->get_parameter("enable_feedback").as_bool();
  torque_feedback_topic_ = get_node()->get_parameter("torque_feedback_topic").as_string();

  subtract_first_bias_ = get_node()->get_parameter("subtract_first_bias").as_bool();

  feedback_scale_ = get_node()->get_parameter("feedback_scale").as_double();
  feedback_max_abs_tau_ = get_node()->get_parameter("feedback_max_abs_tau").as_double();
  feedback_additive_ = get_node()->get_parameter("feedback_additive").as_bool();

  // Move-to-start goal
  auto start_joint_configuration_vector =
    get_node()->get_parameter("start_joint_configuration").as_double_array();

  Eigen::Map<Eigen::VectorXd>(q_goal_.data(), kNumJoints) =
    Eigen::Map<Eigen::VectorXd>(start_joint_configuration_vector.data(), kNumJoints);

  // Gains only required if move_to_start enabled
  if (move_to_start_) {
    auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
    auto d_gains = get_node()->get_parameter("d_gains").as_double_array();

    if (k_gains.empty() || k_gains.size() != static_cast<size_t>(kNumJoints)) {
      RCLCPP_FATAL(get_node()->get_logger(), "k_gains not set or wrong size");
      return CallbackReturn::FAILURE;
    }
    if (d_gains.empty() || d_gains.size() != static_cast<size_t>(kNumJoints)) {
      RCLCPP_FATAL(get_node()->get_logger(), "d_gains not set or wrong size");
      return CallbackReturn::FAILURE;
    }

    for (int i = 0; i < kNumJoints; ++i) {
      k_gains_(i) = k_gains.at(i);
      d_gains_(i) = d_gains.at(i);
    }
  }

  dq_filtered_.setZero();

  // reset bias each configure (and set NaNs in buffer)
  bias_initialized_ = false;
  tau_bias_.setZero();

  Vector7d init_tau;
  init_tau.setConstant(std::numeric_limits<double>::quiet_NaN());
  tau_feedback_rt_.writeFromNonRT(init_tau);

  // (Re)subscribe only if feedback enabled
  tau_sub_.reset();
  if (enable_feedback_) {
    tau_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      torque_feedback_topic_,
      rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { onTauArray(*msg); });

    RCLCPP_INFO(get_node()->get_logger(),
                "Torque feedback ENABLED, subscribing to '%s', subtract_first_bias=%s",
                torque_feedback_topic_.c_str(),
                subtract_first_bias_ ? "true" : "false");
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Torque feedback DISABLED (enable_feedback=false)");
  }

  mode_ = move_to_start_ ? Mode::MOVE_TO_START : Mode::FEEDBACK_GRAVITY;
  return CallbackReturn::SUCCESS;
}

CallbackReturn GravityCompensationWithJointTorqueFeedbackController::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/) {

  updateJointStates();

  if (move_to_start_) {
    motion_generator_ = std::make_unique<MotionGenerator>(0.2, q_, q_goal_);
    start_time_ = this->get_node()->now();
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type GravityCompensationWithJointTorqueFeedbackController::update(
  const rclcpp::Time& /*time*/,
  const rclcpp::Duration& /*period*/) {

  updateJointStates();

  // If feedback is disabled: command 0 always (pure gravity compensation behavior)
  if (!enable_feedback_) {
    for (int i = 0; i < kNumJoints; ++i) {
      command_interfaces_[i].set_value(0.0);
    }
    return controller_interface::return_type::OK;
  }

  // Read latest feedback torques (RT-safe)
  const Vector7d tau_fb = *(tau_feedback_rt_.readFromRT());

  auto write_feedback_or_zero = [&]() {
    for (int i = 0; i < kNumJoints; ++i) {
      double t = tau_fb(i);
      if (!isFinite(t)) t = 0.0;
      t = feedback_scale_ * clampAbs(t, feedback_max_abs_tau_);
      command_interfaces_[i].set_value(t);
    }
  };

  if (mode_ == Mode::MOVE_TO_START) {
    auto trajectory_time = this->get_node()->now() - start_time_;
    auto out = motion_generator_->getDesiredJointPositions(trajectory_time);
    const Vector7d q_desired = out.first;
    const bool finished = out.second;

    if (!finished) {
      const double kAlpha = 0.99;
      dq_filtered_ = (1.0 - kAlpha) * dq_filtered_ + kAlpha * dq_;

      const Vector7d tau_move =
        k_gains_.cwiseProduct(q_desired - q_) + d_gains_.cwiseProduct(-dq_filtered_);

      if (feedback_additive_) {
        for (int i = 0; i < kNumJoints; ++i) {
          double t = tau_fb(i);
          if (!isFinite(t)) t = 0.0;
          t = feedback_scale_ * clampAbs(t, feedback_max_abs_tau_);
          command_interfaces_[i].set_value(tau_move(i) + t);
        }
      } else {
        for (int i = 0; i < kNumJoints; ++i) {
          command_interfaces_[i].set_value(tau_move(i));
        }
      }
    } else {
      mode_ = Mode::FEEDBACK_GRAVITY;

      // reset bias when switching modes too
      bias_initialized_ = false;
      tau_bias_.setZero();

      write_feedback_or_zero();
    }

    return controller_interface::return_type::OK;
  }

  write_feedback_or_zero();
  return controller_interface::return_type::OK;
}

void GravityCompensationWithJointTorqueFeedbackController::updateJointStates() {
  for (int i = 0; i < kNumJoints; ++i) {
    const auto& pos = state_interfaces_.at(2 * i);
    const auto& vel = state_interfaces_.at(2 * i + 1);

    assert(pos.get_interface_name() == "position");
    assert(vel.get_interface_name() == "velocity");

    q_(i) = pos.get_value();
    dq_(i) = vel.get_value();
  }
}

void GravityCompensationWithJointTorqueFeedbackController::onTauArray(
  const std_msgs::msg::Float64MultiArray& msg) {

  // If feedback is disabled, ignore callbacks (extra safety)
  if (!enable_feedback_) {
    return;
  }

  const auto& d = msg.data;

  if (d.size() != static_cast<size_t>(kNumJoints)) {
    Vector7d tau;
    tau.setConstant(std::numeric_limits<double>::quiet_NaN());
    tau_feedback_rt_.writeFromNonRT(tau);
    return;
  }

  Vector7d tau_in;
  for (int i = 0; i < kNumJoints; ++i) {
    tau_in(i) = d[static_cast<size_t>(i)];
  }

  // Initialize bias from first *fully finite* sample
  if (subtract_first_bias_ && !bias_initialized_) {
    bool ok = true;
    for (int i = 0; i < kNumJoints; ++i) {
      if (!std::isfinite(tau_in(i))) { ok = false; break; }
    }
    if (ok) {
      tau_bias_ = tau_in;
      bias_initialized_ = true;
    }
  }

  Vector7d tau;
  if (subtract_first_bias_ && bias_initialized_) {
    tau = -(tau_in - tau_bias_);
  } else {
    tau = -tau_in;
  }
  tau_feedback_rt_.writeFromNonRT(tau);
}

}  // namespace teleop_controllers

PLUGINLIB_EXPORT_CLASS(
  teleop_controllers::GravityCompensationWithJointTorqueFeedbackController,
  controller_interface::ControllerInterface)
