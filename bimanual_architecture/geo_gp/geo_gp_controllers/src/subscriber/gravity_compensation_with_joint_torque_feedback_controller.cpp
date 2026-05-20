#include "geo_gp_controllers/subscriber/gravity_compensation_with_joint_torque_feedback_controller.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <exception>
#include <limits>

#include <franka/rate_limiting.h>
#include <pluginlib/class_list_macros.hpp>

namespace geo_gp_controllers {

namespace {
inline bool isFinite(double x) { return std::isfinite(x); }

inline double clampAbs(double x, double max_abs) {
  if (!isFinite(x)) return 0.0;
  if (x > max_abs) return max_abs;
  if (x < -max_abs) return -max_abs;
  return x;
}

inline double safePeriodSeconds(const rclcpp::Duration& period) {
  const double dt = period.seconds();
  if (!std::isfinite(dt) || dt <= 0.0) {
    return 0.001;
  }
  return dt;
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

  if (franka_robot_model_) {
    for (const auto& name : franka_robot_model_->get_state_interface_names()) {
      config.names.push_back(name);
    }
  } else if (TDPA_active_) {
    franka_semantic_components::FrankaRobotModel tmp_model(arm_id_ + "/robot_model", arm_id_);
    for (const auto& name : tmp_model.get_state_interface_names()) {
      config.names.push_back(name);
    }
  }

  if (franka_robot_state_) {
    for (const auto& name : franka_robot_state_->get_state_interface_names()) {
      config.names.push_back(name);
    }
  } else if (TDPA_active_) {
    franka_semantic_components::FrankaRobotState tmp_state(arm_id_ + "/robot_state", arm_id_);
    for (const auto& name : tmp_state.get_state_interface_names()) {
      config.names.push_back(name);
    }
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

    auto_declare<bool>("enable_feedback", true);
    auto_declare<std::string>("feedback_source", "measured");
    auto_declare<std::string>("torque_feedback_topic", "");
    auto_declare<bool>("subtract_first_bias", true);
    auto_declare<double>("feedback_scale", 1.0);
    auto_declare<double>("feedback_max_abs_tau", 30.0);
    auto_declare<bool>("feedback_additive", false);
    auto_declare<bool>("suppress_feedback_during_execution", true);
    auto_declare<std::string>("execution_running_topic", "/execution/running");
    auto_declare<std::string>("blend_running_topic", "/execution/blend_to_leader_running");
    auto_declare<double>("execution_feedback_scale", 0.0);

    auto_declare<bool>("TDPA_active", false);
    auto_declare<bool>("tau_ext_feedback", true);
    auto_declare<double>("eta_passivity_shortage", 0.0);
    auto_declare<std::string>("remote_state_topic", "follower/tdpa_joint_state");
    auto_declare<std::string>("local_state_topic", "leader/tdpa_joint_state");
  }
  catch (const std::exception& e) {
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
  feedback_source_ = get_node()->get_parameter("feedback_source").as_string();
  torque_feedback_topic_ = get_node()->get_parameter("torque_feedback_topic").as_string();

  TDPA_active_ = get_node()->get_parameter("TDPA_active").as_bool();
  tau_ext_feedback_ = get_node()->get_parameter("tau_ext_feedback").as_bool();
  eta_ = get_node()->get_parameter("eta_passivity_shortage").as_double();
  remote_state_topic_ = get_node()->get_parameter("remote_state_topic").as_string();
  local_state_topic_ = get_node()->get_parameter("local_state_topic").as_string();

  if (torque_feedback_topic_.empty() && !TDPA_active_) {
    if (feedback_source_ == "commanded") {
      torque_feedback_topic_ = "/follower/franka_torque_broadcaster/tau_j_d";
    } else if (feedback_source_ == "measured") {
      torque_feedback_topic_ = "/follower/franka_torque_broadcaster/tau_ext_hat_filtered";
    } else {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Invalid feedback_source='%s'. Supported values: commanded, measured",
        feedback_source_.c_str());
      return CallbackReturn::FAILURE;
    }
  }

  subtract_first_bias_ = get_node()->get_parameter("subtract_first_bias").as_bool();
  feedback_scale_ = get_node()->get_parameter("feedback_scale").as_double();
  feedback_max_abs_tau_ = get_node()->get_parameter("feedback_max_abs_tau").as_double();
  feedback_additive_ = get_node()->get_parameter("feedback_additive").as_bool();

  suppress_feedback_during_execution_ =
    get_node()->get_parameter("suppress_feedback_during_execution").as_bool();
  execution_running_topic_ = get_node()->get_parameter("execution_running_topic").as_string();
  blend_running_topic_ = get_node()->get_parameter("blend_running_topic").as_string();
  execution_feedback_scale_ = get_node()->get_parameter("execution_feedback_scale").as_double();

  if (TDPA_active_) {
    franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id_ + "/robot_model", arm_id_));
    franka_robot_state_ = std::make_unique<franka_semantic_components::FrankaRobotState>(
      franka_semantic_components::FrankaRobotState(arm_id_ + "/robot_state", arm_id_));
    tdpaReset_();
  }

  auto start_joint_configuration_vector =
    get_node()->get_parameter("start_joint_configuration").as_double_array();

  if (start_joint_configuration_vector.size() != static_cast<std::size_t>(kNumJoints)) {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "start_joint_configuration must have size %d",
      kNumJoints);
    return CallbackReturn::FAILURE;
  }

  q_goal_ = Eigen::Map<const Vector7d>(start_joint_configuration_vector.data());

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
  bias_initialized_ = false;
  tau_bias_.setZero();

  Vector7d init_tau;
  init_tau.setConstant(std::numeric_limits<double>::quiet_NaN());
  tau_feedback_rt_.writeFromNonRT(init_tau);

  tau_sub_.reset();
  execution_running_sub_.reset();
  blend_running_sub_.reset();
  remote_state_sub_.reset();
  local_state_pub_.reset();
  local_state_pub_raw_.reset();
  execution_running_.store(false, std::memory_order_release);
  blend_running_.store(false, std::memory_order_release);

  if (TDPA_active_) {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    remote_state_sub_ =
      get_node()->create_subscription<geo_gp_interfaces::msg::TDPAJointState>(
        remote_state_topic_, qos,
        std::bind(&GravityCompensationWithJointTorqueFeedbackController::remoteStateCallback,
                  this, std::placeholders::_1));
    local_state_pub_raw_ =
      get_node()->create_publisher<geo_gp_interfaces::msg::TDPAJointState>(
        local_state_topic_, qos);
    local_state_pub_ =
      std::make_unique<
        realtime_tools::RealtimePublisher<geo_gp_interfaces::msg::TDPAJointState>>(
          local_state_pub_raw_);
  } else if (enable_feedback_) {
    tau_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      torque_feedback_topic_,
      rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { onTauArray(*msg); });

    if (suppress_feedback_during_execution_) {
      execution_running_sub_ =
        get_node()->create_subscription<std_msgs::msg::Bool>(
          execution_running_topic_,
          rclcpp::QoS(1).transient_local(),
          [this](const std_msgs::msg::Bool::SharedPtr msg) { onExecutionRunning(*msg); });
      blend_running_sub_ =
        get_node()->create_subscription<std_msgs::msg::Bool>(
          blend_running_topic_,
          rclcpp::QoS(1).transient_local(),
          [this](const std_msgs::msg::Bool::SharedPtr msg) { onBlendRunning(*msg); });
    }
  }

  mode_ = move_to_start_ ? Mode::MOVE_TO_START : Mode::FEEDBACK_GRAVITY;
  return CallbackReturn::SUCCESS;
}

CallbackReturn GravityCompensationWithJointTorqueFeedbackController::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/) {

  updateJointStates();

  if (TDPA_active_) {
    franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
    franka_robot_state_->assign_loaned_state_interfaces(state_interfaces_);
    pandatime_ = 0.0;
    remote_pandatime_ = 0.0;
    local_seq_ = 0;
    last_received_remote_seq_ = 0;
    last_received_remote_tx_time_ns_ = 0;
    tdpaReset_();
    last_tau_cmd_.setZero();
    last_tau_cmd_initialized_ = false;
  }

  if (move_to_start_) {
    motion_generator_ = std::make_unique<MotionGenerator>(0.2, q_, q_goal_);
    start_time_ = this->get_node()->now();
  }

  mode_ = move_to_start_ ? Mode::MOVE_TO_START : Mode::FEEDBACK_GRAVITY;
  return CallbackReturn::SUCCESS;
}

CallbackReturn GravityCompensationWithJointTorqueFeedbackController::on_deactivate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  if (franka_robot_model_) {
    franka_robot_model_->release_interfaces();
  }
  if (franka_robot_state_) {
    franka_robot_state_->release_interfaces();
  }
  return CallbackReturn::SUCCESS;
}

GravityCompensationWithJointTorqueFeedbackController::Vector7d
GravityCompensationWithJointTorqueFeedbackController::computeTdpaFeedbackTorque_(double dt) {
  const auto* robot_state = franka_robot_state_->get_robot_state_ptr();

  for (int i = 0; i < kNumJoints; ++i) {
    tau_ext_(i) = -robot_state->tau_ext_hat_filtered[static_cast<std::size_t>(i)];
  }

  for (int i = 0; i < 6; ++i) {
    f_local_(i) = robot_state->O_F_ext_hat_K[static_cast<std::size_t>(i)];
  }

  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state->O_T_EE.data()));
  position_ = transform.translation();

  dq_local_ = dq_;

  for (int i = 0; i < kNumJoints; ++i) {
    q_local_delta_(i) = q_(i) - q_goal_(i);
  }

  Vector7d tau_remote_in = tau_remote_;
  if (!tdpa_remote_ready_.load(std::memory_order_acquire)) {
    return Vector7d::Zero();
  }

  Vector7d tau_d_calculated = Vector7d::Zero();
  Vector7d tau_local = tau_remote_in;

  double dq_local_double[7]{};
  double tau_remote_double[7]{};
  double tau_local_double[7]{};

  const double gain_dq_local = tau_ext_feedback_ ? -1.0 : 1.0;

  for (int i = 0; i < kNumJoints; ++i) {
    dq_local_double[i] = gain_dq_local * dq_local_(i);
    tau_remote_double[i] = tau_remote_in(i);
    tau_local_double[i] = tau_remote_double[i];
  }

  leaderPC_.energyObserver(dq_local_double, tau_remote_double, dt);
  std::memcpy(tau_local_double, tau_remote_double, 7 * sizeof(double));

  const Matrix7d friction = Matrix7d::Zero();
  const double dissipation = dq_.transpose() * friction * dq_;
  shortage_ += dt * eta_ * dissipation;

  leaderPC_.energyController(
    tau_local_double,
    dq_local_double,
    E_F_in_delayed_ + shortage_,
    dt);

  for (int i = 0; i < kNumJoints; ++i) {
    tau_diff_(i) = tau_remote_double[i] - tau_local_double[i];
    tau_local(i) = tau_local_double[i];
  }

  alpha_ = leaderPC_.getAlpha();
  E_L_in_ = leaderPC_.getInputEnergyFlow();
  E_L_out_ = leaderPC_.getOutputEnergyFlow();
  E_L_diss_ = leaderPC_.getDissipatedEnergyFlow();

  const double gain_tau_ld = tau_ext_feedback_ ? 1.0 : -1.0;

  for (int i = 0; i < kNumJoints; ++i) {
    tau_d_calculated(i) = gain_tau_ld * tau_local(i);
  }

  return tau_d_calculated;
}

controller_interface::return_type GravityCompensationWithJointTorqueFeedbackController::update(
  const rclcpp::Time& /*time*/,
  const rclcpp::Duration& period) {

  updateJointStates();

  if (TDPA_active_) {
    const double dt = safePeriodSeconds(period);
    pandatime_ += dt;

    auto apply_tau = [&](const Vector7d& tau_cmd) {
      std::array<double, 7> tau_des{};
      std::array<double, 7> tau_prev{};
      for (int i = 0; i < kNumJoints; ++i) {
        tau_des[static_cast<std::size_t>(i)] = tau_cmd(i);
        tau_prev[static_cast<std::size_t>(i)] =
          last_tau_cmd_initialized_ ? last_tau_cmd_(i) : tau_cmd(i);
      }
      const auto tau_limited =
        franka::limitRate(franka::kMaxTorqueRate, tau_des, tau_prev);
      for (int i = 0; i < kNumJoints; ++i) {
        command_interfaces_[i].set_value(tau_limited[static_cast<std::size_t>(i)]);
        last_tau_cmd_(i) = tau_limited[static_cast<std::size_t>(i)];
      }
      last_tau_cmd_initialized_ = true;
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
        apply_tau(tau_move);
        return controller_interface::return_type::OK;
      }

      mode_ = Mode::FEEDBACK_GRAVITY;
      tdpaReset_();
    }

    const Vector7d tau_cmd = computeTdpaFeedbackTorque_(dt);
    apply_tau(tau_cmd);

    if (local_state_pub_ && local_state_pub_->trylock()) {
      auto& msg = local_state_pub_->msg_;
      const rclcpp::Time now = get_node()->now();
      const int64_t now_ns = now.nanoseconds();

      msg.header.stamp = now;
      msg.seq = local_seq_++;
      msg.tx_time_ns = now_ns;
      msg.echo_seq = last_received_remote_seq_;
      msg.echo_tx_time_ns = last_received_remote_tx_time_ns_;
      msg.pandatime = pandatime_;

      for (int i = 0; i < kNumJoints; ++i) {
        msg.q_local_delta[static_cast<std::size_t>(i)] = q_local_delta_(i);
        msg.dq_local[static_cast<std::size_t>(i)] = dq_local_(i);
        msg.tau_local[static_cast<std::size_t>(i)] =
          tau_ext_feedback_ ? tau_ext_(i) : last_tau_cmd_(i);
      }

      msg.energy = E_L_in_;
      local_state_pub_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
  }

  if (!enable_feedback_) {
    for (int i = 0; i < kNumJoints; ++i) {
      command_interfaces_[i].set_value(0.0);
    }
    return controller_interface::return_type::OK;
  }

  const Vector7d tau_fb = *(tau_feedback_rt_.readFromRT());
  const double eff_scale = effectiveFeedbackScale();

  auto write_feedback_or_zero = [&]() {
    for (int i = 0; i < kNumJoints; ++i) {
      double tau_fb_i = tau_fb(i);
      if (!isFinite(tau_fb_i)) tau_fb_i = 0.0;
      tau_fb_i = eff_scale * clampAbs(tau_fb_i, feedback_max_abs_tau_);
      command_interfaces_[i].set_value(tau_fb_i);
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
          double tau_fb_i = tau_fb(i);
          if (!isFinite(tau_fb_i)) tau_fb_i = 0.0;
          tau_fb_i = eff_scale * clampAbs(tau_fb_i, feedback_max_abs_tau_);
          command_interfaces_[i].set_value(tau_move(i) + tau_fb_i);
        }
      } else {
        for (int i = 0; i < kNumJoints; ++i) {
          command_interfaces_[i].set_value(tau_move(i));
        }
      }
    } else {
      mode_ = Mode::FEEDBACK_GRAVITY;
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

  if (subtract_first_bias_ && !bias_initialized_) {
    bool ok = true;
    for (int i = 0; i < kNumJoints; ++i) {
      if (!std::isfinite(tau_in(i))) {
        ok = false;
        break;
      }
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

void GravityCompensationWithJointTorqueFeedbackController::onExecutionRunning(
  const std_msgs::msg::Bool& msg) {
  execution_running_.store(msg.data, std::memory_order_release);
}

void GravityCompensationWithJointTorqueFeedbackController::onBlendRunning(
  const std_msgs::msg::Bool& msg) {
  blend_running_.store(msg.data, std::memory_order_release);
}

void GravityCompensationWithJointTorqueFeedbackController::remoteStateCallback(
  const geo_gp_interfaces::msg::TDPAJointState& msg) {
  remote_pandatime_ = msg.pandatime;
  last_received_remote_seq_ = msg.seq;
  last_received_remote_tx_time_ns_ = msg.tx_time_ns;

  for (int i = 0; i < kNumJoints; ++i) {
    q_remote_delta_(i) = msg.q_local_delta[static_cast<std::size_t>(i)];
    dq_remote_(i) = msg.dq_local[static_cast<std::size_t>(i)];
    tau_remote_(i) = msg.tau_local[static_cast<std::size_t>(i)];
  }

  if (std::isfinite(msg.energy)) {
    E_F_in_delayed_ = msg.energy;
  }

  tdpa_remote_ready_.store(true, std::memory_order_release);
}

double GravityCompensationWithJointTorqueFeedbackController::effectiveFeedbackScale() const {
  if (!suppress_feedback_during_execution_) {
    return feedback_scale_;
  }
  const bool feedback_suppressed =
    execution_running_.load(std::memory_order_acquire) ||
    blend_running_.load(std::memory_order_acquire);
  if (!feedback_suppressed) {
    return feedback_scale_;
  }
  return feedback_scale_ * execution_feedback_scale_;
}

}  // namespace geo_gp_controllers

PLUGINLIB_EXPORT_CLASS(
  geo_gp_controllers::GravityCompensationWithJointTorqueFeedbackController,
  controller_interface::ControllerInterface)
