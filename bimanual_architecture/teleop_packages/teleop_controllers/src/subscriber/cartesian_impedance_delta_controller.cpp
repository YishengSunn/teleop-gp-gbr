#include <teleop_controllers/subscriber/cartesian_impedance_delta_controller.hpp>

#include <cmath>
#include <exception>
#include <string>

#include <franka/model.h>
#include <pluginlib/class_list_macros.hpp>

namespace teleop_controllers {

controller_interface::InterfaceConfiguration
CartesianImpedanceDeltaController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
CartesianImpedanceDeltaController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto& name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(name);
  }
  return config;
}

controller_interface::return_type
CartesianImpedanceDeltaController::update(const rclcpp::Time& time,
                                          const rclcpp::Duration& /*period*/) {
  // --- Robot state/model (franka semantic components) ---
  Eigen::Map<const Matrix4d> current(
      franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data());
  Vector3d current_position(current.block<3, 1>(0, 3));
  Quaterniond current_orientation(current.block<3, 3>(0, 0));

  Eigen::Map<const Vector7d> coriolis(franka_robot_model_->getCoriolisForceVector().data());
  Eigen::Matrix<double, 6, 7> jacobian(
      franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector).data());
  Eigen::Map<const Vector7d> qD(franka_robot_model_->getRobotState()->dq.data());
  Eigen::Map<const Vector7d> q(franka_robot_model_->getRobotState()->q.data());

  bool skip_delta_apply_this_cycle = false;

  const auto mode_now = static_cast<Mode>(mode_.load(std::memory_order_relaxed));
  if (mode_now == Mode::MOVE_TO_START) {
    // Latch start time from update() time source (fixes "different time sources" crash)
    if (!start_time_set_) {
      start_time_ = time;
      start_time_set_ = true;
    }

    const auto t = time - start_time_;  // same clock type => safe
    const auto out = motion_generator_->getDesiredJointPositions(t);

    const Vector7d q_desired = out.first;
    const bool finished = out.second;

    if (!finished) {
      const double kAlpha = 0.99;
      dq_filtered_ = (1.0 - kAlpha) * dq_filtered_ + kAlpha * qD;

      const Vector7d tau =
          k_start_.cwiseProduct(q_desired - q) +
          d_start_.cwiseProduct(-dq_filtered_);

      for (int i = 0; i < num_joints; ++i) {
        command_interfaces_[i].set_value(tau(i));
      }
      return controller_interface::return_type::OK;
    }

    // Finished: seed desired pose from current EE
    current_orientation.normalize();
    desired_position_ = current_position;
    desired_orientation_ = current_orientation;
    desired_qn_ = q;

    // Mark all deltas as already handled; DO NOT write to RealtimeBuffer from RT thread.
    last_delta_seq_ = delta_seq_.load(std::memory_order_acquire);

    accept_delta_.store(true, std::memory_order_release);
    mode_.store(static_cast<uint8_t>(Mode::DELTA_CARTESIAN), std::memory_order_release);

    // Don't apply delta in the same cycle as the switch.
    skip_delta_apply_this_cycle = true;
  }

  // Apply at most one delta per update cycle (latest-wins), only if a NEW msg arrived.
  if (!skip_delta_apply_this_cycle && accept_delta_.load(std::memory_order_relaxed)) {
    const uint64_t seq = delta_seq_.load(std::memory_order_acquire);
    if (seq != last_delta_seq_) {
      const auto* cmd = delta_buffer_.readFromRT();
      if (cmd && cmd->valid) {
        Vector3d dp(cmd->dx, cmd->dy, cmd->dz);
        const double dp_norm = dp.norm();

        Matrix3d R_delta;
        R_delta << cmd->R[0], cmd->R[1], cmd->R[2],
                   cmd->R[3], cmd->R[4], cmd->R[5],
                   cmd->R[6], cmd->R[7], cmd->R[8];

        Eigen::AngleAxisd aa(R_delta);
        const double d_angle = std::fabs(aa.angle());

        const bool apply_translation = (std::isfinite(dp_norm) && dp_norm >= delta_deadzone_);
        const bool apply_orientation = (std::isfinite(d_angle) && d_angle > 1e-6);

        if (dp_norm > delta_max_step_) {
          dp *= (delta_max_step_ / dp_norm);
        }
        desired_position_ += dp;

        Matrix3d R_des = desired_orientation_.toRotationMatrix();
        R_des = R_delta * R_des;
        desired_orientation_ = Quaterniond(R_des);
        desired_orientation_.normalize();

        // Consume this delta (so if publisher stops, we don't re-apply it).
        DeltaCmdRT consumed = *cmd;
        consumed.valid = false;
        delta_buffer_.writeFromNonRT(consumed);
      }

      // Consume by sequence only (RT-safe)
      last_delta_seq_ = seq;
    }
  }

  // --- Cartesian impedance ---
  Vector6d error;
  error.head(3) = current_position - desired_position_;

  if (desired_orientation_.coeffs().dot(current_orientation.coeffs()) < 0.0) {
    current_orientation.coeffs() = -current_orientation.coeffs();
  }

  Quaterniond rot_error(current_orientation * desired_orientation_.inverse());
  Eigen::AngleAxisd rot_error_aa(rot_error);
  error.tail(3) = rot_error_aa.axis() * rot_error_aa.angle();

  Vector7d tau_task = jacobian.transpose() *
                      (-stiffness_ * error - damping_ * (jacobian * qD));

  // --- RT-friendly nullspace projector (DLS), replaces SVD pseudoInverse ---
  const double lambda = std::max(0.0, nullspace_damping_lambda_);
  Eigen::Matrix<double, 6, 6> A = jacobian * jacobian.transpose();
  A.diagonal().array() += lambda * lambda;

  // pinv(J^T) ≈ (J J^T + λ² I)^{-1} J   (6x7)
  Eigen::Matrix<double, 6, 7> pinv_Jt = A.ldlt().solve(jacobian);

  // P = I - J^T * pinv(J^T)
  Matrix7d P = Matrix7d::Identity() - jacobian.transpose() * pinv_Jt;

  Vector7d tau_nullspace =
      P * (n_stiffness_ * (desired_qn_ - q) - (2.0 * std::sqrt(n_stiffness_)) * qD);

  Vector7d tau_d = tau_task + coriolis + tau_nullspace;

  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
  }

  return controller_interface::return_type::OK;
}

CallbackReturn CartesianImpedanceDeltaController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<double>("pos_stiff", 100.0);
    auto_declare<double>("rot_stiff", 10.0);
    auto_declare<double>("n_stiffness", 10.0);

    auto_declare<std::string>("cartesian_delta_topic", "/cartesian_impedance/pose_delta");
    auto_declare<double>("delta_deadzone", 1e-4);
    auto_declare<double>("delta_max_step", 0.05);

    auto_declare<double>("nullspace_damping_lambda", 0.2);

    auto_declare<bool>("move_to_start", false);
    auto_declare<std::vector<double>>(
        "start_joint_configuration",
        {0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4});

    auto_declare<std::vector<double>>("start_k_gains", {});
    auto_declare<std::vector<double>>("start_d_gains", {});
  } catch (const std::exception& e) {
    std::fprintf(stderr,
                 "Exception thrown during init stage with message: %s \n",
                 e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceDeltaController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();

  pos_stiff_ = get_node()->get_parameter("pos_stiff").as_double();
  rot_stiff_ = get_node()->get_parameter("rot_stiff").as_double();
  n_stiffness_ = get_node()->get_parameter("n_stiffness").as_double();

  cartesian_delta_topic_ = get_node()->get_parameter("cartesian_delta_topic").as_string();
  delta_deadzone_ = get_node()->get_parameter("delta_deadzone").as_double();
  delta_max_step_ = get_node()->get_parameter("delta_max_step").as_double();

  nullspace_damping_lambda_ = get_node()->get_parameter("nullspace_damping_lambda").as_double();

  move_to_start_ = get_node()->get_parameter("move_to_start").as_bool();
  const auto start_q = get_node()->get_parameter("start_joint_configuration").as_double_array();
  if (start_q.size() != static_cast<size_t>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "start_joint_configuration must have size %d (got %ld)",
                 num_joints, start_q.size());
    return CallbackReturn::FAILURE;
  }
  q_start_ = Eigen::Map<const Vector7d>(start_q.data());

  const auto k_start = get_node()->get_parameter("start_k_gains").as_double_array();
  const auto d_start = get_node()->get_parameter("start_d_gains").as_double_array();
  if (move_to_start_) {
    if (k_start.size() != static_cast<size_t>(num_joints) ||
        d_start.size() != static_cast<size_t>(num_joints)) {
      RCLCPP_FATAL(get_node()->get_logger(),
                   "start_k_gains and start_d_gains must be size %d when move_to_start=true",
                   num_joints);
      return CallbackReturn::FAILURE;
    }
    for (int i = 0; i < num_joints; ++i) {
      k_start_(i) = k_start.at(i);
      d_start_(i) = d_start.at(i);
    }
  }

  franka_robot_model_ =
      std::make_unique<franka_semantic_components::FrankaRobotModel>(
          arm_id_ + "/robot_model", arm_id_);

  sub_desired_cartesian_delta_ =
      get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
          cartesian_delta_topic_,
          1,
          std::bind(&CartesianImpedanceDeltaController::desiredCartesianDeltaCallback,
                    this,
                    std::placeholders::_1));

  // Initialize buffer from non-RT context
  DeltaCmdRT init;
  init.valid = false;
  delta_buffer_.writeFromNonRT(init);

  // Treat as already applied until a new delta arrives.
  last_delta_seq_ = delta_seq_.load(std::memory_order_acquire);

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceDeltaController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  accept_delta_.store(false, std::memory_order_release);

  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  Matrix4d T0(franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data());
  desired_position_ = Vector3d(T0.block<3, 1>(0, 3));
  desired_orientation_ = Quaterniond(T0.block<3, 3>(0, 0));
  desired_orientation_.normalize();

  desired_qn_ = Vector7d(franka_robot_model_->getRobotState()->q.data());

  stiffness_.setIdentity();
  stiffness_.topLeftCorner(3, 3) = pos_stiff_ * Matrix3d::Identity();
  stiffness_.bottomRightCorner(3, 3) = rot_stiff_ * Matrix3d::Identity();

  damping_.setIdentity();
  damping_.topLeftCorner(3, 3) = 2.0 * std::sqrt(pos_stiff_) * Matrix3d::Identity();
  damping_.bottomRightCorner(3, 3) =
      0.8 * 2.0 * std::sqrt(rot_stiff_) * Matrix3d::Identity();

  dq_filtered_.setZero();

  // Clear buffer from non-RT context
  DeltaCmdRT clear;
  clear.valid = false;
  delta_buffer_.writeFromNonRT(clear);
  last_delta_seq_ = delta_seq_.load(std::memory_order_acquire);

  if (move_to_start_) {
    // Start time must be taken from update()'s time source -> latch in update()
    start_time_set_ = false;

    Eigen::Map<const Vector7d> q(franka_robot_model_->getRobotState()->q.data());
    motion_generator_ = std::make_unique<MotionGenerator>(0.2, q, q_start_);
    mode_.store(static_cast<uint8_t>(Mode::MOVE_TO_START), std::memory_order_release);
    // accept_delta stays false until finished
  } else {
    mode_.store(static_cast<uint8_t>(Mode::DELTA_CARTESIAN), std::memory_order_release);
    accept_delta_.store(true, std::memory_order_release);
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceDeltaController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

void CartesianImpedanceDeltaController::desiredCartesianDeltaCallback(
    const std_msgs::msg::Float64MultiArray& msg) {
  if (!accept_delta_.load(std::memory_order_relaxed)) {
    return;
  }
  if (msg.data.size() < 12) {
    return;
  }

  DeltaCmdRT cmd;
  cmd.valid = true;

  cmd.dx = msg.data[0];
  cmd.dy = msg.data[1];
  cmd.dz = msg.data[2];

  for (int i = 0; i < 9; ++i) {
    cmd.R[i] = msg.data[3 + i];
  }

  // Non-RT callback writing to RT-readable buffer is intended usage
  delta_buffer_.writeFromNonRT(cmd);
  delta_seq_.fetch_add(1, std::memory_order_release);
}

}  // namespace teleop_controllers

PLUGINLIB_EXPORT_CLASS(
    teleop_controllers::CartesianImpedanceDeltaController,
    controller_interface::ControllerInterface)
