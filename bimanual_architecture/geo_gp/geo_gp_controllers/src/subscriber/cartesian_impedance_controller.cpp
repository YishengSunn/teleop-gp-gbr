#include "geo_gp_controllers/subscriber/cartesian_impedance_controller.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <exception>
#include <string>

#include <franka/model.h>
#include <franka/rate_limiting.h>

inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {
  double lambda_ = damped ? 0.2 : 0.0;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
  Eigen::MatrixXd S_ = M_;
  S_.setZero();

  for (int i = 0; i < sing_vals_.size(); i++) {
    S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);
  }

  M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}

namespace geo_gp_controllers {

namespace {

inline double safePeriodSeconds(const rclcpp::Duration& period) {
  const double dt = period.seconds();
  if (!std::isfinite(dt) || dt <= 0.0) {
    return 0.001;
  }
  return dt;
}

Quaterniond slerpShortestArc(const Quaterniond& q0_in, const Quaterniond& q1_in, double s) {
  Quaterniond q0 = q0_in.normalized();
  Quaterniond q1 = q1_in.normalized();

  if (q0.dot(q1) < 0.0) {
    q1.coeffs() *= -1.0;
  }

  return q0.slerp(s, q1).normalized();
}
}  // namespace

controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
CartesianImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  if (TDPA_active_) {
    for (int i = 1; i <= num_joints; ++i) {
      config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
      config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
    }

    if (franka_robot_model_) {
      for (const auto& name : franka_robot_model_->get_state_interface_names()) {
        config.names.push_back(name);
      }
    }

    if (franka_robot_state_) {
      for (const auto& n : franka_robot_state_->get_state_interface_names()) {
        config.names.push_back(n);
      }
    } else {
      franka_semantic_components::FrankaRobotState tmp_state(arm_id_ + "/robot_state", arm_id_);
      for (const auto& n : tmp_state.get_state_interface_names()) {
        config.names.push_back(n);
      }
    }
  } else {
    for (const auto& name : franka_robot_model_->get_state_interface_names()) {
      config.names.push_back(name);
    }
  }

  return config;
}

Vector6d
CartesianImpedanceController::wrenchFromTorque(
    const Matrix6x7d& jacobian,
    const Matrix7d& mass,
    const Vector7d& tau_ext) {
  const Matrix7d mass_inv = mass.inverse();
  const Matrix6d mass_c = (jacobian * mass_inv * jacobian.transpose()).inverse();
  return (mass_inv * jacobian.transpose() * mass_c).transpose() * tau_ext;
}

Vector3d
CartesianImpedanceController::quatErrorToRotvec(
    const Quaterniond& current,
    const Quaterniond& desired) {
  Quaterniond q_cur = current;
  Quaterniond q_des = desired;

  if (q_cur.coeffs().dot(q_des.coeffs()) < 0.0) {
    q_des.coeffs() = -q_des.coeffs();
  }

  Quaterniond q_err = q_des * q_cur.inverse();
  q_err.normalize();

  Eigen::AngleAxisd aa(q_err);
  if (!std::isfinite(aa.angle()) || std::abs(aa.angle()) < 1e-12) {
    return Vector3d::Zero();
  }

  return aa.axis() * aa.angle();
}

Matrix3d
CartesianImpedanceController::skewMatrix(const Vector3d& w) {
  Matrix3d S = Matrix3d::Zero();

  S(0, 1) = -w(2);
  S(0, 2) =  w(1);
  S(1, 0) =  w(2);
  S(1, 2) = -w(0);
  S(2, 0) = -w(1);
  S(2, 1) =  w(0);

  return S;
}

Matrix3d
CartesianImpedanceController::projectToSO3(const Matrix3d& R) {
  Eigen::JacobiSVD<Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Matrix3d R_proj = svd.matrixU() * svd.matrixV().transpose();

  if (R_proj.determinant() < 0.0) {
    Matrix3d U = svd.matrixU();
    U.col(2) *= -1.0;
    R_proj = U * svd.matrixV().transpose();
  }

  return R_proj;
}

Matrix3d
CartesianImpedanceController::integrateRotationWorld(
    const Matrix3d& R,
    const Vector3d& omega_world,
    double dt) {
  Matrix3d R_next = R + dt * skewMatrix(omega_world) * R;
  return projectToSO3(R_next);
}

void CartesianImpedanceController::updateJointStates() {
  for (int i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

controller_interface::return_type
CartesianImpedanceController::updateTdpa_(const rclcpp::Time& /*time*/,
                                           const rclcpp::Duration& period) {
  updateJointStates();

  bool skip_buffer_read_this_cycle = false;

  const auto mode_now = static_cast<Mode>(mode_.load(std::memory_order_relaxed));

  if (mode_now == Mode::MOVE_TO_START) {
    const auto t = this->get_node()->now() - start_time_;
    const auto out = motion_generator_->getDesiredJointPositions(t);

    const Vector7d q_desired_move = out.first;
    const bool finished = out.second;

    if (!finished) {
      const double kAlpha = 0.99;
      dq_filtered_ = (1.0 - kAlpha) * dq_filtered_ + kAlpha * dq_;

      const Vector7d tau =
          k_start_.cwiseProduct(q_desired_move - q_) +
          d_start_.cwiseProduct(-dq_filtered_);

      for (int i = 0; i < num_joints; ++i) {
        command_interfaces_[i].set_value(tau(i));
      }

      return controller_interface::return_type::OK;
    }

    Eigen::Map<const Matrix4d> pose_after_move(
        franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data());

    position_start_ = pose_after_move.block<3, 1>(0, 3);
    orientation_start_ = Quaterniond(pose_after_move.block<3, 3>(0, 0));
    orientation_start_.normalize();

    desired_position_ = position_start_;
    desired_orientation_ = orientation_start_;
    desired_rotation_ = orientation_start_.toRotationMatrix();
    desired_qn_ = q_;

    CartesianArray cur{};
    cur.fill(0.0);
    xdot_cmd_buffer_.writeFromNonRT(cur);

    last_desired_seq_ = desired_seq_.load(std::memory_order_acquire);

    pandatime_ = 0.0;
    remote_pandatime_ = 0.0;

    tdpaReset_();

    accept_desired_.store(true, std::memory_order_release);
    mode_.store(static_cast<uint8_t>(Mode::CARTESIAN), std::memory_order_release);

    skip_buffer_read_this_cycle = true;
  }

  const double dt = safePeriodSeconds(period);
  pandatime_ += dt;

  if (!skip_buffer_read_this_cycle) {
    const uint64_t seq = desired_seq_.load(std::memory_order_acquire);

    if (seq != last_desired_seq_) {
      const auto* xdot_ptr = xdot_cmd_buffer_.readFromRT();

      if (xdot_ptr != nullptr) {
        for (int i = 0; i < cart_dims; ++i) {
          xdot_remote_(i) = (*xdot_ptr)[static_cast<size_t>(i)];
        }
      }

      last_desired_seq_ = seq;
    }
  }

  Eigen::Map<const Matrix4d> current_pose(
      franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data());

  const Vector3d current_position(current_pose.block<3, 1>(0, 3));

  Quaterniond current_orientation(current_pose.block<3, 3>(0, 0));
  current_orientation.normalize();

  Matrix6x7d jacobian(
      franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector).data());

  const auto mass_array = franka_robot_model_->getMassMatrix();
  const Matrix7d mass = Eigen::Map<const Matrix7d>(mass_array.data());

  xdot_local_ = jacobian * dq_;

  x_local_delta_.head<3>() = current_position - position_start_;
  x_local_delta_.tail<3>() =
      quatErrorToRotvec(orientation_start_, current_orientation);

  Vector6d xdot_remote_mod = xdot_remote_;
  for (int i = 0; i < 3; ++i) {
    xdot_remote_mod(i) -= K_drift_ * position_error_(i);
  }

  for (int i = 0; i < num_joints; ++i) {
    tau_ext_(i) = -franka_robot_state_->get_robot_state_ptr()->tau_ext_hat_filtered[i];
  }

  wrench_ext_ = wrenchFromTorque(jacobian, mass, tau_ext_);

  // Predicted execution and blend-back use local Cartesian pose targets. Normal
  // bilateral teleoperation continues through the TDPA path below.
  bool use_pose_target = execution_running_.load(std::memory_order_acquire);

  if (pending_blend_to_leader_.load(std::memory_order_acquire)) {
    if (!blend_to_leader_enabled_) {
      pending_blend_to_leader_.store(false, std::memory_order_release);
      desired_position_ = current_position;
      desired_orientation_ = current_orientation;
      desired_rotation_ = current_orientation.toRotationMatrix();
    } else {
      const auto* goal = leader_pose_cache_.readFromRT();
      if (goal) {
        pending_blend_to_leader_.store(false, std::memory_order_release);
        blend_pose_start_.px = current_position.x();
        blend_pose_start_.py = current_position.y();
        blend_pose_start_.pz = current_position.z();
        desiredPoseFromQuaternion(current_orientation, &blend_pose_start_);
        blend_pose_goal_ = *goal;

        const Vector3d p0(blend_pose_start_.px, blend_pose_start_.py, blend_pose_start_.pz);
        const Vector3d p1(blend_pose_goal_.px, blend_pose_goal_.py, blend_pose_goal_.pz);
        const double pos_dist = (p1 - p0).norm();
        const Quaterniond q0 = quatFromDesiredPose(blend_pose_start_);
        Quaterniond q1 = quatFromDesiredPose(blend_pose_goal_);
        if (q0.dot(q1) < 0.0) {
          q1.coeffs() *= -1.0;
          desiredPoseFromQuaternion(q1, &blend_pose_goal_);
        }
        blend_duration_sec_ =
            blend_seconds_per_meter_ * pos_dist +
            blend_seconds_per_rad_ * q0.angularDistance(q1);
        blend_duration_sec_ =
            std::clamp(blend_duration_sec_, blend_duration_min_, blend_duration_max_);
        blend_t0_ = this->get_node()->now();
        blending_to_leader_.store(true, std::memory_order_release);
        mode_.store(static_cast<uint8_t>(Mode::BLEND_TO_LEADER), std::memory_order_release);
      }
    }
  }

  if (blending_to_leader_.load(std::memory_order_acquire)) {
    const double elapsed = (this->get_node()->now() - blend_t0_).seconds();
    const double blend_s =
        (blend_duration_sec_ > 1e-6)
            ? std::min(1.0, elapsed / blend_duration_sec_)
            : 1.0;
    const Vector3d p0(blend_pose_start_.px, blend_pose_start_.py, blend_pose_start_.pz);
    const Vector3d p1(blend_pose_goal_.px, blend_pose_goal_.py, blend_pose_goal_.pz);
    const Vector3d p = (1.0 - blend_s) * p0 + blend_s * p1;
    const Quaterniond q_interp = slerpShortestArc(
        quatFromDesiredPose(blend_pose_start_),
        quatFromDesiredPose(blend_pose_goal_), blend_s);

    desired_pose_rt_.px = p.x();
    desired_pose_rt_.py = p.y();
    desired_pose_rt_.pz = p.z();
    desiredPoseFromQuaternion(q_interp, &desired_pose_rt_);
    use_pose_target = true;

    if (blend_s >= 1.0 - 1e-6) {
      desired_pose_rt_ = blend_pose_goal_;
      desired_pose_buffer_.writeFromNonRT(blend_pose_goal_);
      desired_pose_seq_.fetch_add(1, std::memory_order_release);
      last_desired_pose_seq_ = desired_pose_seq_.load(std::memory_order_acquire);
      blending_to_leader_.store(false, std::memory_order_release);
      blend_running_hold_until_ =
          this->get_node()->now() + rclcpp::Duration::from_seconds(blend_running_hold_sec_);
      mode_.store(static_cast<uint8_t>(Mode::CARTESIAN), std::memory_order_release);

      // Resume TDPA integration at the blend endpoint instead of jumping back
      // to the pre-execution integrated target.
      desired_position_ = p;
      desired_orientation_ = q_interp;
      desired_rotation_ = q_interp.toRotationMatrix();
      x_remote_delta_intgl_.head<3>() = desired_position_ - position_start_;
      x_remote_delta_intgl_.tail<3>() =
          quatErrorToRotvec(orientation_start_, desired_orientation_);
    }
  } else if (use_pose_target) {
    const uint64_t pose_seq = desired_pose_seq_.load(std::memory_order_acquire);
    if (pose_seq != last_desired_pose_seq_) {
      const auto* pose = desired_pose_buffer_.readFromRT();
      if (pose) {
        desired_pose_rt_ = *pose;
      }
      last_desired_pose_seq_ = pose_seq;
    }
  }

  const double gain_tau_f = tau_ext_feedback_ ? 1.0 : -1.0;

  double v_remote_linear[6]{};
  double v_remote_rotational[6]{};
  double f_old_linear[6]{};
  double f_old_rotational[6]{};
  double v_des_linear[6]{};
  double v_des_rotational[6]{};

  for (int i = 0; i < 3; ++i) {
    const double old_linear =
        tau_ext_feedback_ ? wrench_ext_(i)
                          : (wrench_applied_initialized_ ? wrench_applied_(i) : 0.0);

    const double old_rotational =
        tau_ext_feedback_ ? wrench_ext_(i + 3)
                          : (wrench_applied_initialized_ ? wrench_applied_(i + 3) : 0.0);

    v_remote_linear[i] = xdot_remote_mod(i);
    v_remote_rotational[i + 3] = xdot_remote_mod(i + 3);

    f_old_linear[i] = gain_tau_f * old_linear;
    f_old_rotational[i + 3] = gain_tau_f * old_rotational;

    v_des_linear[i] = v_remote_linear[i];
    v_des_rotational[i + 3] = v_remote_rotational[i + 3];
  }

  followerPC_linear_.energyObserver(v_remote_linear, f_old_linear, dt);
  followerPC_rotational_.energyObserver(v_remote_rotational, f_old_rotational, dt);

  const Matrix3d damping_linear = damping_.topLeftCorner<3, 3>();
  const Matrix3d damping_rotational = damping_.bottomRightCorner<3, 3>();

  const double dissipation_linear =
      xdot_local_.head<3>().dot(damping_linear * xdot_local_.head<3>());

  const double dissipation_rotational =
      xdot_local_.tail<3>().dot(damping_rotational * xdot_local_.tail<3>());

  shortage_linear_ += dt * eta_ * dissipation_linear;
  shortage_rotational_ += dt * eta_ * dissipation_rotational;
  followerPC_linear_.energyController(
      v_des_linear,
      f_old_linear,
      E_L_in_delayed_linear_ + shortage_linear_,
      dt);

  followerPC_rotational_.energyController(
      v_des_rotational,
      f_old_rotational,
      E_L_in_delayed_rotational_ + shortage_rotational_,
      dt);

  Vector6d xdot_des = Vector6d::Zero();

  for (int i = 0; i < 3; ++i) {
    xdot_des(i) = v_des_linear[i];
    xdot_des(i + 3) = v_des_rotational[i + 3];
  }

  desired_position_ += dt * xdot_des.head<3>();

  desired_rotation_ =
      integrateRotationWorld(desired_rotation_, xdot_des.tail<3>(), dt);

  desired_orientation_ = Quaterniond(desired_rotation_);
  desired_orientation_.normalize();

  if (tdpa_integrated_pose_pub_ && tdpa_integrated_pose_pub_->trylock()) {
    auto& msg = tdpa_integrated_pose_pub_->msg_;
    msg.header.stamp = get_node()->now();
    msg.header.frame_id = arm_id_ + "_base";
    msg.pose.position.x = desired_position_.x();
    msg.pose.position.y = desired_position_.y();
    msg.pose.position.z = desired_position_.z();
    msg.pose.orientation.x = desired_orientation_.x();
    msg.pose.orientation.y = desired_orientation_.y();
    msg.pose.orientation.z = desired_orientation_.z();
    msg.pose.orientation.w = desired_orientation_.w();
    tdpa_integrated_pose_pub_->unlockAndPublish();
  }

  x_remote_delta_intgl_.head<3>() = desired_position_ - position_start_;
  x_remote_delta_intgl_.tail<3>() =
      quatErrorToRotvec(orientation_start_, desired_orientation_);

  position_error_.setZero();
  position_error_.head<3>() =
      x_remote_delta_intgl_.head<3>() - x_remote_delta_.head<3>();

  Vector6d error = Vector6d::Zero();
  error.head<3>() = desired_position_ - current_position;
  error.tail<3>() =
      quatErrorToRotvec(current_orientation, desired_orientation_);

  const Vector6d vel_error_tdpa = xdot_des - xdot_local_;

  wrench_c_ = stiffness_ * error + damping_ * vel_error_tdpa;

  E_F_in_linear_ = followerPC_linear_.getInputEnergyFlow();
  E_F_in_rotational_ = followerPC_rotational_.getInputEnergyFlow();
  E_F_in_ = E_F_in_linear_ + E_F_in_rotational_;

  if (use_pose_target) {
    const Vector3d pose_target(
        desired_pose_rt_.px, desired_pose_rt_.py, desired_pose_rt_.pz);
    const Quaterniond orientation_target = quatFromDesiredPose(desired_pose_rt_);
    Vector6d pose_error = Vector6d::Zero();
    pose_error.head<3>() = pose_target - current_position;
    pose_error.tail<3>() = quatErrorToRotvec(current_orientation, orientation_target);

    // Online_fuser supplies x_d. The TDPA-integrated reference remains x_L
    const Vector6d f_total = stiffness_ * pose_error - damping_ * xdot_local_;
    Vector6d f_cmd = f_total;
    if (autonomy_passivity_with_online_fuser_ &&
        online_fuser_active_.load(std::memory_order_acquire)) {
      Vector6d leader_error = Vector6d::Zero();
      leader_error.head<3>() = desired_position_ - current_position;
      leader_error.tail<3>() =
          quatErrorToRotvec(current_orientation, desired_orientation_);
      const Vector6d f_leader = stiffness_ * leader_error - damping_ * xdot_local_;
      const AutonomyPassivityOutput safe =
          autonomy_pc_.update(f_leader, f_total, xdot_local_, dt);
      f_cmd = f_leader + safe.wrench_autonomy_safe;
    }
    const Vector7d tau_task_execution = jacobian.transpose() * f_cmd;
    Eigen::MatrixXd jacobian_transpose_pinv;
    pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
    const Vector7d tau_nullspace_execution =
        (Matrix7d::Identity() - jacobian.transpose() * jacobian_transpose_pinv) *
        (n_stiffness_ * (desired_qn_ - q_) -
         (2.0 * std::sqrt(n_stiffness_)) * dq_);
    const auto coriolis_array = franka_robot_model_->getCoriolisForceVector();
    const Vector7d coriolis = Eigen::Map<const Vector7d>(coriolis_array.data());
    const Vector7d tau_d_execution =
        tau_task_execution + coriolis + tau_nullspace_execution;

    std::array<double, 7> tau_des{};
    std::array<double, 7> tau_prev{};
    for (int i = 0; i < num_joints; ++i) {
      tau_des[static_cast<size_t>(i)] = tau_d_execution(i);
      tau_prev[static_cast<size_t>(i)] =
          last_tau_cmd_initialized_ ? last_tau_cmd_(i) : tau_d_execution(i);
    }
    const auto tau_limited = franka::limitRate(franka::kMaxTorqueRate, tau_des, tau_prev);
    Vector7d tau_applied = Vector7d::Zero();
    for (int i = 0; i < num_joints; ++i) {
      tau_applied(i) = tau_limited[static_cast<size_t>(i)];
      command_interfaces_[i].set_value(tau_applied(i));
      last_tau_cmd_(i) = tau_applied(i);
    }
    last_tau_cmd_initialized_ = true;
    wrench_applied_ = wrenchFromTorque(
        jacobian, mass, tau_applied - tau_nullspace_execution - coriolis);
    wrench_applied_initialized_ = true;

    if (local_state_pub_ && local_state_pub_->trylock()) {
      auto& msg = local_state_pub_->msg_;
      const rclcpp::Time now = get_node()->now();
      msg.header.stamp = now;
      msg.seq = local_seq_++;
      msg.tx_time_ns = now.nanoseconds();
      msg.echo_seq = last_received_remote_seq_;
      msg.echo_tx_time_ns = last_received_remote_tx_time_ns_;
      msg.pandatime = pandatime_;
      for (int i = 0; i < cart_dims; ++i) {
        msg.x_local_delta[static_cast<size_t>(i)] = x_local_delta_(i);
        msg.xdot_local[static_cast<size_t>(i)] = xdot_local_(i);
        msg.wrench_local[static_cast<size_t>(i)] =
            tau_ext_feedback_ ? wrench_ext_(i) : wrench_applied_(i);
      }
      msg.energy = E_F_in_;
      msg.energy_linear = E_F_in_linear_;
      msg.energy_rotational = E_F_in_rotational_;
      local_state_pub_->unlockAndPublish();
    }

    const bool blend_now =
        blending_to_leader_.load(std::memory_order_relaxed) ||
        (this->get_node()->now() < blend_running_hold_until_);
    if (pub_blend_running_ && blend_now != last_blend_running_published_) {
      std_msgs::msg::Bool msg;
      msg.data = blend_now;
      pub_blend_running_->publish(msg);
      last_blend_running_published_ = blend_now;
    }
    return controller_interface::return_type::OK;
  }

  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  const Vector7d tau_task = jacobian.transpose() * wrench_c_;

  const Vector7d tau_nullspace =
      (Matrix7d::Identity() - jacobian.transpose() * jacobian_transpose_pinv) *
      (n_stiffness_ * (desired_qn_ - q_) -
       (2.0 * std::sqrt(n_stiffness_)) * dq_);

  const Vector7d tau_d = tau_task + tau_nullspace;

  std::array<double, 7> tau_des{};
  std::array<double, 7> tau_prev{};

  for (int i = 0; i < num_joints; ++i) {
    tau_des[static_cast<size_t>(i)] = tau_d(i);
    tau_prev[static_cast<size_t>(i)] =
        last_tau_cmd_initialized_ ? last_tau_cmd_(i) : tau_d(i);
  }

  const auto tau_limited =
      franka::limitRate(franka::kMaxTorqueRate, tau_des, tau_prev);

  Vector7d tau_applied = Vector7d::Zero();

  for (int i = 0; i < num_joints; ++i) {
    tau_applied(i) = tau_limited[static_cast<size_t>(i)];
    command_interfaces_[i].set_value(tau_applied(i));
    last_tau_cmd_(i) = tau_applied(i);
  }

  last_tau_cmd_initialized_ = true;

  const Vector7d tau_task_applied = tau_applied - tau_nullspace;
  wrench_applied_ = wrenchFromTorque(jacobian, mass, tau_task_applied);
  wrench_applied_initialized_ = true;

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

    for (int i = 0; i < cart_dims; ++i) {
      msg.x_local_delta[static_cast<size_t>(i)] = x_local_delta_(i);
      msg.xdot_local[static_cast<size_t>(i)] = xdot_local_(i);
      msg.wrench_local[static_cast<size_t>(i)] =
          tau_ext_feedback_ ? wrench_ext_(i) : wrench_applied_(i);
    }

    msg.energy = E_F_in_;
    msg.energy_linear = E_F_in_linear_;
    msg.energy_rotational = E_F_in_rotational_;

    local_state_pub_->unlockAndPublish();
  }

  const bool blend_now =
      blending_to_leader_.load(std::memory_order_relaxed) ||
      (this->get_node()->now() < blend_running_hold_until_);
  if (pub_blend_running_ && blend_now != last_blend_running_published_) {
    std_msgs::msg::Bool msg;
    msg.data = blend_now;
    pub_blend_running_->publish(msg);
    last_blend_running_published_ = blend_now;
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type
CartesianImpedanceController::update(const rclcpp::Time& time,
                                     const rclcpp::Duration& period) {
  if (TDPA_active_) {
    return updateTdpa_(time, period);
  }

  Eigen::Map<const Matrix4d> current(
      franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data());

  Vector3d current_position(current.block<3, 1>(0, 3));
  Quaterniond current_orientation(current.block<3, 3>(0, 0));

  Eigen::Map<const Vector7d> coriolis(franka_robot_model_->getCoriolisForceVector().data());
  Eigen::Matrix<double, 6, 7> jacobian(
      franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector).data());

  Eigen::Map<const Vector7d> qD(franka_robot_model_->getRobotState()->dq.data());
  Eigen::Map<const Vector7d> q(franka_robot_model_->getRobotState()->q.data());

  bool skip_pose_read_this_cycle = false;

  const auto mode_now = static_cast<Mode>(mode_.load(std::memory_order_relaxed));
  if (mode_now == Mode::MOVE_TO_START) {
    const auto t = this->get_node()->now() - start_time_;
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

    // Move finished: seed desired pose from current EE pose so the impedance law starts "where we are".
    current_orientation.normalize();
    DesiredPoseRT init;
    init.px = current_position.x();
    init.py = current_position.y();
    init.pz = current_position.z();
    init.qw = current_orientation.w();
    init.qx = current_orientation.x();
    init.qy = current_orientation.y();
    init.qz = current_orientation.z();

    desired_pose_rt_ = init;
    desired_pose_buffer_.writeFromNonRT(init);

    // Mark the buffer as already applied until a NEW PoseStamped arrives.
    last_desired_pose_seq_ = desired_pose_seq_.load(std::memory_order_acquire);

    accept_desired_.store(true, std::memory_order_release);
    mode_.store(static_cast<uint8_t>(Mode::CARTESIAN), std::memory_order_release);

    // Don't read the buffer again in this cycle.
    skip_pose_read_this_cycle = true;
  }

  // Prediction ended: optional linear blend in pose space to cached leader EE (duration from distance).
  if (pending_blend_to_leader_.load(std::memory_order_acquire)) {
    if (!blend_to_leader_enabled_) {
      pending_blend_to_leader_.store(false, std::memory_order_release);
    }
    else {
      const auto* goal = leader_pose_cache_.readFromRT();
      if (goal) {
        pending_blend_to_leader_.store(false, std::memory_order_release);

        blend_pose_start_.px = current_position.x();
        blend_pose_start_.py = current_position.y();
        blend_pose_start_.pz = current_position.z();
        current_orientation.normalize();
        desiredPoseFromQuaternion(current_orientation, &blend_pose_start_);

        blend_pose_goal_ = *goal;

        const Vector3d p0(blend_pose_start_.px, blend_pose_start_.py, blend_pose_start_.pz);
        const Vector3d p1(blend_pose_goal_.px, blend_pose_goal_.py, blend_pose_goal_.pz);
        const double pos_dist = (p1 - p0).norm();
        const Quaterniond q0 = quatFromDesiredPose(blend_pose_start_);
        Quaterniond q1 = quatFromDesiredPose(blend_pose_goal_);
        if (q0.dot(q1) < 0.0) {
          q1.coeffs() *= -1.0;
          desiredPoseFromQuaternion(q1, &blend_pose_goal_);
        }
        const double ang_dist = q0.angularDistance(q1);

        blend_duration_sec_ =
          blend_seconds_per_meter_ * pos_dist + blend_seconds_per_rad_ * ang_dist;
        blend_duration_sec_ =
          std::clamp(blend_duration_sec_, blend_duration_min_, blend_duration_max_);

        blend_t0_ = this->get_node()->now();
        blending_to_leader_.store(true, std::memory_order_release);
        mode_.store(static_cast<uint8_t>(Mode::BLEND_TO_LEADER), std::memory_order_release);
        skip_pose_read_this_cycle = true;
      }
      else {
        static int log_ctr = 0;
        if (++log_ctr % 500 == 0) {
          RCLCPP_WARN(
            get_node()->get_logger(),
            "Blend-to-leader waiting for leader pose cache (will retry)...");
        }
      }
    }
  }

  if (static_cast<Mode>(mode_.load(std::memory_order_relaxed)) == Mode::BLEND_TO_LEADER) {
    const double elapsed = (this->get_node()->now() - blend_t0_).seconds();
    const double s =
      (blend_duration_sec_ > 1e-6) ? std::min(1.0, elapsed / blend_duration_sec_) : 1.0;

    const Vector3d p0(blend_pose_start_.px, blend_pose_start_.py, blend_pose_start_.pz);
    const Vector3d p1(blend_pose_goal_.px, blend_pose_goal_.py, blend_pose_goal_.pz);
    const Vector3d p = (1.0 - s) * p0 + s * p1;

    const Quaterniond q0 = quatFromDesiredPose(blend_pose_start_);
    const Quaterniond q1 = quatFromDesiredPose(blend_pose_goal_);
    const Quaterniond q_interp = slerpShortestArc(q0, q1, s);

    desired_pose_rt_.px = p.x();
    desired_pose_rt_.py = p.y();
    desired_pose_rt_.pz = p.z();
    desiredPoseFromQuaternion(q_interp, &desired_pose_rt_);

    if (s >= 1.0 - 1e-6) {
      desired_pose_rt_ = blend_pose_goal_;
      desired_pose_buffer_.writeFromNonRT(blend_pose_goal_);
      desired_pose_seq_.fetch_add(1, std::memory_order_release);
      last_desired_pose_seq_ = desired_pose_seq_.load(std::memory_order_acquire);
      blending_to_leader_.store(false, std::memory_order_release);
      blend_running_hold_until_ =
        this->get_node()->now() + rclcpp::Duration::from_seconds(blend_running_hold_sec_);
      mode_.store(static_cast<uint8_t>(Mode::CARTESIAN), std::memory_order_release);
    }

    skip_pose_read_this_cycle = true;
  }

  // Cartesian mode: only apply desired pose if a new message arrived.
  if (!skip_pose_read_this_cycle &&
      static_cast<Mode>(mode_.load(std::memory_order_relaxed)) != Mode::BLEND_TO_LEADER) {
    const uint64_t seq = desired_pose_seq_.load(std::memory_order_acquire);
    if (seq != last_desired_pose_seq_) {
      const auto* dp = desired_pose_buffer_.readFromRT();
      if (dp) {
        desired_pose_rt_ = *dp;
      }
      last_desired_pose_seq_ = seq;
    }
  }

  Vector3d desired_position;
  desired_position << desired_pose_rt_.px, desired_pose_rt_.py, desired_pose_rt_.pz;

  Quaterniond desired_orientation(desired_pose_rt_.qw,
                                  desired_pose_rt_.qx,
                                  desired_pose_rt_.qy,
                                  desired_pose_rt_.qz);
  if (desired_orientation.norm() < 1e-9) {
    desired_orientation = Quaterniond::Identity();
  }
  else {
    desired_orientation.normalize();
  }

  Vector6d error;
  error.head(3) = current_position - desired_position;

  if (desired_orientation.coeffs().dot(current_orientation.coeffs()) < 0.0) {
    current_orientation.coeffs() = -current_orientation.coeffs();
  }

  Quaterniond rot_error(current_orientation * desired_orientation.inverse());
  Eigen::AngleAxisd rot_error_aa(rot_error);
  error.tail(3) = rot_error_aa.axis() * rot_error_aa.angle();

  Vector7d tau_task = jacobian.transpose() *
                      (-stiffness_ * error - damping_ * (jacobian * qD));

  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  Vector7d tau_nullspace =
      (Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose() * jacobian_transpose_pinv) *
      (n_stiffness_ * (desired_qn_ - q) - (2.0 * std::sqrt(n_stiffness_)) * qD);

  Vector7d tau_d = tau_task + coriolis + tau_nullspace;

  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
  }

  const bool blend_now =
    blending_to_leader_.load(std::memory_order_relaxed) ||
    (this->get_node()->now() < blend_running_hold_until_);
  if (pub_blend_running_ && blend_now != last_blend_running_published_) {
    std_msgs::msg::Bool m;
    m.data = blend_now;
    pub_blend_running_->publish(m);
    last_blend_running_published_ = blend_now;
  }

  return controller_interface::return_type::OK;
}

CallbackReturn
CartesianImpedanceController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");

    auto_declare<double>("pos_stiff", 100.0);
    auto_declare<double>("rot_stiff", 10.0);
    auto_declare<double>("n_stiffness", 10.0);

    auto_declare<std::string>("leader_robot_state_topic", "/leader/franka_robot_state_broadcaster/robot_state");
    auto_declare<std::string>("execution_pose_topic", "/execution/desired_pose");
    auto_declare<std::string>("execution_running_topic", "/execution/running");
    auto_declare<std::string>("blend_running_topic", "/execution/blend_to_leader_running");

    auto_declare<bool>("blend_to_leader_enabled", true);
    // Larger => slower return to leader after prediction (duration = pos_m * a + ang_rad * b).
    auto_declare<double>("blend_seconds_per_meter", 2.0);
    auto_declare<double>("blend_seconds_per_rad", 1.2);
    auto_declare<double>("blend_duration_min", 0.25);
    auto_declare<double>("blend_duration_max", 8.0);
    auto_declare<double>("blend_running_hold_sec", 0.5);

    auto_declare<bool>("move_to_start", false);
    auto_declare<std::vector<double>>(
        "start_joint_configuration",
        {0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4});

    auto_declare<std::vector<double>>("start_k_gains", { 600.0 ,600.0 ,600.0 ,600.0 ,250.0 ,150.0 ,50.0 });
    auto_declare<std::vector<double>>("start_d_gains", { 30.0 ,30.0 ,30.0 ,30.0 ,10.0 ,10.0 ,5.0 });

    auto_declare<bool>("TDPA_active", false);
    auto_declare<bool>("tau_ext_feedback", false);
    auto_declare<double>("k_pos_drift", 0.0);
    auto_declare<double>("eta_passivity_shortage", 0.0);
    auto_declare<std::string>("remote_state_topic", "leader/tdpa_cartesian_state");
    auto_declare<std::string>("local_state_topic", "follower/tdpa_cartesian_state");
    auto_declare<std::string>("tdpa_integrated_pose_topic", "/tdpa/integrated_desired_pose");
    auto_declare<bool>("autonomy_passivity_with_online_fuser", false);
    auto_declare<std::string>("online_fuser_active_topic", "/execution/online_fuser_active");
    auto_declare<double>("autonomy_tank_initial_energy", 2.0);
    auto_declare<double>("autonomy_tank_max_energy", 5.0);
    auto_declare<double>("autonomy_tank_recharge_efficiency", 0.8);
    auto_declare<double>("autonomy_wrench_max_abs", 100.0);

  }

  catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn
CartesianImpedanceController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();

  pos_stiff_ = get_node()->get_parameter("pos_stiff").as_double();
  rot_stiff_ = get_node()->get_parameter("rot_stiff").as_double();
  n_stiffness_ = get_node()->get_parameter("n_stiffness").as_double();

  leader_robot_state_topic_ =
      get_node()->get_parameter("leader_robot_state_topic").as_string();
  execution_pose_topic_ =
      get_node()->get_parameter("execution_pose_topic").as_string();
  execution_running_topic_ =
      get_node()->get_parameter("execution_running_topic").as_string();
  blend_running_topic_ =
      get_node()->get_parameter("blend_running_topic").as_string();

  blend_to_leader_enabled_ = get_node()->get_parameter("blend_to_leader_enabled").as_bool();
  blend_seconds_per_meter_ = get_node()->get_parameter("blend_seconds_per_meter").as_double();
  blend_seconds_per_rad_ = get_node()->get_parameter("blend_seconds_per_rad").as_double();
  blend_duration_min_ = get_node()->get_parameter("blend_duration_min").as_double();
  blend_duration_max_ = get_node()->get_parameter("blend_duration_max").as_double();
  blend_running_hold_sec_ = get_node()->get_parameter("blend_running_hold_sec").as_double();

  TDPA_active_ = get_node()->get_parameter("TDPA_active").as_bool();
  tau_ext_feedback_ = get_node()->get_parameter("tau_ext_feedback").as_bool();
  K_drift_ = get_node()->get_parameter("k_pos_drift").as_double();
  eta_ = get_node()->get_parameter("eta_passivity_shortage").as_double();
  remote_state_topic_ = get_node()->get_parameter("remote_state_topic").as_string();
  local_state_topic_ = get_node()->get_parameter("local_state_topic").as_string();
  tdpa_integrated_pose_topic_ =
      get_node()->get_parameter("tdpa_integrated_pose_topic").as_string();
  autonomy_passivity_with_online_fuser_ =
      get_node()->get_parameter("autonomy_passivity_with_online_fuser").as_bool();
  online_fuser_active_topic_ =
      get_node()->get_parameter("online_fuser_active_topic").as_string();
  autonomy_passivity_params_.tank.initial_energy =
      get_node()->get_parameter("autonomy_tank_initial_energy").as_double();
  autonomy_passivity_params_.tank.max_energy =
      get_node()->get_parameter("autonomy_tank_max_energy").as_double();
  autonomy_passivity_params_.tank.recharge_efficiency =
      get_node()->get_parameter("autonomy_tank_recharge_efficiency").as_double();
  autonomy_passivity_params_.tank.wrench_max_abs =
      get_node()->get_parameter("autonomy_wrench_max_abs").as_double();
  autonomy_pc_.configure(autonomy_passivity_params_);

  move_to_start_ = get_node()->get_parameter("move_to_start").as_bool();
  const auto start_q = get_node()->get_parameter("start_joint_configuration").as_double_array();
  if (start_q.size() != static_cast<size_t>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "start_joint_configuration must have size %d (got %ld)",
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

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id_ + "/robot_model", arm_id_));

  if (TDPA_active_) {
    franka_robot_state_ = std::make_unique<franka_semantic_components::FrankaRobotState>(
        franka_semantic_components::FrankaRobotState(arm_id_ + "/robot_state", arm_id_));

    stiffness_.setZero();
    stiffness_.topLeftCorner<3, 3>() = pos_stiff_ * Matrix3d::Identity();
    stiffness_.bottomRightCorner<3, 3>() = rot_stiff_ * Matrix3d::Identity();

    damping_.setZero();
    damping_.topLeftCorner<3, 3>() = 2.0 * std::sqrt(pos_stiff_) * Matrix3d::Identity();
    damping_.bottomRightCorner<3, 3>() =
        0.8 * 2.0 * std::sqrt(rot_stiff_) * Matrix3d::Identity();

    desired_rotation_.setIdentity();

    CartesianArray init{};
    init.fill(0.0);
    xdot_cmd_buffer_.writeFromNonRT(init);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

    remote_state_sub_ =
        get_node()->create_subscription<geo_gp_interfaces::msg::TDPACartesianState>(
            remote_state_topic_, qos,
            std::bind(&CartesianImpedanceController::remoteStateCallback, this,
                      std::placeholders::_1));

    local_state_pub_raw_ =
        get_node()->create_publisher<geo_gp_interfaces::msg::TDPACartesianState>(
            local_state_topic_, qos);

    local_state_pub_ =
        std::make_unique<
            realtime_tools::RealtimePublisher<geo_gp_interfaces::msg::TDPACartesianState>>(
            local_state_pub_raw_);

    tdpa_integrated_pose_pub_raw_ =
        get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
            tdpa_integrated_pose_topic_, qos);
    tdpa_integrated_pose_pub_ =
        std::make_unique<
            realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
            tdpa_integrated_pose_pub_raw_);

    tdpaReset_();
  }

  sub_leader_robot_state_ =
      get_node()->create_subscription<franka_msgs::msg::FrankaState>(
          leader_robot_state_topic_, rclcpp::QoS(1),
          std::bind(&CartesianImpedanceController::leaderRobotStateCallback, this,
                    std::placeholders::_1));

  sub_execution_pose_ =
      get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
          execution_pose_topic_, rclcpp::QoS(10),
          std::bind(&CartesianImpedanceController::executionDesiredPoseCallback, this,
                    std::placeholders::_1));

  sub_execution_running_ =
      get_node()->create_subscription<std_msgs::msg::Bool>(
          execution_running_topic_, rclcpp::QoS(1).transient_local(),
          std::bind(&CartesianImpedanceController::executionRunningCallback, this,
                    std::placeholders::_1));

  sub_online_fuser_active_ =
      get_node()->create_subscription<std_msgs::msg::Bool>(
          online_fuser_active_topic_, rclcpp::QoS(1).transient_local(),
          std::bind(&CartesianImpedanceController::onlineFuserActiveCallback, this,
                    std::placeholders::_1));

  pub_blend_running_ = get_node()->create_publisher<std_msgs::msg::Bool>(
      blend_running_topic_, rclcpp::QoS(1).transient_local());

  return CallbackReturn::SUCCESS;
}

CallbackReturn
CartesianImpedanceController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  accept_desired_.store(false, std::memory_order_release);

  if (TDPA_active_) {
    updateJointStates();
  }

  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  if (TDPA_active_) {
    franka_robot_state_->assign_loaned_state_interfaces(state_interfaces_);
  }

  Eigen::Map<const Matrix4d> desired_init(
      franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data());

  Vector3d init_pos(desired_init.block<3, 1>(0, 3));
  Quaterniond init_ori(desired_init.block<3, 3>(0, 0));
  init_ori.normalize();

  DesiredPoseRT init;
  init.px = init_pos.x();
  init.py = init_pos.y();
  init.pz = init_pos.z();
  init.qw = init_ori.w();
  init.qx = init_ori.x();
  init.qy = init_ori.y();
  init.qz = init_ori.z();

  desired_pose_rt_ = init;
  desired_pose_buffer_.writeFromNonRT(init);

  // Treat current desired as already applied until a new PoseStamped arrives.
  last_desired_pose_seq_ = desired_pose_seq_.load(std::memory_order_acquire);

  desired_qn_ = Vector7d(franka_robot_model_->getRobotState()->q.data());

  prev_execution_running_ = false;
  execution_running_.store(false, std::memory_order_release);
  pending_blend_to_leader_.store(false, std::memory_order_release);
  blending_to_leader_.store(false, std::memory_order_release);
  blend_running_hold_until_ = this->get_node()->now();
  last_blend_running_published_ = false;
  autonomy_pc_.reset();
  if (pub_blend_running_) {
    std_msgs::msg::Bool msg;
    msg.data = false;
    pub_blend_running_->publish(msg);
  }

  if (TDPA_active_) {
    position_start_ = init_pos;
    orientation_start_ = init_ori;
    desired_position_ = position_start_;
    desired_orientation_ = orientation_start_;
    desired_rotation_ = orientation_start_.toRotationMatrix();

    CartesianArray init_arr{};
    init_arr.fill(0.0);
    xdot_cmd_buffer_.writeFromNonRT(init_arr);

    last_desired_seq_ = desired_seq_.load(std::memory_order_acquire);

    pandatime_ = 0.0;
    remote_pandatime_ = 0.0;
    local_seq_ = 0;
    last_received_remote_seq_ = 0;
    last_received_remote_tx_time_ns_ = 0;

    tdpaReset_();

    last_tau_cmd_.setZero();
    last_tau_cmd_initialized_ = false;
    wrench_applied_.setZero();
    wrench_applied_initialized_ = false;
  } else {
    stiffness_.setIdentity();
    stiffness_.topLeftCorner(3, 3) = pos_stiff_ * Matrix3d::Identity();
    stiffness_.bottomRightCorner(3, 3) = rot_stiff_ * Matrix3d::Identity();

    damping_.setIdentity();
    damping_.topLeftCorner(3, 3) = 2.0 * std::sqrt(pos_stiff_) * Matrix3d::Identity();
    damping_.bottomRightCorner(3, 3) = 0.8 * 2.0 * std::sqrt(rot_stiff_) * Matrix3d::Identity();
  }

  dq_filtered_.setZero();

  if (move_to_start_) {
    const Vector7d q_init = TDPA_active_ ? q_
                                         : Vector7d(franka_robot_model_->getRobotState()->q.data());
    motion_generator_ = std::make_unique<MotionGenerator>(0.2, q_init, q_start_);
    start_time_ = this->get_node()->now();
    mode_.store(static_cast<uint8_t>(Mode::MOVE_TO_START), std::memory_order_release);
    // accept_desired stays false until finished
  }
  else {
    mode_.store(static_cast<uint8_t>(Mode::CARTESIAN), std::memory_order_release);
    accept_desired_.store(true, std::memory_order_release);
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn
CartesianImpedanceController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  if (pub_blend_running_) {
    std_msgs::msg::Bool m;
    m.data = false;
    pub_blend_running_->publish(m);
  }
  franka_robot_model_->release_interfaces();
  if (franka_robot_state_) {
    franka_robot_state_->release_interfaces();
  }
  return CallbackReturn::SUCCESS;
}

void CartesianImpedanceController::remoteStateCallback(
    const geo_gp_interfaces::msg::TDPACartesianState& msg) {
  if (!accept_desired_.load(std::memory_order_relaxed) &&
      static_cast<Mode>(mode_.load(std::memory_order_relaxed)) != Mode::CARTESIAN) {
    return;
  }

  remote_pandatime_ = msg.pandatime;

  last_received_remote_seq_ = msg.seq;
  last_received_remote_tx_time_ns_ = msg.tx_time_ns;

  CartesianArray xdot_remote_arr{};
  xdot_remote_arr.fill(0.0);

  for (int i = 0; i < cart_dims; ++i) {
    x_remote_delta_(i) = msg.x_local_delta[static_cast<size_t>(i)];
    xdot_remote_arr[static_cast<size_t>(i)] = msg.xdot_local[static_cast<size_t>(i)];
  }

  xdot_cmd_buffer_.writeFromNonRT(xdot_remote_arr);

  desired_seq_.fetch_add(1, std::memory_order_release);

  if (std::isfinite(msg.energy_linear)) {
    E_L_in_delayed_linear_ = msg.energy_linear;
  }

  if (std::isfinite(msg.energy_rotational)) {
    E_L_in_delayed_rotational_ = msg.energy_rotational;
  }

  E_L_in_delayed_ = E_L_in_delayed_linear_ + E_L_in_delayed_rotational_;
}

void CartesianImpedanceController::leaderRobotStateCallback(
    const franka_msgs::msg::FrankaState& msg) {
  Eigen::Map<const Matrix4d> leader_T_EE(msg.o_t_ee.data());

  Vector3d pos = leader_T_EE.block<3, 1>(0, 3);
  Quaterniond ori(leader_T_EE.block<3, 3>(0, 0));

  if (!std::isfinite(pos.x()) || !std::isfinite(pos.y()) || !std::isfinite(pos.z()) ||
      !std::isfinite(ori.w()) || !std::isfinite(ori.x()) ||
      !std::isfinite(ori.y()) || !std::isfinite(ori.z())) {
    return;
  }

  const double n = ori.norm();
  if (n < 1e-9) {
    return;
  }
  ori.normalize();

  DesiredPoseRT d;
  d.px = pos.x();
  d.py = pos.y();
  d.pz = pos.z();
  d.qw = ori.w();
  d.qx = ori.x();
  d.qy = ori.y();
  d.qz = ori.z();

  leader_pose_cache_.writeFromNonRT(d);

  if (!accept_desired_.load(std::memory_order_relaxed)) {
    return;
  }
  if (execution_running_.load(std::memory_order_relaxed)) {
    return;
  }
  if (pending_blend_to_leader_.load(std::memory_order_relaxed)) {
    return;
  }
  if (blending_to_leader_.load(std::memory_order_relaxed)) {
    return;
  }

  desired_pose_buffer_.writeFromNonRT(d);
  desired_pose_seq_.fetch_add(1, std::memory_order_release);
}

void CartesianImpedanceController::executionDesiredPoseCallback(
    const std_msgs::msg::Float64MultiArray& msg) {
  if (!accept_desired_.load(std::memory_order_relaxed)) {
    return;
  }
  if (!execution_running_.load(std::memory_order_acquire)) {
    return;
  }
  if (pending_blend_to_leader_.load(std::memory_order_acquire) ||
      blending_to_leader_.load(std::memory_order_acquire)) {
    return;
  }
  if (msg.data.size() < 7) {
    return;
  }

  DesiredPoseRT d;
  d.px = msg.data[0];
  d.py = msg.data[1];
  d.pz = msg.data[2];
  d.qx = msg.data[3];
  d.qy = msg.data[4];
  d.qz = msg.data[5];
  d.qw = msg.data[6];

  if (!std::isfinite(d.px) || !std::isfinite(d.py) || !std::isfinite(d.pz) ||
      !std::isfinite(d.qw) || !std::isfinite(d.qx) ||
      !std::isfinite(d.qy) || !std::isfinite(d.qz)) {
    return;
  }

  Quaterniond ori(d.qw, d.qx, d.qy, d.qz);
  const double n = ori.norm();
  if (n < 1e-9) {
    return;
  }
  ori.normalize();
  d.qw = ori.w();
  d.qx = ori.x();
  d.qy = ori.y();
  d.qz = ori.z();

  desired_pose_buffer_.writeFromNonRT(d);
  desired_pose_seq_.fetch_add(1, std::memory_order_release);
}

void CartesianImpedanceController::executionRunningCallback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  if (!msg) {
    return;
  }
  const bool now = msg->data;
  const bool prev = prev_execution_running_;
  prev_execution_running_ = now;
  if (prev && !now) {
    pending_blend_to_leader_.store(true, std::memory_order_release);
  }
  execution_running_.store(now, std::memory_order_release);
}

void CartesianImpedanceController::onlineFuserActiveCallback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  online_fuser_active_.store(msg && msg->data, std::memory_order_release);
}

Quaterniond CartesianImpedanceController::quatFromDesiredPose(const DesiredPoseRT& p) {
  Quaterniond q(p.qw, p.qx, p.qy, p.qz);
  if (q.norm() < 1e-9) {
    return Quaterniond::Identity();
  }
  q.normalize();
  return q;
}

void CartesianImpedanceController::desiredPoseFromQuaternion(const Quaterniond& q_in, DesiredPoseRT* out) {
  Quaterniond q = q_in;
  q.normalize();
  out->qw = q.w();
  out->qx = q.x();
  out->qy = q.y();
  out->qz = q.z();
}

}  // namespace geo_gp_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(geo_gp_controllers::CartesianImpedanceController,
                       controller_interface::ControllerInterface)
