#include <geo_gp_controllers/subscriber/hybrid_position_force_controller.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <string>

#include <franka/model.h>
#include <pluginlib/class_list_macros.hpp>

namespace {

inline void pseudoInverse(const Eigen::MatrixXd& matrix, Eigen::MatrixXd& matrix_pinv, bool damped = true) {
  const double lambda = damped ? 0.2 : 0.0;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto singular_values = svd.singularValues();
  Eigen::MatrixXd singular_value_inv = matrix;
  singular_value_inv.setZero();

  for (int i = 0; i < singular_values.size(); ++i) {
    singular_value_inv(i, i) =
        singular_values(i) / (singular_values(i) * singular_values(i) + lambda * lambda);
  }

  matrix_pinv =
      Eigen::MatrixXd(svd.matrixV() * singular_value_inv.transpose() * svd.matrixU().transpose());
}

inline double clampAbs(double value, double max_abs) {
  if (!std::isfinite(value)) {
    return 0.0;
  }
  if (value > max_abs) {
    return max_abs;
  }
  if (value < -max_abs) {
    return -max_abs;
  }
  return value;
}

inline double clampRange(double value, double min_value, double max_value) {
  return std::max(min_value, std::min(max_value, value));
}

Eigen::Quaterniond slerpShortestArc(
    const Eigen::Quaterniond& q0_in, const Eigen::Quaterniond& q1_in, double s) {
  Eigen::Quaterniond q0 = q0_in.normalized();
  Eigen::Quaterniond q1 = q1_in.normalized();
  if (q0.dot(q1) < 0.0) {
    q1.coeffs() *= -1.0;
  }
  return q0.slerp(s, q1).normalized();
}

}  // namespace

namespace geo_gp_controllers {

controller_interface::InterfaceConfiguration
HybridPositionForceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
HybridPositionForceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  if (franka_robot_model_) {
    for (const auto& name : franka_robot_model_->get_state_interface_names()) {
      config.names.push_back(name);
    }
  }
  else {
    franka_semantic_components::FrankaRobotModel temp_model(
        arm_id_ + "/robot_model", arm_id_);
    for (const auto& name : temp_model.get_state_interface_names()) {
      config.names.push_back(name);
    }
  }
  return config;
}

controller_interface::return_type HybridPositionForceController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  const auto current_pose_array =
      franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Map<const Matrix4d> current_pose(current_pose_array.data());

  Vector3d current_position(current_pose.block<3, 1>(0, 3));
  Quaterniond current_orientation(current_pose.block<3, 3>(0, 0));
  current_orientation.normalize();

  const auto coriolis_array =
      franka_robot_model_->getCoriolisForceVector();
  Eigen::Map<const Vector7d> coriolis(coriolis_array.data());
  const auto jacobian_array =
      franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

  const auto* robot_state = franka_robot_model_->getRobotState();
  Eigen::Map<const Vector7d> qD(robot_state->dq.data());
  Eigen::Map<const Vector7d> q(robot_state->q.data());

  bool skip_pose_read_this_cycle = false;

  const auto mode_now = static_cast<Mode>(mode_.load(std::memory_order_relaxed));
  if (mode_now == Mode::MOVE_TO_START) {
    const auto t = this->get_node()->now() - start_time_;
    const auto out = motion_generator_->getDesiredJointPositions(t);

    const Vector7d q_desired = out.first;
    const bool finished = out.second;

    if (!finished) {
      const double k_alpha = 0.99;
      dq_filtered_ = (1.0 - k_alpha) * dq_filtered_ + k_alpha * qD;

      const Vector7d tau = k_start_.cwiseProduct(q_desired - q) +
                           d_start_.cwiseProduct(-dq_filtered_);

      for (int i = 0; i < kNumJoints; ++i) {
        command_interfaces_[i].set_value(tau(i));
      }
      return controller_interface::return_type::OK;
    }

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
    last_desired_pose_seq_ = desired_pose_seq_.load(std::memory_order_acquire);

    accept_desired_.store(true, std::memory_order_release);
    mode_.store(static_cast<uint8_t>(Mode::HYBRID), std::memory_order_release);
    skip_pose_read_this_cycle = true;
  }

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
        desiredPoseFromQuaternion(current_orientation, &blend_pose_start_);

        blend_pose_goal_ = *goal;

        const Vector3d p0(
            blend_pose_start_.px, blend_pose_start_.py, blend_pose_start_.pz);
        const Vector3d p1(
            blend_pose_goal_.px, blend_pose_goal_.py, blend_pose_goal_.pz);
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
        blend_duration_sec_ = clampRange(
            blend_duration_sec_, blend_duration_min_, blend_duration_max_);

        blend_t0_ = this->get_node()->now();
        blending_to_leader_.store(true, std::memory_order_release);
        mode_.store(
            static_cast<uint8_t>(Mode::BLEND_TO_LEADER), std::memory_order_release);
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

  if (static_cast<Mode>(mode_.load(std::memory_order_relaxed)) ==
      Mode::BLEND_TO_LEADER) {
    const double elapsed = (this->get_node()->now() - blend_t0_).seconds();
    const double s =
        (blend_duration_sec_ > 1e-6) ? std::min(1.0, elapsed / blend_duration_sec_) : 1.0;

    const Vector3d p0(
        blend_pose_start_.px, blend_pose_start_.py, blend_pose_start_.pz);
    const Vector3d p1(
        blend_pose_goal_.px, blend_pose_goal_.py, blend_pose_goal_.pz);
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
          this->get_node()->now() +
          rclcpp::Duration::from_seconds(blend_running_hold_sec_);
      mode_.store(static_cast<uint8_t>(Mode::HYBRID), std::memory_order_release);
    }

    skip_pose_read_this_cycle = true;
  }

  if (!skip_pose_read_this_cycle &&
      static_cast<Mode>(mode_.load(std::memory_order_relaxed)) !=
          Mode::BLEND_TO_LEADER) {
    const uint64_t seq = desired_pose_seq_.load(std::memory_order_acquire);
    if (seq != last_desired_pose_seq_) {
      const auto* dp = desired_pose_buffer_.readFromRT();
      if (dp) {
        desired_pose_rt_ = *dp;
      }
      last_desired_pose_seq_ = seq;
    }
  }

  const uint64_t force_seq = desired_force_seq_.load(std::memory_order_acquire);
  if (force_seq != last_desired_force_seq_) {
    const auto* desired_force = desired_force_buffer_.readFromRT();
    if (desired_force) {
      desired_force_rt_ = *desired_force;
    }
    last_desired_force_seq_ = force_seq;
  }

  Vector3d desired_position;
  desired_position << desired_pose_rt_.px, desired_pose_rt_.py, desired_pose_rt_.pz;

  Quaterniond desired_orientation(
      desired_pose_rt_.qw,
      desired_pose_rt_.qx,
      desired_pose_rt_.qy,
      desired_pose_rt_.qz);
  if (desired_orientation.norm() < 1e-9) {
    desired_orientation = Quaterniond::Identity();
  }
  else {
    desired_orientation.normalize();
  }

  Vector3d position_error = current_position - desired_position;
  Vector6d cartesian_velocity = jacobian * qD;
  Vector3d linear_velocity = cartesian_velocity.head(3);
  Vector3d angular_velocity = cartesian_velocity.tail(3);

  if (desired_orientation.coeffs().dot(current_orientation.coeffs()) < 0.0) {
    current_orientation.coeffs() = -current_orientation.coeffs();
  }

  Quaterniond rot_error(current_orientation * desired_orientation.inverse());
  Eigen::AngleAxisd rot_error_aa(rot_error);
  Vector3d orientation_error = rot_error_aa.axis() * rot_error_aa.angle();

  // Z (force_axis_) uses Cartesian impedance while teleoperating; during prediction
  // execution (execution_running) it uses the force controller on that axis.
  const bool force_control_on_hybrid_axis =
      execution_running_.load(std::memory_order_relaxed);
  if (force_control_on_hybrid_axis != last_execution_running_for_z_axis_) {
    force_error_integral_ = 0.0;
    last_execution_running_for_z_axis_ = force_control_on_hybrid_axis;
  }

  Vector6d wrench_command = Vector6d::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    if (force_control_on_hybrid_axis && axis == force_axis_) {
      continue;
    }
    wrench_command(axis) = -pos_stiff_ * position_error(axis) -
                           translational_damping_ * linear_velocity(axis);
  }

  if (force_control_on_hybrid_axis) {
    double raw_force_measurement = robot_state->O_F_ext_hat_K[force_axis_];
    if (!std::isfinite(raw_force_measurement)) {
      raw_force_measurement =
          force_filter_initialized_ ? filtered_force_measurement_ : 0.0;
    }

    if (!force_filter_initialized_) {
      filtered_force_measurement_ = raw_force_measurement;
      force_filter_initialized_ = true;
    }
    else {
      filtered_force_measurement_ =
          measured_force_filter_alpha_ * filtered_force_measurement_ +
          (1.0 - measured_force_filter_alpha_) * raw_force_measurement;
    }

    const double force_error = desired_force_rt_.value - filtered_force_measurement_;
    const double dt = std::max(1e-6, period.seconds());
    force_error_integral_ =
        clampRange(force_error_integral_ + dt * force_error,
                   -force_integral_limit_, force_integral_limit_);

    double hybrid_axis_wrench =
        force_feedforward_scale_ * desired_force_rt_.value +
        force_kp_ * force_error +
        force_ki_ * force_error_integral_ -
        force_damping_ * linear_velocity(force_axis_);
    hybrid_axis_wrench = clampAbs(hybrid_axis_wrench, force_command_max_abs_);
    wrench_command(force_axis_) = hybrid_axis_wrench;
  }

  wrench_command.tail(3) =
      -rot_stiff_ * orientation_error - rotational_damping_ * angular_velocity;

  Vector7d tau_task = jacobian.transpose() * wrench_command;

  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  Vector7d tau_nullspace =
      (Eigen::MatrixXd::Identity(kNumJoints, kNumJoints) -
       jacobian.transpose() * jacobian_transpose_pinv) *
      (n_stiffness_ * (desired_qn_ - q) -
       (2.0 * std::sqrt(n_stiffness_)) * qD);

  Vector7d tau_d = tau_task + coriolis + tau_nullspace;

  for (int i = 0; i < kNumJoints; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
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

CallbackReturn HybridPositionForceController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");

    auto_declare<double>("pos_stiff", 100.0);
    auto_declare<double>("rot_stiff", 10.0);
    auto_declare<double>("n_stiffness", 10.0);

    auto_declare<std::string>("leader_robot_state_topic", "/leader/franka_robot_state_broadcaster/robot_state");
    auto_declare<std::string>("execution_pose_topic", "/execution/desired_pose");
    auto_declare<std::string>("desired_force_topic", "/execution/desired_force");
    auto_declare<std::string>("execution_running_topic", "/execution/running");
    auto_declare<std::string>("blend_running_topic", "/execution/blend_to_leader_running");

    auto_declare<bool>("blend_to_leader_enabled", true);
    auto_declare<double>("blend_seconds_per_meter", 2.0);
    auto_declare<double>("blend_seconds_per_rad", 1.2);
    auto_declare<double>("blend_duration_min", 0.25);
    auto_declare<double>("blend_duration_max", 8.0);
    auto_declare<double>("blend_running_hold_sec", 0.5);

    auto_declare<std::string>("force_axis", "z");
    auto_declare<double>("force_kp", 1.0);
    auto_declare<double>("force_ki", 0.0);
    auto_declare<double>("force_damping", 5.0);
    auto_declare<double>("force_feedforward_scale", 1.0);
    auto_declare<double>("force_integral_limit", 10.0);
    auto_declare<double>("force_command_max_abs", 30.0);
    auto_declare<double>("measured_force_filter_alpha", 0.95);
    auto_declare<double>("initial_desired_force", 0.0);

    auto_declare<bool>("move_to_start", false);
    auto_declare<std::vector<double>>(
        "start_joint_configuration",
        {0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4});
    auto_declare<std::vector<double>>(
        "start_k_gains", {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0});
    auto_declare<std::vector<double>>(
        "start_d_gains", {30.0, 30.0, 30.0, 30.0, 10.0, 10.0, 5.0});
  }
  catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn HybridPositionForceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();

  pos_stiff_ = get_node()->get_parameter("pos_stiff").as_double();
  rot_stiff_ = get_node()->get_parameter("rot_stiff").as_double();
  n_stiffness_ = get_node()->get_parameter("n_stiffness").as_double();
  translational_damping_ = 2.0 * std::sqrt(pos_stiff_);
  rotational_damping_ = 0.8 * 2.0 * std::sqrt(rot_stiff_);

  leader_robot_state_topic_ =
      get_node()->get_parameter("leader_robot_state_topic").as_string();
  execution_pose_topic_ =
      get_node()->get_parameter("execution_pose_topic").as_string();
  desired_force_topic_ =
      get_node()->get_parameter("desired_force_topic").as_string();
  execution_running_topic_ =
      get_node()->get_parameter("execution_running_topic").as_string();
  blend_running_topic_ =
      get_node()->get_parameter("blend_running_topic").as_string();

  blend_to_leader_enabled_ =
      get_node()->get_parameter("blend_to_leader_enabled").as_bool();
  blend_seconds_per_meter_ =
      get_node()->get_parameter("blend_seconds_per_meter").as_double();
  blend_seconds_per_rad_ =
      get_node()->get_parameter("blend_seconds_per_rad").as_double();
  blend_duration_min_ =
      get_node()->get_parameter("blend_duration_min").as_double();
  blend_duration_max_ =
      get_node()->get_parameter("blend_duration_max").as_double();
  blend_running_hold_sec_ =
      get_node()->get_parameter("blend_running_hold_sec").as_double();

  force_axis_name_ = get_node()->get_parameter("force_axis").as_string();
  if (!forceAxisFromString(force_axis_name_, &force_axis_)) {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "Invalid force_axis='%s'. Supported values: x, y, z",
        force_axis_name_.c_str());
    return CallbackReturn::FAILURE;
  }

  force_kp_ = get_node()->get_parameter("force_kp").as_double();
  force_ki_ = get_node()->get_parameter("force_ki").as_double();
  force_damping_ = get_node()->get_parameter("force_damping").as_double();
  force_feedforward_scale_ =
      get_node()->get_parameter("force_feedforward_scale").as_double();
  force_integral_limit_ =
      std::abs(get_node()->get_parameter("force_integral_limit").as_double());
  force_command_max_abs_ =
      std::abs(get_node()->get_parameter("force_command_max_abs").as_double());
  measured_force_filter_alpha_ = clampRange(
      get_node()->get_parameter("measured_force_filter_alpha").as_double(), 0.0, 0.999);
  initial_desired_force_ =
      get_node()->get_parameter("initial_desired_force").as_double();

  move_to_start_ =
      get_node()->get_parameter("move_to_start").as_bool();
  const auto start_q =
      get_node()->get_parameter("start_joint_configuration").as_double_array();
  if (start_q.size() != static_cast<size_t>(kNumJoints)) {
    RCLCPP_FATAL(
        get_node()->get_logger(),
        "start_joint_configuration must have size %d (got %ld)",
        kNumJoints, start_q.size());
    return CallbackReturn::FAILURE;
  }
  q_start_ = Eigen::Map<const Vector7d>(start_q.data());

  const auto k_start = get_node()->get_parameter("start_k_gains").as_double_array();
  const auto d_start = get_node()->get_parameter("start_d_gains").as_double_array();
  if (move_to_start_) {
    if (k_start.size() != static_cast<size_t>(kNumJoints) ||
        d_start.size() != static_cast<size_t>(kNumJoints)) {
      RCLCPP_FATAL(
          get_node()->get_logger(),
          "start_k_gains and start_d_gains must be size %d when move_to_start=true",
          kNumJoints);
      return CallbackReturn::FAILURE;
    }
    for (int i = 0; i < kNumJoints; ++i) {
      k_start_(i) = k_start.at(i);
      d_start_(i) = d_start.at(i);
    }
  }

  franka_robot_model_ =
      std::make_unique<franka_semantic_components::FrankaRobotModel>(
          franka_semantic_components::FrankaRobotModel(arm_id_ + "/robot_model", arm_id_));

  sub_leader_robot_state_.reset();
  sub_execution_pose_.reset();
  sub_desired_force_.reset();
  sub_execution_running_.reset();
  pub_blend_running_.reset();

  sub_leader_robot_state_ =
      get_node()->create_subscription<franka_msgs::msg::FrankaState>(
          leader_robot_state_topic_,
          rclcpp::QoS(1),
          std::bind(
              &HybridPositionForceController::leaderRobotStateCallback,
              this,
              std::placeholders::_1));

  sub_execution_pose_ =
      get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
          execution_pose_topic_,
          rclcpp::QoS(10),
          std::bind(
              &HybridPositionForceController::executionDesiredPoseCallback,
              this,
              std::placeholders::_1));

  sub_desired_force_ = get_node()->create_subscription<std_msgs::msg::Float64>(
      desired_force_topic_,
      rclcpp::QoS(10),
      std::bind(
          &HybridPositionForceController::executionDesiredForceCallback,
          this,
          std::placeholders::_1));

  sub_execution_running_ = get_node()->create_subscription<std_msgs::msg::Bool>(
      execution_running_topic_,
      rclcpp::QoS(1).transient_local(),
      std::bind(
          &HybridPositionForceController::executionRunningCallback,
          this,
          std::placeholders::_1));

  pub_blend_running_ = get_node()->create_publisher<std_msgs::msg::Bool>(
      blend_running_topic_,
      rclcpp::QoS(1).transient_local());

  return CallbackReturn::SUCCESS;
}

CallbackReturn HybridPositionForceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  accept_desired_.store(false, std::memory_order_release);

  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  const auto desired_init_array =
      franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Map<const Matrix4d> desired_init(desired_init_array.data());

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
  last_desired_pose_seq_ = desired_pose_seq_.load(std::memory_order_acquire);

  desired_force_rt_.value = initial_desired_force_;
  desired_force_buffer_.writeFromNonRT(desired_force_rt_);
  last_desired_force_seq_ = desired_force_seq_.load(std::memory_order_acquire);

  Eigen::Map<const Vector7d> q_initial(franka_robot_model_->getRobotState()->q.data());
  desired_qn_ = q_initial;

  prev_execution_running_ = false;
  execution_running_.store(false, std::memory_order_release);
  pending_blend_to_leader_.store(false, std::memory_order_release);
  blending_to_leader_.store(false, std::memory_order_release);
  blend_running_hold_until_ = this->get_node()->now();
  last_blend_running_published_ = false;
  force_error_integral_ = 0.0;
  filtered_force_measurement_ = 0.0;
  force_filter_initialized_ = false;
  last_execution_running_for_z_axis_ = false;
  dq_filtered_.setZero();

  if (pub_blend_running_) {
    std_msgs::msg::Bool msg;
    msg.data = false;
    pub_blend_running_->publish(msg);
  }

  if (move_to_start_) {
    Eigen::Map<const Vector7d> q(franka_robot_model_->getRobotState()->q.data());
    motion_generator_ = std::make_unique<MotionGenerator>(0.2, q, q_start_);
    start_time_ = this->get_node()->now();
    mode_.store(static_cast<uint8_t>(Mode::MOVE_TO_START), std::memory_order_release);
  }
  else {
    mode_.store(static_cast<uint8_t>(Mode::HYBRID), std::memory_order_release);
    accept_desired_.store(true, std::memory_order_release);
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn HybridPositionForceController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (pub_blend_running_) {
    std_msgs::msg::Bool msg;
    msg.data = false;
    pub_blend_running_->publish(msg);
  }
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

void HybridPositionForceController::leaderRobotStateCallback(
    const franka_msgs::msg::FrankaState& msg) {
  Eigen::Map<const Matrix4d> leader_pose(msg.o_t_ee.data());

  Vector3d pos = leader_pose.block<3, 1>(0, 3);
  Quaterniond ori(leader_pose.block<3, 3>(0, 0));

  if (!std::isfinite(pos.x()) || !std::isfinite(pos.y()) || !std::isfinite(pos.z()) ||
      !std::isfinite(ori.w()) || !std::isfinite(ori.x()) ||
      !std::isfinite(ori.y()) || !std::isfinite(ori.z())) {
    return;
  }

  const double norm = ori.norm();
  if (norm < 1e-9) {
    return;
  }
  ori.normalize();

  DesiredPoseRT desired_pose;
  desired_pose.px = pos.x();
  desired_pose.py = pos.y();
  desired_pose.pz = pos.z();
  desired_pose.qw = ori.w();
  desired_pose.qx = ori.x();
  desired_pose.qy = ori.y();
  desired_pose.qz = ori.z();

  leader_pose_cache_.writeFromNonRT(desired_pose);

  if (!accept_desired_.load(std::memory_order_relaxed) ||
      execution_running_.load(std::memory_order_relaxed) ||
      pending_blend_to_leader_.load(std::memory_order_relaxed) ||
      blending_to_leader_.load(std::memory_order_relaxed)) {
    return;
  }

  desired_pose_buffer_.writeFromNonRT(desired_pose);
  desired_pose_seq_.fetch_add(1, std::memory_order_release);
}

void HybridPositionForceController::executionDesiredPoseCallback(
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

  DesiredPoseRT desired_pose;
  desired_pose.px = msg.data[0];
  desired_pose.py = msg.data[1];
  desired_pose.pz = msg.data[2];
  desired_pose.qx = msg.data[3];
  desired_pose.qy = msg.data[4];
  desired_pose.qz = msg.data[5];
  desired_pose.qw = msg.data[6];

  if (!std::isfinite(desired_pose.px) || !std::isfinite(desired_pose.py) ||
      !std::isfinite(desired_pose.pz) || !std::isfinite(desired_pose.qw) ||
      !std::isfinite(desired_pose.qx) || !std::isfinite(desired_pose.qy) ||
      !std::isfinite(desired_pose.qz)) {
    return;
  }

  Quaterniond ori(
      desired_pose.qw, desired_pose.qx, desired_pose.qy, desired_pose.qz);
  if (ori.norm() < 1e-9) {
    return;
  }
  ori.normalize();
  desired_pose.qw = ori.w();
  desired_pose.qx = ori.x();
  desired_pose.qy = ori.y();
  desired_pose.qz = ori.z();

  desired_pose_buffer_.writeFromNonRT(desired_pose);
  desired_pose_seq_.fetch_add(1, std::memory_order_release);
}

void HybridPositionForceController::executionDesiredForceCallback(
    const std_msgs::msg::Float64& msg) {
  if (!accept_desired_.load(std::memory_order_relaxed) || !std::isfinite(msg.data)) {
    return;
  }

  DesiredForceRT desired_force;
  desired_force.value = msg.data;
  desired_force_buffer_.writeFromNonRT(desired_force);
  desired_force_seq_.fetch_add(1, std::memory_order_release);
}

void HybridPositionForceController::executionRunningCallback(
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

bool HybridPositionForceController::forceAxisFromString(
    const std::string& axis, int* axis_index) {
  if (!axis_index) {
    return false;
  }
  if (axis == "x" || axis == "X") {
    *axis_index = 0;
    return true;
  }
  if (axis == "y" || axis == "Y") {
    *axis_index = 1;
    return true;
  }
  if (axis == "z" || axis == "Z") {
    *axis_index = 2;
    return true;
  }
  return false;
}

Quaterniond HybridPositionForceController::quatFromDesiredPose(const DesiredPoseRT& p) {
  Quaterniond q(p.qw, p.qx, p.qy, p.qz);
  if (q.norm() < 1e-9) {
    return Quaterniond::Identity();
  }
  q.normalize();
  return q;
}

void HybridPositionForceController::desiredPoseFromQuaternion(
    const Quaterniond& q_in, DesiredPoseRT* out) {
  Quaterniond q = q_in;
  q.normalize();
  out->qw = q.w();
  out->qx = q.x();
  out->qy = q.y();
  out->qz = q.z();
}

}  // namespace geo_gp_controllers

PLUGINLIB_EXPORT_CLASS(
    geo_gp_controllers::HybridPositionForceController,
    controller_interface::ControllerInterface)
