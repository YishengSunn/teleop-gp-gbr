#include <geo_gp_controllers/subscriber/cartesian_impedance_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <chrono>
#include <algorithm>

#include <franka/model.h>

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

  for (const auto& name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(name);
  }
  return config;
}

controller_interface::return_type
CartesianImpedanceController::update(const rclcpp::Time& /*time*/,
                                     const rclcpp::Duration& /*period*/) {
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
        const Quaterniond q1 = quatFromDesiredPose(blend_pose_goal_);
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
    const Quaterniond q_interp = q0.slerp(s, q1);

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

    sub_leader_robot_state_ =
        get_node()->create_subscription<franka_msgs::msg::FrankaState>(
            get_node()->get_parameter("leader_robot_state_topic").as_string(),
            rclcpp::QoS(1),
            std::bind(&CartesianImpedanceController::leaderRobotStateCallback, this,
                      std::placeholders::_1));

    sub_execution_pose_ =
        get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
            get_node()->get_parameter("execution_pose_topic").as_string(),
            rclcpp::QoS(10),
            std::bind(&CartesianImpedanceController::executionDesiredPoseCallback, this,
                      std::placeholders::_1));

    sub_execution_running_ =
        get_node()->create_subscription<std_msgs::msg::Bool>(
            get_node()->get_parameter("execution_running_topic").as_string(),
            rclcpp::QoS(1).transient_local(),
            std::bind(&CartesianImpedanceController::executionRunningCallback, this,
                      std::placeholders::_1));

    pub_blend_running_ = get_node()->create_publisher<std_msgs::msg::Bool>(
        get_node()->get_parameter("blend_running_topic").as_string(),
        rclcpp::QoS(1).transient_local());

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

  return CallbackReturn::SUCCESS;
}

CallbackReturn
CartesianImpedanceController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  accept_desired_.store(false, std::memory_order_release);

  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

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
  if (pub_blend_running_) {
    std_msgs::msg::Bool m;
    m.data = false;
    pub_blend_running_->publish(m);
  }

  stiffness_.setIdentity();
  stiffness_.topLeftCorner(3, 3) = pos_stiff_ * Matrix3d::Identity();
  stiffness_.bottomRightCorner(3, 3) = rot_stiff_ * Matrix3d::Identity();

  damping_.setIdentity();
  damping_.topLeftCorner(3, 3) = 2.0 * std::sqrt(pos_stiff_) * Matrix3d::Identity();
  damping_.bottomRightCorner(3, 3) = 0.8 * 2.0 * std::sqrt(rot_stiff_) * Matrix3d::Identity();

  dq_filtered_.setZero();

  if (move_to_start_) {
    Eigen::Map<const Vector7d> q(franka_robot_model_->getRobotState()->q.data());
    motion_generator_ = std::make_unique<MotionGenerator>(0.2, q, q_start_);
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
  return CallbackReturn::SUCCESS;
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
  execution_running_.store(now, std::memory_order_release);
  if (prev && !now) {
    pending_blend_to_leader_.store(true, std::memory_order_release);
  }
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
