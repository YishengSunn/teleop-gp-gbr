#include <teleop_controllers/subscriber/cartesian_impedance_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

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

namespace teleop_controllers {

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

  // Cartesian mode: only apply desired pose if a new message arrived.
  if (!skip_pose_read_this_cycle) {
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
  } else {
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

  return controller_interface::return_type::OK;
}

CallbackReturn
CartesianImpedanceController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");

    auto_declare<double>("pos_stiff", 100.0);
    auto_declare<double>("rot_stiff", 10.0);
    auto_declare<double>("n_stiffness", 10.0);

    auto_declare<std::string>("cartesian_pose_topic", "/cartesian_impedance/pose_desired");

    auto_declare<bool>("move_to_start", false);
    auto_declare<std::vector<double>>(
        "start_joint_configuration",
        {0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4});

    auto_declare<std::vector<double>>("start_k_gains", {});
    auto_declare<std::vector<double>>("start_d_gains", {});

    sub_desired_cartesian_ =
        get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
            get_node()->get_parameter("cartesian_pose_topic").as_string(),
            rclcpp::QoS(1),
            std::bind(&CartesianImpedanceController::desiredCartesianCallback, this,
                      std::placeholders::_1));

  } catch (const std::exception& e) {
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

  cartesian_pose_topic_ = get_node()->get_parameter("cartesian_pose_topic").as_string();

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
  } else {
    mode_.store(static_cast<uint8_t>(Mode::CARTESIAN), std::memory_order_release);
    accept_desired_.store(true, std::memory_order_release);
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn
CartesianImpedanceController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

void CartesianImpedanceController::desiredCartesianCallback(const geometry_msgs::msg::PoseStamped& msg) {
  if (!accept_desired_.load(std::memory_order_relaxed)) {
    return;
  }

  Quaterniond ori(msg.pose.orientation.w,
                  msg.pose.orientation.x,
                  msg.pose.orientation.y,
                  msg.pose.orientation.z);

  if (!std::isfinite(ori.w()) || !std::isfinite(ori.x()) ||
      !std::isfinite(ori.y()) || !std::isfinite(ori.z())) {
    return;
  }
  const double n = ori.norm();
  if (n < 1e-9) {
    return;
  }
  ori.normalize();

  DesiredPoseRT d;
  d.px = msg.pose.position.x;
  d.py = msg.pose.position.y;
  d.pz = msg.pose.position.z;

  d.qw = ori.w();
  d.qx = ori.x();
  d.qy = ori.y();
  d.qz = ori.z();

  desired_pose_buffer_.writeFromNonRT(d);
  desired_pose_seq_.fetch_add(1, std::memory_order_release);
}

}  // namespace teleop_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(teleop_controllers::CartesianImpedanceController,
                       controller_interface::ControllerInterface)
