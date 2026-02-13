
#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <algorithm>

#include "garmi_controllers/dual_cartesian_impedance_controller.hpp"

namespace garmi_controllers {

/*---------------- SVD-based pseudo-inverse ---------------*/
inline void DualCartesianImpedanceController::pseudoInverse(
  const Eigen::MatrixXd& M, Eigen::MatrixXd& M_pinv, bool damped) {
const double lambda = damped ? 0.2 : 0.0;
Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
const auto& S = svd.singularValues();

// Build S+ with shape (n x m) where M is (m x n)
Eigen::MatrixXd Splus = Eigen::MatrixXd::Zero(M.cols(), M.rows());
for (int i = 0; i < S.size(); ++i) {
  const double s = S(i);
  Splus(i, i) = s / (s*s + lambda*lambda);
}

// M+ = V * S+ * U^T  (NO transpose on Splus)
M_pinv = svd.matrixV() * Splus * svd.matrixU().transpose();
}


/*---------------- Small rotation integration ----------------*/
inline Eigen::Quaterniond DualCartesianImpedanceController::smallAngleQuat(
    const Eigen::Vector3d& dtheta) {
  double ang = dtheta.norm();
  if (ang < 1e-9) return Eigen::Quaterniond::Identity();
  Eigen::Vector3d axis = dtheta / ang;
  double half = 0.5 * ang;
  return Eigen::Quaterniond(std::cos(half), axis.x()*std::sin(half),
                            axis.y()*std::sin(half), axis.z()*std::sin(half));
}

inline void DualCartesianImpedanceController::integrate_delta(
    Arm& a, const geometry_msgs::msg::Twist& d, double max_lin, double max_rot) {
  Eigen::Vector3d dpos(d.linear.x, d.linear.y, d.linear.z);
  Eigen::Vector3d drot(d.angular.x, d.angular.y, d.angular.z);

  if (dpos.norm() > max_lin) dpos *= (max_lin / std::max(1e-12, dpos.norm()));
  if (drot.norm() > max_rot) drot *= (max_rot / std::max(1e-12, drot.norm()));

  a.desired_position += dpos;                            // base-frame deltas
  a.desired_orientation = smallAngleQuat(drot) * a.desired_orientation;
  a.desired_orientation.normalize();
}

/*---------------- Interface declarations ----------------*/
controller_interface::InterfaceConfiguration
DualCartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Prefer parameterized IDs if arms_ not filled yet
  std::vector<std::string> ids;
  if (!arm_order_.empty()) {
    ids = arm_order_;
  } else {
    ids.push_back(arm_id_1_);
    ids.push_back(arm_id_2_);
  }
  for (const auto& id : ids) {
    for (int i = 1; i <= num_joints; ++i) {
      config.names.push_back(id + "_joint" + std::to_string(i) + "/effort");
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration
DualCartesianImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Cache to avoid spamming "Initialized FrankaRobotModel ..."
  static std::vector<std::string> cached_names;
  static std::string cached_arm1, cached_arm2;

  // Rebuild cache only if empty or arm IDs changed
  if (cached_names.empty() || cached_arm1 != arm_id_1_ || cached_arm2 != arm_id_2_) {
    cached_names.clear();
    cached_arm1 = arm_id_1_;
    cached_arm2 = arm_id_2_;

    {
      franka_semantic_components::FrankaRobotModel model(arm_id_1_ + "/robot_model", arm_id_1_);
      for (const auto& n : model.get_state_interface_names()) cached_names.push_back(n);
    }
    {
      franka_semantic_components::FrankaRobotModel model(arm_id_2_ + "/robot_model", arm_id_2_);
      for (const auto& n : model.get_state_interface_names()) cached_names.push_back(n);
    }
  }

  config.names = cached_names;
  return config;
}

/*---------------- Lifecycle ----------------*/
controller_interface::CallbackReturn
DualCartesianImpedanceController::on_init() {
  try {
    auto_declare<std::string>("arm_1.arm_id", "left");
    auto_declare<std::string>("arm_2.arm_id", "right");
    auto_declare<std::string>("arm_1.delta_topic", "/teleop/left/ee_delta");
    auto_declare<std::string>("arm_2.delta_topic", "/teleop/right/ee_delta");

    auto_declare<double>("pos_stiff", 400.0);
    auto_declare<double>("rot_stiff", 20.0);
    auto_declare<double>("nullspace_stiff", 10.0);
    auto_declare<double>("max_delta_lin", 0.03);
    auto_declare<double>("max_delta_rot", 0.15);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception during init: %s\n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
DualCartesianImpedanceController::on_configure(const rclcpp_lifecycle::State&) {
  arm_id_1_ = get_node()->get_parameter("arm_1.arm_id").as_string();
  arm_id_2_ = get_node()->get_parameter("arm_2.arm_id").as_string();
  delta_topic_1_ = get_node()->get_parameter("arm_1.delta_topic").as_string();
  delta_topic_2_ = get_node()->get_parameter("arm_2.delta_topic").as_string();

  pos_stiff_ = get_node()->get_parameter("pos_stiff").as_double();
  rot_stiff_ = get_node()->get_parameter("rot_stiff").as_double();
  nullspace_stiff_ = get_node()->get_parameter("nullspace_stiff").as_double();
  max_delta_lin_ = get_node()->get_parameter("max_delta_lin").as_double();
  max_delta_rot_ = get_node()->get_parameter("max_delta_rot").as_double();

  if (arm_id_1_.empty() || arm_id_2_.empty() || arm_id_1_ == arm_id_2_) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Invalid arm IDs: '%s' vs '%s'", arm_id_1_.c_str(), arm_id_2_.c_str());
    return CallbackReturn::ERROR;
  }

  arms_.clear();
  arm_order_.clear();
  for (auto [id, topic] : { std::pair{arm_id_1_, delta_topic_1_},
                            std::pair{arm_id_2_, delta_topic_2_} }) {
    Arm a;
    a.arm_id = id;
    a.delta_topic = topic;
    a.model = std::make_unique<franka_semantic_components::FrankaRobotModel>(
                  id + "/robot_model", id);

    // Gains (same structure as your example: build from scalars)
    a.stiffness.setIdentity();
    a.stiffness.topLeftCorner(3,3)     = pos_stiff_ * Eigen::Matrix3d::Identity();
    a.stiffness.bottomRightCorner(3,3) = rot_stiff_ * Eigen::Matrix3d::Identity();

    a.damping.setIdentity();
    a.damping.topLeftCorner(3,3)       = 2.0 * std::sqrt(pos_stiff_) * Eigen::Matrix3d::Identity();
    a.damping.bottomRightCorner(3,3)   = 0.8 * 2.0 * std::sqrt(rot_stiff_) * Eigen::Matrix3d::Identity();

    a.n_stiffness = nullspace_stiff_;
    a.max_delta_lin = max_delta_lin_;
    a.max_delta_rot = max_delta_rot_;

    arms_.emplace(id, std::move(a));
    arm_order_.push_back(id);
  }

  RCLCPP_INFO(get_node()->get_logger(),
              "Configured DualCartesianImpedanceController for arms '%s' & '%s'",
              arm_id_1_.c_str(), arm_id_2_.c_str());
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
DualCartesianImpedanceController::on_activate(const rclcpp_lifecycle::State&) {
  // Bind model interfaces and initialize setpoints from current pose/q
  for (auto& [id, a] : arms_) {
    a.model->assign_loaned_state_interfaces(state_interfaces_);

    Eigen::Map<const Matrix4d> H(a.model->getPoseMatrix(franka::Frame::kEndEffector).data());
    a.desired_position   = Vector3d(H.block<3,1>(0,3));
    a.desired_orientation = Quaterniond(H.block<3,3>(0,0));
    a.desired_orientation.normalize();

    a.desired_qn = Vector7d(a.model->getRobotState()->q.data());

    // Subscribe teleop deltas
    a.sub_delta = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
      a.delta_topic, rclcpp::SystemDefaultsQoS(),
      [this, &a](const geometry_msgs::msg::TwistStamped& msg) {
        integrate_delta(a, msg.twist, a.max_delta_lin, a.max_delta_rot);
      });
  }

  start_time_ = get_node()->now();
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
DualCartesianImpedanceController::on_deactivate(const rclcpp_lifecycle::State&) {
  for (auto& [id, a] : arms_) {
    a.sub_delta.reset();
    a.model->release_interfaces();
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
DualCartesianImpedanceController::on_error(const rclcpp_lifecycle::State&) {
  RCLCPP_ERROR(get_node()->get_logger(), "DualCartesianImpedanceController error");
  return CallbackReturn::ERROR;
}

/*---------------- Control loop ----------------*/
controller_interface::return_type
DualCartesianImpedanceController::update(const rclcpp::Time&, const rclcpp::Duration&) {
  // Command interfaces are ordered by arm_order_: joint1..7 (arm1), then joint1..7 (arm2)
  std::size_t k_cmd = 0;

  for (const auto& id : arm_order_) {
    auto& a = arms_.at(id);

    // Read model state (same calls as your single-arm code)
    Eigen::Map<const Matrix4d> H(a.model->getPoseMatrix(franka::Frame::kEndEffector).data());
    Eigen::Vector3d current_position(H.block<3,1>(0,3));
    Eigen::Quaterniond current_orientation(H.block<3,3>(0,0));
    Eigen::Map<const Matrix7d> inertia(a.model->getMassMatrix().data());      // not used directly here
    Eigen::Map<const Vector7d> coriolis(a.model->getCoriolisForceVector().data());
    Eigen::Matrix<double,6,7> jacobian(a.model->getZeroJacobian(franka::Frame::kEndEffector).data());
    Eigen::Map<const Vector7d> qD(a.model->getRobotState()->dq.data());
    Eigen::Map<const Vector7d> q(a.model->getRobotState()->q.data());

    // Error (position + quaternion orientation, same structure as yours)
    Vector6d error;
    error.head<3>() = current_position - a.desired_position;

    // Ensure shortest quaternion path
    if (a.desired_orientation.coeffs().dot(current_orientation.coeffs()) < 0.0) {
      current_orientation.coeffs() << -current_orientation.coeffs();
    }
    Eigen::Quaterniond rot_error(current_orientation * a.desired_orientation.inverse());
    Eigen::AngleAxisd rot_error_aa(rot_error);
    error.tail<3>() = rot_error_aa.axis() * rot_error_aa.angle();

    // Task-space torque
    Vector7d tau_task = Vector7d::Zero();
    tau_task = jacobian.transpose() * (-a.stiffness * error - a.damping * (jacobian * qD));

    // Nullspace
    Eigen::MatrixXd JT = jacobian.transpose();
    Eigen::MatrixXd JT_pinv;
    pseudoInverse(JT, JT_pinv);
    Vector7d tau_null = (Eigen::Matrix<double,7,7>::Identity() - JT * JT_pinv) *
                         ( a.n_stiffness * (a.desired_qn - q)
                           - (2.0 * std::sqrt(a.n_stiffness)) * qD );

    // Final torque with coriolis feedforward (Franka example style)
    Vector7d tau_d = tau_task + coriolis + tau_null;

    // Write commands
    for (int j = 0; j < num_joints; ++j, ++k_cmd) {
      command_interfaces_[k_cmd].set_value(tau_d(j));
    }
  }

  return controller_interface::return_type::OK;
}

/*---------------- Plugin export ----------------*/
} // namespace garmi_controllers

// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(garmi_controllers::DualCartesianImpedanceController,
                       controller_interface::ControllerInterface)
