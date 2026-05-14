#include <teleop_controllers/subscriber/haptic_gravity_compensation_controller.hpp>

#include <exception>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/qos.hpp>

namespace teleop_controllers {

// ----------- Interfaces -----------

controller_interface::InterfaceConfiguration
HapticGravityCompensationController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
HapticGravityCompensationController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Leader joint position + velocity (used for desired q and impedance error)
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }

  // Franka model state interfaces for FK (Cartesian desired).
  // NOTE: RobotModel uses "<arm_id>/robot_model" as its interface prefix.
  franka_semantic_components::FrankaRobotModel tmp(arm_id_ + "/robot_model", arm_id_);
  for (const auto& name : tmp.get_state_interface_names()) {
    config.names.push_back(name);
  }
  return config;
}

// ----------- Lifecycle -----------

HapticGravityCompensationController::CallbackReturn
HapticGravityCompensationController::on_init() {
  try {
    // Core params
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<std::string>("follower_input_mode", "torque_topic");   // or "joint_states"
    auto_declare<std::string>("follower_torque_topic", "/haptic/follower_tau");
    auto_declare<std::string>("joint_state_topic", "/follower/joint_states");
    auto_declare<double>("lpf_alpha", 0.1);
    auto_declare<std::vector<double>>("torque_clamp",
                                      std::vector<double>(num_joints, 5.0));

    // Impedance gains for "joint_states" mode
    auto_declare<std::vector<double>>("k_gains",
                                      std::vector<double>(num_joints, 0.0));
    auto_declare<std::vector<double>>("d_gains",
                                      std::vector<double>(num_joints, 0.0));

    // Always-publish desired joint/cartesian topics (no booleans)
    auto_declare<std::string>("desired_q_topic",
                              "/joint_impedance/joints_desired");
    auto_declare<std::string>("desired_xd_topic",
                              "/cartesian_impedance/pose_desired");
    auto_declare<std::string>("ee_frame", "panda_link8");  // or "tool"
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception during on_init: %s\n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

HapticGravityCompensationController::CallbackReturn
HapticGravityCompensationController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Load params
  arm_id_               = get_node()->get_parameter("arm_id").as_string();
  follower_input_mode_  = get_node()->get_parameter("follower_input_mode").as_string();
  follower_torque_topic_= get_node()->get_parameter("follower_torque_topic").as_string();
  joint_state_topic_    = get_node()->get_parameter("joint_state_topic").as_string();
  lpf_alpha_            = get_node()->get_parameter("lpf_alpha").as_double();

  desired_q_topic_      = get_node()->get_parameter("desired_q_topic").as_string();
  desired_xd_topic_     = get_node()->get_parameter("desired_xd_topic").as_string();
  ee_frame_             = get_node()->get_parameter("ee_frame").as_string();

  if (lpf_alpha_ < 0.0 || lpf_alpha_ > 1.0) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "lpf_alpha must be in [0,1], got %f", lpf_alpha_);
    return CallbackReturn::FAILURE;
  }

  // Clamp vector
  auto clamp_vec = get_node()->get_parameter("torque_clamp").as_double_array();
  if (clamp_vec.size() != static_cast<size_t>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "torque_clamp must have %d elements, got %zu",
                 num_joints, clamp_vec.size());
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < num_joints; ++i) {
    torque_clamp_(i) = std::abs(clamp_vec[i]);
  }

  // Impedance gains
  auto k_vec = get_node()->get_parameter("k_gains").as_double_array();
  auto d_vec = get_node()->get_parameter("d_gains").as_double_array();
  if (k_vec.size() != static_cast<size_t>(num_joints) ||
      d_vec.size() != static_cast<size_t>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "k_gains and d_gains must have %d elements.", num_joints);
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < num_joints; ++i) {
    k_gains_(i) = k_vec[i];
    d_gains_(i) = d_vec[i];
  }

  // Subscribers (overlay inputs)
  sub_follower_tau_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      follower_torque_topic_, rclcpp::QoS(10),
      std::bind(&HapticGravityCompensationController::followerTorqueCallback,
                this, std::placeholders::_1));

  sub_joint_state_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::QoS(50),
      std::bind(&HapticGravityCompensationController::jointStateCallback,
                this, std::placeholders::_1));

  // Publishers (pre-size messages; avoid allocs in update)
  {
    auto pub_q = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
        desired_q_topic_, rclcpp::QoS(10));
    rt_pub_qd_ = std::make_shared<
        realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(pub_q);
    rt_pub_qd_->msg_.data.resize(num_joints);
  }
  {
    auto pub_x = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
        desired_xd_topic_, rclcpp::QoS(1));
    rt_pub_xd_ = std::make_shared<
        realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(pub_x);
    rt_pub_xd_->msg_.data.resize(12);  // [px,py,pz,R00..R22] row-major
  }

  // Create robot model (bind interfaces in on_activate)
  try {
    franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
        arm_id_ + "/robot_model", arm_id_);
    model_ready_ = true;  // will be confirmed in on_activate
  } catch (const std::exception& e) {
    model_ready_ = false;
    RCLCPP_WARN(get_node()->get_logger(),
                "FrankaRobotModel creation failed; Cartesian publishing will be disabled. err=%s",
                e.what());
  }

  // Init runtime state
  tau_raw_.setZero();
  tau_filtered_.setZero();
  tau_cmd_.setZero();
  have_first_msg_ = false;

  q_.setZero();
  dq_.setZero();
  q_follower_.setZero();
  dq_follower_.setZero();
  have_follower_state_ = false;

  // Validate mode
  if (follower_input_mode_ != "torque_topic" && follower_input_mode_ != "joint_states") {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Invalid follower_input_mode '%s'. Use 'torque_topic' or 'joint_states'.",
                 follower_input_mode_.c_str());
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_node()->get_logger(),
              "Configured HGC | arm='%s' mode='%s' tau_topic='%s' js_topic='%s' lpf_alpha=%.3f "
              "qd_topic='%s' xd_topic='%s' ee_frame='%s'",
              arm_id_.c_str(), follower_input_mode_.c_str(),
              follower_torque_topic_.c_str(), joint_state_topic_.c_str(), lpf_alpha_,
              desired_q_topic_.c_str(), desired_xd_topic_.c_str(), ee_frame_.c_str());
  return CallbackReturn::SUCCESS;
}

HapticGravityCompensationController::CallbackReturn
HapticGravityCompensationController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Reset efforts to zero baseline
  for (auto& ci : command_interfaces_) {
    ci.set_value(0.0);
  }

  // Bind model interfaces now that state_interfaces_ are loaned
  if (franka_robot_model_) {
    try {
      franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
      model_ready_ = true;
    } catch (const std::exception& e) {
      model_ready_ = false;
      RCLCPP_WARN(get_node()->get_logger(),
                  "assign_loaned_state_interfaces failed: %s", e.what());
    }
  }
  return CallbackReturn::SUCCESS;
}

HapticGravityCompensationController::CallbackReturn
HapticGravityCompensationController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (franka_robot_model_) {
    try {
      franka_robot_model_->release_interfaces();
    } catch (...) {}
  }
  return CallbackReturn::SUCCESS;
}

// ----------- Subscribers -----------

void HapticGravityCompensationController::followerTorqueCallback(
    const std_msgs::msg::Float64MultiArray& msg) {
  if (follower_input_mode_ != "torque_topic") return;
  if (msg.data.size() < static_cast<size_t>(num_joints)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                         "Follower tau length %zu < %d; ignoring.", msg.data.size(), num_joints);
    return;
  }
  std::lock_guard<std::mutex> lock(data_mutex_);
  for (int i = 0; i < num_joints; ++i) {
    tau_raw_(i) = static_cast<double>(msg.data[i]);
  }
  if (!have_first_msg_) {
    tau_filtered_ = tau_raw_;
    have_first_msg_ = true;
  }
}

void HapticGravityCompensationController::jointStateCallback(
    const sensor_msgs::msg::JointState& msg) {
  if (follower_input_mode_ != "joint_states") return;

  if (msg.position.size() < static_cast<size_t>(num_joints) ||
      msg.velocity.size() < static_cast<size_t>(num_joints)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "JointState pos/vel length too small (pos=%zu vel=%zu), need %d; ignoring.",
        msg.position.size(), msg.velocity.size(), num_joints);
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  for (int i = 0; i < num_joints; ++i) {
    q_follower_(i)  = static_cast<double>(msg.position[i]);
    dq_follower_(i) = static_cast<double>(msg.velocity[i]);
  }
  have_follower_state_ = true;
}

// ----------- Helpers -----------

void HapticGravityCompensationController::readLeaderJointStates() {
  // We requested position then velocity for each joint
  for (int i = 0; i < num_joints; ++i) {
    const auto& pos_iface = state_interfaces_.at(2 * i);
    const auto& vel_iface = state_interfaces_.at(2 * i + 1);
    q_(i)  = pos_iface.get_value();
    dq_(i) = vel_iface.get_value();
  }
}

bool HapticGravityCompensationController::fill_cartesian_row_major(
    std_msgs::msg::Float64MultiArray& m) {
  if (!model_ready_ || !franka_robot_model_) return false;

  // Pose of the end-effector in base (4x4)
  Eigen::Map<const Eigen::Matrix4d> T(
      franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data());
  const Eigen::Vector3d p = T.block<3,1>(0,3);
  const Eigen::Matrix3d R = T.block<3,3>(0,0);

  m.data[0] = p.x(); m.data[1] = p.y(); m.data[2] = p.z();
  m.data[3] = R(0,0); m.data[4] = R(0,1); m.data[5] = R(0,2);
  m.data[6] = R(1,0); m.data[7] = R(1,1); m.data[8] = R(1,2);
  m.data[9] = R(2,0); m.data[10]= R(2,1); m.data[11]= R(2,2);
  return true;
}

// ----------- Runtime -----------

controller_interface::return_type
HapticGravityCompensationController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {

  // 1) Read leader states
  readLeaderJointStates();

  // 2) Determine tau_raw_ source
  if (follower_input_mode_ == "joint_states") {
    // Impedance overlay from follower state vs leader state
    Vector7d tau_imp = Vector7d::Zero();

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (have_follower_state_) {
        const Vector7d q_err  = q_  - q_follower_;
        const Vector7d dq_err = dq_ - dq_follower_;
        tau_imp = k_gains_.cwiseProduct(q_err) + d_gains_.cwiseProduct(dq_err);
        printf("q_err: %.4f %.4f %.4f %.4f %.4f %.4f %.4f | "
               "dq_err: %.4f %.4f %.4f %.4f %.4f %.4f %.4f | "
               "tau_imp: %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",
               q_err(0), q_err(1), q_err(2), q_err(3), q_err(4), q_err(5), q_err(6),
               dq_err(0), dq_err(1), dq_err(2), dq_err(3), dq_err(4), dq_err(5), dq_err(6),
               tau_imp(0), tau_imp(1), tau_imp(2), tau_imp(3), tau_imp(4), tau_imp(5), tau_imp(6));
        tau_raw_ = tau_imp;
        if (!have_first_msg_) {
          tau_filtered_ = tau_raw_;
          have_first_msg_ = true;
        }
      }
    }
  }
  // else: "torque_topic" mode already updates tau_raw_ in its callback

  // 3) LPF + clamp (common path)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    tau_filtered_ = lpf_alpha_ * tau_filtered_ + (1.0 - lpf_alpha_) * tau_raw_;
    for (int i = 0; i < num_joints; ++i) {
      const double lim = torque_clamp_(i);
      tau_cmd_(i) = 0.5 * clamp(tau_filtered_(i), -lim, +lim);
    }
  }
  printf("tau_cmd: %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",
         tau_cmd_(0), tau_cmd_(1), tau_cmd_(2), tau_cmd_(3),
         tau_cmd_(4), tau_cmd_(5), tau_cmd_(6));

  // 4) Send effort commands (overlay on pure GC baseline == 0)
  // for (int i = 0; i < num_joints; ++i) {
  //   command_interfaces_[i].set_value(tau_cmd_(i));
  // }

  // 5) Publish desired joints (leader q)
  if (rt_pub_qd_ && rt_pub_qd_->trylock()) {
    auto& msg = rt_pub_qd_->msg_;
    for (int i = 0; i < num_joints; ++i) msg.data[i] = q_(i);
    rt_pub_qd_->unlockAndPublish();
  }

  // 6) Publish desired Cartesian pose [px py pz R00..R22] row-major
  if (rt_pub_xd_ && rt_pub_xd_->trylock()) {
    if (fill_cartesian_row_major(rt_pub_xd_->msg_)) {
      rt_pub_xd_->unlockAndPublish();
    } else {
      rt_pub_xd_->unlock();  // model not ready; skip
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace teleop_controllers

// Export plugin
PLUGINLIB_EXPORT_CLASS(teleop_controllers::HapticGravityCompensationController,
                       controller_interface::ControllerInterface)
