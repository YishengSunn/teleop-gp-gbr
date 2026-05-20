#include "geo_gp_controllers/subscriber/joint_impedance_controller.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <exception>
#include <string>

#include <Eigen/Eigen>
#include <franka/rate_limiting.h>

namespace geo_gp_controllers {

namespace {

inline double safePeriodSeconds(const rclcpp::Duration& period) {
  const double dt = period.seconds();
  if (!std::isfinite(dt) || dt <= 0.0) {
    return 0.001;
  }
  return dt;
}

}  // namespace

controller_interface::InterfaceConfiguration
JointImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(command_arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(command_arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(command_arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }

  if (franka_robot_model_) {
    for (const auto& n : franka_robot_model_->get_state_interface_names()) {
      config.names.push_back(n);
    }
  }

  if (franka_robot_state_) {
    for (const auto& n : franka_robot_state_->get_state_interface_names()) {
      config.names.push_back(n);
    }
  } else if (TDPA_active_) {
    franka_semantic_components::FrankaRobotState tmp_state(
        command_arm_id_ + "/robot_state", command_arm_id_);
    for (const auto& n : tmp_state.get_state_interface_names()) {
      config.names.push_back(n);
    }
  }

  return config;
}

controller_interface::return_type
JointImpedanceController::update(const rclcpp::Time& /*time*/,
                                 const rclcpp::Duration& period) {
  updateJointStates();

  if (!TDPA_active_) {
    bool skip_buffer_read_this_cycle = false;

    const auto mode_now = static_cast<Mode>(mode_.load(std::memory_order_relaxed));
    if (mode_now == Mode::MOVE_TO_START) {
      const auto t = this->get_node()->now() - start_time_;
      const auto out = motion_generator_->getDesiredJointPositions(t);

      const Vector7d q_desired = out.first;
      const bool finished = out.second;

      if (!finished) {
        const double kAlpha = 0.99;
        dq_filtered_ = (1.0 - kAlpha) * dq_filtered_ + kAlpha * dq_;

        const Vector7d tau =
            start_k_gains_.cwiseProduct(q_desired - q_) +
            start_d_gains_.cwiseProduct(-dq_filtered_);

        for (int i = 0; i < num_joints; ++i) {
          command_interfaces_[i].set_value(tau(i));
        }
        return controller_interface::return_type::OK;
      }

      q_d_rt_ = q_;

      JointsArray cur{};
      for (int i = 0; i < num_joints; ++i) {
        cur[static_cast<size_t>(i)] = q_(i);
      }
      qd_buffer_.writeFromNonRT(cur);

      last_desired_seq_ = desired_seq_.load(std::memory_order_acquire);

      accept_desired_.store(true, std::memory_order_release);
      mode_.store(static_cast<uint8_t>(Mode::IMPEDANCE), std::memory_order_release);

      skip_buffer_read_this_cycle = true;
    }

    if (!skip_buffer_read_this_cycle) {
      const uint64_t seq = desired_seq_.load(std::memory_order_acquire);
      if (seq != last_desired_seq_) {
        const auto* qd_ptr = qd_buffer_.readFromRT();
        if (qd_ptr != nullptr) {
          for (int i = 0; i < num_joints; ++i) {
            q_d_rt_(i) = (*qd_ptr)[static_cast<size_t>(i)];
          }
        }
        last_desired_seq_ = seq;
      }
    }

    Eigen::Map<const Vector7d> coriolis(franka_robot_model_->getCoriolisForceVector().data());

    const double kAlpha = 0.99;
    dq_filtered_ = (1.0 - kAlpha) * dq_filtered_ + kAlpha * dq_;

    const Vector7d tau =
        k_gains_.cwiseProduct(q_d_rt_ - q_) +
        d_gains_.cwiseProduct(-dq_filtered_) +
        coriolis;

    for (int i = 0; i < num_joints; ++i) {
      command_interfaces_[i].set_value(tau(i));
    }

    return controller_interface::return_type::OK;
  }

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

      Vector7d tau =
          start_k_gains_.cwiseProduct(q_desired_move - q_) +
          start_d_gains_.cwiseProduct(-dq_filtered_);

      for (int i = 0; i < num_joints; ++i) {
        command_interfaces_[i].set_value(tau(i));
      }
      return controller_interface::return_type::OK;
    }

    q_remote_delta_intgl_.setZero();
    position_error_.setZero();
    tau_c_.setZero();

    JointsArray cur{};
    cur.fill(0.0);
    qd_cmd_buffer_.writeFromNonRT(cur);

    last_desired_seq_ = desired_seq_.load(std::memory_order_acquire);

    accept_desired_.store(true, std::memory_order_release);
    mode_.store(static_cast<uint8_t>(Mode::IMPEDANCE), std::memory_order_release);

    pandatime_ = 0.0;
    remote_pandatime_ = 0.0;

    tdpaReset_();
    skip_buffer_read_this_cycle = true;
  }

  const double dt = safePeriodSeconds(period);
  pandatime_ += dt;

  if (!skip_buffer_read_this_cycle) {
    const uint64_t seq = desired_seq_.load(std::memory_order_acquire);
    if (seq != last_desired_seq_) {
      const auto* qd_ptr = qd_cmd_buffer_.readFromRT();
      if (qd_ptr != nullptr) {
        for (int i = 0; i < num_joints; ++i) {
          dq_remote_(i) = (*qd_ptr)[static_cast<std::size_t>(i)];
        }
      }
      last_desired_seq_ = seq;
    }
  }

  const auto* robot_state = franka_robot_state_->get_robot_state_ptr();

  for (int i = 0; i < num_joints; ++i) {
    tau_ext_(i) = -robot_state->tau_ext_hat_filtered[static_cast<std::size_t>(i)];
  }

  dq_local_ = dq_;

  for (int i = 0; i < num_joints; ++i) {
    q_local_delta_(i) = q_(i) - q_start_(i);
  }

  for (int i = 0; i < num_joints; ++i) {
    dq_remote_(i) = dq_remote_(i) - K_drift_ * position_error_(i);
  }

  double dq_remote_double[7]{};
  double tau_c_old_double[7]{};
  double dq_des_double[7]{};

  const double gain_tau_f = tau_ext_feedback_ ? 1.0 : -1.0;

  for (int i = 0; i < num_joints; ++i) {
    dq_remote_double[i] = dq_remote_(i);
    tau_c_old_double[i] = tau_ext_feedback_ ? tau_ext_(i) : last_tau_cmd_(i);
    tau_c_old_double[i] = gain_tau_f * tau_c_old_double[i];
  }

  followerPC_.energyObserver(dq_remote_double, tau_c_old_double, dt);

  std::memcpy(dq_des_double, dq_remote_double, 7 * sizeof(double));

  const Matrix7d damping = d_gains_.asDiagonal();
  const Matrix7d friction = Matrix7d::Zero();

  const double dissipation = dq_.transpose() * (damping + friction) * dq_;
  shortage_ += dt * eta_ * dissipation;

  followerPC_.energyController(
      dq_des_double,
      tau_c_old_double,
      E_L_in_delayed_ + shortage_,
      dt);

  for (int i = 0; i < num_joints; ++i) {
    q_remote_delta_intgl_(i) += dt * dq_des_double[i];
    q_des_(i) = q_start_(i) + q_remote_delta_intgl_(i);
    position_error_(i) = q_remote_delta_intgl_(i) - q_remote_delta_(i);
  }

  for (int i = 0; i < num_joints; ++i) {
    tau_c_(i) =
        k_gains_(i) * (q_des_(i) - q_(i)) +
        0.5 * d_gains_(i) * (dq_des_double[i] - dq_local_(i));

    tau_c_no_mod_dq_(i) =
        k_gains_(i) * (q_des_(i) - q_(i)) +
        0.5 * d_gains_(i) * (dq_remote_double[i] - dq_local_(i));

    tau_diff_(i) = tau_c_no_mod_dq_(i) - tau_c_(i);
  }

  beta_ = followerPC_.getBeta();
  E_F_in_ = followerPC_.getInputEnergyFlow();
  E_F_out_ = followerPC_.getOutputEnergyFlow();
  E_F_diss_ = followerPC_.getDissipatedEnergyFlow();

  std::array<double, 7> tau_des{};
  std::array<double, 7> tau_prev{};

  for (int i = 0; i < num_joints; ++i) {
    tau_des[static_cast<std::size_t>(i)] = tau_c_(i);
    tau_prev[static_cast<std::size_t>(i)] =
        last_tau_cmd_initialized_ ? last_tau_cmd_(i) : tau_c_(i);
  }

  const auto tau_limited =
      franka::limitRate(franka::kMaxTorqueRate, tau_des, tau_prev);

  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_limited[static_cast<std::size_t>(i)]);
    last_tau_cmd_(i) = tau_limited[static_cast<std::size_t>(i)];
  }

  last_tau_cmd_initialized_ = true;

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

    for (int i = 0; i < num_joints; ++i) {
      msg.q_local_delta[static_cast<std::size_t>(i)] = q_local_delta_(i);
      msg.dq_local[static_cast<std::size_t>(i)] = dq_local_(i);
      msg.tau_local[static_cast<std::size_t>(i)] =
          tau_ext_feedback_ ? tau_ext_(i) : tau_limited[static_cast<std::size_t>(i)];
    }

    msg.energy = E_F_in_;
    local_state_pub_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

CallbackReturn
JointImpedanceController::on_init() {
  try {
    auto_declare<std::string>("command_arm_id", "panda");
    auto_declare<std::string>("source_arm_id", "panda");

    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});

    auto_declare<std::string>("joint_impedance_topic", "joint_impedance/joints_desired");

    auto_declare<bool>("move_to_start", false);
    auto_declare<std::vector<double>>(
        "start_joint_configuration",
        {0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4});

    auto_declare<std::vector<double>>("start_k_gains", {});
    auto_declare<std::vector<double>>("start_d_gains", {});

    auto_declare<bool>("TDPA_active", false);
    auto_declare<bool>("tau_ext_feedback", false);
    auto_declare<double>("k_pos_drift", 0.0);
    auto_declare<double>("eta_passivity_shortage", 0.0);

    auto_declare<std::string>("remote_state_topic", "leader/tdpa_joint_state");
    auto_declare<std::string>("local_state_topic", "follower/tdpa_joint_state");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn
JointImpedanceController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  command_arm_id_ = get_node()->get_parameter("command_arm_id").as_string();
  source_arm_id_ = get_node()->get_parameter("source_arm_id").as_string();

  move_to_start_ = get_node()->get_parameter("move_to_start").as_bool();
  TDPA_active_ = get_node()->get_parameter("TDPA_active").as_bool();
  tau_ext_feedback_ = get_node()->get_parameter("tau_ext_feedback").as_bool();
  K_drift_ = get_node()->get_parameter("k_pos_drift").as_double();
  eta_ = get_node()->get_parameter("eta_passivity_shortage").as_double();

  remote_state_topic_ = get_node()->get_parameter("remote_state_topic").as_string();
  local_state_topic_ = get_node()->get_parameter("local_state_topic").as_string();

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(
          command_arm_id_ + "/robot_model",
          command_arm_id_));

  if (TDPA_active_) {
    franka_robot_state_ = std::make_unique<franka_semantic_components::FrankaRobotState>(
        franka_semantic_components::FrankaRobotState(
            command_arm_id_ + "/robot_state",
            command_arm_id_));
  }

  const auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  const auto d_gains = get_node()->get_parameter("d_gains").as_double_array();

  if (k_gains.size() != static_cast<size_t>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains must have size %d (got %ld)",
                 num_joints, k_gains.size());
    return CallbackReturn::FAILURE;
  }
  if (d_gains.size() != static_cast<size_t>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains must have size %d (got %ld)",
                 num_joints, d_gains.size());
    return CallbackReturn::FAILURE;
  }

  for (int i = 0; i < num_joints; ++i) {
    k_gains_(i) = k_gains.at(i);
    d_gains_(i) = d_gains.at(i);
  }

  const auto start_q = get_node()->get_parameter("start_joint_configuration").as_double_array();
  if (start_q.size() != static_cast<size_t>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "start_joint_configuration must have size %d (got %ld)",
                 num_joints, start_q.size());
    return CallbackReturn::FAILURE;
  }
  q_start_ = Eigen::Map<const Vector7d>(start_q.data());

  if (move_to_start_) {
    const auto start_k = get_node()->get_parameter("start_k_gains").as_double_array();
    const auto start_d = get_node()->get_parameter("start_d_gains").as_double_array();

    if (start_k.size() != static_cast<size_t>(num_joints) ||
        start_d.size() != static_cast<size_t>(num_joints)) {
      RCLCPP_FATAL(get_node()->get_logger(),
                   "start_k_gains and start_d_gains must have size %d when move_to_start=true",
                   num_joints);
      return CallbackReturn::FAILURE;
    }

    for (int i = 0; i < num_joints; ++i) {
      start_k_gains_(i) = start_k.at(i);
      start_d_gains_(i) = start_d.at(i);
    }
  }

  expected_joint_names_.clear();
  expected_name_to_index_.clear();
  expected_joint_names_.reserve(num_joints);

  for (int i = 1; i <= num_joints; ++i) {
    const std::string jn = source_arm_id_ + "_joint" + std::to_string(i);
    expected_joint_names_.push_back(jn);
    expected_name_to_index_.emplace(jn, static_cast<size_t>(i - 1));
  }

  warned_bad_jointstate_ = false;
  dq_filtered_.setZero();

  JointsArray init{};
  init.fill(0.0);
  qd_buffer_.writeFromNonRT(init);
  qd_cmd_buffer_.writeFromNonRT(init);

  sub_desired_joint_.reset();
  if (!TDPA_active_) {
    sub_desired_joint_ =
        get_node()->create_subscription<sensor_msgs::msg::JointState>(
            get_node()->get_parameter("joint_impedance_topic").as_string(),
            rclcpp::QoS(1),
            std::bind(&JointImpedanceController::desiredJointCallback, this,
                      std::placeholders::_1));
  } else {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

    remote_state_sub_ =
        get_node()->create_subscription<geo_gp_interfaces::msg::TDPAJointState>(
            remote_state_topic_, qos,
            std::bind(&JointImpedanceController::remoteStateCallback, this,
                      std::placeholders::_1));

    local_state_pub_raw_ =
        get_node()->create_publisher<geo_gp_interfaces::msg::TDPAJointState>(
            local_state_topic_, qos);
    local_state_pub_ =
        std::make_unique<
            realtime_tools::RealtimePublisher<geo_gp_interfaces::msg::TDPAJointState>>(
            local_state_pub_raw_);

    tdpaReset_();
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn
JointImpedanceController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  accept_desired_.store(false, std::memory_order_release);

  updateJointStates();
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  if (TDPA_active_) {
    franka_robot_state_->assign_loaned_state_interfaces(state_interfaces_);
  }

  q_d_rt_ = q_;
  dq_filtered_.setZero();

  JointsArray init{};
  for (int i = 0; i < num_joints; ++i) {
    init[static_cast<size_t>(i)] = q_(i);
  }
  qd_buffer_.writeFromNonRT(init);
  qd_cmd_buffer_.writeFromNonRT(init);

  last_desired_seq_ = desired_seq_.load(std::memory_order_acquire);

  if (TDPA_active_) {
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
    motion_generator_ = std::make_unique<MotionGenerator>(0.2, q_, q_start_);
    start_time_ = this->get_node()->now();
    mode_.store(static_cast<uint8_t>(Mode::MOVE_TO_START), std::memory_order_release);
  } else {
    mode_.store(static_cast<uint8_t>(Mode::IMPEDANCE), std::memory_order_release);
    accept_desired_.store(true, std::memory_order_release);
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn
JointImpedanceController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->release_interfaces();
  if (franka_robot_state_) {
    franka_robot_state_->release_interfaces();
  }
  return CallbackReturn::SUCCESS;
}

void JointImpedanceController::updateJointStates() {
  for (int i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

void JointImpedanceController::desiredJointCallback(const sensor_msgs::msg::JointState& msg) {
  if (!accept_desired_.load(std::memory_order_relaxed)) {
    return;
  }

  if (msg.position.empty()) {
    if (!warned_bad_jointstate_) {
      RCLCPP_WARN(get_node()->get_logger(), "Desired JointState has empty position[]. Ignoring.");
      warned_bad_jointstate_ = true;
    }
    return;
  }

  JointsArray desired{};
  const auto* cur = qd_buffer_.readFromRT();
  if (cur != nullptr) {
    desired = *cur;
  } else {
    desired.fill(0.0);
  }

  if (!msg.name.empty()) {
    const size_t n = std::min(msg.name.size(), msg.position.size());
    for (size_t j = 0; j < n; ++j) {
      auto it = expected_name_to_index_.find(msg.name[j]);
      if (it == expected_name_to_index_.end()) {
        continue;
      }
      const size_t idx = it->second;
      if (idx < static_cast<size_t>(num_joints)) {
        desired[idx] = msg.position[j];
      }
    }
    qd_buffer_.writeFromNonRT(desired);
    desired_seq_.fetch_add(1, std::memory_order_release);
    return;
  }

  const size_t n = std::min(static_cast<size_t>(num_joints), msg.position.size());
  for (size_t i = 0; i < n; ++i) {
    desired[i] = msg.position[i];
  }

  qd_buffer_.writeFromNonRT(desired);
  desired_seq_.fetch_add(1, std::memory_order_release);
}

void JointImpedanceController::remoteStateCallback(
    const geo_gp_interfaces::msg::TDPAJointState& msg) {
  if (!accept_desired_.load(std::memory_order_relaxed) &&
      static_cast<Mode>(mode_.load(std::memory_order_relaxed)) != Mode::IMPEDANCE) {
    return;
  }

  remote_pandatime_ = msg.pandatime;

  last_received_remote_seq_ = msg.seq;
  last_received_remote_tx_time_ns_ = msg.tx_time_ns;

  JointsArray dq_remote_arr{};
  dq_remote_arr.fill(0.0);

  for (int i = 0; i < num_joints; ++i) {
    q_remote_delta_(i) = msg.q_local_delta[static_cast<std::size_t>(i)];
    dq_remote_arr[static_cast<std::size_t>(i)] = msg.dq_local[static_cast<std::size_t>(i)];
    tau_remote_(i) = msg.tau_local[static_cast<std::size_t>(i)];
  }

  qd_cmd_buffer_.writeFromNonRT(dq_remote_arr);
  desired_seq_.fetch_add(1, std::memory_order_release);

  if (std::isfinite(msg.energy)) {
    E_L_in_delayed_ = msg.energy;
  }
}

}  // namespace geo_gp_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(geo_gp_controllers::JointImpedanceController,
                       controller_interface::ControllerInterface)
