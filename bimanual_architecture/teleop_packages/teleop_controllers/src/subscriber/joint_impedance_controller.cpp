#include <teleop_controllers/subscriber/joint_impedance_controller.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace teleop_controllers {

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

  // Can be called before on_configure() depending on controller_manager ordering.
  if (franka_robot_model_) {
    for (const auto& n : franka_robot_model_->get_state_interface_names()) {
      config.names.push_back(n);
    }
  }

  return config;
}

controller_interface::return_type
JointImpedanceController::update(const rclcpp::Time& /*time*/,
                                 const rclcpp::Duration& /*period*/) {
  updateJointStates();

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

    // Move finished. Switch to impedance but do NOT pull stale desired from qd_buffer_.
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

  // Impedance mode: only apply new desired if callback delivered something new.
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

CallbackReturn
JointImpedanceController::on_init() {
  try {
    auto_declare<std::string>("command_arm_id", "panda");
    auto_declare<std::string>("source_arm_id", "panda");

    // Gains for IMPEDANCE mode
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});

    auto_declare<std::string>("joint_impedance_topic", "joint_impedance/joints_desired");

    auto_declare<bool>("move_to_start", false);
    auto_declare<std::vector<double>>(
        "start_joint_configuration",
        {0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4});

    // Gains for MOVE_TO_START PD
    auto_declare<std::vector<double>>("start_k_gains", {});
    auto_declare<std::vector<double>>("start_d_gains", {});

    sub_desired_joint_ =
        get_node()->create_subscription<sensor_msgs::msg::JointState>(
            get_node()->get_parameter("joint_impedance_topic").as_string(),
            rclcpp::QoS(1),
            std::bind(&JointImpedanceController::desiredJointCallback, this,
                      std::placeholders::_1));
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn
JointImpedanceController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  command_arm_id_ = get_node()->get_parameter("command_arm_id").as_string();
  source_arm_id_  = get_node()->get_parameter("source_arm_id").as_string();

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(
          command_arm_id_ + "/robot_model",
          command_arm_id_));

  // --- IMPEDANCE gains ---
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

  // --- MOVE_TO_START config ---
  move_to_start_ = get_node()->get_parameter("move_to_start").as_bool();

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

  // Expected incoming name mapping based on SOURCE robot
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

  // Seed buffer with zeros; real seeding happens in on_activate.
  JointsArray init{};
  init.fill(0.0);
  qd_buffer_.writeFromNonRT(init);

  return CallbackReturn::SUCCESS;
}

CallbackReturn
JointImpedanceController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  // Block desired updates while we initialize.
  accept_desired_.store(false, std::memory_order_release);

  updateJointStates();
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  q_d_rt_ = q_;
  dq_filtered_.setZero();

  // Seed buffer with current q
  JointsArray init{};
  for (int i = 0; i < num_joints; ++i) {
    init[static_cast<size_t>(i)] = q_(i);
  }
  qd_buffer_.writeFromNonRT(init);

  // Treat existing buffer as already applied until a new message arrives.
  last_desired_seq_ = desired_seq_.load(std::memory_order_acquire);

  if (move_to_start_) {
    motion_generator_ = std::make_unique<MotionGenerator>(0.2, q_, q_start_);
    start_time_ = this->get_node()->now();
    mode_.store(static_cast<uint8_t>(Mode::MOVE_TO_START), std::memory_order_release);
    // accept_desired stays false until finished
  } else {
    mode_.store(static_cast<uint8_t>(Mode::IMPEDANCE), std::memory_order_release);
    accept_desired_.store(true, std::memory_order_release);
  }

  return CallbackReturn::SUCCESS;
}

void JointImpedanceController::updateJointStates() {
  for (int i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i)  = position_interface.get_value();
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

  // Start from last buffered value so partial updates don't trash other joints.
  JointsArray desired{};
  const auto* cur = qd_buffer_.readFromNonRT();
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

  // No name[]: assume joint1..joint7 ordering
  const size_t n = std::min(static_cast<size_t>(num_joints), msg.position.size());
  for (size_t i = 0; i < n; ++i) {
    desired[i] = msg.position[i];
  }

  qd_buffer_.writeFromNonRT(desired);
  desired_seq_.fetch_add(1, std::memory_order_release);
}

}  // namespace teleop_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(teleop_controllers::JointImpedanceController,
                       controller_interface::ControllerInterface)
