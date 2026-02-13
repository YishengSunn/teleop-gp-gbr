#pragma once

#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace garmi_controllers {

/**
 * A controller that commands joint velocities to two arms.
 * The two arms are identified by their unique arm IDs, which are provided as parameters.
 * The controller subscribes to a topic to receive desired joint velocities for both arms.
 */

class DualJointImpedanceController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  struct ArmContainer{
    std::string arm_id_;
    
    Vector7d q_;
    Vector7d initial_q_;
    Vector7d dq_;
    Vector7d dq_filtered_;
    Vector7d k_gains_;
    Vector7d d_gains_;
    Vector7d q_des_;
  };
  
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

  private:
  const int num_joints = 7;
  std::map<std::string, ArmContainer> arms_;
  rclcpp::Time start_time_;
  void updateJointStates();

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_desired_joint_; 
  void desiredJointCallback(const sensor_msgs::msg::JointState& msg);
};

}  // namespace garmi_controllers