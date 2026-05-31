#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <franka_msgs/msg/franka_state.hpp>
#include <geo_gp_interfaces/msg/predicted_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>


struct LeaderSample {
    double time{0.0};
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Vector3 force;
};


class TrajectoryExecutor : public rclcpp::Node {
public:
    TrajectoryExecutor();

private:
    void trajectory_callback(const geo_gp_interfaces::msg::PredictedTrajectory::SharedPtr msg);
    void leader_state_callback(const franka_msgs::msg::FrankaState::SharedPtr msg);
    void timer_callback();
    void publish_pose(const geometry_msgs::msg::Pose & pose);
    void publish_force(const geometry_msgs::msg::Vector3 & force);
    double select_force_axis(const geometry_msgs::msg::Vector3 & force) const;
    bool can_extend_trajectory_in_place(
        const geo_gp_interfaces::msg::PredictedTrajectory::SharedPtr msg) const;
    void begin_leader_recording(const std::string & skill_name);
    void finalize_leader_recording();
    void save_leader_trajectory_csv();
    static std::string format_execution_timestamp(const rclcpp::Time & stamp);

    rclcpp::Subscription<geo_gp_interfaces::msg::PredictedTrajectory>::SharedPtr sub_;
    rclcpp::Subscription<franka_msgs::msg::FrankaState>::SharedPtr leader_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr force_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_running_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<geometry_msgs::msg::Pose> trajectory_;
    std::vector<geometry_msgs::msg::Vector3> force_trajectory_;
    std::vector<double> trajectory_time_;
    std::vector<LeaderSample> leader_samples_;

    std::string input_topic_;
    std::string output_topic_;
    std::string force_output_topic_;
    std::string force_axis_;
    std::string running_topic_;
    std::string leader_input_topic_;
    std::string csv_output_dir_;
    std::string recording_skill_name_;
    std::string recording_timestamp_;
    double publish_rate_;
    bool progressive_update_enabled_;
    double progressive_match_pos_eps_;
    double progressive_match_time_eps_;
    bool save_leader_csv_;

    size_t index_;
    bool executing_;
    bool recording_leader_;
    rclcpp::Time start_time_;
};
