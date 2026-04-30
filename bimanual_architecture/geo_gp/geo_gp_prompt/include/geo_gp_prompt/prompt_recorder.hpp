#pragma once

#include <chrono>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <franka_msgs/msg/franka_state.hpp>
#include <geo_gp_interfaces/msg/prompt_trajectory.hpp>
#include <std_msgs/msg/bool.hpp>


class PromptRecorder : public rclcpp::Node {
public:
    PromptRecorder();

private:
    void pose_callback(const franka_msgs::msg::FrankaState::SharedPtr msg);
    void execution_running_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void blend_running_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void enabled_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void reset_recording_state();
    void request_rearm();
    bool publish_paused() const;
    bool can_publish_online() const;
    void publish_prompt();

    // ROS
    rclcpp::Subscription<franka_msgs::msg::FrankaState>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr execution_running_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr blend_running_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enabled_sub_;
    rclcpp::Publisher<geo_gp_interfaces::msg::PromptTrajectory>::SharedPtr prompt_pub_;

    // Data
    std::vector<geometry_msgs::msg::Pose> poses_;
    std::vector<geometry_msgs::msg::Vector3> forces_;
    std::vector<double> time_from_start_;
    rclcpp::Time motion_start_time_;

    //Parameters
    std::string input_topic_;
    std::string output_topic_;
    std::string execution_running_topic_;
    std::string blend_running_topic_;
    std::string enabled_topic_;
    bool execution_running_ = false;
    bool blend_running_ = false;
    bool enabled_ = false;

    int moving_counter_ = 0;
    int stop_counter_ = 0;
    int settle_counter_ = 0;

    int moving_count_threshold_;
    int stop_count_threshold_;

    double start_threshold_;
    double stop_threshold_;
    size_t min_points_ = 10;
    bool online_mode_ = false;
    int publish_period_ms_ = 150;
    size_t publish_min_points_ = 10;
    rclcpp::Time last_online_publish_time_;
    bool waiting_for_settle_ = false;

    // State machine
    enum class State {
        IDLE,
        MOVING,
        STOPPING
    };

    State state_;
};
