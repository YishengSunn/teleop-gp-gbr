#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <franka_msgs/msg/franka_state.hpp>
#include <geo_gp_interfaces/msg/prompt_trajectory.hpp>
#include <std_msgs/msg/bool.hpp>

#include <Eigen/Dense>


class PromptRecorder : public rclcpp::Node {
public:
    PromptRecorder();

private:
    void pose_callback(const franka_msgs::msg::FrankaState::SharedPtr msg);
    void execution_running_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void blend_running_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void publish_prompt();

    // ROS
    rclcpp::Subscription<franka_msgs::msg::FrankaState>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr execution_running_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr blend_running_sub_;
    rclcpp::Publisher<geo_gp_interfaces::msg::PromptTrajectory>::SharedPtr prompt_pub_;

    // Data
    std::vector<geometry_msgs::msg::Pose> poses_;
    std::vector<double> time_from_start_;
    rclcpp::Time motion_start_time_;

    //Parameters
    std::string input_topic_;
    std::string output_topic_;
    std::string execution_running_topic_;
    std::string blend_running_topic_;
    bool execution_running_ = false;
    bool blend_running_ = false;

    int moving_counter_ = 0;
    int stop_counter_ = 0;

    int moving_count_threshold_;
    int stop_count_threshold_;

    double start_threshold_;
    double stop_threshold_;
    size_t min_points_ = 10;

    // State machine
    enum class State {
        IDLE,
        MOVING,
        STOPPING
    };

    State state_;
};
