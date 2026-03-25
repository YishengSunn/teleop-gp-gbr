#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geo_gp_interfaces/msg/predicted_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>


class TrajectoryExecutor : public rclcpp::Node {
public:
    TrajectoryExecutor();

private:
    void trajectory_callback(const geo_gp_interfaces::msg::PredictedTrajectory::SharedPtr msg);
    void timer_callback();
    void publish_pose(const geometry_msgs::msg::Pose & pose);

    rclcpp::Subscription<geo_gp_interfaces::msg::PredictedTrajectory>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<geometry_msgs::msg::Pose> trajectory_;
    std::vector<double> trajectory_time_;

    std::string input_topic_;
    std::string output_topic_;
    double publish_rate_;
    double hold_time_;

    size_t index_;
    bool executing_;
    rclcpp::Time start_time_;
};
