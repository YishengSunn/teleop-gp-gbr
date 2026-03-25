#include "geo_gp_execution/trajectory_executor.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;


TrajectoryExecutor::TrajectoryExecutor()
: Node("trajectory_executor"),
  index_(0),
  executing_(false) {

    input_topic_ = this->declare_parameter<std::string>(
        "input_topic", "/gp_predicted_trajectory");

    output_topic_ = this->declare_parameter<std::string>(
        "output_topic", "/execution/desired_pose");

    publish_rate_ = this->declare_parameter<double>(
        "rate", 200.0);

    hold_time_ = this->declare_parameter<double>(
        "hold_time", 0.5);

    sub_ = this->create_subscription<geo_gp_interfaces::msg::PredictedTrajectory>(
        input_topic_, 10,
        std::bind(&TrajectoryExecutor::trajectory_callback, this, _1)
    );

    pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        output_topic_, 10
    );

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
        std::bind(&TrajectoryExecutor::timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Trajectory Executor started.");
    RCLCPP_INFO(this->get_logger(), "Listening on: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic_.c_str());
}

void TrajectoryExecutor::trajectory_callback(
    const geo_gp_interfaces::msg::PredictedTrajectory::SharedPtr msg) {

    if (!msg->success) {
        RCLCPP_WARN(this->get_logger(), 
            "Prediction failed, skip execution. skill=%s", 
            msg->skill_name.c_str());
        return;
    }

    if (msg->poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty predicted trajectory.");
        return;
    }

    trajectory_ = msg->poses;
    trajectory_time_ = msg->time_from_start;

    if (trajectory_time_.size() != trajectory_.size()) {
        trajectory_time_.clear();

        const double dt = 1.0 / publish_rate_;
        trajectory_time_.reserve(trajectory_.size());

        for (size_t i = 0; i < trajectory_.size(); ++i) {
            trajectory_time_.push_back(i * dt);
        }

        RCLCPP_WARN(this->get_logger(),
            "PredictedTrajectory.time_from_start size mismatch, fallback to fixed dt=%.6f s",
            dt);
    }
    index_ = 0;
    executing_ = true;
    start_time_ = this->now();

    const double planned_traj_time = trajectory_time_.empty() ? 0.0 : trajectory_time_.back();
    const double planned_total_time = hold_time_ + planned_traj_time;
    RCLCPP_INFO(this->get_logger(),
        "Received predicted trajectory with %zu poses | planned_traj=%.3f s | hold=%.3f s | planned_total=%.3f s",
        trajectory_.size(),
        planned_traj_time,
        hold_time_,
        planned_total_time);
}

void TrajectoryExecutor::timer_callback() {
    if (!executing_ || trajectory_.empty()) {
        return;
    }

    double elapsed = (this->now() - start_time_).seconds();

    if (elapsed < hold_time_) {
        publish_pose(trajectory_.front());
        return;
    }

    const double exec_elapsed = elapsed - hold_time_;

    while (index_ + 1 < trajectory_time_.size() &&
           trajectory_time_[index_ + 1] <= exec_elapsed) {
        ++index_;
    }

    if (exec_elapsed >= trajectory_time_.back()) {
        publish_pose(trajectory_.back());
        executing_ = false;
        const double total_elapsed = elapsed;
        RCLCPP_INFO(this->get_logger(),
            "Trajectory execution finished | actual_total=%.3f s | actual_traj=%.3f s | hold=%.3f s",
            total_elapsed,
            exec_elapsed,
            hold_time_);
        return;
    }

    publish_pose(trajectory_[index_]);
    
    if (index_ + 1 < trajectory_.size() &&
        trajectory_time_[index_ + 1] <= exec_elapsed) {
        ++index_;
    }
}

void TrajectoryExecutor::publish_pose(const geometry_msgs::msg::Pose & pose) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    };
    pub_->publish(msg);
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryExecutor>());
    rclcpp::shutdown();
    return 0;
}
