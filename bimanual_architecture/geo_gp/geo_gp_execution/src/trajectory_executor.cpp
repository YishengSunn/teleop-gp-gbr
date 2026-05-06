#include "geo_gp_execution/trajectory_executor.hpp"

#include <cmath>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace {
void publish_running(const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr& pub, bool running) {
  std_msgs::msg::Bool msg;
  msg.data = running;
  pub->publish(msg);
}
}  // namespace

TrajectoryExecutor::TrajectoryExecutor()
: Node("trajectory_executor"),
  index_(0),
  executing_(false) {

    input_topic_ = this->declare_parameter<std::string>(
        "input_topic", "/gp_predicted_trajectory");
    output_topic_ = this->declare_parameter<std::string>(
        "output_topic", "/execution/desired_pose");
    force_output_topic_ = this->declare_parameter<std::string>(
        "force_output_topic", "/execution/desired_force");
    force_axis_ = this->declare_parameter<std::string>(
        "force_axis", "z");
    running_topic_ = this->declare_parameter<std::string>(
        "running_topic", "/execution/running");
    publish_rate_ = this->declare_parameter<double>(
        "rate", 200.0);
    progressive_update_enabled_ = this->declare_parameter<bool>(
        "progressive_update_enabled", true);
    progressive_match_pos_eps_ = this->declare_parameter<double>(
        "progressive_match_pos_eps", 1e-3);
    progressive_match_time_eps_ = this->declare_parameter<double>(
        "progressive_match_time_eps", 1e-3);

    sub_ = this->create_subscription<geo_gp_interfaces::msg::PredictedTrajectory>(
        input_topic_, 10,
        std::bind(&TrajectoryExecutor::trajectory_callback, this, _1)
    );

    pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        output_topic_, 10
    );

    force_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        force_output_topic_, 10
    );

    pub_running_ = this->create_publisher<std_msgs::msg::Bool>(
        running_topic_,
        rclcpp::QoS(1).transient_local());

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
        std::bind(&TrajectoryExecutor::timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Trajectory Executor started.");

    publish_running(pub_running_, false);
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

    const bool extend_in_place =
        progressive_update_enabled_ && can_extend_trajectory_in_place(msg);

    trajectory_ = msg->poses;
    if (msg->forces.size() == msg->poses.size()) {
        force_trajectory_ = msg->forces;
    }
    else {
        force_trajectory_.clear();
        if (!msg->forces.empty()) {
            RCLCPP_WARN(this->get_logger(),
                "PredictedTrajectory.forces size mismatch, skip force execution. poses=%zu forces=%zu",
                msg->poses.size(), msg->forces.size());
        }
    }
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
    if (extend_in_place) {
        RCLCPP_INFO(
            this->get_logger(),
            "Extended running trajectory in-place | points=%zu",
            trajectory_.size());
        return;
    }

    index_ = 0;
    executing_ = true;
    start_time_ = this->now();
    publish_running(pub_running_, true);
}

void TrajectoryExecutor::timer_callback() {
    if (!executing_ || trajectory_.empty()) {
        return;
    }

    double elapsed = (this->now() - start_time_).seconds();

    while (index_ + 1 < trajectory_time_.size() &&
           trajectory_time_[index_ + 1] <= elapsed) {
        ++index_;
    }

    if (elapsed >= trajectory_time_.back()) {
        publish_pose(trajectory_.back());
        if (!force_trajectory_.empty()) {
            publish_force(force_trajectory_.back());
        }
        executing_ = false;
        publish_running(pub_running_, false);
        RCLCPP_INFO(this->get_logger(),
            "Trajectory execution finished | elapsed=%.3f s",
            elapsed);
        return;
    }

    publish_pose(trajectory_[index_]);
    if (!force_trajectory_.empty() && index_ < force_trajectory_.size()) {
        publish_force(force_trajectory_[index_]);
    }

    if (index_ + 1 < trajectory_.size() &&
        trajectory_time_[index_ + 1] <= elapsed) {
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

void TrajectoryExecutor::publish_force(const geometry_msgs::msg::Vector3 & force) {
    std_msgs::msg::Float64 msg;
    msg.data = select_force_axis(force);
    force_pub_->publish(msg);
}

double TrajectoryExecutor::select_force_axis(const geometry_msgs::msg::Vector3 & force) const {
    if (force_axis_ == "x" || force_axis_ == "X") {
        return force.x;
    }
    if (force_axis_ == "y" || force_axis_ == "Y") {
        return force.y;
    }
    return force.z;
}

bool TrajectoryExecutor::can_extend_trajectory_in_place(
    const geo_gp_interfaces::msg::PredictedTrajectory::SharedPtr msg) const {
    if (!executing_ || trajectory_.empty() || trajectory_time_.empty()) {
        return false;
    }
    if (msg->poses.size() <= trajectory_.size()) {
        return false;
    }
    if (msg->time_from_start.size() != msg->poses.size()) {
        return false;
    }
    if (trajectory_time_.size() != trajectory_.size()) {
        return false;
    }

    for (size_t i = 0; i < trajectory_.size(); ++i) {
        const auto & old_p = trajectory_[i].position;
        const auto & new_p = msg->poses[i].position;
        const double dx = old_p.x - new_p.x;
        const double dy = old_p.y - new_p.y;
        const double dz = old_p.z - new_p.z;
        const double pos_err = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (pos_err > progressive_match_pos_eps_) {
            return false;
        }
        if (std::fabs(trajectory_time_[i] - msg->time_from_start[i]) >
            progressive_match_time_eps_) {
            return false;
        }
    }

    return true;
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryExecutor>());
    rclcpp::shutdown();
    return 0;
}
