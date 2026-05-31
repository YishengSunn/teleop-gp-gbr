#include "geo_gp_execution/trajectory_executor.hpp"

#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace {
void publish_running(const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr& pub, bool running) {
  std_msgs::msg::Bool msg;
  msg.data = running;
  pub->publish(msg);
}

double round_to(double value, int precision) {
  const double scale = std::pow(10.0, precision);
  return std::round(value * scale) / scale;
}
}  // namespace

TrajectoryExecutor::TrajectoryExecutor()
: Node("trajectory_executor"),
  index_(0),
  executing_(false),
  recording_leader_(false) {

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
    leader_input_topic_ = this->declare_parameter<std::string>(
        "leader_input_topic", "/leader/franka_robot_state_broadcaster/robot_state");
    csv_output_dir_ = this->declare_parameter<std::string>(
        "csv_output_dir", "");
    save_leader_csv_ = this->declare_parameter<bool>(
        "save_leader_csv", true);
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

    leader_state_sub_ = this->create_subscription<franka_msgs::msg::FrankaState>(
        leader_input_topic_, rclcpp::QoS(10),
        std::bind(&TrajectoryExecutor::leader_state_callback, this, _1)
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
    RCLCPP_INFO(
        this->get_logger(),
        "Leader recording: %s | topic=%s | csv_output_dir=%s",
        save_leader_csv_ ? "enabled" : "disabled",
        leader_input_topic_.c_str(),
        csv_output_dir_.empty() ? "(cwd)" : csv_output_dir_.c_str());

    publish_running(pub_running_, false);
}

std::string TrajectoryExecutor::format_execution_timestamp(const rclcpp::Time & stamp) {
    const int64_t nanoseconds = stamp.nanoseconds();
    const int64_t seconds = nanoseconds / 1000000000LL;

    std::time_t time_sec = static_cast<std::time_t>(seconds);
    std::tm tm_local{};
#if defined(_WIN32)
    localtime_s(&tm_local, &time_sec);
#else
    localtime_r(&time_sec, &tm_local);
#endif

    std::ostringstream oss;
    oss << std::put_time(&tm_local, "%Y-%m-%d_%H-%M-%S");
    return oss.str();
}

void TrajectoryExecutor::begin_leader_recording(const std::string & skill_name) {
    if (!save_leader_csv_) {
        return;
    }

    leader_samples_.clear();
    recording_skill_name_ = skill_name;
    recording_timestamp_ = format_execution_timestamp(start_time_);
    recording_leader_ = true;
}

void TrajectoryExecutor::save_leader_trajectory_csv() {
    if (!save_leader_csv_ || leader_samples_.empty()) {
        return;
    }

    std::filesystem::path output_dir = csv_output_dir_.empty()
        ? std::filesystem::current_path()
        : std::filesystem::path(csv_output_dir_);
    std::error_code ec;
    std::filesystem::create_directories(output_dir, ec);
    if (ec) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to create leader CSV output directory '%s': %s",
            output_dir.string().c_str(),
            ec.message().c_str());
        return;
    }

    const std::string filename =
        "leader_execution_" + recording_timestamp_ + ".csv";
    const std::filesystem::path filepath = output_dir / filename;

    std::ofstream out(filepath);
    if (!out.is_open()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to open leader CSV file '%s'",
            filepath.string().c_str());
        return;
    }

    out << "time,x,y,z,qx,qy,qz,qw,fx,fy,fz\n";
    out << std::fixed;
    for (const auto & sample : leader_samples_) {
        const auto & pose = sample.pose;
        const auto & force = sample.force;
        out << round_to(sample.time, 4) << ","
            << round_to(pose.position.x, 6) << ","
            << round_to(pose.position.y, 6) << ","
            << round_to(pose.position.z, 6) << ","
            << round_to(pose.orientation.x, 6) << ","
            << round_to(pose.orientation.y, 6) << ","
            << round_to(pose.orientation.z, 6) << ","
            << round_to(pose.orientation.w, 6) << ","
            << round_to(force.x, 6) << ","
            << round_to(force.y, 6) << ","
            << round_to(force.z, 6) << "\n";
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Saved leader execution trajectory | skill=%s | samples=%zu | file=%s",
        recording_skill_name_.c_str(),
        leader_samples_.size(),
        filepath.string().c_str());
}

void TrajectoryExecutor::finalize_leader_recording() {
    if (!recording_leader_) {
        return;
    }

    save_leader_trajectory_csv();
    leader_samples_.clear();
    recording_leader_ = false;
    recording_skill_name_.clear();
    recording_timestamp_.clear();
}

void TrajectoryExecutor::leader_state_callback(
    const franka_msgs::msg::FrankaState::SharedPtr msg) {
    if (!executing_ || !recording_leader_) {
        return;
    }

    const double elapsed = std::max(0.0, (this->now() - start_time_).seconds());

    Eigen::Map<const Eigen::Matrix4d> T(msg->o_t_ee.data());

    LeaderSample sample;
    sample.time = elapsed;
    sample.pose.position.x = T(0, 3);
    sample.pose.position.y = T(1, 3);
    sample.pose.position.z = T(2, 3);

    Eigen::Quaterniond q(T.block<3, 3>(0, 0));
    q.normalize();
    sample.pose.orientation.x = q.x();
    sample.pose.orientation.y = q.y();
    sample.pose.orientation.z = q.z();
    sample.pose.orientation.w = q.w();

    sample.force.x = msg->o_f_ext_hat_k[0];
    sample.force.y = msg->o_f_ext_hat_k[1];
    sample.force.z = msg->o_f_ext_hat_k[2];

    leader_samples_.push_back(sample);
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

    if (executing_) {
        finalize_leader_recording();
    }

    index_ = 0;
    executing_ = true;
    start_time_ = this->now();
    begin_leader_recording(msg->skill_name);
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
        finalize_leader_recording();
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
