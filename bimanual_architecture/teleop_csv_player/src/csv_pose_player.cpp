#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <chrono>

using namespace std::chrono_literals;

class CsvPosePlayer : public rclcpp::Node {
public:
  CsvPosePlayer()
  : Node("csv_pose_player")
  {
    topic_ = declare_parameter<std::string>(
      "topic",
      "/cartesian_impedance/pose_desired");

    csv_path_ = declare_parameter<std::string>(
      "csv_path",
      "trajectory.csv");

    publish_rate_ = declare_parameter<double>(
      "rate",
      200.0);   // Hz

    hold_time_ = declare_parameter<double>(
      "hold_time", 2.0);  // Seconds hold first pose

    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(topic_, 10);

    load_csv(csv_path_);

    start_time_ = now();

    timer_ = create_wall_timer(
      std::chrono::milliseconds((int)(1000.0 / publish_rate_)),
      std::bind(&CsvPosePlayer::timer_callback, this));

    RCLCPP_INFO(get_logger(), "CSV Pose Player Started");
  }

private:
  struct PoseRow
  {
    double time;
    std::vector<double> pose;  // x y z qx qy qz qw
  };

  void load_csv(const std::string &path)
  {
    std::ifstream file(path);
    std::string line;

    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open CSV: %s", path.c_str());
      return;
    }

    std::getline(file, line);  // Skip header

    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string item;
      std::vector<double> row;

      while (std::getline(ss, item, ','))
        row.push_back(std::stod(item));

      if (row.size() < 8)
        continue;

      PoseRow p;

      p.time = row[0];

      p.pose = {
        row[1],
        row[2],
        row[3],
        row[4],
        row[5],
        row[6],
        row[7]
      };

      trajectory_.push_back(p);
    }

    RCLCPP_INFO(get_logger(), "Loaded %zu trajectory points", trajectory_.size());
  }

  void timer_callback()
  {
    if (trajectory_.empty())
      return;

    double elapsed = (now() - start_time_).seconds();

    // Hold first pose to avoid reflex
    if (elapsed < hold_time_) {
      publish_pose(trajectory_[0].pose);
      return;
    }

    double t = elapsed - hold_time_;

    while (index_ + 1 < trajectory_.size() &&
           trajectory_[index_ + 1].time < t) {
      index_++;
    }

    if (index_ >= trajectory_.size()) {
      publish_pose(trajectory_.back().pose);
      RCLCPP_INFO_ONCE(get_logger(), "Trajectory finished");
      return;
    }

    publish_pose(trajectory_[index_].pose);
  }

  void publish_pose(const std::vector<double>& pose) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = pose;
    pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<PoseRow> trajectory_;

  std::string topic_;
  std::string csv_path_;
  double publish_rate_;
  double hold_time_;

  rclcpp::Time start_time_;
  size_t index_ = 0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CsvPosePlayer>());
  rclcpp::shutdown();
  return 0;
}
