#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <franka_msgs/msg/franka_state.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <array>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;


std::string getTimeStamp() {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);

    std::tm tm{};
    localtime_r(&t, &tm);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    return oss.str();
}

class EETrajectoryRecorder : public rclcpp::Node {
public:
  EETrajectoryRecorder()
  : Node("ee_trajectory_recorder"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    base_frame_ = "leader_link0";
    ee_frame_   = "leader_link8";
    robot_state_topic_ = "/leader/franka_robot_state_broadcaster/robot_state";

    std::string filename = "ee_trajectory_" + getTimeStamp() + ".csv";
    file_.open(filename);
    file_ << "time,x,y,z,qx,qy,qz,qw,fx,fy,fz\n";

    const auto latest_only_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    robot_state_sub_ = this->create_subscription<franka_msgs::msg::FrankaState>(
      robot_state_topic_,
      latest_only_qos,
      std::bind(&EETrajectoryRecorder::robotStateCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Waiting for TF tree...");

    while (rclcpp::ok() &&
           !tf_buffer_.canTransform(base_frame_, ee_frame_, tf2::TimePointZero, 1s))
    {
      rclcpp::sleep_for(500ms);
    }

    RCLCPP_INFO(this->get_logger(), "TF ready. Start recording.");
    start_time_ = this->now();

    timer_ = this->create_wall_timer(
      5ms, std::bind(&EETrajectoryRecorder::record, this));
  }

private:
  void robotStateCallback(const franka_msgs::msg::FrankaState::SharedPtr msg) {
    latest_force_ = {
      msg->o_f_ext_hat_k[0],
      msg->o_f_ext_hat_k[1],
      msg->o_f_ext_hat_k[2],
    };
  }

  void record() {
    try {
      auto tf = tf_buffer_.lookupTransform(
          base_frame_, ee_frame_, tf2::TimePointZero);

      double t = (this->now() - start_time_).seconds();
      std::ostringstream time_stream;
      time_stream << std::fixed << std::setprecision(2) << t;

      auto &p = tf.transform.translation;
      auto &q = tf.transform.rotation;

      file_ << time_stream.str() << ","
            << p.x << "," << p.y << "," << p.z << ","
            << q.x << "," << q.y << "," << q.z << "," << q.w << ","
            << latest_force_[0] << "," << latest_force_[1] << "," << latest_force_[2]
            << "\n";
    }
    catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    }
  }

  std::string base_frame_, ee_frame_, robot_state_topic_;
  std::array<double, 3> latest_force_{0.0, 0.0, 0.0};
  std::ofstream file_;
  rclcpp::Time start_time_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<franka_msgs::msg::FrankaState>::SharedPtr robot_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EETrajectoryRecorder>());
  rclcpp::shutdown();
  return 0;
}
