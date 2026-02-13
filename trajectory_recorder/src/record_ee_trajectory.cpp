#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;


std::string getTimeStamp()
{
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);

    std::tm tm{};
    localtime_r(&t, &tm);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    return oss.str();
}

class EETrajectoryRecorder : public rclcpp::Node
{
public:
  EETrajectoryRecorder()
  : Node("ee_trajectory_recorder"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    base_frame_ = "leader_link0";
    ee_frame_   = "leader_link8";

    std::string filename = "ee_trajectory_" + getTimeStamp() + ".csv";
    file_.open(filename);
    file_ << "time,x,y,z,qx,qy,qz,qw\n";

    RCLCPP_INFO(this->get_logger(), "Waiting for TF tree...");

    while (rclcpp::ok() &&
           !tf_buffer_.canTransform(base_frame_, ee_frame_, tf2::TimePointZero, 1s))
    {
      rclcpp::sleep_for(500ms);
    }

    RCLCPP_INFO(this->get_logger(), "TF ready. Start recording.");

    timer_ = this->create_wall_timer(
      50ms, std::bind(&EETrajectoryRecorder::record, this));
  }

private:
  void record()
  {
    try
    {
      auto tf = tf_buffer_.lookupTransform(
          base_frame_, ee_frame_, tf2::TimePointZero);

      double t = this->now().seconds();

      auto &p = tf.transform.translation;
      auto &q = tf.transform.rotation;

      file_ << t << ","
            << p.x << "," << p.y << "," << p.z << ","
            << q.x << "," << q.y << "," << q.z << "," << q.w
            << "\n";
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    }
  }

  std::string base_frame_, ee_frame_;
  std::ofstream file_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EETrajectoryRecorder>());
  rclcpp::shutdown();
  return 0;
}
