#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <vector>

using namespace std::chrono_literals;

class MoveToPose : public rclcpp::Node
{
public:
  MoveToPose()
  : Node("move_to_pose"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    target_ = declare_parameter<std::vector<double>>(
        "target_pose",
        std::vector<double>{0.4, 0.0, 0.4, 1, 0, 0, 0});

    duration_ = declare_parameter<double>("duration", 5.0);
    rate_ = declare_parameter<double>("rate", 200.0);

    base_frame_ = declare_parameter<std::string>("base_frame", "follower_link0");
    ee_frame_ = declare_parameter<std::string>("ee_frame", "follower_link8");

    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/cartesian_impedance/pose_desired", 10);

    RCLCPP_INFO(get_logger(), "Waiting for TF...");
    rclcpp::sleep_for(1s);

    current_ = getCurrentPose();

    if (current_.empty()) {
      RCLCPP_ERROR(get_logger(), "Failed to read current pose");
      rclcpp::shutdown();
      return;
    }

    start_time_ = now();

    timer_ = create_wall_timer(
        std::chrono::milliseconds((int)(1000.0 / rate_)),
        std::bind(&MoveToPose::update, this));

    RCLCPP_INFO(get_logger(), "MoveToPose started");
  }

private:
  std::vector<double> getCurrentPose() {
    geometry_msgs::msg::TransformStamped tf;

    try {
      tf = tf_buffer_.lookupTransform(
          base_frame_,
          ee_frame_,
          tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "TF error: %s", ex.what());
      return {};
    }

    std::vector<double> pose(7);

    pose[0] = tf.transform.translation.x;
    pose[1] = tf.transform.translation.y;
    pose[2] = tf.transform.translation.z;

    pose[3] = tf.transform.rotation.x;
    pose[4] = tf.transform.rotation.y;
    pose[5] = tf.transform.rotation.z;
    pose[6] = tf.transform.rotation.w;

    return pose;
  }

  std::vector<double> interpolate(double s) {
    std::vector<double> out(7);

    // Position
    for (int i = 0; i < 3; i++)
      out[i] = current_[i] + s * (target_[i] - current_[i]);

    // Quaternion slerp
    tf2::Quaternion q0(
        current_[3],
        current_[4],
        current_[5],
        current_[6]);

    tf2::Quaternion q1(
        target_[3],
        target_[4],
        target_[5],
        target_[6]);

    q0.normalize();
    q1.normalize();

    if (q0.dot(q1) < 0.0) {
        q1 = tf2::Quaternion(-q1.x(), -q1.y(), -q1.z(), -q1.w());
    }

    tf2::Quaternion q_interp = q0.slerp(q1, s);

    out[3] = q_interp.x();
    out[4] = q_interp.y();
    out[5] = q_interp.z();
    out[6] = q_interp.w();

    return out;
  }

  void update() {
    double t = (now() - start_time_).seconds();
    double s = std::min(t / duration_, 1.0);

    auto pose = interpolate(s);

    std_msgs::msg::Float64MultiArray msg;
    msg.data = pose;
    pub_->publish(msg);

    if (s >= 1.0)
      RCLCPP_INFO_ONCE(get_logger(), "Target reached");
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<double> target_;
  std::vector<double> current_;

  double duration_;
  double rate_;

  std::string base_frame_;
  std::string ee_frame_;

  rclcpp::Time start_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveToPose>());
  rclcpp::shutdown();
  return 0;
}
