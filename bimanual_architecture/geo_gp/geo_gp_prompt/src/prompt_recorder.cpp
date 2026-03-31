#include "geo_gp_prompt/prompt_recorder.hpp"

using std::placeholders::_1;


PromptRecorder::PromptRecorder()
: Node("prompt_recorder"),
  moving_count_threshold_(5),
  stop_count_threshold_(15),
  start_threshold_(0.05),
  stop_threshold_(0.02) {

    state_ = State::IDLE;

    input_topic_ = this->declare_parameter<std::string>(
        "input_topic",
        "/follower/franka_robot_state_broadcaster/robot_state");

    output_topic_ = this->declare_parameter<std::string>(
        "output_topic",
        "/gp_prompt_trajectory");

    execution_running_topic_ = this->declare_parameter<std::string>(
        "execution_running_topic",
        "/execution/running");
    blend_running_topic_ = this->declare_parameter<std::string>(
        "blend_running_topic",
        "/execution/blend_to_leader_running");

    pose_sub_ = this->create_subscription<franka_msgs::msg::FrankaState>(
        input_topic_, 10,
        std::bind(&PromptRecorder::pose_callback, this, _1)
    );

    execution_running_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        execution_running_topic_, 10,
        std::bind(&PromptRecorder::execution_running_callback, this, _1)
    );
    blend_running_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        blend_running_topic_, 10,
        std::bind(&PromptRecorder::blend_running_callback, this, _1)
    );

    prompt_pub_ = this->create_publisher<geo_gp_interfaces::msg::PromptTrajectory>(
        output_topic_, 10
    );

    RCLCPP_INFO(this->get_logger(), "Prompt Recorder started.");
    RCLCPP_INFO(this->get_logger(), "Listening on: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Execution running topic: %s", execution_running_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Blend running topic: %s", blend_running_topic_.c_str());
}

void PromptRecorder::execution_running_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
    execution_running_ = msg->data;
}

void PromptRecorder::blend_running_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data != blend_running_) {
        RCLCPP_INFO(
            this->get_logger(),
            "Blend-to-leader %s",
            msg->data ? "START (record disabled)" : "END (record enabled)");
    }
    blend_running_ = msg->data;
}

void PromptRecorder::pose_callback(
    const franka_msgs::msg::FrankaState::SharedPtr msg) {
    if (execution_running_ || blend_running_) {
        if (blend_running_) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Skipping prompt recording while blend_to_leader is active");
        }
        if (state_ != State::IDLE) {
            poses_.clear();
            time_from_start_.clear();
            moving_counter_ = 0;
            stop_counter_ = 0;
            state_ = State::IDLE;
        }
        return;
    }

    auto now = this->now();

    // 1. Detect motion based on joint velocities
    double dq_norm = 0.0;
    for (double v : msg->dq) {
        dq_norm += v * v;
    }
    dq_norm = std::sqrt(dq_norm);

    // 2. Extract end-effector pose
    Eigen::Map<const Eigen::Matrix4d> T(msg->o_t_ee.data());

    geometry_msgs::msg::Pose pose;

    pose.position.x = T(0, 3);
    pose.position.y = T(1, 3);
    pose.position.z = T(2, 3);

    Eigen::Quaterniond q(T.block<3,3>(0,0));
    q.normalize();

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    // 3. State machine
    switch (state_) {
        case State::IDLE: {
            if (dq_norm > start_threshold_) moving_counter_++;
            else moving_counter_ = 0;

            if (moving_counter_ > moving_count_threshold_) {
                poses_.clear();
                time_from_start_.clear();
                motion_start_time_ = now;
                poses_.push_back(pose);
                time_from_start_.push_back(0.0);

                state_ = State::MOVING;
                moving_counter_ = 0;

                RCLCPP_INFO(this->get_logger(), "IDLE -> MOVING");
            }
            break;
        }

        case State::MOVING: {
            poses_.push_back(pose);
            time_from_start_.push_back((now - motion_start_time_).seconds());

            if (poses_.size() > 2000) poses_.erase(poses_.begin());
            if (time_from_start_.size() > 2000) time_from_start_.erase(time_from_start_.begin());

            if (dq_norm < stop_threshold_) stop_counter_++;
            else stop_counter_ = 0;

            if (stop_counter_ > stop_count_threshold_) {
                state_ = State::STOPPING;
                stop_counter_ = 0;

                RCLCPP_INFO(this->get_logger(), "MOVING -> STOPPING");
            }
            break;
        }

        case State::STOPPING: {
            if (poses_.size() >= min_points_) {
                publish_prompt();

                RCLCPP_INFO(this->get_logger(),
                    "STOPPING -> IDLE (sent %zu poses)", poses_.size());
            }
            else {
                RCLCPP_WARN(this->get_logger(),
                    "Trajectory too short (%zu), skip.", poses_.size());
            }

            poses_.clear();
            time_from_start_.clear();
            state_ = State::IDLE;
            break;
        }
    }
}

void PromptRecorder::publish_prompt() {
    if (poses_.empty()) {
        return;
    }

    geo_gp_interfaces::msg::PromptTrajectory msg;
    msg.header.frame_id = "world";
    msg.poses = poses_;
    msg.time_from_start = time_from_start_;

    prompt_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(),
        "Published prompt trajectory with %zu poses.", poses_.size());
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PromptRecorder>());
    rclcpp::shutdown();
    return 0;
}
