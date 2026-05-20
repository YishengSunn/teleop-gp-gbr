from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='geo_gp_prompt',
            executable='prompt_recorder',
            name='prompt_recorder',
            output='screen',
            parameters=[{
                "input_topic": "/follower/franka_robot_state_broadcaster/robot_state",
                "output_topic": "/gp_prompt_trajectory",
                "execution_running_topic": "/execution/running",
                "blend_running_topic": "/execution/blend_to_leader_running",
                "enabled_topic": "/geo_gp/enabled",
                "online_mode": False,
                "publish_period_ms": 2000,
            }]
        ),
        Node(
            package='geo_gp_prediction',
            executable='prediction_node',
            name='prediction_node',
            output='screen',
            parameters=[{
                "config_path": "/home/user/geo-gp/config/default.yaml",
                "model_dir": "/home/user/geo-gp/data/05-08/models/6d/line1",
                "input_topic": "/gp_prompt_trajectory",
                "output_topic": "/gp_predicted_trajectory",
                "execution_running_topic": "/execution/running",
                "enabled_topic": "/geo_gp/enabled",
                "force_enabled_topic": "/geo_gp/force_prediction_enabled",
            }]
        ),
        Node(
            package='geo_gp_execution',
            executable='trajectory_executor',
            name='trajectory_executor',
            output='screen',
            parameters=[{
                "input_topic": "/gp_predicted_trajectory",
                "output_topic": "/execution/desired_pose",
                "force_output_topic": "/execution/desired_force",
                "force_axis": "z",
                "running_topic": "/execution/running",
            }]
        )
    ])
