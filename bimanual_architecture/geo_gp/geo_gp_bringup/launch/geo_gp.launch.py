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
                "blend_running_topic": "/execution/blend_to_leader_running"
            }]
        ),
        Node(
            package='geo_gp_prediction',
            executable='prediction_node',
            name='prediction_node',
            output='screen',
            parameters=[{
                "config_path": "/home/user/geo-gp/config/default.yaml",
                "model_dir": "/home/user/geo-gp/data/02-26/models/6d",
                "input_topic": "/gp_prompt_trajectory",
                "output_topic": "/gp_predicted_trajectory"
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
                "running_topic": "/execution/running",
            }]
        )
    ])
