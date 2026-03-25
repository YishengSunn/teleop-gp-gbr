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
                "input_topic": "/leader/franka_robot_state_broadcaster/robot_state",
                "output_topic": "/gp_prompt_trajectory"
            }]
        ),
        Node(
            package='geo_gp_prediction',
            executable='prediction_node',
            name='prediction_node',
            output='screen',
            parameters=[{
                "config_path": "/home/user/geo-gp/config/default.yaml",
                "model_dir": "/home/user/geo-gp/data/02-26/models",
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
                "output_topic": "/execution/desired_pose"
            }]
        )
    ])
