from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_trajectory_executor = LaunchConfiguration('use_trajectory_executor')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_trajectory_executor',
            default_value='true',
            description='Run trajectory_executor when true, otherwise run online_fuser.',
        ),
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
                "model_dir": "/home/user/geo-gp/data/06-02/models/6d",
                "input_topic": "/gp_prompt_trajectory",
                "output_topic": "/gp_predicted_trajectory",
                "execution_running_topic": "/execution/running",
                "enabled_topic": "/geo_gp/enabled",
                "force_enabled_topic": "/geo_gp/force_prediction_enabled",
                "save_csv": False,
                "csv_output_dir": "/home/user/geo-gp/data/05-08/preds/arc1/arc1_1",
            }]
        ),
        Node(
            package='geo_gp_execution',
            executable='trajectory_executor',
            name='trajectory_executor',
            output='screen',
            condition=IfCondition(use_trajectory_executor),
            parameters=[{
                "input_topic": "/gp_predicted_trajectory",
                "output_topic": "/execution/desired_pose",
                "force_output_topic": "/execution/desired_force",
                "force_axis": "z",
                "running_topic": "/execution/running",
                "leader_input_topic": "/leader/franka_robot_state_broadcaster/robot_state",
                "save_leader_csv": False,
                "csv_output_dir": "/home/user/geo-gp/data/05-08/preds/arc1/arc1_1",
            }]
        ),
        Node(
            package='geo_gp_fusion',
            executable='online_fuser',
            name='online_fuser',
            output='screen',
            condition=UnlessCondition(use_trajectory_executor),
            parameters=[{
                "prediction_topic": "/gp_predicted_trajectory",
                "tdpa_pose_topic": "/tdpa/integrated_desired_pose",
                "output_pose_topic": "/execution/desired_pose",
                "running_topic": "/execution/running",
                "rate": 200.0,
                "leader_timeout_sec": 0.1,
                "confidence_gain": 1.0,
                "min_prediction_weight": 0.0,
                "max_prediction_weight": 1.0,
                "progressive_update_enabled": True,
                "progressive_match_pos_eps": 1e-3,
                "progressive_match_time_eps": 1e-3,
            }]
        )
    ])
