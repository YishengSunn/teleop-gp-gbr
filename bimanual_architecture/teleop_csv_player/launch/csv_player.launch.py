from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_csv_player',
            executable='csv_pose_player',
            parameters=[
                {
                    # "Alpha"
                    # "csv_path": "/home/user/humble_ws/src/data/02-16-1/refs_1/ref_2026-02-16_15-01-33.csv",
                    # "csv_path": "/home/user/humble_ws/src/data/02-16-1/probes_1/probe_2026-02-16_15-03-57.csv",
                    # "csv_path": "/home/user/humble_ws/src/data/02-16-1/preds_1/prediction_2026-02-16_15-03-57.csv",

                    # "Treble clef"
                    # "csv_path": "/home/user/humble_ws/src/data/02-16-1/refs_2/ref_2026-02-16_15-33-07.csv",
                    # "csv_path": "/home/user/humble_ws/src/data/02-16-1/probes_2/probe_2026-02-16_15-34-03.csv",
                    "csv_path": "/home/user/humble_ws/src/data/02-16-1/preds_2/prediction_2026-02-16_15-34-03.csv",

                    "rate": 20.0
                }
            ],
            output='screen'
        )
    ])