from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    move_node = Node(
        package="teleop_csv_player",
        executable="move_to_pose",
        name="move_to_pose",
        output="screen",

        parameters=[
            {
                # "target_pose": [0.308, -0.001, 0.591, 0.927, -0.375, 0.002, 0.001],  # Start pose

                # "Alpha"
                # "target_pose": [0.336751, -0.00765189, 0.453625, 0.913649, -0.406355, -0.00365943, -0.0104158],  # Reference
                # "target_pose": [0.405954, -0.0776077, 0.455011, 0.887463, -0.460395, -0.00158706, 0.0210644],  # Probe
                # "target_pose": [0.346887, 0.006606, 0.453219, 0.929524, -0.36709, 0.026844, 0.022569],  # Prediction

                # "Treble clef"
                # "target_pose": [0.330647, 0.00070572, 0.454163, 0.920275, -0.390777, -0.0176255, 0.00876145],  # Reference
                "target_pose": [0.350315, -0.0211602, 0.454292, 0.930813, -0.36454, -0.0180702, 0.0192617],  # Probe
                # "target_pose": [0.435769, -0.07834, 0.455725, 0.89721, -0.436484, -0.03254, 0.058631],  # Prediction

                "duration": 1.0,
                "rate": 200.0,
                "base_frame": "follower_link0",
                "ee_frame": "follower_link8"
            }
        ]
    )

    return LaunchDescription([move_node])