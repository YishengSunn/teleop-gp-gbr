#  Copyright (c) 2021 Franka Emika GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip_parameter_name = 'robot_ip'
    arm_id_parameter_name = 'arm_id'
    load_gripper_parameter_name = 'load_gripper'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_rviz_parameter_name = 'use_rviz'
    ns_parameter_name = 'ns'
    move_to_start_parameter_name = 'move_to_start'
    control_mode_parameter_name = 'control_mode'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    arm_id = LaunchConfiguration(arm_id_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)
    move_to_start = LaunchConfiguration(move_to_start_parameter_name)
    control_mode = LaunchConfiguration(control_mode_parameter_name)

    ns = LaunchConfiguration(ns_parameter_name)
    cm_abs = [TextSubstitution(text='/'), ns, TextSubstitution(text='/controller_manager')]

    franka_xacro_file = os.path.join(get_package_share_directory('franka_description'), 'robots', 'real',
                                     'panda_arm.urdf.xacro')
    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=', load_gripper,
         ' robot_ip:=', robot_ip, ' use_fake_hardware:=', use_fake_hardware,
         ' fake_sensor_commands:=', fake_sensor_commands, ' arm_id:=', arm_id])

    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_franka.rviz')

    franka_controllers = PathJoinSubstitution(
        [
            FindPackageShare('teleop_bringup'),
            'config', 'single',
            'single_controllers.yaml',
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            arm_id_parameter_name,
            description='Arm ID of the robot.'),
        DeclareLaunchArgument(
            robot_ip_parameter_name,
            description='Hostname or IP address of the robot.'),
        DeclareLaunchArgument(
            use_rviz_parameter_name,
            default_value='false',
            description='Visualize the robot in Rviz'),
        DeclareLaunchArgument(
            use_fake_hardware_parameter_name,
            default_value='false',
            description='Use fake hardware'),
        DeclareLaunchArgument(
            fake_sensor_commands_parameter_name,
            default_value='false',
            description="Fake sensor commands. Only valid when '{}' is true".format(
                use_fake_hardware_parameter_name)),
        DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value='false',
            description='Use Franka Gripper as an end-effector, otherwise, the robot is loaded '
                        'without an end-effector.'),
        DeclareLaunchArgument(
            ns_parameter_name,
            description='Top-level namespace'),
        DeclareLaunchArgument(
            move_to_start_parameter_name,
            default_value='false',
            description='Enable movement'),
        DeclareLaunchArgument(
            control_mode_parameter_name,
            default_value='joint_impedance',
            description='Control mode'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=ns,
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=ns,
            parameters=[{
                    'source_list': [
                        [TextSubstitution(text='/'), ns, TextSubstitution(text='/joint_states')],
                        'panda_gripper/joint_states'
                    ],
                    'rate': 30
                    }],
        ),
        Node(
            package='franka_control2',
            executable='franka_control2_node',
            parameters=[{'robot_description': robot_description}, franka_controllers],
            # remappings=[('joint_states', 'joint_states')],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
            namespace=ns,
            on_exit=Shutdown(),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', cm_abs],
            namespace=ns,
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['franka_robot_state_broadcaster', '--controller-manager', cm_abs],
            namespace=ns,
            output='screen',
            condition=UnlessCondition(use_fake_hardware),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gravity_compensation_with_joint_torque_feedback_controller', '--controller-manager', cm_abs],
            namespace=ns,
            output='screen',
            condition=IfCondition(
                PythonExpression([
                "'", ns, "' == 'leader' and '", move_to_start, "' == 'false'"
                ]),
            )
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['move_to_start_example_controller', '--controller-manager', cm_abs],
            namespace=ns,
            output='screen',
            condition=IfCondition(move_to_start),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_impedance_controller', '--controller-manager', cm_abs],
            namespace=ns,
            output='screen',
            condition=IfCondition(
                PythonExpression([
                "'", ns, "' == 'follower' and '", move_to_start, "' == 'false' and '", control_mode, "' == 'joint_impedance'"
                ]),
            ),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['cartesian_impedance_controller', '--controller-manager', cm_abs],
            namespace=ns,
            output='screen',
            condition=IfCondition(
                PythonExpression([
                "'", ns, "' == 'follower' and '", move_to_start, "' == 'false' and '", control_mode, "' == 'cartesian_impedance'"
                ]),
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution(
                [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
            launch_arguments={robot_ip_parameter_name: robot_ip,
                              use_fake_hardware_parameter_name: use_fake_hardware}.items(),
            condition=IfCondition(load_gripper)

        ),
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file],
             condition=IfCondition(use_rviz)
             )
    ])
