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
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            Shutdown)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

def concatenate_ns(ns1, ns2, absolute=False):
    
    if(len(ns1) == 0):
        return ns2
    if(len(ns2) == 0):
        return ns1
    
    # check for /s at the end and start
    if(ns1[0] == '/'):
        ns1 = ns1[1:]
    if(ns1[-1] == '/'):
        ns1 = ns1[:-1]
    if(ns2[0] == '/'):
        ns2 = ns2[1:]
    if(ns2[-1] == '/'):
        ns2 = ns2[:-1]
    if(absolute):
        ns1 = '/' + ns1
    return ns1 + '/' + ns2


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    # Parameters as launch arguments
    ns = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description = LaunchConfiguration('robot_description')
    arm_id_1 = LaunchConfiguration('arm_id_1')
    arm_id_2 = LaunchConfiguration('arm_id_2')
    load_gripper_1 = LaunchConfiguration('load_gripper_1')
    load_gripper_2 = LaunchConfiguration('load_gripper_2')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('use_fake_hardware')

    use_rviz_param = 'use_rviz'


    db_arg = DeclareLaunchArgument(
        'db', default_value='True', description='Database flag'
    )
    
    sqlite_database = "/garmi-docker/ros2_ws/src/gs_study_pkg/config/warehouse_db.sqlite"
    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": sqlite_database,
    }

    # Semantic robot description
    garmi_semantic_xacro_file = os.path.join(get_package_share_directory('garmi_moveit_config'),
                                              'srdf',
                                              'garmi.srdf.xacro')
    
    robot_description_semantic_config = Command(
        [
            FindExecutable(name='xacro'),
            ' ', garmi_semantic_xacro_file,
            ' arm_id_1:=', arm_id_1,
            ' arm_id_2:=', arm_id_2,
            ' hand_1:=',load_gripper_1,  # Hardcoded values can stay as they are
            ' hand_2:=',load_gripper_2   # Hardcoded values can stay as they are
        ]
    )
    
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config
    }

    # Kinematics
    kinematics_yaml = load_yaml(
        'garmi_moveit_config', 'config/kinematics.yaml'
    )

    # Planning Functionality
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        'garmi_moveit_config', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        'garmi_moveit_config', 'config/real_garmi_controllers_ns.yaml'
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                     '/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'publish_robot_description': True,
        'publish_robot_description_semantic': True,
        'monitor_dynamics': False,
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=ns,
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            warehouse_ros_config
        ],
    )

    # RViz
    rviz_base = os.path.join(get_package_share_directory('garmi_moveit_config'), 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'garmi_moveit_ns.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace=ns,
        name='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            warehouse_ros_config
        ],
    )

    # Warehouse mongodb server
    db_config = LaunchConfiguration('db')
    mongodb_server_node = Node(
        package='warehouse_ros_mongo',
        executable='mongo_wrapper_ros.py',
        parameters=[
            {'warehouse_port': 33829},
            {'warehouse_host': 'localhost'},
            {'warehouse_plugin': 'warehouse_ros_mongo::MongoDatabaseConnection'},
        ],
        output='screen',
        condition=IfCondition(db_config)
    )

    # Gripper launch files
    # ***********************
    # TODO
    # ***********************
    # gripper_1_launch_file = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([PathJoinSubstitution(
    #         [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
    #     launch_arguments={'robot_ip': robot_ip_1,
    #                       use_fake_hardware_parameter_name: use_fake_hardware}.items(),
    #     condition=IfCondition(load_gripper_1)
    # )
    
    # gripper_2_launch_file = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([PathJoinSubstitution(
    #         [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
    #     launch_arguments={'robot_ip': robot_ip_2,
    #                       use_fake_hardware_parameter_name: use_fake_hardware}.items(),
    #     condition=IfCondition(load_gripper_2)
    # )

    return LaunchDescription(
        [
            # Parameters as launch arguments
            DeclareLaunchArgument('namespace', default_value='garmi'),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('robot_description', default_value=''),
            DeclareLaunchArgument('arm_id_1', default_value='left'),
            DeclareLaunchArgument('arm_id_2', default_value='right'),
            DeclareLaunchArgument('load_gripper_1', default_value='false'),
            DeclareLaunchArgument('load_gripper_2', default_value='false'),
            DeclareLaunchArgument('use_fake_hardware', default_value='false'),
            DeclareLaunchArgument('use_fake_hardware', default_value='false'),
            

            db_arg,
            rviz_node,
            run_move_group_node,
            # mongodb_server_node,
            # gripper_1_launch_file,
            # gripper_2_launch_file,
            Node( 
                package='controller_manager',
                executable='spawner',
                namespace= ns,
                arguments=['moveit_arm_controller'],
                output='screen',
            ),
        ]
    )