from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

import xacro
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

def generate_launch_description():
    initial_positions_1_param = 'initial_positions_1'
    initial_positions_2_param = 'initial_positions_2'
    use_rviz_param = 'use_rviz'

    initial_positions_1 = LaunchConfiguration(initial_positions_1_param)
    initial_positions_2 = LaunchConfiguration(initial_positions_2_param)
    use_rviz = LaunchConfiguration(use_rviz_param)

    # Fixed values
    load_gripper = False # We make gripper a fixed variable, mainly because parsing the argument 
                        # within generate_launch_description is a fairly unintuitive process, 
                        # and it's not worth doing just for a single boolean.
    
    load_lidars = False # Lidar sensor simulation takes up a pretty significant amount of resources,
                        # so it should only be enabled if it's necessary.
    
    yaml_config = 'sim_garmi_teleop.yaml'
    scene_file = 'garmi.xml'
    if(load_gripper): # mujoco scene file must be manually adjusted since there's no way to pass parameters
        if(load_lidars):
            scene_file = 'garmi_lidar.xml'
            yaml_config = 'sim_garmi_lidarless.yaml'
        else:
            scene_file = 'garmi.xml'
    else:
        scene_file = 'garmi_ng.xml'
    garmi_xacro_file = os.path.join(get_package_share_directory('garmi_description'), 'robots',
                                     'garmi_sim.urdf.xacro')
    xml_path = os.path.join(get_package_share_directory('garmi_description'), 'mujoco', 'garmi', 'assets', 'xml', scene_file)
    mjros_config_file = os.path.join(get_package_share_directory('teleop_bringup'), 'config', 'sim',
                                     yaml_config)
    ns = "garmi"     # this must match the namespace argument under mujoco_ros2_control in the plugin's parameter yaml file. 
                # See the ros2_control_plugins_example_with_ns.yaml file for more details.

    # Robot state publisher setup
    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', garmi_xacro_file, 
            ' arm_id_1:=left', 
            ' arm_id_2:=right',
            ' hand_1:=', str(load_gripper).lower(),
            ' hand_2:=', str(load_gripper).lower(),
            ' initial_positions_1:=', initial_positions_1,
            ' initial_positions_2:=', initial_positions_2])
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace= ns,
        parameters=[{'robot_description': robot_description}]
    )
    
    # Joint state publisher setup
    jsp_source_list = [concatenate_ns(ns, 'joint_states', True)]
    if(load_gripper):
        jsp_source_list.append(concatenate_ns(ns, 'left_gripper_sim_node/joint_states', True))
        jsp_source_list.append(concatenate_ns(ns, 'right_gripper_sim_node/joint_states', True))

    node_joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace= ns,
            parameters=[
                {'source_list': jsp_source_list,
                 'rate': 30}],
    )

    return LaunchDescription([
        # Launch args
        DeclareLaunchArgument(
            use_rviz_param,
            default_value='false',
            description='Visualize the robot in Rviz'),
        DeclareLaunchArgument(
            initial_positions_1_param,
            default_value='"-1.571 -0.785 0.0 -2.356 0.0 1.571 0.785"',
            description='Initial joint positions of robot 1. Must be enclosed in quotes, and in pure number.'
                        'Defaults to the "communication_test" pose.'),
        DeclareLaunchArgument(
            initial_positions_2_param,
            default_value='"1.571 -0.785 0.0 -2.356 0.0 1.571 0.785"',
            description='Initial joint positions of robot 2. Must be enclosed in quotes, and in pure number.'
                        'Defaults to the "communication_test" pose.'),
        
        node_robot_state_publisher,
        node_joint_state_publisher,

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(get_package_share_directory('teleop_bringup') + '/launch/sim/launch_mujoco_ros_server_ns.launch'),
            launch_arguments={
                'use_sim_time': "true",
                'modelfile': xml_path,
                'unpause': 'true',
                'verbose': "true",
                'ns': ns,
                'debug': "false",
                'mujoco_plugin_config': mjros_config_file

            }.items()
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            namespace= ns,
            arguments=['joint_state_broadcaster', '-c', concatenate_ns(ns, 'controller_manager', True)],
            output='screen',
        ),
        # Franka State Broadcasters
        # Left Arm
        Node( 
            package='controller_manager',
            executable='spawner',
            namespace= ns,
            arguments=['left_state_broadcaster', '-c', concatenate_ns(ns, 'controller_manager', True)],
            output='screen',
        ),
        # Right Arm
        Node(
            package='controller_manager',
            executable='spawner',
            namespace= ns,
            arguments=['right_state_broadcaster', '-c', concatenate_ns(ns, 'controller_manager', True)],
            output='screen',
        ),
        # Start Head Controller
        Node( 
            package='controller_manager',
            executable='spawner',
              namespace= ns,
            arguments=['garmi_head_controller', '-c', concatenate_ns(ns, 'controller_manager', True)],
            output='screen',
        ),
        # Start Base Controller
        Node(
            package='controller_manager',
            executable='spawner',
            namespace= ns,
            arguments=['garmi_base_controller', '-c', concatenate_ns(ns, 'controller_manager', True)],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            namespace= ns,
            arguments=['left_move_to_start_controller', '-c', concatenate_ns(ns, 'controller_manager', True)],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            namespace= ns,
            arguments=['right_move_to_start_controller', '-c', concatenate_ns(ns, 'controller_manager', True)],
            output='screen',
        ),


        # # Include MoveIt launch file
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(
        #             get_package_share_directory('teleop_bringup'),
        #             'launch', 'sim',
        #             'sim_garmi_moveit.launch.py'
        #         )
        #     ),
        #     launch_arguments={
        #         'use_sim_time': 'true',
        #         'namespace': ns,
        #         'robot_description': robot_description,
        #     }.items(),
        # ),
    ])
