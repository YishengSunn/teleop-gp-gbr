from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
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

    # Namespace arg (default matches your comment about plugin YAML)
    ns_arg = DeclareLaunchArgument('ns', default_value='garmi', description='Top-level namespace')
    ns = LaunchConfiguration('ns')   # this must match the namespace argument under mujoco_ros2_control in the plugin's parameter yaml file. 
                                     # See the ros2_control_plugins_example_with_ns.yaml file for more details.

    initial_positions_1 = LaunchConfiguration(initial_positions_1_param)
    initial_positions_2 = LaunchConfiguration(initial_positions_2_param)
    use_rviz = LaunchConfiguration(use_rviz_param)

    # Fixed values
    load_gripper = True # We make gripper a fixed variable, mainly because parsing the argument 
                        # within generate_launch_description is a fairly unintuitive process, 
                        # and it's not worth doing just for a single boolean.
    
    load_lidars = False # Lidar sensor simulation takes up a pretty significant amount of resources,
                        # so it should only be enabled if it's necessary.
    
    yaml_config = 'sim_garmi.yaml'
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
    mjros_config_file = os.path.join(get_package_share_directory('garmi_bringup'), 'config', 'sim',
                                     yaml_config)
    # ns value is taken from the launch arg above.

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
        parameters=[{'robot_description': robot_description}]
    )
    
    # Joint state publisher setup
    # Build absolute topics using ns LaunchConfiguration (works at launch-time)
    jsp_source_list = [PythonExpression(['"/" + ', ns, ' + "/joint_states"'])]
    if(load_gripper):
        jsp_source_list.append(PythonExpression(['"/" + ', ns, ' + "/left_gripper_sim_node/joint_states/joint_states"']))
        jsp_source_list.append(PythonExpression(['"/" + ', ns, ' + "/right_gripper_sim_node/joint_states/joint_states"']))

    node_joint_state_publisher = Node( # RVIZ dependency
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {'source_list': jsp_source_list,
                 'rate': 30}],
    )

    # Others
    rviz_file = os.path.join(get_package_share_directory('garmi_description'), 'rviz',
                             'visualize_garmi.rviz')
    
    # Controller spawners – target the namespaced controller_manager
    spawner_jsb = Node( # RVIZ dependency
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', ['/', ns, '/controller_manager']],
        output='screen',
    )
    spawner_dual_imp = Node( # Dual Joint Impedance Controller
        package='controller_manager',
        executable='spawner',
        arguments=['dual_joint_impedance_controller', '-c', ['/', ns, '/controller_manager']],
        output='screen',
    )

    # MuJoCo server include (pass ns through)
    mujoco_server = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(get_package_share_directory('garmi_bringup') + '/launch/sim/launch_mujoco_ros_server.launch'),
        launch_arguments={
            'use_sim_time': "true",
            'modelfile': xml_path,
            'verbose': "true",
            'ns': ns,
            'mujoco_plugin_config': mjros_config_file
        }.items()
    )

    rviz_node = Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file],
             condition=IfCondition(use_rviz)
             )

    # Group everything under the namespace 
    stack = GroupAction([
        PushRosNamespace(ns),
        node_robot_state_publisher,
        # node_joint_state_publisher, 
        spawner_jsb,
        spawner_dual_imp,
        rviz_node,
    ])

    return LaunchDescription([
        # Launch args
        ns_arg,
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
        mujoco_server,
        stack,
    ])
