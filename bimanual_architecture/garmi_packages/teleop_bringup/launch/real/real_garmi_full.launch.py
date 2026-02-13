import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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
    # Main namespace
    ns = 'garmi'

    # Declare parameter names
    robot_ip_1_parameter_name = 'robot_ip_1'
    robot_ip_2_parameter_name = 'robot_ip_2'
    
    load_gripper_1_parameter_name = 'load_gripper_1'
    load_gripper_2_parameter_name = 'load_gripper_2'

    arm_id_1_parameter_name = 'arm_id_1'
    arm_id_2_parameter_name = 'arm_id_2'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_rviz_parameter_name = 'use_rviz'

    # Fixed values
    load_gripper = False # We make gripper a fixed variable, mainly because parsing the argument 
                        # within generate_launch_description is a fairly unintuitive process, 
                        # and it's not worth doing just for a single boolean.

    # Launch configurations
    robot_ip_1 = LaunchConfiguration(robot_ip_1_parameter_name)
    robot_ip_2 = LaunchConfiguration(robot_ip_2_parameter_name)

    arm_id_1 = LaunchConfiguration(arm_id_1_parameter_name)
    arm_id_2 = LaunchConfiguration(arm_id_2_parameter_name)

    load_gripper_1 = LaunchConfiguration(load_gripper_1_parameter_name)
    load_gripper_2 = LaunchConfiguration(load_gripper_2_parameter_name)
    
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)

    garmi_hand_control_share = FindPackageShare('garmi_hand_control')

    
    # Path to communication handler launch file
    qb_hands_launch_path = PathJoinSubstitution([
        garmi_hand_control_share,
        'launch',
        'garmi_qb_hands.launch.py'
    ])

    qb_hands_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(qb_hands_launch_path),
        launch_arguments={
        'namespace': ns, 
        }.items(),
    )

    # Robot description (includes arms, base, and head)
    franka_xacro_file = os.path.join(
        get_package_share_directory('garmi_description'), 
        'robots',
        'garmi_real.urdf.xacro'
    )
    
    robot_description = Command([
        FindExecutable(name='xacro'), ' ', franka_xacro_file, 
        ' hand_1:=', load_gripper_1, 
        ' hand_2:=', load_gripper_2,
        ' robot_ip_1:=', robot_ip_1, 
        ' robot_ip_2:=', robot_ip_2, 
        ' arm_id_1:=', arm_id_1, 
        ' arm_id_2:=', arm_id_2,
        ' use_fake_hardware:=', use_fake_hardware,
        ' fake_sensor_commands:=', fake_sensor_commands
    ])

    # RViz configuration
    rviz_file = os.path.join(
        get_package_share_directory('garmi_description'), 
        'rviz',
        'visualize_garmi.rviz'
    )

    # Combined controller configuration
    combined_controllers = PathJoinSubstitution([
        FindPackageShare('teleop_bringup'),
        'config', 'real',
        'garmi_ns.yaml',
    ])

    # ========== Node Definitions ==========
    
    # Robot state publisher - publishes TF transforms
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=ns,
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # Joint state publisher - aggregates joint states from all sources
    jsp_source_list = [concatenate_ns(ns, 'joint_states', True)]
    
    ###########
    # ToDo: Panda gripper to joint states 
    ###########
    if(load_gripper):
        jsp_source_list.append(concatenate_ns(ns, 'left_gripper_sim_node/joint_states', True))
        jsp_source_list.append(concatenate_ns(ns, 'right_gripper_sim_node/joint_states', True))

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=ns,
        parameters=[{
            'source_list':  jsp_source_list,
        }],
    )

    # Single controller manager for all hardware interfaces
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=ns,
        parameters=[
            {'robot_description': robot_description}, 
            combined_controllers
        ],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        on_exit=Shutdown(),
    )

    # ========== Controller Spawners ==========
    
    # Joint state broadcaster (handles all joints: arms + base + head)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=ns,
        arguments=['joint_state_broadcaster'],
        # parameters=[{'joints': [
        # 'left_joint1', 'left_joint2', 'left_joint3', 'left_joint4', 'left_joint5', 
        # 'left_joint6', 'left_joint7', 'right_joint1', 'right_joint2', 'right_joint3', 
        # 'right_joint4', 'right_joint5', 'right_joint6', 'right_joint7', 'garmi_head_joint1',
        # 'garmi_head_joint2', 'garmi_base_joint_left', 'garmi_base_joint_right'
        # ]}],
        output='screen',
    )

    # Franka arm state broadcasters
    franka_left_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=ns,
        arguments=['left_arm_state'],
        output='screen',
        condition=UnlessCondition(use_fake_hardware),
    )

    franka_left_model_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=ns,
        arguments=['left_arm_model'],
        output='screen',
        condition=UnlessCondition(use_fake_hardware),
    )

    franka_right_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=ns,
        arguments=['right_arm_state'],
        output='screen',
        condition=UnlessCondition(use_fake_hardware),
    )

    franka_right_model_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=ns,
        arguments=['right_arm_model'],
        output='screen',
        condition=UnlessCondition(use_fake_hardware),
    )

    # Base controller spawner
    garmi_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=ns,
        arguments=['garmi_base_controller'],
        output='screen',
    )

    # Head controller spawner
    garmi_head_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=ns,
        arguments=['garmi_head_controller'],
        output='screen',
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=ns,
        arguments=['--display-config', rviz_file],
        condition=IfCondition(use_rviz)
    )

    # ========== Launch Description ==========
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            robot_ip_1_parameter_name,
            default_value='192.168.3.101',
            description='Hostname or IP address of robot 1.'
        ),
        DeclareLaunchArgument(
            robot_ip_2_parameter_name,
            default_value='192.168.3.102',
            description='Hostname or IP address of robot 2.'
        ),
        DeclareLaunchArgument(
            arm_id_1_parameter_name,
            default_value='left',
            description='Unique arm ID of robot 1.'
        ),
        DeclareLaunchArgument(
            arm_id_2_parameter_name,
            default_value='right',
            description='Unique arm ID of robot 2.'
        ),
        DeclareLaunchArgument(
            use_rviz_parameter_name,
            default_value='false',
            description='Visualize the robot in Rviz'
        ),
        DeclareLaunchArgument(
            use_fake_hardware_parameter_name,
            default_value='false',
            description='Use fake hardware'
        ),
        DeclareLaunchArgument(
            fake_sensor_commands_parameter_name,
            default_value='false',
            description=f"Fake sensor commands. Only valid when '{use_fake_hardware_parameter_name}' is true"
        ),
        DeclareLaunchArgument(
            load_gripper_1_parameter_name,
            default_value='false',
            description='Use Franka Gripper as end-effector for robot 1'
        ),
        DeclareLaunchArgument(
            load_gripper_2_parameter_name,
            default_value='false',
            description='Use Franka Gripper as end-effector for robot 2'
        ),
        
        # Start core nodes immediately
        robot_state_publisher_node,
        joint_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,

         
        # Wait 2 seconds, then start arm state broadcasters
        TimerAction(
            period=2.0,
            actions=[
                franka_left_state_broadcaster_spawner,
                franka_right_state_broadcaster_spawner,
            ]
        ),
        
        # # Wait 2 seconds, then start arm model broadcasters
        TimerAction(
            period=2.0,
            actions=[
                franka_left_model_broadcaster_spawner,
                franka_right_model_broadcaster_spawner,
            ]
        ),
        
        # Wait 2 seconds, then start base and head controllers
        # TimerAction(
        #     period=2.0,
        #     actions=[
        #         # garmi_base_controller_spawner,
        #         # garmi_head_controller_spawner,
        #     ]
        # ),

        # # Wait 2 seconds, then start RViz (optional)
        # TimerAction(
        #     period=2.0,
        #     actions=[rviz_node]
        # ),

        # # Start qb_hands control
        # qb_hands_launch,

        # Gripper launch
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([PathJoinSubstitution(
        #         [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
        #     launch_arguments={robot_ip_1_parameter_name: robot_ip_1,
        #                       use_fake_hardware_parameter_name: use_fake_hardware}.items(),
        #     condition=IfCondition(load_gripper_1)
        # ),
        # IncludeLaunchDescription(
        # PythonLaunchDescriptionSource([PathJoinSubstitution(
        #     [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
        # launch_arguments={robot_ip_2_parameter_name: robot_ip_2,
        #                     use_fake_hardware_parameter_name: use_fake_hardware}.items(),
        # condition=IfCondition(load_gripper_1)
        #   ),


        # Include MoveIt launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('teleop_bringup'),
                    'launch', 'real',
                    'real_garmi_moveit.launch.py'
                )
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'namespace': ns,
                'robot_description': robot_description,
                'arm_id_1': arm_id_1,
                'arm_id_2': arm_id_2,
                'load_gripper_1': load_gripper_1,
                'load_gripper_2': load_gripper_2,

            }.items(),
        ),
    ])
