import os
import sys
# from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnExecutionComplete
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # ld = LaunchDescription()
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")
    
    namespace = LaunchConfiguration("namespace")

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='former2',
        description='Top-level namespace')
    
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('/diagnostics', 'diagnostics')]

    upload_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('former_description'),
            '/launch/ns_upload_robot.launch.py']
        ),
        launch_arguments = {
            'use_gazebo_sim': 'false',
            'namespace': namespace
        }.items()
    )

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('former_description'),
            'urdf/robot.urdf.xacro',
        ]),
        ' use_gazebo_sim:=', 'false',
        ' namespace:=', namespace
    ])

    robot_description = {"robot_description": robot_description_content}

    # robot_localization_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     remappings=remappings,
    #     parameters=[
    #         PathJoinSubstitution([
    #             FindPackageShare('former_bringup'),
    #             'config/ekf.yaml'
    #         ]),
    #         {'use_sim_time': LaunchConfiguration('use_sim_time')}
    #     ],
    # )

    robot_controllers = PathJoinSubstitution([FindPackageShare('former_bringup'), "config", "controllers_former.yaml"])

    param_substitutions = {}

    configured_robot_controllers_param = RewrittenYaml(
        source_file=robot_controllers,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            configured_robot_controllers_param
            ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        remappings=remappings,
        respawn=True,
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        
        executable="spawner",
        remappings=remappings,
        arguments=["joint_state_broadcaster", "--controller-manager", "controller_manager"],
    )

    load_base_controller = Node(
        package="controller_manager",
        executable="spawner",
        remappings=remappings,
        arguments=["base_controller", "--controller-manager", "controller_manager"],
    )

    load_former_io_controller = Node(
        package="controller_manager",
        executable="spawner",
        remappings=remappings,
        arguments=["former_io_controller", "--controller-manager", "controller_manager"],
    )

    lidar_bringup = Node(
        package="sick_scan",
        executable="sick_generic_caller",
        respawn=True,
        arguments=[
            PathJoinSubstitution([
                FindPackageShare('sick_scan'),
                'launch/sick_tim_5xx.launch'
            ]),
            'hostname:=192.168.10.11',
            'frame_id:=laser_link',
            # 'min_ang:=-1.0',  // ERROR: Need setting in launch file
            # 'max_ang:=1.0',  // ERROR Need setting in launch file
            'nodename:=front_lidar',
            'range_min:=0.05',
            'sw_pll_only_publish:=false',
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },   
        remappings=remappings + [('front_lidar/scan', 'scan')]
    )

    auto_docking_bringup = Node(
        package="former_auto_docking",
        executable="auto_docking_node",
        respawn=True,
        parameters=[
            {"distance_approach": 0.260},
        ],
        remappings=remappings +
            [('odom', 'base_controller/odom'),
            ('cmd_vel', 'base_controller/cmd_vel_unstamped')]
        ,
        output={
            "stdout": "screen",
            "stderr": "screen",
        }
    )

    gpio_board_bringup = Node(
        package="former_gpio_board",
        executable="main_node",
        respawn=True,
        remappings=remappings,
        parameters=[
            {"port_name": "/dev/ttyARDUINO"},
            {"baudrate": 115200},
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        }
    )

    imu_bringup = Node(
        package="imu_xg6000_ros2",
        executable="main_node",
        respawn=True,
        remappings=remappings,
        parameters=[
            {'port_name': '/dev/ttyIMU'},
            {'baudrate': 38400},
            {'frame_id': 'imu_link'},
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        }
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        remappings=remappings,
        parameters=[{
            'dev': '/dev/js0',
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }],
    )

    joy_param = PathJoinSubstitution([FindPackageShare('former_bringup'), 'config/ps5.config.yaml']),

    configured_joy_param = RewrittenYaml(
        source_file=joy_param,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    teleop_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[
            configured_joy_param
        ],
        remappings= remappings +
            [('cmd_vel', 'base_controller/cmd_vel_unstamped')]
    )

    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            namespace=namespace),
        upload_robot,
        control_node,
        load_joint_state_broadcaster,
        load_base_controller,
        load_former_io_controller,
        # # robot_localization_node,
        lidar_bringup,
        imu_bringup,
        gpio_board_bringup,
        auto_docking_bringup,
        joy_node,
        teleop_joy_node
    ])

    return LaunchDescription([
        use_sim_time,
        declare_namespace_cmd,
        bringup_cmd_group
    ])
