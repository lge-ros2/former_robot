import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnExecutionComplete
from launch_ros.actions import PushRosNamespace
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command

def generate_launch_description():

    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false")

    upload_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('former_description'),
            '/launch/upload_robot.launch.py']
        ),
        launch_arguments = {
            'namespace': namespace,
            'use_gazebo_sim': 'false'
        }.items()
    )

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('former_description'),
            'urdf/robot.urdf.xacro',
        ]),
        ' use_gazebo_sim:=', 'false'
    ])

    robot_description = {"robot_description": robot_description_content}

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('former_bringup'),
                'config/ekf.yaml'
            ]),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    robot_controllers = PathJoinSubstitution([
            FindPackageShare('former_bringup'),
            "config",
            "controllers_former.yaml"
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            robot_controllers
            ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        respawn=True,
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    load_base_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["base_controller", "--controller-manager", "/controller_manager"],
    )

    load_former_io_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["former_io_controller", "--controller-manager", "/controller_manager"],
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
        remappings=[
            ('front_lidar/scan', 'scan')
        ]
    )

    auto_docking_bringup = Node(
        package="former_auto_docking",
        executable="auto_docking_node",
        respawn=True,
        parameters=[
            {"distance_approach": 0.260},
        ],
        remappings=[
            ('odom', 'base_controller/odom'),
            ('cmd_vel', 'base_controller/cmd_vel_unstamped')
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        }
    )

    gpio_board_bringup = Node(
        package="former_gpio_board",
        executable="main_node",
        respawn=True,
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
        parameters=[{
            'dev': '/dev/js0',
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }],
    )

    teleop_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('former_bringup'),
                'config/ps5.config.yaml'
            ]),
        ],
        remappings=[
            ('cmd_vel', 'base_controller/cmd_vel_unstamped')
        ]
    )

    bringup_cmd_group = GroupAction([
        PushRosNamespace(namespace=namespace),
        control_node,
        load_joint_state_broadcaster,
        load_base_controller,
        load_former_io_controller,
        # robot_localization_node,
        lidar_bringup,
        imu_bringup,
        gpio_board_bringup,
        auto_docking_bringup,
        joy_node,
        teleop_joy_node
    ])

    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_sim_time,
        upload_robot,
        bringup_cmd_group
    ])
