from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.substitutions import FindPackageShare, PathJoinSubstitution


def generate_launch_description():

    ld = LaunchDescription()

    use_gazebo_sim = DeclareLaunchArgument("use_gazebo_sim", default_value="false")
    tf_prefix = DeclareLaunchArgument("tf_prefix", default_value="")
    namespace = DeclareLaunchArgument("namespace", default_value="")

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('/diagnostics', 'diagnostics')]

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        remappings=remappings,
        parameters=[{
            'ignore_timestamp': False,
            'robot_description':
                Command([
                    'xacro ',
                    PathJoinSubstitution([
                        FindPackageShare('former_description'),
                        'urdf/robot.urdf.xacro',
                    ]),
                    ' use_gazebo_sim:=', LaunchConfiguration('use_gazebo_sim'),
                    ' namespace:=', LaunchConfiguration('namespace'),
                    ' tf_prefix:=', LaunchConfiguration('tf_prefix'),

                ]),
            'use_time_time': LaunchConfiguration('use_gazebo_sim'),
        }]
    )

    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            "source_list": ['joint_states'],
            "rate": 50.0,
            "use_sim_time": LaunchConfiguration('use_gazebo_sim')
        }]
    )

    ld.add_action(use_gazebo_sim)
    ld.add_action(tf_prefix)
    ld.add_action(namespace)
    ld.add_action(rsp_node)
    ld.add_action(jsp_node)

    return ld
