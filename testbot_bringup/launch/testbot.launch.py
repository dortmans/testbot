from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, LogInfo,
                            SetEnvironmentVariable, ExecuteProcess)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  FindExecutable)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace, SetParameter

ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    ),
    DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    ),
    DeclareLaunchArgument(
        'ip',
        default_value='0.0.0.0',
        description='Robot IP-address'
    ),
    DeclareLaunchArgument(
        'domain',
        default_value='1',
        description='ROS_DOMAIN_ID'
    ),
    DeclareLaunchArgument(
        'protocol',
        default_value='udp',
        description='Zenoh Bridge Protocol (udp or tcp)'
    ),
    DeclareLaunchArgument(
        'mode',
        default_value='peer',
        description='Zenoh Bridge Mode (peer or client)'
    ),
]


def generate_launch_description():

    zenoh_bridge_config_path = PathJoinSubstitution(
        [FindPackageShare('testbot_bringup'), 'config',
         'zenoh-bridge-ros2dds.json5']
    )

    robot_launch_path = PathJoinSubstitution(
        [FindPackageShare('dummy_robot_bringup'), 'launch',
         'dummy_robot_bringup.launch.py']
    )

    cam2image_params = PathJoinSubstitution(
        [FindPackageShare('testbot_bringup'), 'config',
         'cam2image_params.yaml']
    )

    log_launching = LogInfo(msg="Launching Testbot")

    set_rmw_implementation = SetEnvironmentVariable(
        name='RMW_IMPLEMENTATION', value='rmw_cyclonedds_cpp')

    set_cyclonedds_uri = SetEnvironmentVariable(
        name='CYCLONEDDS_URI',
        value=PathJoinSubstitution(
            [FindPackageShare('testbot_bringup'), 'config', 'cyclonedds.xml']))

    set_ros_local_host_only = SetEnvironmentVariable(
        name='ROS_LOCALHOST_ONLY', value='0')

    set_ros_domain_id = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID', value=LaunchConfiguration('domain'))

    set_use_sim_time = SetParameter(
        name='use_sim_time', value=LaunchConfiguration('use_sim_time'))

    launch_zenoh_bridge = ExecuteProcess(
        cmd=[FindExecutable(name='zenoh-bridge-ros2dds'),
             '-c',
             zenoh_bridge_config_path,
             '-m',
             LaunchConfiguration('mode'),
             '-l',
             [LaunchConfiguration('protocol'), '/',
              LaunchConfiguration('ip'), ':7447'],
             '--no-multicast-scouting'
             ],
        shell=True)

    launch_robot_nodes = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch_path)),

        Node(
            package='image_tools',
            name='cam2image',
            executable='cam2image',
            remappings=[('/image', '/image')],
            parameters=[cam2image_params],
            output='screen'
        )
    ])

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(log_launching)
    ld.add_action(set_rmw_implementation)
    ld.add_action(set_cyclonedds_uri)
    ld.add_action(set_ros_local_host_only)
    ld.add_action(set_ros_domain_id)
    ld.add_action(set_use_sim_time)
    ld.add_action(launch_zenoh_bridge)
    ld.add_action(launch_robot_nodes)

    return ld
