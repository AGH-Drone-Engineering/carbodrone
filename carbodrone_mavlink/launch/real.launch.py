import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import AnyLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    urdf_path = PathJoinSubstitution([FindPackageShare('carbodrone_common'), 'models', 'x500', 'model.urdf'])
    robot_description_content = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    mavros_config_path = PathJoinSubstitution([FindPackageShare('carbodrone_mavlink'), 'launch', 'apm_config.yaml'])
    mavros_pluginlists_path = PathJoinSubstitution([FindPackageShare('carbodrone_mavlink'), 'launch', 'apm_pluginlists.yaml'])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
            }],
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('mavros'),
                    'launch', 'node.launch'
                ])
            ]),
            launch_arguments={
                'fcu_url': 'serial:///dev/ttyAMA0:57600',
                'gcs_url': '',
                'tgt_system': '1',
                'tgt_component': '1',
                'config_yaml': mavros_config_path,
                'pluginlists_yaml': mavros_pluginlists_path,
            }.items(),
        ),
    ])
