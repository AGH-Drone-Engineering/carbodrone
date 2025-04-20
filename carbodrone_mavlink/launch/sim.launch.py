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

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    rviz = os.path.join(
        get_package_share_directory('carbodrone_common'),
        'rviz/main.rviz')

    urdf_path = PathJoinSubstitution([FindPackageShare('carbodrone_common'), 'models', 'x500', 'model.urdf'])
    robot_description_content = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    mavros_config_path = PathJoinSubstitution([FindPackageShare('carbodrone_mavlink'), 'launch', 'px4_config.yaml'])
    mavros_pluginlists_path = PathJoinSubstitution([FindPackageShare('carbodrone_mavlink'), 'launch', 'px4_pluginlists.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description_content,
            }],
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('mavros'),
                    'launch', 'node.launch'
                ])
            ]),
            launch_arguments={
                'fcu_url': 'udp://:14540@127.0.0.1:14557',
                'gcs_url': '',
                'tgt_system': '1',
                'tgt_component': '1',
                'config_yaml': mavros_config_path,
                'pluginlists_yaml': mavros_pluginlists_path,
            }.items(),
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='parameter_bridge',
            output='screen',
            arguments=[
                '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/depth_camera/image/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='image_bridge',
            output='screen',
            arguments=[
                '/camera/image',
                '/depth_camera/image',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        ExecuteProcess(
            cmd=['/bin/bash', '-c', 'QT_QPA_PLATFORM=xcb QGroundControl'],
            name='QGroundControl',
        ),
