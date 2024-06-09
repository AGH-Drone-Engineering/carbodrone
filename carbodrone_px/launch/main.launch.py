import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    urdf = os.path.join(
        get_package_share_directory('carbodrone_px'),
        "models/x500_depth/model.sdf")
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

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
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='carbodrone_px',
            executable='odom_publisher',
            name='odom_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='parameter_bridge',
            output='screen',
            arguments=[
                '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
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
                '/camera',
                '/bottom_camera',
                '/depth_camera',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        ExecuteProcess(
            cmd=['QGroundControl'],
            name='QGroundControl',
        ),

        Node(
            package='carbodrone_px',
            executable='imu',
            name='imu',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
