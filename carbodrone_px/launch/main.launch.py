import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    rviz = os.path.join(
        get_package_share_directory('carbodrone_px'),
        'rviz/main.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'description.launch.py']),
            launch_arguments={
                'urdf_package': 'carbodrone_px',
                'urdf_package_path': PathJoinSubstitution(['models', 'x500', 'model.urdf'])}.items()
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
