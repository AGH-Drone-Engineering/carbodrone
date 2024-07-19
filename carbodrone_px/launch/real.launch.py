from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    urdf_path = PathJoinSubstitution([FindPackageShare('carbodrone_px'), 'models', 'x500', 'model.urdf'])
    robot_description_content = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

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

        Node(
            package='carbodrone_px',
            executable='odom_publisher',
            name='odom_publisher',
            output='screen',
            respawn=True,
        ),

        Node(
            package='carbodrone_px',
            executable='imu',
            name='imu',
            output='screen',
            respawn=True,
        ),

        Node(
            package='carbodrone_px',
            executable='ground_pub',
            name='ground_pub',
            output='screen',
            respawn=True,
        ),
    ])
