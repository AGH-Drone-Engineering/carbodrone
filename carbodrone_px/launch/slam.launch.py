import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    carto_config_dir = os.path.join(
        get_package_share_directory('carbodrone_px'),
        'config')
    carto_config_file = 'carto.lua'

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments = [
                '-configuration_directory', carto_config_dir,
                '-configuration_basename', carto_config_file,
            ],
            remappings=[('points2', '/depth_camera/points')],
        ),
    ])
