from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'dataset_collector_pkg'

    config_file = LaunchConfiguration('config_file')
    default_config_file = PathJoinSubstitution([
        get_package_share_directory(package_name),
        'config',
        'replicate_trajectory_config.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='Path to the replicate trajectory YAML config file.',
        ),
        Node(
            package=package_name,
            executable='replicate_trajectory',
            name='replicate_trajectory',
            output='screen',
            parameters=[config_file],
        ),
    ])
