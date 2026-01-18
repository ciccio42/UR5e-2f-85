import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    teleop_pkg = get_package_share_directory('ur5e_2f_85_teleoperation')

    teleop_conf_file_default = os.path.join(
        teleop_pkg,
        'config',
        'tele_node_config.yaml'
    )

    teleop_conf_arg = DeclareLaunchArgument(
        'teleop_conf_file',
        default_value=TextSubstitution(text=teleop_conf_file_default),
        description='Path to the teleoperation YAML config file'
    )

    teleop_conf_file = LaunchConfiguration('teleop_conf_file')

    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    # Teleoperation node
    teleoperation_node = Node(
        package='ur5e_2f_85_teleoperation',
        executable='teleoperation',
        name='teleoperation',
        parameters=[teleop_conf_file],
        output='screen'
    )

    return LaunchDescription([
        teleop_conf_arg,
        joy_node,
        teleoperation_node
    ])
