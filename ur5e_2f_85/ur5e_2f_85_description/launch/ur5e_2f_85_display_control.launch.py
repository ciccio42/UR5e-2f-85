from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from launch import LaunchDescription


def generate_launch_description():
    
    ur_type = LaunchConfiguration("ur_type", default=TextSubstitution(text="ur5e"))
    robot_ip = LaunchConfiguration("robot_ip", default=TextSubstitution(text="192.168.56.101"))


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ur_robot_driver'),
                    'launch',
                    'ur_rsp.launch.py'
                ])
            ]),
            launch_arguments={
                'ur_type': ur_type,
                'robot_ip': robot_ip,
                'description_file': PathJoinSubstitution([
                    FindPackageShare("ur5e_2f_85_description"),
                    'urdf',
                    'ur5e_2f_85_platform_control.urdf.xacro'
                ]),
            }.items()
        )
    ])