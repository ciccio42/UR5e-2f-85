from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("ur5e_2f_85_description"),
            "urdf",
            "ur5e_2f_85_platform.urdf.xacro"
        ]),
        " safety_limits:=true",
        " ur_type:=ur5e",
        " parent:=world",
        " use_fake_hardware:=true",
    ])

    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content, value_type=str
        )
    }

    return LaunchDescription([
        Node(
            package="moveit_setup_assistant",
            executable="moveit_setup_assistant",
            parameters=[robot_description],
            output="screen",
        )
    ])
