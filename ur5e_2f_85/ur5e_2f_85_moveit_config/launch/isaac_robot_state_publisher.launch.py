import os
import xml.etree.ElementTree as ET

import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():

    package_share = get_package_share_directory(
        "ur5e_2f_85_moveit_config"
    )

    xacro_file = os.path.join(
        package_share,
        "config",
        "ur5e_2f_85.isaac.urdf.xacro",
    )

    initial_positions_file = os.path.join(
        package_share,
        "config",
        "initial_positions_isaac.yaml",
    )

    # ---------------------------------------------------------
    # Process Isaac-specific Xacro
    # ---------------------------------------------------------

    doc = xacro.process_file(
        xacro_file,
        mappings={
            "initial_positions_file": initial_positions_file
        },
    )

    robot_description = doc.toxml()

    # ---------------------------------------------------------
    # Remove legacy Robotiq ros2_control block
    #
    # The flattened robot description already contains:
    #
    #   <ros2_control name="RobotiqGripperHardwareInterface">
    #
    # Isaac instead uses the gripper through IsaacSystem /
    # TopicBasedSystem, therefore the legacy hardware interface
    # must not be present.
    # ---------------------------------------------------------

    root = ET.fromstring(robot_description)

    removed = False

    for elem in list(root):
        if (
            elem.tag == "ros2_control"
            and elem.get("name") == "RobotiqGripperHardwareInterface"
        ):
            root.remove(elem)
            removed = True

    robot_description = ET.tostring(
        root,
        encoding="unicode",
    )

    # ---------------------------------------------------------
    # robot_state_publisher
    # ---------------------------------------------------------

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
            }
        ],
    )

    status = (
        "Removed legacy RobotiqGripperHardwareInterface "
        "from Isaac robot description."
        if removed
        else
        "WARNING: legacy RobotiqGripperHardwareInterface "
        "was not found."
    )

    return LaunchDescription([
        LogInfo(msg=status),
        robot_state_publisher,
    ])