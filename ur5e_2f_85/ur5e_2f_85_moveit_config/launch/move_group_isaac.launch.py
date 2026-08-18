from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "ur5e_2f_85",
            package_name="ur5e_2f_85_moveit_config",
        )
        .robot_description(
            file_path="config/ur5e_2f_85.isaac.urdf.xacro"
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml"
        )
        .to_moveit_configs()
    )

    # Isaac integration: disable the legacy depth-camera OctoMap
    # configuration during arm planning tests.
    moveit_config.sensors_3d = {}

    return generate_move_group_launch(moveit_config)