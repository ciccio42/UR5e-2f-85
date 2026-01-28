import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import TimerAction


# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

# =========================
# Default paths
# =========================

default_config_common = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'config',
    'common_stereo.yaml'
)

default_camera_config = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'config',
    'zed2.yaml'
)

default_xacro_path = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'urdf',
    'zed_descr.urdf.xacro'
)

default_cameras_yaml = os.path.join(
    get_package_share_directory('zed_camera_driver'),
    'config',
    'multi_cameras.yaml'
)

# =========================
# Launch setup
# =========================

def launch_setup(context, *args, **kwargs):

    actions = []

    camera_model = LaunchConfiguration('camera_model')
    zed_node_name = LaunchConfiguration('zed_node_name')

    config_common_path = LaunchConfiguration('config_path')
    config_camera_path = LaunchConfiguration('config_camera_path')

    publish_urdf = LaunchConfiguration('publish_urdf')
    publish_tf = LaunchConfiguration('publish_tf')
    publish_map_tf = LaunchConfiguration('publish_map_tf')
    publish_imu_tf = LaunchConfiguration('publish_imu_tf')
    gravity_alignment = LaunchConfiguration('gravity_alignment')
    xacro_path = LaunchConfiguration('xacro_path')
    svo_path = LaunchConfiguration('svo_path')

    start_rviz = LaunchConfiguration('rviz')
    cameras_yaml = LaunchConfiguration('cameras_yaml')

    camera_model_val = camera_model.perform(context)

    # =========================
    # Load cameras YAML
    # =========================

    with open(cameras_yaml.perform(context), 'r') as f:
        cameras_config = yaml.safe_load(f)

    cameras = cameras_config.get('cameras', [])
    if not cameras:
        raise RuntimeError('No cameras defined in cameras YAML file')

    # =========================
    # RViz (once)
    # =========================

    rviz_node = Node(
        condition=IfCondition(start_rviz),
        package='rviz2',
        executable='rviz2',
        name='zed_multi_rviz2',
        output='screen'
    )
    actions.append(rviz_node)

    # =========================
    # Cameras loop
    # =========================

    for idx, cam in enumerate(cameras):

        camera_name = cam['camera_name']
        serial_number = int(cam['serial_number'])

        # Robot State Publisher (can start immediately)
        rsp_node = Node(
            condition=IfCondition(publish_urdf),
            package='robot_state_publisher',
            namespace=camera_name,
            executable='robot_state_publisher',
            name='zed_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    'xacro ',
                    xacro_path,
                    ' camera_name:=', camera_name,
                    ' camera_model:=', camera_model_val
                ])
            }]
        )

        # ZED component
        zed_component = ComposableNode(
            package='zed_components',
            namespace=camera_name,
            plugin='stereolabs::ZedCamera',
            name=zed_node_name,
            parameters=[
                config_common_path,
                config_camera_path,
                {
                    'general.camera_name': camera_name,
                    'general.camera_model': camera_model_val,
                    'general.serial_number': serial_number,
                    'svo.svo_path': svo_path,
                    'pos_tracking.publish_tf': publish_tf,
                    'pos_tracking.publish_map_tf': publish_map_tf,
                    'sensors.publish_imu_tf': publish_imu_tf,
                    'pos_tracking.set_gravity_as_origin': gravity_alignment,

                    # Optional but helpful
                    'general.grab_timeout': 10000
                }
            ],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

        container = ComposableNodeContainer(
            name=f'{camera_name}_container',
            namespace=camera_name,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[zed_component],
            output='screen'
        )

        # --- STAGGERED STARTUP ---
        delay_sec = idx * 3.0   # 3 seconds between cameras

        delayed_container = TimerAction(
            period=delay_sec,
            actions=[container]
        )

        actions.append(rsp_node)
        actions.append(delayed_container)



    return actions


# =========================
# Launch description
# =========================

def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            'camera_model',
            description='ZED camera model',
            choices=['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'virtual'],
        ),

        DeclareLaunchArgument(
            'zed_node_name',
            default_value='zed_node'
        ),

        DeclareLaunchArgument(
            'config_path',
            default_value=TextSubstitution(text=default_config_common)
        ),

        DeclareLaunchArgument(
            'config_camera_path',
            default_value=TextSubstitution(text=default_camera_config)
        ),

        DeclareLaunchArgument(
            'cameras_yaml',
            default_value=TextSubstitution(text=default_cameras_yaml)
        ),

        DeclareLaunchArgument('publish_urdf', default_value='true'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('publish_map_tf', default_value='true'),
        DeclareLaunchArgument('publish_imu_tf', default_value='true'),
        DeclareLaunchArgument('gravity_alignment', default_value='false'),

        DeclareLaunchArgument(
            'xacro_path',
            default_value=TextSubstitution(text=default_xacro_path)
        ),

        DeclareLaunchArgument('svo_path', default_value='live'),
        DeclareLaunchArgument('rviz', default_value='true'),

        OpaqueFunction(function=launch_setup),
    ])
