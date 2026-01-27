import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution

from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

# =========================
# Default configuration paths
# =========================

default_config_common = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'config',
    'common_stereo.yaml'
)

default_config_aruco = os.path.join(
    get_package_share_directory('zed_camera_calibration'),
    'config',
    'aruco_frontal_camera.yaml'
)

default_xacro_path = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'urdf',
    'zed_descr.urdf.xacro'
)

default_cameras_yaml = os.path.join(
    get_package_share_directory('zed_camera_calibration'),
    'config',
    'multi_cameras.yaml'
)

# =========================
# Launch setup
# =========================

def launch_setup(context, *args, **kwargs):

    actions = []

    # Launch configurations
    camera_model = LaunchConfiguration('camera_model')
    zed_node_name = LaunchConfiguration('zed_node_name')
    aruco_node_name = LaunchConfiguration('aruco_node_name')

    config_common_path = LaunchConfiguration('config_path')
    config_camera_path = LaunchConfiguration('config_camera_path')
    config_path_aruco = LaunchConfiguration('config_path_aruco')

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
    # Load cameras from YAML
    # =========================

    cameras_yaml_path = cameras_yaml.perform(context)

    with open(cameras_yaml_path, 'r') as f:
        cameras_config = yaml.safe_load(f)

    cameras = cameras_config.get('cameras', [])

    if not cameras:
        raise RuntimeError('No cameras defined in cameras YAML file')

    # =========================
    # RViz (only once)
    # =========================

    rviz2_node = Node(
        condition=IfCondition(start_rviz),
        package='rviz2',
        executable='rviz2',
        name='zed_multi_rviz2',
        output='screen',
        arguments=[
            '-d',
            os.path.join(
                get_package_share_directory('zed_aruco_localization'),
                'rviz2',
                'aruco.rviz'
            )
        ],
    )
    actions.append(rviz2_node)

    # =========================
    # Multi-camera loop
    # =========================

    for cam in cameras:

        camera_name = cam['camera_name']
        serial_number = int(cam['serial_number'])

        # Robot State Publisher
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

        # ZED wrapper component
        zed_wrapper_component = ComposableNode(
            package='zed_components',
            namespace=camera_name,
            plugin='stereolabs::ZedCamera',
            name=zed_node_name,
            parameters=[
                config_common_path,
                config_camera_path,
                config_path_aruco,
                {
                    'general.camera_name': camera_name,
                    'general.camera_model': camera_model_val,
                    'general.serial_number': int(serial_number),
                    'svo.svo_path': svo_path,
                    'pos_tracking.publish_tf': publish_tf,
                    'pos_tracking.publish_map_tf': publish_map_tf,
                    'sensors.publish_imu_tf': publish_imu_tf,
                    'pos_tracking.set_gravity_as_origin': gravity_alignment
                }
            ],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

        # ArUco localization component
        zed_aruco_component = ComposableNode(
            package='zed_aruco_localization',
            namespace=camera_name,
            plugin='stereolabs::ZedArucoLoc',
            name=aruco_node_name,
            parameters=[config_path_aruco,    
                        {
                            'general.camera_name': camera_name
                        }
                    ],
            remappings=[
                ('in/zed_image',
                 zed_node_name.perform(context) + '/left/color/rect/image'),
                ('in/camera_info',
                 zed_node_name.perform(context) + '/left/color/rect/camera_info'),
                ('set_pose',
                 zed_node_name.perform(context) + '/set_pose')
            ],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

        # Component container
        
        container = ComposableNodeContainer(
            name=f'{camera_name}_container',
            namespace=camera_name,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                zed_wrapper_component,
                zed_aruco_component
            ],
            output='screen'
        )

        actions.extend([rsp_node, container])

    return actions


# =========================
# Generate launch description
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
            'aruco_node_name',
            default_value='aruco_node'
        ),

        DeclareLaunchArgument(
            'config_path',
            default_value=TextSubstitution(text=default_config_common)
        ),

        DeclareLaunchArgument(
            'config_camera_path',
            default_value=TextSubstitution(
                text=os.path.join(
                    get_package_share_directory('zed_wrapper'),
                    'config',
                    'zed2.yaml'
                )
            )
        ),

        DeclareLaunchArgument(
            'config_path_aruco',
            default_value=TextSubstitution(text=default_config_aruco)
        ),

        DeclareLaunchArgument(
            'cameras_yaml',
            default_value=TextSubstitution(text=default_cameras_yaml),
            description='YAML file describing multiple ZED cameras'
        ),

        DeclareLaunchArgument(
            'publish_urdf',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'publish_tf',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'publish_map_tf',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'publish_imu_tf',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'gravity_alignment',
            default_value='false'
        ),

        DeclareLaunchArgument(
            'xacro_path',
            default_value=TextSubstitution(text=default_xacro_path)
        ),

        DeclareLaunchArgument(
            'svo_path',
            default_value='live'
        ),

        DeclareLaunchArgument(
            'rviz',
            default_value='true'
        ),

        OpaqueFunction(function=launch_setup),
    ])
