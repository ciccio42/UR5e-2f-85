#!/usr/bin/env python3

from __future__ import annotations

import argparse
import copy
import json
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np
import rclpy
import yaml

from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import JointState
from tf2_ros import StaticTransformBroadcaster


# =====================================================================
# Repository / helper imports
# =====================================================================

REPO_ROOT = Path(
    "/home/ros2_ws/src/UR5e-2f-85"
)

SEEDO_DIR = (
    REPO_ROOT
    / "ai_controller"
    / "ai_controller"
    / "models"
    / "seedo_controller"
)

if str(SEEDO_DIR) not in sys.path:
    sys.path.insert(
        0,
        str(SEEDO_DIR),
    )

from inspect_pkl import (  # noqa: E402
    get_trajectory_step,
    load_pickle,
)

from ai_controller.ai_controller_node import (  # noqa: E402
    AIControllerNode,
)

from ai_controller.models.seedo_controller.ai_controller_node_utils import (  # noqa: E402
    get_seedo_base_to_table_transform,
    get_seedo_grasp_runtime_input,
    transform_dict_to_matrix,
)

from ai_controller.utils.utils import (  # noqa: E402
    EEF_POS_NAME,
    EEF_QUAT_NAME,
)


# =====================================================================
# Defaults
# =====================================================================

DEFAULT_GRASP_PKL = Path(
    "/scene_capture/traj_001.pkl"
)

DEFAULT_GRASP_STEP = 16

DEFAULT_ARTIFACTS_DIR = Path(
    "/seedo_tests/"
    "ai_controller_node_grasp_pipeline"
)

DEFAULT_ROLLOUTS_DIR = (
    DEFAULT_ARTIFACTS_DIR
    / "rollouts"
)

DEFAULT_EXPECTED_PRIMITIVES = (
    "reach",
    "approaching",
    "pick",
    "lift_up",
    "moving",
    "placing",
)

EXPECTED_M2T2_GRIPPER_DEPTH_M = 0.1034


# =====================================================================
# Eye-in-hand calibration
#
# Same calibration used by the already validated offline
# test_grasp_planner.py.
# =====================================================================

DEFAULT_GRIPPER_K = np.array(
    [
        [
            363.8071594238281,
            0.0,
            335.86553955078125,
        ],
        [
            0.0,
            363.8071594238281,
            183.81210327148438,
        ],
        [
            0.0,
            0.0,
            1.0,
        ],
    ],
    dtype=np.float64,
)


# tcp_link -> eye-in-hand optical frame.
T_TCP_CAMERA = np.array(
    [
        [
            0.999262529,
            -0.0383979045,
            0.0,
            -0.0209004103,
        ],
        [
            0.0383979045,
            0.999262529,
            0.0,
            -0.0681027558,
        ],
        [
            0.0,
            0.0,
            1.0,
            -0.1455,
        ],
        [
            0.0,
            0.0,
            0.0,
            1.0,
        ],
    ],
    dtype=np.float64,
)


CAMERA_NAMES = (
    "camera_front",
    "camera_lateral_left",
    "camera_lateral_right",
    "eye_in_hand",
)

EXPECTED_ROLLOUT_KEYS = {
    "camera_front_image",
    "camera_front_depth",
    "camera_lateral_left_image",
    "camera_lateral_left_depth",
    "camera_lateral_right_image",
    "camera_lateral_right_depth",
    "eye_in_hand_image",
    "eye_in_hand_depth",
}


# =====================================================================
# Geometry helpers
# =====================================================================

def _quaternion_to_rotation(
    quaternion: np.ndarray,
) -> np.ndarray:
    quaternion = np.asarray(
        quaternion,
        dtype=np.float64,
    )

    if quaternion.shape != (4,):
        raise ValueError(
            "Quaternion must have shape (4,)."
        )

    norm = np.linalg.norm(
        quaternion
    )

    if norm <= 1e-12:
        raise ValueError(
            "Quaternion has near-zero norm."
        )

    x, y, z, w = (
        quaternion
        / norm
    )

    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def _rotation_matrix_to_quaternion(
    rotation: np.ndarray,
) -> np.ndarray:
    rotation = np.asarray(
        rotation,
        dtype=np.float64,
    )

    if rotation.shape != (3, 3):
        raise ValueError(
            "Rotation matrix must have shape (3, 3)."
        )

    trace = float(
        np.trace(
            rotation
        )
    )

    if trace > 0.0:
        s = np.sqrt(
            trace + 1.0
        ) * 2.0

        w = 0.25 * s
        x = (
            rotation[2, 1]
            - rotation[1, 2]
        ) / s
        y = (
            rotation[0, 2]
            - rotation[2, 0]
        ) / s
        z = (
            rotation[1, 0]
            - rotation[0, 1]
        ) / s

    elif (
        rotation[0, 0]
        > rotation[1, 1]
        and rotation[0, 0]
        > rotation[2, 2]
    ):
        s = np.sqrt(
            1.0
            + rotation[0, 0]
            - rotation[1, 1]
            - rotation[2, 2]
        ) * 2.0

        w = (
            rotation[2, 1]
            - rotation[1, 2]
        ) / s

        x = 0.25 * s

        y = (
            rotation[0, 1]
            + rotation[1, 0]
        ) / s

        z = (
            rotation[0, 2]
            + rotation[2, 0]
        ) / s

    elif (
        rotation[1, 1]
        > rotation[2, 2]
    ):
        s = np.sqrt(
            1.0
            + rotation[1, 1]
            - rotation[0, 0]
            - rotation[2, 2]
        ) * 2.0

        w = (
            rotation[0, 2]
            - rotation[2, 0]
        ) / s

        x = (
            rotation[0, 1]
            + rotation[1, 0]
        ) / s

        y = 0.25 * s

        z = (
            rotation[1, 2]
            + rotation[2, 1]
        ) / s

    else:
        s = np.sqrt(
            1.0
            + rotation[2, 2]
            - rotation[0, 0]
            - rotation[1, 1]
        ) * 2.0

        w = (
            rotation[1, 0]
            - rotation[0, 1]
        ) / s

        x = (
            rotation[0, 2]
            + rotation[2, 0]
        ) / s

        y = (
            rotation[1, 2]
            + rotation[2, 1]
        ) / s

        z = 0.25 * s

    quaternion = np.array(
        [
            x,
            y,
            z,
            w,
        ],
        dtype=np.float64,
    )

    quaternion /= np.linalg.norm(
        quaternion
    )

    return quaternion


def _pose_to_transform(
    position: np.ndarray,
    quaternion: np.ndarray,
) -> np.ndarray:
    transform = np.eye(
        4,
        dtype=np.float64,
    )

    transform[
        :3,
        :3,
    ] = _quaternion_to_rotation(
        quaternion
    )

    transform[
        :3,
        3,
    ] = np.asarray(
        position,
        dtype=np.float64,
    )

    return transform


def _transform_to_msg(
    transform_matrix: np.ndarray,
    *,
    parent_frame: str,
    child_frame: str,
    stamp,
) -> TransformStamped:
    transform_matrix = np.asarray(
        transform_matrix,
        dtype=np.float64,
    )

    if transform_matrix.shape != (4, 4):
        raise ValueError(
            "Transform matrix must have shape (4, 4)."
        )

    quaternion = (
        _rotation_matrix_to_quaternion(
            transform_matrix[
                :3,
                :3,
            ]
        )
    )

    translation = (
        transform_matrix[
            :3,
            3,
        ]
    )

    msg = TransformStamped()

    msg.header.stamp = stamp
    msg.header.frame_id = parent_frame
    msg.child_frame_id = child_frame

    msg.transform.translation.x = float(
        translation[0]
    )

    msg.transform.translation.y = float(
        translation[1]
    )

    msg.transform.translation.z = float(
        translation[2]
    )

    msg.transform.rotation.x = float(
        quaternion[0]
    )

    msg.transform.rotation.y = float(
        quaternion[1]
    )

    msg.transform.rotation.z = float(
        quaternion[2]
    )

    msg.transform.rotation.w = float(
        quaternion[3]
    )

    return msg


def _load_transform_yaml(
    path: Path,
) -> np.ndarray:
    with path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        data = yaml.safe_load(
            stream
        )

    transform = np.eye(
        4,
        dtype=np.float64,
    )

    transform[
        :3,
        :3,
    ] = np.asarray(
        data[
            "rotation"
        ],
        dtype=np.float64,
    )

    transform[
        :3,
        3,
    ] = np.asarray(
        data[
            "translation"
        ],
        dtype=np.float64,
    )

    return transform


# =====================================================================
# PKL helpers
# =====================================================================

def _get_eye_in_hand_rgbd(
    observation: dict,
) -> tuple[
    np.ndarray,
    np.ndarray,
]:
    candidates = (
        (
            "eye_in_hand_image",
            "eye_in_hand_depth",
        ),
        (
            "camera_gripper_image",
            "camera_gripper_depth",
        ),
    )

    for (
        image_key,
        depth_key,
    ) in candidates:

        if (
            image_key in observation
            and depth_key in observation
        ):
            image = np.asarray(
                observation[
                    image_key
                ]
            )

            depth = np.asarray(
                observation[
                    depth_key
                ],
                dtype=np.float32,
            )

            if (
                image.ndim != 3
                or image.shape[2] != 3
            ):
                raise RuntimeError(
                    "Invalid eye-in-hand image shape: "
                    f"{image.shape}"
                )

            if depth.ndim != 2:
                raise RuntimeError(
                    "Invalid eye-in-hand depth shape: "
                    f"{depth.shape}"
                )

            return (
                image,
                depth,
            )

    raise RuntimeError(
        "No eye-in-hand RGB-D pair found "
        "in the selected PKL observation."
    )


def _load_grasp_fixture(
    pkl_path: Path,
    step_index: int,
) -> dict:
    data = load_pickle(
        pkl_path
    )

    if "traj" not in data:
        raise RuntimeError(
            "Grasp PKL does not contain 'traj'."
        )

    trajectory = data[
        "traj"
    ]

    if (
        step_index < 0
        or step_index >= len(
            trajectory
        )
    ):
        raise IndexError(
            "Grasp step is outside trajectory range: "
            f"step={step_index}, length={len(trajectory)}"
        )

    step = get_trajectory_step(
        trajectory,
        step_index,
    )

    observation = step.get(
        "obs"
    )

    if not isinstance(
        observation,
        dict,
    ):
        raise RuntimeError(
            "Selected grasp trajectory step "
            "does not contain an observation dictionary."
        )

    (
        image_bgr,
        depth,
    ) = _get_eye_in_hand_rgbd(
        observation
    )

    if EEF_POS_NAME not in observation:
        raise RuntimeError(
            "Grasp PKL observation does not contain "
            f"{EEF_POS_NAME!r}."
        )

    if EEF_QUAT_NAME not in observation:
        raise RuntimeError(
            "Grasp PKL observation does not contain "
            f"{EEF_QUAT_NAME!r}."
        )

    eef_position = np.asarray(
        observation[
            EEF_POS_NAME
        ],
        dtype=np.float64,
    )

    eef_orientation = np.asarray(
        observation[
            EEF_QUAT_NAME
        ],
        dtype=np.float64,
    )

    if eef_position.shape != (3,):
        raise RuntimeError(
            "PKL EEF position must have shape (3,)."
        )

    if eef_orientation.shape != (4,):
        raise RuntimeError(
            "PKL EEF orientation must have shape (4,)."
        )

    return {
        "image_bgr": image_bgr,
        "image_rgb": cv2.cvtColor(
            image_bgr,
            cv2.COLOR_BGR2RGB,
        ),
        "depth": depth,
        "eef_position": eef_position,
        "eef_orientation": eef_orientation,
    }


# =====================================================================
# CameraInfo helpers
# =====================================================================

def _load_front_camera_info(
    path: Path,
) -> dict:
    with path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        return yaml.safe_load(
            stream
        )


def _load_gripper_camera_matrix(
    path: Path | None,
) -> np.ndarray:
    if path is None:
        return (
            DEFAULT_GRIPPER_K.copy()
        )

    with path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        data = yaml.safe_load(
            stream
        )

    return np.asarray(
        data[
            "k"
        ],
        dtype=np.float64,
    ).reshape(
        3,
        3,
    )


def _build_camera_info_from_yaml(
    data: dict,
    *,
    stamp,
    frame_id: str,
) -> CameraInfo:
    msg = CameraInfo()

    msg.header.stamp = stamp
    msg.header.frame_id = frame_id

    msg.height = int(
        data[
            "height"
        ]
    )

    msg.width = int(
        data[
            "width"
        ]
    )

    msg.distortion_model = str(
        data.get(
            "distortion_model",
            "plumb_bob",
        )
    )

    msg.d = [
        float(value)
        for value
        in data.get(
            "d",
            [],
        )
    ]

    msg.k = [
        float(value)
        for value
        in data[
            "k"
        ]
    ]

    msg.r = [
        float(value)
        for value
        in data.get(
            "r",
            np.eye(
                3
            ).reshape(
                -1
            ),
        )
    ]

    msg.p = [
        float(value)
        for value
        in data.get(
            "p",
            [
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
            ],
        )
    ]

    return msg


def _build_gripper_camera_info(
    K: np.ndarray,
    *,
    width: int,
    height: int,
    stamp,
    frame_id: str,
) -> CameraInfo:
    K = np.asarray(
        K,
        dtype=np.float64,
    )

    msg = CameraInfo()

    msg.header.stamp = stamp
    msg.header.frame_id = frame_id

    msg.height = int(
        height
    )

    msg.width = int(
        width
    )

    msg.distortion_model = (
        "plumb_bob"
    )

    msg.d = [
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]

    msg.k = [
        float(value)
        for value
        in K.reshape(
            -1
        )
    ]

    msg.r = [
        float(value)
        for value
        in np.eye(
            3,
            dtype=np.float64,
        ).reshape(
            -1
        )
    ]

    msg.p = [
        float(K[0, 0]),
        float(K[0, 1]),
        float(K[0, 2]),
        0.0,
        float(K[1, 0]),
        float(K[1, 1]),
        float(K[1, 2]),
        0.0,
        float(K[2, 0]),
        float(K[2, 1]),
        float(K[2, 2]),
        0.0,
    ]

    return msg


# =====================================================================
# Complete simulated ROS runtime
# =====================================================================

class SeeDoGraspPipelinePublisher(
    Node
):
    """
    Publish all ROS data required by the SeeDo integration test.

    Front/left/right cameras:
        runtime scene fixture.

    Eye-in-hand camera:
        selected observation from the grasp test PKL.

    TF:
        base_link -> table
        base_link -> tcp_link
        tcp_link  -> eye-in-hand optical camera

    The TCP and eye-in-hand transform are intentionally pinned to
    the selected PKL observation. This reproduces the same grasp
    geometry already validated by test_grasp_planner.py while the
    AIControllerNode executes a no-robot dry-run.
    """

    def __init__(
        self,
        *,
        scene_dir: Path,
        base_to_table_path: Path,
        grasp_pkl_path: Path,
        grasp_step: int,
        gripper_camera_info_path: Path | None,
        rgb_topics: list[str],
        depth_topics: list[str],
        front_camera_info_topic: str,
        gripper_camera_info_topic: str,
        joint_states_topic: str,
        joint_robot_names: list[str],
        gripper_robot_names: list[str],
        base_frame: str,
        table_frame: str,
        eef_frame: str,
        gripper_optical_frame: str,
    ) -> None:
        super().__init__(
            "seedo_grasp_pipeline_test_publisher"
        )

        self.bridge = CvBridge()

        if len(
            rgb_topics
        ) != 4:
            raise ValueError(
                "Expected exactly four RGB topics."
            )

        if len(
            depth_topics
        ) != 4:
            raise ValueError(
                "Expected exactly four depth topics."
            )

        self.base_frame = (
            base_frame
        )

        self.table_frame = (
            table_frame
        )

        self.eef_frame = (
            eef_frame
        )

        self.gripper_optical_frame = (
            gripper_optical_frame
        )

        self.joint_robot_names = list(
            joint_robot_names
        )

        self.gripper_robot_names = list(
            gripper_robot_names
        )

        # ---------------------------------------------------------
        # Runtime front-scene fixture
        # ---------------------------------------------------------

        rgb_path = (
            scene_dir
            / "rgb.png"
        )

        depth_path = (
            scene_dir
            / "depth.npy"
        )

        camera_info_path = (
            scene_dir
            / "camera_info.yaml"
        )

        for path in (
            rgb_path,
            depth_path,
            camera_info_path,
            base_to_table_path,
            grasp_pkl_path,
        ):
            if not path.is_file():
                raise FileNotFoundError(
                    "Required test input does not exist: "
                    f"{path}"
                )

        runtime_bgr = cv2.imread(
            str(
                rgb_path
            ),
            cv2.IMREAD_COLOR,
        )

        if runtime_bgr is None:
            raise RuntimeError(
                "Could not load runtime RGB image: "
                f"{rgb_path}"
            )

        self.runtime_rgb = cv2.cvtColor(
            runtime_bgr,
            cv2.COLOR_BGR2RGB,
        )

        self.runtime_depth = np.asarray(
            np.load(
                depth_path
            ),
            dtype=np.float32,
        )

        self.front_camera_info_data = (
            _load_front_camera_info(
                camera_info_path
            )
        )

        # ---------------------------------------------------------
        # Eye-in-hand fixture from PKL
        # ---------------------------------------------------------

        self.grasp_fixture = (
            _load_grasp_fixture(
                grasp_pkl_path,
                grasp_step,
            )
        )

        self.grasp_rgb = (
            self.grasp_fixture[
                "image_rgb"
            ]
        )

        self.grasp_bgr = (
            self.grasp_fixture[
                "image_bgr"
            ]
        )

        self.grasp_depth = (
            self.grasp_fixture[
                "depth"
            ]
        )

        self.grasp_eef_position = (
            self.grasp_fixture[
                "eef_position"
            ]
        )

        self.grasp_eef_orientation = (
            self.grasp_fixture[
                "eef_orientation"
            ]
        )

        self.gripper_K = (
            _load_gripper_camera_matrix(
                gripper_camera_info_path
            )
        )

        # ---------------------------------------------------------
        # Static geometry
        # ---------------------------------------------------------

        self.T_base_table = (
            _load_transform_yaml(
                base_to_table_path
            )
        )

        self.T_base_tcp = (
            _pose_to_transform(
                self.grasp_eef_position,
                self.grasp_eef_orientation,
            )
        )

        self.T_base_camera = (
            self.T_base_tcp
            @ T_TCP_CAMERA
        )

        # ---------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------

        self.rgb_publishers = {
            camera_name: self.create_publisher(
                RosImage,
                topic,
                qos_profile_sensor_data,
            )
            for (
                camera_name,
                topic,
            )
            in zip(
                CAMERA_NAMES,
                rgb_topics,
                strict=True,
            )
        }

        self.depth_publishers = {
            camera_name: self.create_publisher(
                RosImage,
                topic,
                qos_profile_sensor_data,
            )
            for (
                camera_name,
                topic,
            )
            in zip(
                CAMERA_NAMES,
                depth_topics,
                strict=True,
            )
        }

        self.front_camera_info_publisher = (
            self.create_publisher(
                CameraInfo,
                front_camera_info_topic,
                qos_profile_sensor_data,
            )
        )

        self.gripper_camera_info_publisher = (
            self.create_publisher(
                CameraInfo,
                gripper_camera_info_topic,
                qos_profile_sensor_data,
            )
        )

        self.joint_state_publisher = (
            self.create_publisher(
                JointState,
                joint_states_topic,
                10,
            )
        )

        # ---------------------------------------------------------
        # Static TF
        # ---------------------------------------------------------

        self.static_tf_broadcaster = (
            StaticTransformBroadcaster(
                self
            )
        )

        self._publish_static_transforms()

        # Continuously publish sensor data.
        self.timer = self.create_timer(
            0.10,
            self._publish_runtime,
        )

    def _publish_static_transforms(
        self,
    ) -> None:
        stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        table_transform = (
            _transform_to_msg(
                self.T_base_table,
                parent_frame=(
                    self.base_frame
                ),
                child_frame=(
                    self.table_frame
                ),
                stamp=stamp,
            )
        )

        tcp_transform = (
            _transform_to_msg(
                self.T_base_tcp,
                parent_frame=(
                    self.base_frame
                ),
                child_frame=(
                    self.eef_frame
                ),
                stamp=stamp,
            )
        )

        camera_transform = (
            _transform_to_msg(
                T_TCP_CAMERA,
                parent_frame=(
                    self.eef_frame
                ),
                child_frame=(
                    self.gripper_optical_frame
                ),
                stamp=stamp,
            )
        )

        self.static_tf_broadcaster.sendTransform(
            [
                table_transform,
                tcp_transform,
                camera_transform,
            ]
        )

    def _build_joint_state(
        self,
        stamp,
    ) -> JointState:
        msg = JointState()

        msg.header.stamp = stamp

        msg.name = (
            self.joint_robot_names
            + self.gripper_robot_names
        )

        msg.position = [
            0.0
            for _ in msg.name
        ]

        msg.velocity = [
            0.0
            for _ in msg.name
        ]

        msg.effort = [
            0.0
            for _ in msg.name
        ]

        return msg

    def _image_pair_for_camera(
        self,
        camera_name: str,
    ) -> tuple[
        np.ndarray,
        np.ndarray,
    ]:
        if (
            camera_name
            == "eye_in_hand"
        ):
            return (
                self.grasp_rgb,
                self.grasp_depth,
            )

        return (
            self.runtime_rgb,
            self.runtime_depth,
        )

    def _publish_runtime(
        self,
    ) -> None:
        stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        for camera_name in CAMERA_NAMES:
            (
                rgb,
                depth,
            ) = (
                self._image_pair_for_camera(
                    camera_name
                )
            )

            rgb_msg = (
                self.bridge
                .cv2_to_imgmsg(
                    rgb,
                    encoding="rgb8",
                )
            )

            rgb_msg.header.stamp = (
                stamp
            )

            if (
                camera_name
                == "eye_in_hand"
            ):
                rgb_msg.header.frame_id = (
                    self.gripper_optical_frame
                )

            self.rgb_publishers[
                camera_name
            ].publish(
                rgb_msg
            )

            depth_msg = (
                self.bridge
                .cv2_to_imgmsg(
                    np.asarray(
                        depth,
                        dtype=np.float32,
                    ),
                    encoding="32FC1",
                )
            )

            depth_msg.header.stamp = (
                stamp
            )

            if (
                camera_name
                == "eye_in_hand"
            ):
                depth_msg.header.frame_id = (
                    self.gripper_optical_frame
                )

            self.depth_publishers[
                camera_name
            ].publish(
                depth_msg
            )

        front_info = (
            _build_camera_info_from_yaml(
                self.front_camera_info_data,
                stamp=stamp,
                frame_id="runtime_front_camera",
            )
        )

        self.front_camera_info_publisher.publish(
            front_info
        )

        gripper_info = (
            _build_gripper_camera_info(
                self.gripper_K,
                width=(
                    self.grasp_rgb.shape[
                        1
                    ]
                ),
                height=(
                    self.grasp_rgb.shape[
                        0
                    ]
                ),
                stamp=stamp,
                frame_id=(
                    self.gripper_optical_frame
                ),
            )
        )

        self.gripper_camera_info_publisher.publish(
            gripper_info
        )

        self.joint_state_publisher.publish(
            self._build_joint_state(
                stamp
            )
        )


# =====================================================================
# Executor helpers
# =====================================================================

def _spin_controller_executor(
    *,
    node: AIControllerNode,
    executor: MultiThreadedExecutor,
    stop_event: threading.Event,
) -> None:
    while (
        rclpy.ok()
        and not stop_event.is_set()
    ):
        if node.pause_executor.is_set():
            time.sleep(
                0.01
            )
            continue

        executor.spin_once(
            timeout_sec=0.05
        )


# =====================================================================
# Runtime readiness / ROS fixture validation
# =====================================================================

def _wait_for_runtime_ready(
    controller_node: AIControllerNode,
    *,
    timeout: float = 15.0,
) -> None:
    deadline = (
        time.monotonic()
        + timeout
    )

    while (
        time.monotonic()
        < deadline
    ):
        seedo_front_ready = (
            controller_node
            .seedo_rgbd_event
            .is_set()
            and (
                controller_node
                .seedo_camera_info_msg
                is not None
            )
        )

        gripper_info_ready = (
            controller_node
            .seedo_gripper_camera_info_msg
            is not None
        )

        joint_state_ready = (
            controller_node
            .latest_joint_state
            is not None
        )

        with (
            controller_node
            .seedo_record_lock
        ):
            rgb_ready = (
                set(
                    CAMERA_NAMES
                )
                <= set(
                    controller_node
                    .seedo_record_rgb_msgs
                )
            )

            depth_ready = (
                set(
                    CAMERA_NAMES
                )
                <= set(
                    controller_node
                    .seedo_record_depth_msgs
                )
            )

        table_tf_ready = False
        tcp_tf_ready = False
        gripper_tf_ready = False

        try:
            controller_node.tf_buffer.lookup_transform(
                controller_node.frame_id,
                controller_node.seedo_table_frame,
                rclpy.time.Time(),
            )

            table_tf_ready = True

        except Exception:
            pass

        try:
            controller_node.tf_buffer.lookup_transform(
                controller_node.frame_id,
                controller_node.eef_frame_name,
                rclpy.time.Time(),
            )

            tcp_tf_ready = True

        except Exception:
            pass

        try:
            controller_node.tf_buffer.lookup_transform(
                controller_node.frame_id,
                controller_node.seedo_gripper_optical_frame,
                rclpy.time.Time(),
            )

            gripper_tf_ready = True

        except Exception:
            pass

        if (
            seedo_front_ready
            and gripper_info_ready
            and joint_state_ready
            and rgb_ready
            and depth_ready
            and table_tf_ready
            and tcp_tf_ready
            and gripper_tf_ready
        ):
            return

        time.sleep(
            0.05
        )

    raise TimeoutError(
        "Timed out waiting for the complete simulated ROS "
        "runtime: four RGB-D cameras, both CameraInfo topics, "
        "JointState, table TF, TCP TF and eye-in-hand TF."
    )


def _validate_ros_grasp_fixture(
    controller_node: AIControllerNode,
    publisher_node: SeeDoGraspPipelinePublisher,
) -> None:
    # -------------------------------------------------------------
    # Validate raw four-camera callbacks.
    # -------------------------------------------------------------

    record_data = (
        controller_node
        ._get_seedo_record_camera_data(
            timeout_sec=10.0
        )
    )

    if (
        set(
            record_data
        )
        != EXPECTED_ROLLOUT_KEYS
    ):
        raise AssertionError(
            "ROS fixture did not provide all expected "
            "rollout camera keys: "
            f"{sorted(record_data)}"
        )

    np.testing.assert_array_equal(
        record_data[
            "eye_in_hand_image"
        ],
        publisher_node.grasp_bgr,
    )

    np.testing.assert_allclose(
        record_data[
            "eye_in_hand_depth"
        ],
        publisher_node.grasp_depth,
        atol=0.0,
        rtol=0.0,
    )

    # -------------------------------------------------------------
    # Validate the exact GraspPlanner input produced through the
    # real AIControllerNode helper.
    # -------------------------------------------------------------

    base_to_table = (
        get_seedo_base_to_table_transform(
            controller_node
        )
    )

    grasp_input = (
        get_seedo_grasp_runtime_input(
            controller_node,
            base_to_table_transform=(
                base_to_table
            ),
        )
    )

    np.testing.assert_array_equal(
        grasp_input[
            "rgb_image"
        ],
        publisher_node.grasp_rgb,
    )

    np.testing.assert_allclose(
        grasp_input[
            "depth_image"
        ],
        publisher_node.grasp_depth,
        atol=0.0,
        rtol=0.0,
    )

    np.testing.assert_allclose(
        grasp_input[
            "camera_matrix"
        ],
        publisher_node.gripper_K,
        atol=1e-9,
    )

    np.testing.assert_allclose(
        grasp_input[
            "T_base_camera"
        ],
        publisher_node.T_base_camera,
        atol=1e-8,
    )

    np.testing.assert_allclose(
        grasp_input[
            "T_base_table"
        ],
        publisher_node.T_base_table,
        atol=1e-8,
    )

    robot_state = (
        controller_node
        ._capture_robot_state()
    )

    np.testing.assert_allclose(
        robot_state[
            EEF_POS_NAME
        ],
        publisher_node.grasp_eef_position,
        atol=1e-8,
    )

    _assert_quaternion_equivalent(
        robot_state[
            EEF_QUAT_NAME
        ],
        publisher_node.grasp_eef_orientation,
        atol=1e-8,
    )


# =====================================================================
# Input automation
# =====================================================================

class _ScriptedInput:
    """
    Drive exactly one AIControllerNode trajectory automatically.

    After the first rollout has been saved, the second request to
    start a new control loop raises KeyboardInterrupt, allowing the
    test to exit and validate all generated state/artifacts.
    """

    def __init__(
        self,
        *,
        trajectory_count: int,
        task_id: str,
    ) -> None:
        self.trajectory_count = int(
            trajectory_count
        )

        self.task_id = str(
            task_id
        )

        self._started_trajectory = False

    def __call__(
        self,
        prompt: str = "",
    ) -> str:
        normalized = (
            prompt
            .strip()
            .lower()
        )

        if (
            "write the current trajectory count"
            in normalized
        ):
            print(
                f"[AUTO INPUT] trajectory count = "
                f"{self.trajectory_count}"
            )

            return str(
                self.trajectory_count
            )

        if (
            "press enter to start the control loop"
            in normalized
        ):
            if not self._started_trajectory:
                self._started_trajectory = True

                print(
                    "[AUTO INPUT] starting one trajectory"
                )

                return ""

            print(
                "[AUTO INPUT] first trajectory complete; "
                "stopping control_loop"
            )

            raise KeyboardInterrupt

        if (
            "enter task id"
            in normalized
        ):
            print(
                f"[AUTO INPUT] task ID = {self.task_id}"
            )

            return self.task_id

        if (
            "did the robot successfully reach"
            in normalized
        ):
            return "1"

        if (
            "did the robot successfully pick"
            in normalized
        ):
            return "1"

        if (
            "did the robot successfully place"
            in normalized
        ):
            return "1"

        raise AssertionError(
            "Unexpected input() prompt during integration test: "
            f"{prompt!r}"
        )


# =====================================================================
# Inference/action capture
# =====================================================================

def _install_inference_recorder(
    controller,
) -> dict[
    int,
    list[np.ndarray],
]:
    captured: dict[
        int,
        list[np.ndarray],
    ] = {}

    original_inference = (
        controller.inference
    )

    def wrapped_inference(
        *args,
        **kwargs,
    ):
        t = int(
            kwargs.get(
                "t",
                0,
            )
        )

        output = original_inference(
            *args,
            **kwargs,
        )

        if isinstance(
            output,
            list,
        ):
            captured[
                t
            ] = [
                np.asarray(
                    action,
                    dtype=np.float64,
                ).copy()
                for action
                in output
            ]

        return output

    controller.inference = (
        wrapped_inference
    )

    return captured


def _install_grasp_instruction_override(
    controller,
    instruction: str | None,
) -> None:
    if instruction is None:
        return

    instruction = str(
        instruction
    ).strip()

    if not instruction:
        return

    original_resolver = (
        controller
        ._resolve_grasp_action_step
    )

    def wrapped_resolver(
        primitive_step,
    ):
        (
            action_step_index,
            _action_step,
            grasp_target,
        ) = original_resolver(
            primitive_step
        )

        return (
            action_step_index,
            SimpleNamespace(
                grasp_instruction=(
                    instruction
                )
            ),
            grasp_target,
        )

    controller._resolve_grasp_action_step = (
        wrapped_resolver
    )

    print()
    print(
        "[WARNING] TEST-ONLY grasp instruction override enabled."
    )
    print(
        "[WARNING] Grasp instruction:",
        instruction,
    )


# =====================================================================
# Validation helpers
# =====================================================================

def _assert_quaternion_equivalent(
    actual,
    expected,
    *,
    atol: float = 1e-8,
) -> None:
    actual = np.asarray(
        actual,
        dtype=np.float64,
    )

    expected = np.asarray(
        expected,
        dtype=np.float64,
    )

    if actual.shape != (4,):
        raise AssertionError(
            "Actual quaternion must have shape (4,)."
        )

    if expected.shape != (4,):
        raise AssertionError(
            "Expected quaternion must have shape (4,)."
        )

    if np.dot(
        actual,
        expected,
    ) < 0.0:
        actual = -actual

    np.testing.assert_allclose(
        actual,
        expected,
        atol=atol,
    )


def _validate_actions_format(
    primitive_name: str,
    actions: list[np.ndarray],
) -> None:
    if not actions:
        raise AssertionError(
            f"{primitive_name} generated no actions."
        )

    for index, action in enumerate(
        actions
    ):
        action = np.asarray(
            action,
            dtype=np.float64,
        )

        if action.shape != (8,):
            raise AssertionError(
                f"{primitive_name} action {index} "
                f"has shape {action.shape}; expected (8,)."
            )

        if not np.isfinite(
            action
        ).all():
            raise AssertionError(
                f"{primitive_name} action {index} "
                "contains non-finite values."
            )

        quaternion_norm = (
            np.linalg.norm(
                action[
                    3:7
                ]
            )
        )

        np.testing.assert_allclose(
            quaternion_norm,
            1.0,
            atol=1e-6,
        )


def _validate_pipeline_state(
    controller_node: AIControllerNode,
    *,
    captured_actions: dict[
        int,
        list[np.ndarray],
    ],
    expected_primitives: tuple[
        str,
        ...,
    ],
    instruction_override: str | None,
) -> None:
    controller = (
        controller_node.controller
    )

    required_state = {
        "action_plan": (
            controller.action_plan
        ),
        "demo_structured_scene": (
            controller.demo_structured_scene
        ),
        "scene_state": (
            controller.scene_state
        ),
        "runtime_structured_scene": (
            controller.runtime_structured_scene
        ),
        "structural_matching_result": (
            controller.structural_matching_result
        ),
        "replicability_result": (
            controller.replicability_result
        ),
        "primitive_plan": (
            controller.primitive_plan
        ),
        "grasp_plan": (
            controller.grasp_plan
        ),
    }

    missing = [
        name
        for (
            name,
            value,
        )
        in required_state.items()
        if value is None
    ]

    if missing:
        raise AssertionError(
            "Complete grasp-aware pipeline state "
            "was not populated: "
            f"{missing}"
        )

    if (
        controller.execution_status
        != "completed"
    ):
        raise AssertionError(
            "SeeDo execution state is not completed: "
            f"{controller.execution_status!r}"
        )

    if (
        not controller
        .replicability_result
        .replicable
    ):
        raise AssertionError(
            "Runtime task was not considered replicable: "
            f"{controller.replicability_result.failure_reasons}"
        )

    primitive_steps = (
        controller
        .primitive_plan
        .steps
    )

    primitive_names = tuple(
        str(
            step.name
        )
        .strip()
        .lower()
        for step
        in primitive_steps
    )

    if (
        primitive_names
        != expected_primitives
    ):
        raise AssertionError(
            "Unexpected PrimitivePlan sequence: "
            f"expected={expected_primitives}, "
            f"received={primitive_names}"
        )

    approaching_indices = [
        index
        for (
            index,
            step,
        )
        in enumerate(
            primitive_steps
        )
        if (
            str(
                step.name
            )
            .strip()
            .lower()
            == "approaching"
        )
    ]

    if len(
        approaching_indices
    ) != 1:
        raise AssertionError(
            "Expected exactly one approaching primitive, "
            f"found {len(approaching_indices)}."
        )

    approaching_step = (
        primitive_steps[
            approaching_indices[
                0
            ]
        ]
    )

    approaching_target = str(
        approaching_step.arguments.get(
            "target",
            "",
        )
    ).strip()

    if (
        controller.grasp_plan_target
        != approaching_target
    ):
        raise AssertionError(
            "GraspPlan target does not match the "
            "approaching primitive: "
            f"grasp_plan_target={controller.grasp_plan_target!r}, "
            f"approaching_target={approaching_target!r}"
        )

    grasp_plan = (
        controller.grasp_plan
    )

    for attribute in (
        "grasp_pose_base",
        "grasp_position_base",
        "grasp_orientation_base",
        "tcp_pose_base",
        "tcp_position_base",
        "tcp_orientation_base",
    ):
        if not hasattr(
            grasp_plan,
            attribute,
        ):
            raise AssertionError(
                "GraspPlan is missing new grasp-aware field "
                f"{attribute!r}."
            )

    tcp_position = np.asarray(
        grasp_plan.tcp_position_base,
        dtype=np.float64,
    )

    tcp_orientation = np.asarray(
        grasp_plan.tcp_orientation_base,
        dtype=np.float64,
    )

    if tcp_position.shape != (3,):
        raise AssertionError(
            "GraspPlan.tcp_position_base must have shape (3,)."
        )

    if tcp_orientation.shape != (4,):
        raise AssertionError(
            "GraspPlan.tcp_orientation_base must have shape (4,)."
        )

    if not np.isfinite(
        tcp_position
    ).all():
        raise AssertionError(
            "GraspPlan TCP position contains non-finite values."
        )

    if not np.isfinite(
        tcp_orientation
    ).all():
        raise AssertionError(
            "GraspPlan TCP orientation contains non-finite values."
        )

    np.testing.assert_allclose(
        np.linalg.norm(
            tcp_orientation
        ),
        1.0,
        atol=1e-6,
    )

    if instruction_override:
        if (
            str(
                grasp_plan.grasp_instruction
            ).strip()
            != str(
                instruction_override
            ).strip()
        ):
            raise AssertionError(
                "GraspPlan did not use the requested "
                "test-only grasp instruction override."
            )

    # -------------------------------------------------------------
    # Verify YAML -> SeeDoController -> GraspPlanner propagation.
    # -------------------------------------------------------------

    planner = (
        controller.grasp_planner
    )

    if not hasattr(
        planner,
        "m2t2_gripper_depth_m",
    ):
        raise AssertionError(
            "GraspPlanner does not expose "
            "m2t2_gripper_depth_m."
        )

    np.testing.assert_allclose(
        planner.m2t2_gripper_depth_m,
        EXPECTED_M2T2_GRIPPER_DEPTH_M,
        atol=1e-12,
    )

    # -------------------------------------------------------------
    # Verify GraspPlan -> MotionLayer handoff.
    # -------------------------------------------------------------

    motion_layer = (
        controller.motion_layer
    )

    if (
        motion_layer.active_grasp_position
        is None
        or motion_layer.active_grasp_orientation
        is None
    ):
        raise AssertionError(
            "MotionLayer did not receive the active grasp pose."
        )

    np.testing.assert_allclose(
        motion_layer.active_grasp_position,
        tcp_position,
        atol=1e-8,
    )

    _assert_quaternion_equivalent(
        motion_layer.active_grasp_orientation,
        tcp_orientation,
    )

    # -------------------------------------------------------------
    # Validate all captured primitive actions.
    # -------------------------------------------------------------

    if len(
        captured_actions
    ) != len(
        primitive_steps
    ):
        raise AssertionError(
            "Did not capture one action list for every primitive: "
            f"captured t={sorted(captured_actions)}, "
            f"primitive_count={len(primitive_steps)}"
        )

    open_position = float(
        motion_layer.gripper_open_position
    )

    closed_position = float(
        motion_layer.gripper_closed_position
    )

    for (
        primitive_index,
        primitive_step,
    ) in enumerate(
        primitive_steps,
        start=1,
    ):
        primitive_name = (
            str(
                primitive_step.name
            )
            .strip()
            .lower()
        )

        if primitive_index not in (
            captured_actions
        ):
            raise AssertionError(
                "Missing captured actions for "
                f"t={primitive_index} ({primitive_name})."
            )

        actions = (
            captured_actions[
                primitive_index
            ]
        )

        _validate_actions_format(
            primitive_name,
            actions,
        )

        final_action = (
            actions[
                -1
            ]
        )

        if (
            primitive_name
            == "reach"
        ):
            np.testing.assert_allclose(
                final_action[
                    7
                ],
                open_position,
                atol=1e-9,
            )

        elif (
            primitive_name
            == "approaching"
        ):
            np.testing.assert_allclose(
                final_action[
                    :3
                ],
                tcp_position,
                atol=1e-8,
            )

            _assert_quaternion_equivalent(
                final_action[
                    3:7
                ],
                tcp_orientation,
            )

            for action in actions:
                np.testing.assert_allclose(
                    action[
                        7
                    ],
                    open_position,
                    atol=1e-9,
                )

        elif (
            primitive_name
            == "pick"
        ):
            np.testing.assert_allclose(
                final_action[
                    :3
                ],
                tcp_position,
                atol=1e-8,
            )

            _assert_quaternion_equivalent(
                final_action[
                    3:7
                ],
                tcp_orientation,
            )

            np.testing.assert_allclose(
                final_action[
                    7
                ],
                closed_position,
                atol=1e-9,
            )

        elif primitive_name in (
            "lift_up",
            "moving",
            "aligning",
        ):
            for action in actions:
                _assert_quaternion_equivalent(
                    action[
                        3:7
                    ],
                    tcp_orientation,
                )

                np.testing.assert_allclose(
                    action[
                        7
                    ],
                    closed_position,
                    atol=1e-9,
                )

        elif primitive_name in (
            "placing",
            "inserting",
        ):
            for action in actions[
                :-1
            ]:
                _assert_quaternion_equivalent(
                    action[
                        3:7
                    ],
                    tcp_orientation,
                )

                np.testing.assert_allclose(
                    action[
                        7
                    ],
                    closed_position,
                    atol=1e-9,
                )

            _assert_quaternion_equivalent(
                final_action[
                    3:7
                ],
                tcp_orientation,
            )

            np.testing.assert_allclose(
                final_action[
                    7
                ],
                open_position,
                atol=1e-9,
            )


def _validate_artifacts(
    artifacts_dir: Path,
) -> Path:
    required_paths = (
        artifacts_dir
        / "demo_structured_scene"
        / "demo_structured_scene.json",

        artifacts_dir
        / "action_planning"
        / "action_plan.json",

        artifacts_dir
        / "scene_perceiver"
        / "raw_scene_state.json",

        artifacts_dir
        / "scene_interpreter"
        / "scene_state.json",

        artifacts_dir
        / "runtime_structured_scene"
        / "runtime_structured_scene.json",

        artifacts_dir
        / "structural_matching"
        / "structural_matching_result.json",

        artifacts_dir
        / "replicability_check"
        / "replicability_result.json",

        artifacts_dir
        / "lmp_generator"
        / "primitive_plan.json",

        artifacts_dir
        / "motion_layer"
        / "motion_plan.json",

        artifacts_dir
        / "timings.json",
    )

    for path in required_paths:
        if not path.is_file():
            raise AssertionError(
                "Expected artifact was not generated: "
                f"{path}"
            )

        if path.stat().st_size == 0:
            raise AssertionError(
                "Expected artifact is empty: "
                f"{path}"
            )

    grasp_plans = sorted(
        (
            artifacts_dir
            / "grasp_planner"
        ).rglob(
            "grasp_plan.json"
        )
    )

    if len(
        grasp_plans
    ) != 1:
        raise AssertionError(
            "Expected exactly one grasp_plan.json, "
            f"found {len(grasp_plans)}: {grasp_plans}"
        )

    grasp_plan_path = (
        grasp_plans[
            0
        ]
    )

    with grasp_plan_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        payload = json.load(
            stream
        )

    for key in (
        "tcp_position_base",
        "tcp_orientation_base",
        "m2t2_gripper_depth_m",
    ):
        if key not in payload:
            raise AssertionError(
                "Grasp artifact is missing "
                f"{key!r}."
            )

    np.testing.assert_allclose(
        payload[
            "m2t2_gripper_depth_m"
        ],
        EXPECTED_M2T2_GRIPPER_DEPTH_M,
        atol=1e-12,
    )

    return grasp_plan_path


def _snapshot_rollouts(
    rollout_root: Path,
) -> tuple[
    dict[Path, tuple[int, int]],
    dict[Path, tuple[int, int]],
]:
    """
    Snapshot rollout files using both path and file metadata.

    This allows the integration test to detect not only newly
    created trajectories, but also an existing trajectory file
    that has been overwritten by the current test run.
    """

    if not rollout_root.exists():
        return (
            {},
            {},
        )

    def snapshot(
        pattern: str,
    ) -> dict[
        Path,
        tuple[int, int],
    ]:
        result = {}

        for path in rollout_root.rglob(
            pattern
        ):
            stat = path.stat()

            result[
                path
            ] = (
                stat.st_mtime_ns,
                stat.st_size,
            )

        return result

    return (
        snapshot(
            "traj_*.pkl"
        ),
        snapshot(
            "traj_*.json"
        ),
    )


def _validate_outcome_json(
    path: Path,
) -> None:
    with path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        payload = json.load(
            stream
        )

    if (
        payload.get(
            "program_status"
        )
        != "completed"
    ):
        raise AssertionError(
            "Saved outcome does not report "
            "program_status='completed': "
            f"{payload}"
        )

    required_keys = {
        "object_reached",
        "object_picked",
        "object_placed",
        "reached_wrong",
        "picked_wrong",
        "place_wrong_correct_bin",
        "place_wrong_wrong_bin",
    }

    missing = (
        required_keys
        - set(
            payload
        )
    )

    if missing:
        raise AssertionError(
            "Saved outcome metadata is incomplete: "
            f"{sorted(missing)}"
        )


# =====================================================================
# Input validation
# =====================================================================

def _resolve_inputs(
    args: argparse.Namespace,
) -> dict:
    video_path = (
        Path(
            args.video
        )
        .expanduser()
        .resolve()
    )

    scene_dir = (
        Path(
            args.scene_dir
        )
        .expanduser()
        .resolve()
    )

    base_to_table_path = (
        Path(
            args.base_to_table_transform
        )
        .expanduser()
        .resolve()
    )

    model_config_path = (
        Path(
            args.model_config
        )
        .expanduser()
        .resolve()
    )

    grasp_pkl_path = (
        Path(
            args.grasp_pkl
        )
        .expanduser()
        .resolve()
    )

    gripper_camera_info_path = (
        None
        if (
            args.gripper_camera_info
            is None
        )
        else (
            Path(
                args.gripper_camera_info
            )
            .expanduser()
            .resolve()
        )
    )

    artifacts_dir = (
        Path(
            args.artifacts_dir
        )
        .expanduser()
        .resolve()
    )

    rollout_base_dir = (
        Path(
            args.rollouts_dir
        )
        .expanduser()
        .resolve()
    )

    required_files = (
        video_path,
        base_to_table_path,
        model_config_path,
        grasp_pkl_path,
        scene_dir
        / "rgb.png",
        scene_dir
        / "depth.npy",
        scene_dir
        / "camera_info.yaml",
    )

    for path in required_files:
        if not path.is_file():
            raise FileNotFoundError(
                "Required integration-test input "
                "does not exist: "
                f"{path}"
            )

    if (
        gripper_camera_info_path
        is not None
        and not (
            gripper_camera_info_path
            .is_file()
        )
    ):
        raise FileNotFoundError(
            "Gripper CameraInfo YAML "
            "does not exist: "
            f"{gripper_camera_info_path}"
        )

    artifacts_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    rollout_base_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    return {
        "video_path": video_path,
        "scene_dir": scene_dir,
        "base_to_table_path": (
            base_to_table_path
        ),
        "model_config_path": (
            model_config_path
        ),
        "grasp_pkl_path": (
            grasp_pkl_path
        ),
        "gripper_camera_info_path": (
            gripper_camera_info_path
        ),
        "artifacts_dir": (
            artifacts_dir
        ),
        "rollout_base_dir": (
            rollout_base_dir
        ),
    }


# =====================================================================
# Main test
# =====================================================================

def run_test(
    args: argparse.Namespace,
) -> int:
    inputs = (
        _resolve_inputs(
            args
        )
    )

    print()
    print(
        "=" * 78
    )
    print(
        "COMPLETE SEEDO + GRASP PIPELINE ROS INTEGRATION TEST"
    )
    print(
        "=" * 78
    )

    print(
        "[INPUT] demo video:",
        inputs[
            "video_path"
        ],
    )

    print(
        "[INPUT] runtime scene:",
        inputs[
            "scene_dir"
        ],
    )

    print(
        "[INPUT] grasp PKL:",
        inputs[
            "grasp_pkl_path"
        ],
    )

    print(
        "[INPUT] grasp PKL step:",
        args.grasp_step,
    )

    print(
        "[INPUT] base-to-table:",
        inputs[
            "base_to_table_path"
        ],
    )

    print(
        "[INPUT] model config:",
        inputs[
            "model_config_path"
        ],
    )

    rclpy.init(
        args=[
            "--ros-args",

            "-p",
            "ai_controller_target:=seedo_controller",

            "-p",
            f"model_config_path:="
            f"{inputs['model_config_path']}",

            "-p",
            f"demo_path:="
            f"{inputs['video_path']}",

            "-p",
            f"task_name:="
            f"{args.task_name}",

            "-p",
            "move_robot:=False",

            "-p",
            "seedo_execute_gripper:=False",

            "-p",
            f"seedo_artifacts_dir:="
            f"{inputs['artifacts_dir']}",

            "-p",
            f"save_rollout_path:="
            f"{inputs['rollout_base_dir']}",
        ]
    )

    controller_node = None
    publisher_node = None

    controller_executor = None
    publisher_executor = None

    controller_thread = None
    publisher_thread = None

    controller_stop_event = (
        threading.Event()
    )

    try:
        print()
        print(
            "=== INITIALIZING AI CONTROLLER NODE ==="
        )

        controller_node = (
            AIControllerNode()
        )

        if (
            controller_node
            .ai_controller_target
            != "seedo_controller"
        ):
            raise AssertionError(
                "AIControllerNode is not using "
                "seedo_controller."
            )

        if (
            controller_node.move_robot
        ):
            raise AssertionError(
                "Integration test must run with "
                "move_robot=False."
            )

        if (
            controller_node
            .seedo_execute_gripper
        ):
            raise AssertionError(
                "Integration test must run with "
                "seedo_execute_gripper=False."
            )

        if (
            controller_node
            .controller
            .perception_mode
            != args.expected_perception_mode
        ):
            raise AssertionError(
                "Unexpected perception mode: "
                f"{controller_node.controller.perception_mode!r}"
            )

        planner = (
            controller_node
            .controller
            .grasp_planner
        )

        if not hasattr(
            planner,
            "m2t2_gripper_depth_m",
        ):
            raise AssertionError(
                "Updated GraspPlanner does not expose "
                "m2t2_gripper_depth_m."
            )

        np.testing.assert_allclose(
            planner.m2t2_gripper_depth_m,
            EXPECTED_M2T2_GRIPPER_DEPTH_M,
            atol=1e-12,
        )

        controller_node.demo_path = str(
            inputs[
                "video_path"
            ]
        )

        controller_node.seedo_precomputed_action_plan_path = (
            ""
        )

        controller_node.seedo_precomputed_demo_structured_scene_path = (
            ""
        )

        print(
            "[PASS] AIControllerNode configuration"
        )

        print()
        print(
            "=== INITIALIZING SIMULATED ROS RUNTIME ==="
        )

        publisher_node = (
            SeeDoGraspPipelinePublisher(
                scene_dir=(
                    inputs[
                        "scene_dir"
                    ]
                ),
                base_to_table_path=(
                    inputs[
                        "base_to_table_path"
                    ]
                ),
                grasp_pkl_path=(
                    inputs[
                        "grasp_pkl_path"
                    ]
                ),
                grasp_step=(
                    args.grasp_step
                ),
                gripper_camera_info_path=(
                    inputs[
                        "gripper_camera_info_path"
                    ]
                ),
                rgb_topics=list(
                    controller_node
                    .camera_topic
                ),
                depth_topics=list(
                    controller_node
                    .seedo_record_depth_topics
                ),
                front_camera_info_topic=(
                    controller_node
                    .seedo_camera_info_topic
                ),
                gripper_camera_info_topic=(
                    controller_node
                    .seedo_gripper_camera_info_topic
                ),
                joint_states_topic=(
                    controller_node
                    .joint_states_topic
                ),
                joint_robot_names=list(
                    controller_node
                    .joint_robot_names
                ),
                gripper_robot_names=list(
                    controller_node
                    .gripper_robot_names
                ),
                base_frame=(
                    controller_node
                    .frame_id
                ),
                table_frame=(
                    controller_node
                    .seedo_table_frame
                ),
                eef_frame=(
                    controller_node
                    .eef_frame_name
                ),
                gripper_optical_frame=(
                    controller_node
                    .seedo_gripper_optical_frame
                ),
            )
        )

        # ---------------------------------------------------------
        # Controller executor.
        #
        # This mirrors production behavior. It is essential because
        # wait_for_seedo_grasp_runtime_data() explicitly waits for a
        # NEW eye-in-hand RGB-D callback after REACH.
        # ---------------------------------------------------------

        controller_executor = (
            MultiThreadedExecutor(
                num_threads=1
            )
        )

        controller_executor.add_node(
            controller_node
        )

        controller_thread = (
            threading.Thread(
                target=(
                    _spin_controller_executor
                ),
                kwargs={
                    "node": (
                        controller_node
                    ),
                    "executor": (
                        controller_executor
                    ),
                    "stop_event": (
                        controller_stop_event
                    ),
                },
                daemon=True,
            )
        )

        controller_thread.start()

        # ---------------------------------------------------------
        # External simulated ROS world.
        # ---------------------------------------------------------

        publisher_executor = (
            MultiThreadedExecutor(
                num_threads=1
            )
        )

        publisher_executor.add_node(
            publisher_node
        )

        publisher_thread = (
            threading.Thread(
                target=(
                    publisher_executor.spin
                ),
                daemon=True,
            )
        )

        publisher_thread.start()

        print()
        print(
            "=== WAITING FOR COMPLETE ROS RUNTIME ==="
        )

        _wait_for_runtime_ready(
            controller_node,
            timeout=args.runtime_timeout,
        )

        print(
            "[PASS] four RGB-D streams"
        )

        print(
            "[PASS] front CameraInfo"
        )

        print(
            "[PASS] eye-in-hand CameraInfo"
        )

        print(
            "[PASS] JointState"
        )

        print(
            "[PASS] base->table TF"
        )

        print(
            "[PASS] base->TCP TF"
        )

        print(
            "[PASS] TCP->eye-in-hand TF"
        )

        print()
        print(
            "=== VALIDATING PKL -> ROS -> GRASP INPUT ==="
        )

        _validate_ros_grasp_fixture(
            controller_node,
            publisher_node,
        )

        print(
            "[PASS] eye-in-hand RGB from PKL"
        )

        print(
            "[PASS] eye-in-hand depth from PKL"
        )

        print(
            "[PASS] eye-in-hand CameraInfo"
        )

        print(
            "[PASS] T_base_camera matches validated PKL calibration"
        )

        print(
            "[PASS] T_base_table matches test transform"
        )

        print(
            "[PASS] TCP pose matches selected PKL observation"
        )

        # ---------------------------------------------------------
        # Capture every primitive output generated by the real
        # SeeDoController.
        # ---------------------------------------------------------

        captured_actions = (
            _install_inference_recorder(
                controller_node.controller
            )
        )

        _install_grasp_instruction_override(
            controller_node.controller,
            args.grasp_instruction_override,
        )

        actual_rollout_root = Path(
            controller_node
            .save_rollout_path
        )

        (
            before_pkl,
            before_json,
        ) = _snapshot_rollouts(
            actual_rollout_root
        )

        scripted_input = (
            _ScriptedInput(
                trajectory_count=(
                    args.trajectory_count
                ),
                task_id=(
                    args.task_id
                ),
            )
        )

        print()
        print(
            "=== STARTING REAL AIControllerNode.control_loop() ==="
        )

        print(
            "move_robot=False, seedo_execute_gripper=False"
        )

        print(
            "No physical robot or gripper command will be sent."
        )

        interrupted_after_rollout = (
            False
        )

        try:
            with patch(
                "builtins.input",
                new=scripted_input,
            ):
                controller_node.control_loop()

        except KeyboardInterrupt:
            interrupted_after_rollout = (
                True
            )

        if not interrupted_after_rollout:
            raise AssertionError(
                "Automated control_loop did not stop "
                "after the first trajectory."
            )

        print()
        print(
            "=== VALIDATING COMPLETE PIPELINE STATE ==="
        )

        _validate_pipeline_state(
            controller_node,
            captured_actions=(
                captured_actions
            ),
            expected_primitives=tuple(
                args.expected_primitives
            ),
            instruction_override=(
                args.grasp_instruction_override
            ),
        )

        print(
            "[PASS] demonstration understanding"
        )

        print(
            "[PASS] runtime perception"
        )

        print(
            "[PASS] scene interpretation"
        )

        print(
            "[PASS] structural matching"
        )

        print(
            "[PASS] replicability"
        )

        print(
            "[PASS] LMP / PrimitivePlan"
        )

        print(
            "[PASS] runtime GraspPlanner"
        )

        print(
            "[PASS] GraspPlan -> MotionLayer handoff"
        )

        print(
            "[PASS] complete primitive action sequence"
        )

        print()
        print(
            "=== VALIDATING ARTIFACTS ==="
        )

        grasp_artifact_path = (
            _validate_artifacts(
                inputs[
                    "artifacts_dir"
                ]
            )
        )

        print(
            "[PASS] complete artifact tree"
        )

        print(
            "[PASS] grasp artifact:",
            grasp_artifact_path,
        )

        print(
            "[PASS] timings.json"
        )

        print()
        print(
            "=== VALIDATING SAVED ROLLOUT ==="
        )

        (
            after_pkl,
            after_json,
        ) = _snapshot_rollouts(
            actual_rollout_root
        )

        new_pkl = sorted(
            path
            for (
                path,
                metadata,
            )
            in after_pkl.items()
            if (
                path not in before_pkl
                or before_pkl[
                    path
                ] != metadata
            )
        )

        new_json = sorted(
            path
            for (
                path,
                metadata,
            )
            in after_json.items()
            if (
                path not in before_json
                or before_json[
                    path
                ] != metadata
            )
        )

        if len(
            new_pkl
        ) != 1:
            raise AssertionError(
                "Expected exactly one new trajectory PKL, "
                f"found {len(new_pkl)}: {new_pkl}"
            )

        if len(
            new_json
        ) != 1:
            raise AssertionError(
                "Expected exactly one new trajectory outcome JSON, "
                f"found {len(new_json)}: {new_json}"
            )

        if (
            new_pkl[
                0
            ].stem
            != new_json[
                0
            ].stem
        ):
            raise AssertionError(
                "Trajectory PKL and outcome JSON "
                "do not share the same stem."
            )

        if (
            new_pkl[
                0
            ].stat()
            .st_size
            == 0
        ):
            raise AssertionError(
                "Saved trajectory is empty."
            )

        _validate_outcome_json(
            new_json[
                0
            ]
        )

        print(
            "[PASS] trajectory:",
            new_pkl[
                0
            ],
        )

        print(
            "[PASS] outcome:",
            new_json[
                0
            ],
        )

        # ---------------------------------------------------------
        # Final summary
        # ---------------------------------------------------------

        controller = (
            controller_node
            .controller
        )

        grasp_plan = (
            controller
            .grasp_plan
        )

        print()
        print(
            "=" * 78
        )

        print(
            "FINAL GRASP SUMMARY"
        )

        print(
            "=" * 78
        )

        print(
            "Target:",
            controller.grasp_plan_target,
        )

        print(
            "Instruction:",
            grasp_plan.grasp_instruction,
        )

        print(
            "M2T2 index:",
            grasp_plan.selected_m2t2_index,
        )

        print(
            "Confidence:",
            grasp_plan.confidence,
        )

        print(
            "TCP position:",
            grasp_plan.tcp_position_base,
        )

        print(
            "TCP orientation:",
            grasp_plan.tcp_orientation_base,
        )

        print(
            "Symmetry flipped:",
            grasp_plan.tcp_symmetry_flipped,
        )

        print()
        print(
            "Primitive actions captured:"
        )

        for (
            t,
            primitive_step,
        ) in enumerate(
            controller
            .primitive_plan
            .steps,
            start=1,
        ):
            print(
                f"  t={t}: "
                f"{primitive_step.name} -> "
                f"{len(captured_actions[t])} actions"
            )

        print()
        print(
            "=" * 78
        )

        print(
            "COMPLETE SEEDO + GRASP PIPELINE TEST PASSED"
        )

        print(
            "=" * 78
        )

        return 0

    finally:
        controller_stop_event.set()

        if (
            controller_executor
            is not None
        ):
            controller_executor.shutdown()

        if (
            publisher_executor
            is not None
        ):
            publisher_executor.shutdown()

        if (
            controller_thread
            is not None
            and controller_thread.is_alive()
        ):
            controller_thread.join(
                timeout=2.0
            )

        if (
            publisher_thread
            is not None
            and publisher_thread.is_alive()
        ):
            publisher_thread.join(
                timeout=2.0
            )

        if (
            publisher_node
            is not None
        ):
            publisher_node.destroy_node()

        if (
            controller_node
            is not None
        ):
            controller_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


# =====================================================================
# CLI
# =====================================================================

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Complete ROS dry-run integration test for the "
            "grasp-aware SeeDo pipeline."
        )
    )

    parser.add_argument(
        "--video",
        required=True,
        help=(
            "Human demonstration video used by SeeDo."
        ),
    )

    parser.add_argument(
        "--scene-dir",
        required=True,
        help=(
            "Runtime scene directory containing "
            "rgb.png, depth.npy and camera_info.yaml."
        ),
    )

    parser.add_argument(
        "--base-to-table-transform",
        required=True,
        help=(
            "YAML containing rotation/translation for "
            "base_link <- table."
        ),
    )

    parser.add_argument(
        "--model-config",
        required=True,
        help=(
            "SeeDo model configuration YAML."
        ),
    )

    parser.add_argument(
        "--grasp-pkl",
        default=str(
            DEFAULT_GRASP_PKL
        ),
        help=(
            "Trajectory PKL used to publish the "
            "eye-in-hand RGB-D grasp fixture."
        ),
    )

    parser.add_argument(
        "--grasp-step",
        type=int,
        default=(
            DEFAULT_GRASP_STEP
        ),
    )

    parser.add_argument(
        "--gripper-camera-info",
        default=None,
        help=(
            "Optional eye-in-hand CameraInfo YAML. "
            "If omitted, the validated built-in intrinsics "
            "from test_grasp_planner.py are used."
        ),
    )

    parser.add_argument(
        "--task-name",
        default="pick_place",
    )

    parser.add_argument(
        "--task-id",
        default="0",
    )

    parser.add_argument(
        "--trajectory-count",
        type=int,
        default=0,
    )

    parser.add_argument(
        "--expected-perception-mode",
        default="generalized",
    )

    parser.add_argument(
        "--expected-primitives",
        nargs="+",
        default=list(
            DEFAULT_EXPECTED_PRIMITIVES
        ),
        help=(
            "Expected PrimitivePlan names in order."
        ),
    )

    parser.add_argument(
        "--grasp-instruction-override",
        default=None,
        help=(
            "TEST ONLY. Override the grasp instruction passed "
            "to GraspPlanner while preserving the real symbolic "
            "target resolution. Leave unset for a strict semantic "
            "end-to-end test. This is useful when reusing the gray-ring "
            "PKL with an older demonstration fixture that describes "
            "a different object."
        ),
    )

    parser.add_argument(
        "--artifacts-dir",
        default=str(
            DEFAULT_ARTIFACTS_DIR
        ),
    )

    parser.add_argument(
        "--rollouts-dir",
        default=str(
            DEFAULT_ROLLOUTS_DIR
        ),
    )

    parser.add_argument(
        "--runtime-timeout",
        type=float,
        default=15.0,
    )

    return parser


def main() -> int:
    parser = build_parser()

    args = parser.parse_args()

    return run_test(
        args
    )


if __name__ == "__main__":
    raise SystemExit(
        main()
    )
