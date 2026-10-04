from __future__ import annotations

import argparse
import json
import threading
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as RosImage
from tf2_ros import StaticTransformBroadcaster

import ai_controller.ai_controller_node as ai_controller_node_module
from ai_controller.ai_controller_node import AIControllerNode
from ai_controller.utils.utils import (
    EEF_POS_NAME,
    EEF_QUAT_NAME,
)


EXPECTED_VIDEO = Path(
    "/test_dataset/pick_place/human_rgb_pick_place/"
    "task_00/traj000/converted/traj000-h264-30fps.mp4"
)

EXPECTED_SCENE_DIR = Path(
    "/scene_capture/without_distractors/"
    "scene_1_no_distractors"
)

EXPECTED_RUNTIME_OBJECT_IDS = {
    "storage_bin_0",
    "storage_bin_1",
    "storage_bin_2",
    "storage_bin_3",
    "green_block_0",
    "yellow_block_0",
    "blue_block_0",
    "red_block_0",
}

EXPECTED_DEMO_PLACE_IDS = {
    "0",
    "1",
    "2",
    "3",
}

EXPECTED_RUNTIME_PLACE_IDS = {
    "storage_bin_0",
    "storage_bin_1",
    "storage_bin_2",
    "storage_bin_3",
}

EXPECTED_STRUCTURAL_MAPPING = {
    "0": "storage_bin_0",
    "1": "storage_bin_1",
    "2": "storage_bin_2",
    "3": "storage_bin_3",
}

EXPECTED_PRIMITIVES = (
    ("reach", "green_block_0"),
    ("approaching", "green_block_0"),
    ("pick", "green_block_0"),
    ("lift_up", "green_block_0"),
    ("moving", "storage_bin_0"),
    ("placing", "storage_bin_0"),
)

ROLLOUT_RGB_KEYS = (
    "camera_front_image",
    "camera_lateral_left_image",
    "camera_lateral_right_image",
    "eye_in_hand_image",
)

ROLLOUT_DEPTH_KEYS = (
    "camera_front_depth",
    "camera_lateral_left_depth",
    "camera_lateral_right_depth",
    "eye_in_hand_depth",
)

INITIAL_EEF_POSITION = np.array(
    [
        -0.15552094619366708,
        0.34869994018501943,
        0.1532803451753288,
    ],
    dtype=np.float64,
)

INITIAL_EEF_ORIENTATION = np.array(
    [
        0.9994452044624775,
        0.03161651380119412,
        0.0021438049655468088,
        0.010251021036213035,
    ],
    dtype=np.float64,
)


def _rotation_matrix_to_quaternion(
    rotation: np.ndarray,
) -> np.ndarray:
    """Convert a 3x3 rotation matrix to quaternion (x, y, z, w)."""

    matrix = np.asarray(
        rotation,
        dtype=np.float64,
    )

    if matrix.shape != (3, 3):
        raise ValueError(
            "Rotation matrix must have shape (3, 3). "
            f"Got {matrix.shape}."
        )

    trace = float(
        np.trace(
            matrix
        )
    )

    if trace > 0.0:
        s = np.sqrt(
            trace + 1.0
        ) * 2.0

        w = 0.25 * s
        x = (
            matrix[2, 1]
            - matrix[1, 2]
        ) / s
        y = (
            matrix[0, 2]
            - matrix[2, 0]
        ) / s
        z = (
            matrix[1, 0]
            - matrix[0, 1]
        ) / s

    elif (
        matrix[0, 0]
        > matrix[1, 1]
        and matrix[0, 0]
        > matrix[2, 2]
    ):
        s = np.sqrt(
            1.0
            + matrix[0, 0]
            - matrix[1, 1]
            - matrix[2, 2]
        ) * 2.0

        w = (
            matrix[2, 1]
            - matrix[1, 2]
        ) / s

        x = 0.25 * s

        y = (
            matrix[0, 1]
            + matrix[1, 0]
        ) / s

        z = (
            matrix[0, 2]
            + matrix[2, 0]
        ) / s

    elif (
        matrix[1, 1]
        > matrix[2, 2]
    ):
        s = np.sqrt(
            1.0
            + matrix[1, 1]
            - matrix[0, 0]
            - matrix[2, 2]
        ) * 2.0

        w = (
            matrix[0, 2]
            - matrix[2, 0]
        ) / s

        x = (
            matrix[0, 1]
            + matrix[1, 0]
        ) / s

        y = 0.25 * s

        z = (
            matrix[1, 2]
            + matrix[2, 1]
        ) / s

    else:
        s = np.sqrt(
            1.0
            + matrix[2, 2]
            - matrix[0, 0]
            - matrix[1, 1]
        ) * 2.0

        w = (
            matrix[1, 0]
            - matrix[0, 1]
        ) / s

        x = (
            matrix[0, 2]
            + matrix[2, 0]
        ) / s

        y = (
            matrix[1, 2]
            + matrix[2, 1]
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

    norm = np.linalg.norm(
        quaternion
    )

    if norm <= 0.0:
        raise ValueError(
            "Could not normalize transform quaternion."
        )

    quaternion /= norm

    return quaternion


class SeeDoRuntimePublisher(Node):
    """Publish the canonical runtime scene through the real ROS interfaces."""

    def __init__(
        self,
        *,
        scene_dir: str | Path,
        transform_path: str | Path,
        rgb_topics: tuple[str, ...],
        depth_topics: tuple[str, ...],
        camera_info_topic: str,
        base_frame: str,
        table_frame: str,
    ) -> None:
        super().__init__(
            "seedo_runtime_test_publisher"
        )

        if len(
            rgb_topics
        ) != 4:
            raise ValueError(
                "ROS integration test expects exactly four RGB topics."
            )

        if len(
            depth_topics
        ) != 4:
            raise ValueError(
                "ROS integration test expects exactly four depth topics."
            )

        self.bridge = CvBridge()

        scene_dir = (
            Path(scene_dir)
            .expanduser()
            .resolve()
        )

        transform_path = (
            Path(transform_path)
            .expanduser()
            .resolve()
        )

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
            transform_path,
        ):
            if not path.is_file():
                raise FileNotFoundError(
                    "Required ROS integration input does not exist: "
                    f"{path}"
                )

        rgb_bgr = cv2.imread(
            str(
                rgb_path
            )
        )

        if rgb_bgr is None:
            raise RuntimeError(
                f"Could not load RGB image: {rgb_path}"
            )

        self.rgb = cv2.cvtColor(
            rgb_bgr,
            cv2.COLOR_BGR2RGB,
        )

        self.depth = np.asarray(
            np.load(
                depth_path
            ),
            dtype=np.float32,
        )

        with camera_info_path.open(
            "r",
            encoding="utf-8",
        ) as stream:
            self.camera_info_data = (
                yaml.safe_load(
                    stream
                )
            )

        with transform_path.open(
            "r",
            encoding="utf-8",
        ) as stream:
            self.transform_data = (
                yaml.safe_load(
                    stream
                )
            )

        if not isinstance(
            self.camera_info_data,
            dict,
        ):
            raise ValueError(
                "camera_info.yaml must contain a mapping."
            )

        if not isinstance(
            self.transform_data,
            dict,
        ):
            raise ValueError(
                "base_to_table_transform.yaml must contain a mapping."
            )

        self.rgb_publishers = tuple(
            self.create_publisher(
                RosImage,
                topic,
                10,
            )
            for topic
            in rgb_topics
        )

        self.depth_publishers = tuple(
            self.create_publisher(
                RosImage,
                topic,
                10,
            )
            for topic
            in depth_topics
        )

        self.camera_info_publisher = (
            self.create_publisher(
                CameraInfo,
                camera_info_topic,
                10,
            )
        )

        self.tf_broadcaster = (
            StaticTransformBroadcaster(
                self
            )
        )

        self.base_frame = (
            base_frame
        )

        self.table_frame = (
            table_frame
        )

        self._publish_transform()

        self.timer = self.create_timer(
            0.25,
            self._publish_scene,
        )

    def _publish_transform(
        self,
    ) -> None:
        rotation = np.asarray(
            self.transform_data[
                "rotation"
            ],
            dtype=np.float64,
        )

        translation = np.asarray(
            self.transform_data[
                "translation"
            ],
            dtype=np.float64,
        )

        quaternion = (
            _rotation_matrix_to_quaternion(
                rotation
            )
        )

        transform = (
            TransformStamped()
        )

        transform.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        transform.header.frame_id = (
            self.base_frame
        )

        transform.child_frame_id = (
            self.table_frame
        )

        transform.transform.translation.x = (
            float(
                translation[0]
            )
        )

        transform.transform.translation.y = (
            float(
                translation[1]
            )
        )

        transform.transform.translation.z = (
            float(
                translation[2]
            )
        )

        transform.transform.rotation.x = (
            float(
                quaternion[0]
            )
        )

        transform.transform.rotation.y = (
            float(
                quaternion[1]
            )
        )

        transform.transform.rotation.z = (
            float(
                quaternion[2]
            )
        )

        transform.transform.rotation.w = (
            float(
                quaternion[3]
            )
        )

        self.tf_broadcaster.sendTransform(
            transform
        )

    def _publish_scene(
        self,
    ) -> None:
        stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        for publisher in (
            self.rgb_publishers
        ):
            rgb_msg = (
                self.bridge
                .cv2_to_imgmsg(
                    self.rgb,
                    encoding="rgb8",
                )
            )

            rgb_msg.header.stamp = (
                stamp
            )

            publisher.publish(
                rgb_msg
            )

        for publisher in (
            self.depth_publishers
        ):
            depth_msg = (
                self.bridge
                .cv2_to_imgmsg(
                    self.depth,
                    encoding="passthrough",
                )
            )

            depth_msg.header.stamp = (
                stamp
            )

            publisher.publish(
                depth_msg
            )

        camera_info_msg = (
            CameraInfo()
        )

        camera_info_msg.header.stamp = (
            stamp
        )

        camera_info_msg.height = int(
            self.camera_info_data[
                "height"
            ]
        )

        camera_info_msg.width = int(
            self.camera_info_data[
                "width"
            ]
        )

        camera_info_msg.distortion_model = str(
            self.camera_info_data[
                "distortion_model"
            ]
        )

        camera_info_msg.d = list(
            self.camera_info_data[
                "d"
            ]
        )

        camera_info_msg.k = list(
            self.camera_info_data[
                "k"
            ]
        )

        camera_info_msg.r = list(
            self.camera_info_data[
                "r"
            ]
        )

        camera_info_msg.p = list(
            self.camera_info_data[
                "p"
            ]
        )

        self.camera_info_publisher.publish(
            camera_info_msg
        )


class _ControlLoopFinished(Exception):
    """Stop the control loop after one complete ROS integration rollout."""


class _RosTestTrajectory:
    """Minimal Trajectory implementation for the SeeDo ROS integration test."""

    def __init__(
        self,
    ) -> None:
        self.entries: list[
            dict
        ] = []

    def __len__(
        self,
    ) -> int:
        return len(
            self.entries
        )

    def append(
        self,
        obs,
        action,
        done,
        reward,
        info=None,
    ) -> None:
        self.entries.append(
            {
                "obs":
                    obs,
                "action":
                    action,
                "done":
                    done,
                "reward":
                    reward,
                "info":
                    info,
            }
        )


def _normalize_task_type(
    value,
) -> str:
    raw_value = getattr(
        value,
        "value",
        value,
    )

    return str(
        raw_value
    ).strip().lower()


def _require_file(
    path: Path,
) -> None:
    if not path.is_file():
        raise AssertionError(
            f"Expected artifact was not generated: {path}"
        )

    if (
        path.stat().st_size
        == 0
    ):
        raise AssertionError(
            f"Expected artifact is empty: {path}"
        )


def _load_json(
    path: Path,
) -> dict:
    _require_file(
        path
    )

    with path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        payload = json.load(
            stream
        )

    if not isinstance(
        payload,
        dict,
    ):
        raise AssertionError(
            f"Expected JSON object in artifact: {path}"
        )

    return payload


def _validate_inputs(
    args: argparse.Namespace,
) -> tuple[
    Path,
    Path,
    Path,
    Path,
]:
    video_path = (
        Path(
            args.video
        )
        .expanduser()
        .resolve()
    )

    expected_video = (
        EXPECTED_VIDEO
        .expanduser()
        .resolve()
    )

    if (
        video_path
        != expected_video
    ):
        raise ValueError(
            "This test is pinned to the canonical demonstration. "
            f"Expected {expected_video}, received {video_path}."
        )

    if not video_path.is_file():
        raise FileNotFoundError(
            f"Demonstration video does not exist: {video_path}"
        )

    scene_dir = (
        Path(
            args.scene_dir
        )
        .expanduser()
        .resolve()
    )

    expected_scene_dir = (
        EXPECTED_SCENE_DIR
        .expanduser()
        .resolve()
    )

    if (
        scene_dir
        != expected_scene_dir
    ):
        raise ValueError(
            "This test is pinned to the canonical runtime scene. "
            f"Expected {expected_scene_dir}, received {scene_dir}."
        )

    transform_path = (
        Path(
            args.base_to_table_transform
        )
        .expanduser()
        .resolve()
    )

    expected_transform = (
        expected_scene_dir
        / "base_to_table_transform.yaml"
    ).resolve()

    if (
        transform_path
        != expected_transform
    ):
        raise ValueError(
            "Unexpected base-to-table transform. "
            f"Expected {expected_transform}, received {transform_path}."
        )

    model_config = (
        Path(
            args.model_config
        )
        .expanduser()
        .resolve()
    )

    if not model_config.is_file():
        raise FileNotFoundError(
            f"Model configuration does not exist: {model_config}"
        )

    artifacts_dir = (
        Path(
            args.artifacts_dir
        )
        .expanduser()
        .resolve()
    )

    artifacts_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    return (
        video_path,
        scene_dir,
        transform_path,
        artifacts_dir,
    )


def _wait_for_ros_runtime_data(
    node: AIControllerNode,
) -> None:
    ai_controller_node_module.wait_for_seedo_runtime_data(
        node,
        timeout=10.0,
    )

    expected_names = set(
        node.seedo_record_camera_names
    )

    deadline = (
        node.get_clock()
        .now()
        .nanoseconds
        + int(
            10.0
            * 1e9
        )
    )

    while (
        node.get_clock()
        .now()
        .nanoseconds
        < deadline
    ):
        with node.seedo_record_lock:
            rgb_ready = (
                expected_names
                <= set(
                    node.seedo_record_rgb_msgs
                )
            )

            depth_ready = (
                expected_names
                <= set(
                    node.seedo_record_depth_msgs
                )
            )

        if (
            rgb_ready
            and depth_ready
        ):
            return

        import time

        time.sleep(
            0.01
        )

    with node.seedo_record_lock:
        missing_rgb = (
            expected_names
            - set(
                node.seedo_record_rgb_msgs
            )
        )

        missing_depth = (
            expected_names
            - set(
                node.seedo_record_depth_msgs
            )
        )

    raise TimeoutError(
        "Timed out waiting for all four SeeDo rollout cameras. "
        f"RGB missing={sorted(missing_rgb)}, "
        f"depth missing={sorted(missing_depth)}"
    )


def _validate_generalized_state(
    controller_node: AIControllerNode,
) -> None:
    controller = (
        controller_node
        .controller
    )

    if (
        controller.perception_mode
        != "generalized"
    ):
        raise AssertionError(
            "ROS integration test requires generalized mode."
        )

    required_values = {
        "action_plan":
            controller.action_plan,
        "demo_structured_scene":
            controller.demo_structured_scene,
        "perception_result":
            controller.perception_result,
        "scene_state":
            controller.scene_state,
        "runtime_structured_scene":
            controller.runtime_structured_scene,
        "structural_matching_result":
            controller.structural_matching_result,
        "replicability_result":
            controller.replicability_result,
        "primitive_plan":
            controller.primitive_plan,
    }

    missing = [
        name
        for name, value
        in required_values.items()
        if value is None
    ]

    if missing:
        raise AssertionError(
            "ROS control loop did not populate complete "
            f"generalized pipeline state: {missing}"
        )

    action_plan = (
        controller.action_plan
    )

    if (
        action_plan.status
        != "completed"
    ):
        raise AssertionError(
            "Action planning did not complete successfully."
        )

    if (
        _normalize_task_type(
            action_plan.task_type
        )
        != "pick_and_place"
    ):
        raise AssertionError(
            "Unexpected demonstration task type: "
            f"{action_plan.task_type!r}"
        )

    if len(
        action_plan.steps
    ) != 1:
        raise AssertionError(
            "Expected exactly one demonstrated action."
        )

    action_step = (
        action_plan.steps[0]
    )

    if (
        action_step.picked_detector_label
        != "green block"
    ):
        raise AssertionError(
            "Unexpected demonstrated pick label: "
            f"{action_step.picked_detector_label!r}"
        )

    if (
        action_step.destination_track_id
        != 0
    ):
        raise AssertionError(
            "Unexpected demonstrated destination track ID: "
            f"{action_step.destination_track_id}"
        )

    demo_ids = {
        obj.object_id
        for obj
        in controller
        .demo_structured_scene
        .objects
    }

    if (
        demo_ids
        != EXPECTED_DEMO_PLACE_IDS
    ):
        raise AssertionError(
            "Unexpected demonstration structured-scene IDs: "
            f"{sorted(demo_ids)}"
        )

    runtime_ids = {
        obj.object_id
        for obj
        in controller
        .scene_state
        .objects
    }

    if (
        runtime_ids
        != EXPECTED_RUNTIME_OBJECT_IDS
    ):
        raise AssertionError(
            "Unexpected runtime SceneState IDs: "
            f"{sorted(runtime_ids)}"
        )

    runtime_place_ids = {
        obj.object_id
        for obj
        in controller
        .runtime_structured_scene
        .objects
    }

    if (
        runtime_place_ids
        != EXPECTED_RUNTIME_PLACE_IDS
    ):
        raise AssertionError(
            "Unexpected runtime structured-scene IDs: "
            f"{sorted(runtime_place_ids)}"
        )

    matching_result = (
        controller
        .structural_matching_result
    )

    if (
        not matching_result.is_valid
        or not matching_result.is_unique
        or len(
            matching_result.valid_mappings
        ) != 1
    ):
        raise AssertionError(
            "Canonical ROS scene did not produce one unique "
            "structural mapping."
        )

    mapping = {
        match.demo_object_id:
            match.runtime_object_id
        for match
        in matching_result
        .valid_mappings[0]
        .matches
    }

    if (
        mapping
        != EXPECTED_STRUCTURAL_MAPPING
    ):
        raise AssertionError(
            "Unexpected structural mapping: "
            f"{mapping}"
        )

    replicability_result = (
        controller
        .replicability_result
    )

    if (
        not replicability_result.replicable
        or replicability_result.failure_reasons
    ):
        raise AssertionError(
            "Canonical ROS task is not replicable: "
            f"{replicability_result.failure_reasons}"
        )

    if len(
        replicability_result.resolved_targets
    ) != 1:
        raise AssertionError(
            "Expected exactly one resolved target pair."
        )

    resolved = (
        replicability_result
        .resolved_targets[0]
    )

    if (
        resolved.runtime_pick_object_id
        != "green_block_0"
        or resolved.runtime_place_object_id
        != "storage_bin_0"
    ):
        raise AssertionError(
            "Unexpected resolved runtime targets: "
            f"pick={resolved.runtime_pick_object_id}, "
            f"place={resolved.runtime_place_object_id}"
        )

    primitive_plan = (
        controller.primitive_plan
    )

    if len(
        primitive_plan.steps
    ) != len(
        EXPECTED_PRIMITIVES
    ):
        raise AssertionError(
            "Unexpected primitive count: "
            f"{len(primitive_plan.steps)}"
        )

    for (
        primitive_step,
        (
            expected_name,
            expected_target,
        ),
    ) in zip(
        primitive_plan.steps,
        EXPECTED_PRIMITIVES,
        strict=True,
    ):
        if (
            primitive_step.name
            != expected_name
            or primitive_step.arguments
            != {
                "target":
                    expected_target,
            }
        ):
            raise AssertionError(
                "Unexpected PrimitivePlan step: "
                f"{primitive_step}"
            )


def _validate_trajectory(
    trajectory: _RosTestTrajectory,
    *,
    total_actions: int,
    expected_depth_shape: tuple[int, ...],
) -> None:
    if len(
        trajectory.entries
    ) != total_actions:
        raise AssertionError(
            "Expected one trajectory entry per low-level action. "
            f"Expected {total_actions}, received "
            f"{len(trajectory.entries)}."
        )

    if not trajectory.entries:
        raise AssertionError(
            "ROS test trajectory is empty."
        )

    done_indices = [
        index
        for index, entry
        in enumerate(
            trajectory.entries
        )
        if bool(
            entry["done"]
        )
    ]

    if done_indices != [
        len(
            trajectory.entries
        )
        - 1
    ]:
        raise AssertionError(
            "Only the final trajectory entry must be done. "
            f"done_indices={done_indices}"
        )

    statuses = []

    for index, entry in enumerate(
        trajectory.entries
    ):
        action = np.asarray(
            entry["action"]
        )

        if (
            action.shape
            != (8,)
        ):
            raise AssertionError(
                f"Trajectory action {index} has invalid shape "
                f"{action.shape}."
            )

        if not np.all(
            np.isfinite(
                action
            )
        ):
            raise AssertionError(
                f"Trajectory action {index} contains non-finite values."
            )

        if float(
            action[7]
        ) not in {
            0.0,
            1.0,
        }:
            raise AssertionError(
                "Dataset gripper action is not binary at entry "
                f"{index}: {action[7]}"
            )

        obs = entry[
            "obs"
        ]

        if not isinstance(
            obs,
            dict,
        ):
            raise AssertionError(
                f"Trajectory observation {index} is not a dictionary."
            )

        for key in (
            ROLLOUT_RGB_KEYS
        ):
            if key not in obs:
                raise AssertionError(
                    f"Trajectory observation {index} is missing {key}."
                )

            encoded = np.asarray(
                obs[key]
            )

            if (
                encoded.size
                == 0
            ):
                raise AssertionError(
                    f"Compressed RGB observation {key} is empty."
                )

        for key in (
            ROLLOUT_DEPTH_KEYS
        ):
            if key not in obs:
                raise AssertionError(
                    f"Trajectory observation {index} is missing {key}."
                )

            depth = np.asarray(
                obs[key]
            )

            if (
                depth.shape
                != expected_depth_shape
            ):
                raise AssertionError(
                    f"Unexpected depth shape for {key}: "
                    f"{depth.shape}"
                )

        obj_bb = obs.get(
            "obj_bb"
        )

        if (
            not isinstance(
                obj_bb,
                dict,
            )
            or not isinstance(
                obj_bb.get(
                    "camera_front"
                ),
                dict,
            )
            or len(
                obj_bb[
                    "camera_front"
                ]
            ) != 8
        ):
            raise AssertionError(
                "Trajectory obj_bb does not contain all 8 runtime objects."
            )

        info = entry.get(
            "info"
        )

        if not isinstance(
            info,
            dict,
        ):
            raise AssertionError(
                f"Trajectory entry {index} has no info dictionary."
            )

        status = str(
            info.get(
                "status",
                "",
            )
        ).strip(
            '"'
        )

        if not status:
            raise AssertionError(
                f"Trajectory entry {index} has an empty status."
            )

        statuses.append(
            status
        )

    expected_statuses = {
        "start",
        "approaching",
        "picking",
        "moving",
        "placing",
        "end",
    }

    missing_statuses = (
        expected_statuses
        - set(
            statuses
        )
    )

    if missing_statuses:
        raise AssertionError(
            "Missing SeeDo rollout statuses: "
            f"{sorted(missing_statuses)}"
        )

    if (
        statuses[-1]
        != "end"
    ):
        raise AssertionError(
            "Final trajectory status must be 'end'."
        )

    if (
        trajectory.entries[-1][
            "reward"
        ]
        != 1
    ):
        raise AssertionError(
            "Final trajectory reward must be 1."
        )


def _validate_artifacts(
    artifacts_dir: Path,
    *,
    total_actions: int,
) -> None:
    required_artifacts = (
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

    for artifact_path in (
        required_artifacts
    ):
        _require_file(
            artifact_path
        )

    motion_plan = _load_json(
        artifacts_dir
        / "motion_layer"
        / "motion_plan.json"
    )

    if (
        motion_plan.get(
            "total_actions"
        )
        != total_actions
    ):
        raise AssertionError(
            "motion_plan.json action count does not match "
            "the ROS control-loop output. "
            f"artifact={motion_plan.get('total_actions')}, "
            f"control_loop={total_actions}"
        )


def run_test(
    args: argparse.Namespace,
) -> int:
    """Exercise AIControllerNode through real ROS RGB-D, CameraInfo and TF."""

    (
        video_path,
        scene_dir,
        transform_path,
        artifacts_dir,
    ) = _validate_inputs(
        args
    )

    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "ai_controller_target:=seedo_controller",
            "-p",
            f"model_config_path:={args.model_config}",
            "-p",
            "move_robot:=False",
            "-p",
            "seedo_execute_gripper:=False",
            "-p",
            f"seedo_artifacts_dir:={artifacts_dir}",
        ]
    )

    controller_node = None
    publisher_node = None
    executor = None
    executor_thread = None

    try:
        print(
            "=== INITIALIZING AI CONTROLLER NODE ==="
        )

        controller_node = (
            AIControllerNode()
        )

        if (
            controller_node.ai_controller_target
            != "seedo_controller"
        ):
            raise AssertionError(
                "AIControllerNode is not using seedo_controller."
            )

        if controller_node.move_robot:
            raise AssertionError(
                "ROS integration test must run with move_robot=False."
            )

        if controller_node.seedo_execute_gripper:
            raise AssertionError(
                "ROS integration test must disable physical gripper "
                "execution."
            )

        if (
            controller_node
            .controller
            .perception_mode
            != "generalized"
        ):
            raise AssertionError(
                "ROS integration test requires generalized mode."
            )

        controller_node.demo_path = str(
            video_path
        )

        # Explicitly force the complete demonstration pipeline.
        controller_node.seedo_precomputed_action_plan_path = ""
        controller_node.seedo_precomputed_demo_structured_scene_path = ""

        print(
            "AIControllerNode initialized successfully"
        )

        print()
        print(
            "=== INITIALIZING ROS TEST PUBLISHER ==="
        )

        publisher_node = (
            SeeDoRuntimePublisher(
                scene_dir=scene_dir,
                transform_path=(
                    transform_path
                ),
                rgb_topics=tuple(
                    controller_node
                    .camera_topic
                ),
                depth_topics=tuple(
                    controller_node
                    .seedo_record_depth_topics
                ),
                camera_info_topic=(
                    controller_node
                    .seedo_camera_info_topic
                ),
                base_frame=(
                    controller_node
                    .frame_id
                ),
                table_frame=(
                    controller_node
                    .seedo_table_frame
                ),
            )
        )

        if (
            controller_node.camera_topic[0]
            != controller_node.seedo_rgb_topic
        ):
            raise AssertionError(
                "Expected camera_topic[0] to match the SeeDo front "
                "RGB topic."
            )

        if (
            controller_node.seedo_record_depth_topics[0]
            != controller_node.seedo_depth_topic
        ):
            raise AssertionError(
                "Expected seedo_record_depth_topics[0] to match "
                "the SeeDo front depth topic."
            )

        executor = (
            MultiThreadedExecutor(
                num_threads=4
            )
        )

        executor.add_node(
            controller_node
        )

        executor.add_node(
            publisher_node
        )

        executor_thread = (
            threading.Thread(
                target=(
                    executor.spin
                ),
                daemon=True,
            )
        )

        executor_thread.start()

        print()
        print(
            "=== VALIDATING REAL ROS INPUT PATH ==="
        )

        _wait_for_ros_runtime_data(
            controller_node
        )

        if (
            controller_node.seedo_rgb_msg
            is None
        ):
            raise AssertionError(
                "Front RGB callback was not triggered."
            )

        if (
            controller_node.seedo_depth_msg
            is None
        ):
            raise AssertionError(
                "Front depth callback was not triggered."
            )

        if (
            controller_node
            .seedo_camera_info_msg
            is None
        ):
            raise AssertionError(
                "CameraInfo callback was not triggered."
            )

        print(
            "[PASS] front RGB-D + CameraInfo callbacks"
        )

        camera_data = (
            controller_node
            ._get_seedo_record_camera_data(
                timeout_sec=10.0
            )
        )

        expected_camera_keys = (
            set(
                ROLLOUT_RGB_KEYS
            )
            | set(
                ROLLOUT_DEPTH_KEYS
            )
        )

        if (
            set(
                camera_data
            )
            != expected_camera_keys
        ):
            raise AssertionError(
                "Four-camera ROS recording data is incomplete: "
                f"{sorted(camera_data)}"
            )

        print(
            "[PASS] four-camera ROS rollout callbacks"
        )

        base_to_table_transform = (
            ai_controller_node_module
            .get_seedo_base_to_table_transform(
                controller_node
            )
        )

        expected_rotation = (
            np.asarray(
                publisher_node
                .transform_data[
                    "rotation"
                ],
                dtype=np.float64,
            )
        )

        expected_translation = (
            np.asarray(
                publisher_node
                .transform_data[
                    "translation"
                ],
                dtype=np.float64,
            )
        )

        np.testing.assert_allclose(
            base_to_table_transform[
                "rotation"
            ],
            expected_rotation,
            atol=1e-6,
        )

        np.testing.assert_allclose(
            base_to_table_transform[
                "translation"
            ],
            expected_translation,
            atol=1e-6,
        )

        print(
            "[PASS] base_link <- table_0 TF"
        )

        runtime_input = (
            ai_controller_node_module
            .get_seedo_runtime_input(
                controller_node,
                base_to_table_transform,
            )
        )

        if (
            runtime_input[
                "rgb"
            ].shape
            != publisher_node
            .rgb
            .shape
        ):
            raise AssertionError(
                "ROS runtime RGB shape differs from published scene."
            )

        if (
            runtime_input[
                "depth"
            ].shape
            != publisher_node
            .depth
            .shape
        ):
            raise AssertionError(
                "ROS runtime depth shape differs from published scene."
            )

        if not np.array_equal(
            runtime_input[
                "rgb"
            ],
            publisher_node.rgb,
        ):
            raise AssertionError(
                "ROS runtime RGB data differs from published RGB."
            )

        if not np.allclose(
            runtime_input[
                "depth"
            ],
            publisher_node.depth,
            equal_nan=True,
        ):
            raise AssertionError(
                "ROS runtime depth differs from published depth."
            )

        print(
            "[PASS] get_seedo_runtime_input() through ROS messages"
        )

        # get_synced_images() must also use the actual front-camera
        # ROS callback rather than a mocked image path.
        synced_images = (
            controller_node
            .get_synced_images(
                timeout_sec=5.0
            )
        )

        if (
            not isinstance(
                synced_images,
                list,
            )
            or len(
                synced_images
            ) != 1
        ):
            raise AssertionError(
                "SeeDo get_synced_images() did not return one front image."
            )

        if not np.array_equal(
            synced_images[0],
            publisher_node.rgb,
        ):
            raise AssertionError(
                "get_synced_images() RGB differs from published RGB."
            )

        print(
            "[PASS] get_synced_images() through real ROS callback"
        )

        print()
        print(
            "=== PREPARING REAL CONTROL LOOP ==="
        )

        robot_state = {
            EEF_POS_NAME:
                INITIAL_EEF_POSITION.copy(),
            EEF_QUAT_NAME:
                INITIAL_EEF_ORIENTATION.copy(),
        }

        def fake_capture_robot_state():
            return {
                EEF_POS_NAME:
                    robot_state[
                        EEF_POS_NAME
                    ].copy(),
                EEF_QUAT_NAME:
                    robot_state[
                        EEF_QUAT_NAME
                    ].copy(),
            }

        original_inference = (
            controller_node
            .controller
            .inference
        )

        inference_results: dict[
            int,
            object,
        ] = {}

        def tracked_inference(
            input_data,
            t=0,
            save_path=None,
        ):
            result = original_inference(
                input_data=input_data,
                t=t,
                save_path=save_path,
            )

            inference_results[
                int(t)
            ] = result

            return result

        saved_trajectory = None
        saved_rollout_call = None

        def fake_save_rollout(
            traj,
            save_path,
            task_id,
            traj_number,
        ):
            nonlocal saved_trajectory, saved_rollout_call

            saved_trajectory = traj

            saved_rollout_call = {
                "save_path":
                    save_path,
                "task_id":
                    task_id,
                "traj_number":
                    traj_number,
            }

            print()
            print(
                "=== CONTROL LOOP REACHED SAVE_ROLLOUT ==="
            )

            raise _ControlLoopFinished()

        input_values = iter(
            [
                "0",
                "",
                str(
                    args.task_id
                ),
            ]
        )

        def fake_input(
            prompt="",
        ):
            try:
                value = next(
                    input_values
                )
            except StopIteration as exc:
                raise AssertionError(
                    "control_loop() requested more interactive "
                    "inputs than expected."
                ) from exc

            print(
                f"[ROS test input] "
                f"{prompt}{value}"
            )

            return value

        print()
        print(
            "=== RUNNING CONTROL LOOP WITH REAL ROS SCENE INPUT ==="
        )

        with (
            patch(
                "builtins.input",
                side_effect=(
                    fake_input
                ),
            ),
            patch.object(
                ai_controller_node_module,
                "_get_trajectory_cls",
                return_value=(
                    _RosTestTrajectory
                ),
            ),
            patch.object(
                controller_node,
                "_capture_robot_state",
                side_effect=(
                    fake_capture_robot_state
                ),
            ),
            patch.object(
                controller_node,
                "save_rollout",
                side_effect=(
                    fake_save_rollout
                ),
            ),
            patch.object(
                controller_node
                .controller,
                "inference",
                side_effect=(
                    tracked_inference
                ),
            ),
        ):
            try:
                controller_node.control_loop()
            except _ControlLoopFinished:
                pass

        print()
        print(
            "=== VALIDATING GENERALIZED ROS PIPELINE ==="
        )

        _validate_generalized_state(
            controller_node
        )

        controller = (
            controller_node
            .controller
        )

        primitive_count = len(
            controller
            .primitive_plan
            .steps
        )

        resolved = (
            controller
            .replicability_result
            .resolved_targets[0]
        )

        print(
            f"Primitive count: "
            f"{primitive_count}"
        )

        print(
            "Resolved runtime targets: "
            f"pick={resolved.runtime_pick_object_id}, "
            f"place={resolved.runtime_place_object_id}"
        )

        print()
        print(
            "=== VALIDATING CONTROL-LOOP INFERENCE / MOTION ==="
        )

        if (
            0
            not in inference_results
        ):
            raise AssertionError(
                "control_loop() did not execute inference(t=0)."
            )

        if (
            inference_results[
                0
            ]
            is not None
        ):
            raise AssertionError(
                "inference(t=0) must return None."
            )

        print(
            "[PASS] t=0 runtime perception/planning"
        )

        total_low_level_actions = 0

        for t in range(
            1,
            primitive_count + 1,
        ):
            if (
                t
                not in inference_results
            ):
                raise AssertionError(
                    "control_loop() did not execute "
                    f"inference(t={t})."
                )

            actions = (
                inference_results[
                    t
                ]
            )

            if (
                not isinstance(
                    actions,
                    list,
                )
                or not actions
            ):
                raise AssertionError(
                    f"Invalid low-level action list at t={t}: "
                    f"{actions!r}"
                )

            primitive_step = (
                controller
                .primitive_plan
                .steps[
                    t - 1
                ]
            )

            (
                expected_name,
                expected_target,
            ) = EXPECTED_PRIMITIVES[
                t - 1
            ]

            if (
                primitive_step.name
                != expected_name
                or primitive_step.arguments
                != {
                    "target":
                        expected_target,
                }
            ):
                raise AssertionError(
                    f"Unexpected primitive at t={t}: "
                    f"{primitive_step}"
                )

            for (
                action_index,
                action,
            ) in enumerate(
                actions
            ):
                if (
                    not isinstance(
                        action,
                        np.ndarray,
                    )
                    or action.shape
                    != (8,)
                    or not np.all(
                        np.isfinite(
                            action
                        )
                    )
                ):
                    raise AssertionError(
                        "Invalid low-level action at "
                        f"t={t}, index={action_index}: "
                        f"{action!r}"
                    )

            total_low_level_actions += (
                len(
                    actions
                )
            )

            print(
                f"[PASS] t={t}: "
                f"{primitive_step.name}"
                f"({primitive_step.arguments}) "
                f"-> {len(actions)} action(s)"
            )

        completion_t = (
            primitive_count
            + 1
        )

        if (
            completion_t
            not in inference_results
        ):
            raise AssertionError(
                "control_loop() did not execute final completion inference."
            )

        if (
            inference_results[
                completion_t
            ]
            is not None
        ):
            raise AssertionError(
                "Completion inference must return None."
            )

        if (
            controller.execution_status
            != "completed"
        ):
            raise AssertionError(
                "SeeDoController did not enter completed state."
            )

        print(
            "[PASS] final completion inference"
        )

        print(
            "Total low-level actions processed: "
            f"{total_low_level_actions}"
        )

        print()
        print(
            "=== VALIDATING ROS-SOURCED DATASET TRAJECTORY ==="
        )

        if saved_trajectory is None:
            raise AssertionError(
                "control_loop() never reached save_rollout()."
            )

        _validate_trajectory(
            saved_trajectory,
            total_actions=(
                total_low_level_actions
            ),
            expected_depth_shape=(
                publisher_node
                .depth
                .shape
            ),
        )

        if saved_rollout_call is None:
            raise AssertionError(
                "save_rollout() call metadata was not captured."
            )

        if (
            saved_rollout_call[
                "task_id"
            ]
            != str(
                args.task_id
            ).zfill(
                2
            )
        ):
            raise AssertionError(
                "Unexpected task ID passed to save_rollout(): "
                f"{saved_rollout_call['task_id']!r}"
            )

        print(
            "[PASS] one dataset entry per low-level action"
        )

        print(
            "[PASS] four-camera data originated from ROS callbacks"
        )

        print(
            "[PASS] final done/reward/status=end"
        )

        print()
        print(
            "=== VALIDATING ARTIFACTS ==="
        )

        if (
            controller.artifacts_dir
            is None
        ):
            raise AssertionError(
                "Controller has no persistent artifact directory."
            )

        actual_artifacts_dir = (
            Path(
                controller
                .artifacts_dir
            )
            .expanduser()
            .resolve()
        )

        if (
            actual_artifacts_dir
            != artifacts_dir
        ):
            raise AssertionError(
                "Controller used unexpected artifact directory: "
                f"{actual_artifacts_dir}"
            )

        _validate_artifacts(
            actual_artifacts_dir,
            total_actions=(
                total_low_level_actions
            ),
        )

        print(
            "[PASS] complete generalized artifact tree"
        )

        print(
            "[PASS] timings.json"
        )

        print(
            "[PASS] motion_plan.json action count"
        )

        if (
            controller_node
            .pause_executor
            .is_set()
        ):
            raise AssertionError(
                "pause_executor remained set after demonstration processing."
            )

        print()
        print(
            "SEEDO NODE ROS END-TO-END TEST PASSED"
        )

        return 0

    finally:
        if (
            executor
            is not None
        ):
            executor.shutdown()

        if (
            executor_thread
            is not None
            and executor_thread.is_alive()
        ):
            executor_thread.join(
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


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--video",
        required=True,
    )

    parser.add_argument(
        "--scene-dir",
        required=True,
    )

    parser.add_argument(
        "--base-to-table-transform",
        required=True,
    )

    parser.add_argument(
        "--model-config",
        required=True,
    )

    parser.add_argument(
        "--artifacts-dir",
        default=(
            "/seedo_tests/seedo_node_ros"
        ),
    )

    parser.add_argument(
        "--task-id",
        default="1",
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
