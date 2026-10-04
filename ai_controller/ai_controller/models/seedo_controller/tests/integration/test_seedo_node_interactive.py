from __future__ import annotations

import argparse
import json
import threading
import time
from pathlib import Path

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
from sensor_msgs.msg import JointState
from tf2_ros import StaticTransformBroadcaster

from ai_controller.ai_controller_node import AIControllerNode

from ai_controller.utils.utils import (
    EEF_POS_NAME,
    EEF_QUAT_NAME,
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


EXPECTED_VIDEO = Path(
    "/test_dataset/pick_place/human_rgb_pick_place/"
    "task_00/traj000/converted/traj000-h264-30fps.mp4"
)

EXPECTED_SCENE_DIR = Path(
    "/scene_capture/without_distractors/"
    "scene_1_no_distractors"
)

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


def _rotation_matrix_to_quaternion(
    rotation: np.ndarray,
) -> np.ndarray:
    """Convert a 3x3 rotation matrix to (x, y, z, w)."""

    matrix = np.asarray(
        rotation,
        dtype=np.float64,
    )

    if matrix.shape != (3, 3):
        raise ValueError(
            "Rotation matrix must have shape (3, 3). "
            f"Got {matrix.shape}."
        )

    trace = np.trace(matrix)

    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0

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
        matrix[0, 0] > matrix[1, 1]
        and matrix[0, 0] > matrix[2, 2]
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

    elif matrix[1, 1] > matrix[2, 2]:
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
        [x, y, z, w],
        dtype=np.float64,
    )

    quaternion /= np.linalg.norm(
        quaternion
    )

    return quaternion


class SeeDoInteractiveRuntimePublisher(Node):
    """Publish a complete simulated ROS runtime for the interactive test."""

    def __init__(
        self,
        scene_dir: str | Path,
        transform_path: str | Path,
        seedo_rgb_topic: str,
        seedo_depth_topic: str,
        seedo_camera_info_topic: str,
        camera_topics: list[str],
        depth_topics: list[str],
        joint_states_topic: str,
        joint_robot_names: list[str],
        gripper_robot_names: list[str],
        base_frame: str,
        table_frame: str,
        eef_frame: str,
    ) -> None:
        super().__init__(
            "seedo_interactive_runtime_test_publisher"
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

        rgb_path = scene_dir / "rgb.png"
        depth_path = scene_dir / "depth.npy"

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
                    "Required interactive test input "
                    f"does not exist: {path}"
                )

        rgb_bgr = cv2.imread(
            str(rgb_path)
        )

        if rgb_bgr is None:
            raise RuntimeError(
                f"Could not load RGB image: {rgb_path}"
            )

        self.rgb = cv2.cvtColor(
            rgb_bgr,
            cv2.COLOR_BGR2RGB,
        )

        self.depth = np.load(
            depth_path
        )

        with camera_info_path.open(
            "r",
            encoding="utf-8",
        ) as stream:
            self.camera_info_data = (
                yaml.safe_load(stream)
            )

        with transform_path.open(
            "r",
            encoding="utf-8",
        ) as stream:
            self.transform_data = (
                yaml.safe_load(stream)
            )

        self.base_frame = base_frame
        self.table_frame = table_frame
        self.eef_frame = eef_frame

        self.joint_robot_names = list(
            joint_robot_names
        )

        self.gripper_robot_names = list(
            gripper_robot_names
        )

        if len(camera_topics) != 4:
            raise ValueError(
                "Interactive test expects exactly four RGB camera topics."
            )

        if len(depth_topics) != 4:
            raise ValueError(
                "Interactive test expects exactly four depth camera topics."
            )

        #
        # SeeDo RGB-D publishers.
        #
        self.seedo_rgb_publisher = (
            self.create_publisher(
                RosImage,
                seedo_rgb_topic,
                10,
            )
        )

        self.seedo_depth_publisher = (
            self.create_publisher(
                RosImage,
                seedo_depth_topic,
                10,
            )
        )

        self.camera_info_publisher = (
            self.create_publisher(
                CameraInfo,
                seedo_camera_info_topic,
                10,
            )
        )

        #
        # The first legacy camera topic is normally the same
        # ZED-front topic used by SeeDo. The SeeDo publisher
        # already publishes that topic, so only create additional
        # publishers for the remaining camera topics.
        #
        self.additional_camera_publishers = []

        for topic in camera_topics:
            if topic == seedo_rgb_topic:
                continue

            self.additional_camera_publishers.append(
                self.create_publisher(
                    RosImage,
                    topic,
                    10,
                )
            )

        self.additional_depth_publishers = []

        for topic in depth_topics:
            if topic == seedo_depth_topic:
                continue

            self.additional_depth_publishers.append(
                self.create_publisher(
                    RosImage,
                    topic,
                    10,
                )
            )

        #
        # Robot-state publisher.
        #
        self.joint_state_publisher = (
            self.create_publisher(
                JointState,
                joint_states_topic,
                10,
            )
        )

        #
        # Static transforms:
        #
        #   base_link -> table_0
        #   base_link -> tcp_link
        #
        self.tf_broadcaster = (
            StaticTransformBroadcaster(
                self
            )
        )

        self._publish_static_transforms()

        #
        # Continuously publish the simulated sensor and robot
        # state so the AIControllerNode consumes them through
        # its normal ROS subscriptions.
        #
        self.timer = self.create_timer(
            0.25,
            self._publish_runtime,
        )

    def _build_table_transform(
        self,
    ) -> TransformStamped:
        rotation = np.asarray(
            self.transform_data["rotation"],
            dtype=np.float64,
        )

        translation = np.asarray(
            self.transform_data["translation"],
            dtype=np.float64,
        )

        quaternion = (
            _rotation_matrix_to_quaternion(
                rotation
            )
        )

        transform = TransformStamped()

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

        transform.transform.translation.x = float(
            translation[0]
        )

        transform.transform.translation.y = float(
            translation[1]
        )

        transform.transform.translation.z = float(
            translation[2]
        )

        transform.transform.rotation.x = float(
            quaternion[0]
        )

        transform.transform.rotation.y = float(
            quaternion[1]
        )

        transform.transform.rotation.z = float(
            quaternion[2]
        )

        transform.transform.rotation.w = float(
            quaternion[3]
        )

        return transform

    def _build_eef_transform(
        self,
    ) -> TransformStamped:
        transform = TransformStamped()

        transform.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        transform.header.frame_id = (
            self.base_frame
        )

        transform.child_frame_id = (
            self.eef_frame
        )

        transform.transform.translation.x = float(
            INITIAL_EEF_POSITION[0]
        )

        transform.transform.translation.y = float(
            INITIAL_EEF_POSITION[1]
        )

        transform.transform.translation.z = float(
            INITIAL_EEF_POSITION[2]
        )

        transform.transform.rotation.x = float(
            INITIAL_EEF_ORIENTATION[0]
        )

        transform.transform.rotation.y = float(
            INITIAL_EEF_ORIENTATION[1]
        )

        transform.transform.rotation.z = float(
            INITIAL_EEF_ORIENTATION[2]
        )

        transform.transform.rotation.w = float(
            INITIAL_EEF_ORIENTATION[3]
        )

        return transform

    def _publish_static_transforms(
        self,
    ) -> None:
        self.tf_broadcaster.sendTransform(
            [
                self._build_table_transform(),
                self._build_eef_transform(),
            ]
        )

    def _publish_joint_state(
        self,
        stamp,
    ) -> None:
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

        self.joint_state_publisher.publish(
            msg
        )

    def _publish_runtime(
        self,
    ) -> None:
        stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        #
        # Front RGB image.
        #
        rgb_msg = self.bridge.cv2_to_imgmsg(
            self.rgb,
            encoding="rgb8",
        )

        rgb_msg.header.stamp = stamp

        self.seedo_rgb_publisher.publish(
            rgb_msg
        )

        #
        # Remaining legacy camera topics.
        #
        for publisher in (
            self.additional_camera_publishers
        ):
            camera_msg = (
                self.bridge.cv2_to_imgmsg(
                    self.rgb,
                    encoding="rgb8",
                )
            )

            camera_msg.header.stamp = stamp

            publisher.publish(
                camera_msg
            )

        #
        # Depth.
        #
        depth_msg = self.bridge.cv2_to_imgmsg(
            self.depth,
            encoding="passthrough",
        )

        depth_msg.header.stamp = stamp

        self.seedo_depth_publisher.publish(
            depth_msg
        )

        for publisher in (
            self.additional_depth_publishers
        ):
            additional_depth_msg = (
                self.bridge.cv2_to_imgmsg(
                    self.depth,
                    encoding="passthrough",
                )
            )

            additional_depth_msg.header.stamp = stamp

            publisher.publish(
                additional_depth_msg
            )

        #
        # CameraInfo.
        #
        camera_info_msg = CameraInfo()

        camera_info_msg.header.stamp = stamp

        camera_info_msg.height = int(
            self.camera_info_data["height"]
        )

        camera_info_msg.width = int(
            self.camera_info_data["width"]
        )

        camera_info_msg.distortion_model = (
            self.camera_info_data[
                "distortion_model"
            ]
        )

        camera_info_msg.d = list(
            self.camera_info_data["d"]
        )

        camera_info_msg.k = list(
            self.camera_info_data["k"]
        )

        camera_info_msg.r = list(
            self.camera_info_data["r"]
        )

        camera_info_msg.p = list(
            self.camera_info_data["p"]
        )

        self.camera_info_publisher.publish(
            camera_info_msg
        )

        #
        # Simulated robot joint state.
        #
        self._publish_joint_state(
            stamp
        )


def _snapshot_rollouts(
    rollout_root: Path,
) -> tuple[set[Path], set[Path]]:
    if not rollout_root.exists():
        return set(), set()

    pkl_files = set(
        rollout_root.rglob(
            "traj_*.pkl"
        )
    )

    json_files = set(
        rollout_root.rglob(
            "traj_*.json"
        )
    )

    return pkl_files, json_files


def _wait_for_runtime_ready(
    controller_node: AIControllerNode,
    timeout: float = 10.0,
) -> None:
    """Preload all simulated ROS state before entering control_loop()."""

    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        rclpy.spin_once(
            controller_node,
            timeout_sec=0.1,
        )

        seedo_ready = (
            controller_node.seedo_rgbd_event.is_set()
            and controller_node.seedo_camera_info_msg
            is not None
        )

        joint_state_ready = (
            controller_node.latest_joint_state
            is not None
        )

        with controller_node.seedo_record_lock:
            expected_camera_names = set(
                controller_node.seedo_record_camera_names
            )

            rollout_rgb_ready = (
                expected_camera_names
                <= set(
                    controller_node.seedo_record_rgb_msgs
                )
            )

            rollout_depth_ready = (
                expected_camera_names
                <= set(
                    controller_node.seedo_record_depth_msgs
                )
            )

        table_tf_ready = False
        eef_tf_ready = False

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

            eef_tf_ready = True

        except Exception:
            pass

        if (
            seedo_ready
            and rollout_rgb_ready
            and rollout_depth_ready
            and joint_state_ready
            and table_tf_ready
            and eef_tf_ready
        ):
            return

    raise TimeoutError(
        "Timed out waiting for the complete simulated ROS "
        "runtime: front RGB-D, all four rollout RGB-D cameras, "
        "CameraInfo, /joint_states, base->table TF and base->EEF TF."
    )


def _validate_inputs(
    args: argparse.Namespace,
) -> tuple[Path, Path, Path, Path, Path]:
    video_path = (
        Path(args.video)
        .expanduser()
        .resolve()
    )

    expected_video = (
        EXPECTED_VIDEO
        .expanduser()
        .resolve()
    )

    if video_path != expected_video:
        raise ValueError(
            "This interactive dry-run is pinned to the canonical "
            "pick-and-place demonstration. "
            f"Expected {expected_video}, received {video_path}."
        )

    if not video_path.is_file():
        raise FileNotFoundError(
            f"Canonical demonstration video does not exist: {video_path}"
        )

    scene_dir = (
        Path(args.scene_dir)
        .expanduser()
        .resolve()
    )

    expected_scene_dir = (
        EXPECTED_SCENE_DIR
        .expanduser()
        .resolve()
    )

    if scene_dir != expected_scene_dir:
        raise ValueError(
            "This interactive dry-run is pinned to the canonical "
            "no-distractors runtime scene. "
            f"Expected {expected_scene_dir}, received {scene_dir}."
        )

    transform_path = (
        Path(args.base_to_table_transform)
        .expanduser()
        .resolve()
    )

    expected_transform = (
        expected_scene_dir
        / "base_to_table_transform.yaml"
    ).resolve()

    if transform_path != expected_transform:
        raise ValueError(
            "Unexpected base-to-table transform. "
            f"Expected {expected_transform}, received {transform_path}."
        )

    model_config_path = (
        Path(args.model_config)
        .expanduser()
        .resolve()
    )

    if not model_config_path.is_file():
        raise FileNotFoundError(
            f"Model configuration does not exist: {model_config_path}"
        )

    artifacts_dir = (
        Path(args.artifacts_dir)
        .expanduser()
        .resolve()
    )

    rollout_base_dir = (
        Path(args.rollouts_dir)
        .expanduser()
        .resolve()
    )

    artifacts_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    rollout_base_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    return (
        video_path,
        scene_dir,
        transform_path,
        artifacts_dir,
        rollout_base_dir,
    )


def _validate_generalized_state(
    controller_node: AIControllerNode,
) -> None:
    controller = controller_node.controller

    if controller.perception_mode != "generalized":
        raise AssertionError(
            "Interactive dry-run requires generalized perception mode."
        )

    required_state = {
        "action_plan": controller.action_plan,
        "demo_structured_scene": controller.demo_structured_scene,
        "scene_state": controller.scene_state,
        "runtime_structured_scene": controller.runtime_structured_scene,
        "structural_matching_result": controller.structural_matching_result,
        "replicability_result": controller.replicability_result,
        "primitive_plan": controller.primitive_plan,
    }

    missing = [
        name
        for name, value
        in required_state.items()
        if value is None
    ]

    if missing:
        raise AssertionError(
            "Interactive control loop did not populate the complete "
            f"generalized pipeline state: {missing}"
        )

    action_plan = controller.action_plan

    task_type = getattr(
        action_plan.task_type,
        "value",
        action_plan.task_type,
    )

    if str(task_type).strip().lower() != "pick_and_place":
        raise AssertionError(
            f"Unexpected action-plan task type: {action_plan.task_type!r}"
        )

    if len(action_plan.steps) != 1:
        raise AssertionError(
            f"Expected one demonstration action, got {len(action_plan.steps)}."
        )

    action_step = action_plan.steps[0]

    if action_step.picked_detector_label != "green block":
        raise AssertionError(
            "Unexpected demonstrated pick label: "
            f"{action_step.picked_detector_label!r}"
        )

    if action_step.destination_track_id != 0:
        raise AssertionError(
            "Unexpected demonstrated destination track ID: "
            f"{action_step.destination_track_id}"
        )

    matching_result = controller.structural_matching_result

    if (
        not matching_result.is_valid
        or not matching_result.is_unique
        or len(matching_result.valid_mappings) != 1
    ):
        raise AssertionError(
            "Canonical interactive scene did not produce one unique "
            "structural mapping."
        )

    mapping = {
        match.demo_object_id: match.runtime_object_id
        for match
        in matching_result.valid_mappings[0].matches
    }

    if mapping != EXPECTED_STRUCTURAL_MAPPING:
        raise AssertionError(
            "Unexpected structural mapping: "
            f"expected={EXPECTED_STRUCTURAL_MAPPING}, received={mapping}"
        )

    replicability_result = controller.replicability_result

    if (
        not replicability_result.replicable
        or replicability_result.failure_reasons
        or len(replicability_result.resolved_targets) != 1
    ):
        raise AssertionError(
            "Canonical interactive task did not resolve as replicable: "
            f"{replicability_result.failure_reasons}"
        )

    resolved = replicability_result.resolved_targets[0]

    if (
        resolved.runtime_pick_object_id != "green_block_0"
        or resolved.runtime_place_object_id != "storage_bin_0"
    ):
        raise AssertionError(
            "Unexpected resolved runtime targets: "
            f"pick={resolved.runtime_pick_object_id}, "
            f"place={resolved.runtime_place_object_id}"
        )

    primitive_plan = controller.primitive_plan

    if len(primitive_plan.steps) != len(EXPECTED_PRIMITIVES):
        raise AssertionError(
            "Unexpected number of primitives: "
            f"{len(primitive_plan.steps)}"
        )

    for primitive_step, (expected_name, expected_target) in zip(
        primitive_plan.steps,
        EXPECTED_PRIMITIVES,
        strict=True,
    ):
        if (
            primitive_step.name != expected_name
            or primitive_step.arguments
            != {"target": expected_target}
        ):
            raise AssertionError(
                f"Unexpected PrimitivePlan step: {primitive_step}"
            )


def _validate_artifacts(
    artifacts_dir: Path,
) -> None:
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
                f"Expected artifact was not generated: {path}"
            )

        if path.stat().st_size == 0:
            raise AssertionError(
                f"Expected artifact is empty: {path}"
            )


def _validate_outcome_json(
    path: Path,
) -> None:
    with path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        payload = json.load(stream)

    if payload.get("program_status") != "completed":
        raise AssertionError(
            "Interactive rollout outcome metadata does not report "
            f"program_status='completed': {payload}"
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

    missing = required_keys - set(payload)

    if missing:
        raise AssertionError(
            "Interactive rollout outcome metadata is incomplete: "
            f"missing={sorted(missing)}"
        )



def run_test(
    args: argparse.Namespace,
) -> int:
    (
        video_path,
        scene_dir,
        transform_path,
        artifacts_dir,
        rollout_base_dir,
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
            f"demo_path:={video_path}",
            "-p",
            "move_robot:=False",
            "-p",
            "seedo_execute_gripper:=False",
            "-p",
            f"seedo_artifacts_dir:={artifacts_dir}",
            "-p",
            f"save_rollout_path:={rollout_base_dir}",
        ]
    )

    controller_node = None
    publisher_node = None
    publisher_executor = None
    publisher_thread = None

    try:
        print(
            "=== INITIALIZING AI CONTROLLER NODE ==="
        )

        controller_node = AIControllerNode()

        if (
            controller_node.ai_controller_target
            != "seedo_controller"
        ):
            raise AssertionError(
                "AIControllerNode is not configured "
                "to use seedo_controller."
            )

        if controller_node.move_robot:
            raise AssertionError(
                "Interactive dry-run must use "
                "move_robot=False."
            )

        if controller_node.seedo_execute_gripper:
            raise AssertionError(
                "Interactive dry-run must use "
                "seedo_execute_gripper=False."
            )

        if (
            controller_node.controller.perception_mode
            != "generalized"
        ):
            raise AssertionError(
                "Interactive dry-run requires generalized mode."
            )

        controller_node.demo_path = str(
            video_path
        )

        controller_node.seedo_precomputed_action_plan_path = ""
        controller_node.seedo_precomputed_demo_structured_scene_path = ""

        print(
            "AIControllerNode initialized successfully"
        )

        print(
            "\n=== INITIALIZING COMPLETE ROS SIMULATOR ==="
        )

        publisher_node = (
            SeeDoInteractiveRuntimePublisher(
                scene_dir=scene_dir,
                transform_path=(
                    transform_path
                ),
                seedo_rgb_topic=(
                    controller_node.seedo_rgb_topic
                ),
                seedo_depth_topic=(
                    controller_node.seedo_depth_topic
                ),
                seedo_camera_info_topic=(
                    controller_node
                    .seedo_camera_info_topic
                ),
                camera_topics=list(
                    controller_node.camera_topic
                ),
                depth_topics=list(
                    controller_node.seedo_record_depth_topics
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
                    controller_node.frame_id
                ),
                table_frame=(
                    controller_node
                    .seedo_table_frame
                ),
                eef_frame=(
                    controller_node
                    .eef_frame_name
                ),
            )
        )

        #
        # Only the simulated external ROS world is spun in a
        # separate executor. AIControllerNode remains outside
        # this executor because its control_loop/get_synced_images
        # already use rclpy.spin_once(self).
        #
        publisher_executor = (
            MultiThreadedExecutor(
                num_threads=1
            )
        )

        publisher_executor.add_node(
            publisher_node
        )

        publisher_thread = threading.Thread(
            target=publisher_executor.spin,
            daemon=True,
        )

        publisher_thread.start()

        print(
            "\n=== WAITING FOR COMPLETE ROS RUNTIME ==="
        )

        _wait_for_runtime_ready(
            controller_node,
            timeout=10.0,
        )

        print(
            "RGB-D, four-camera recording data, CameraInfo, "
            "JointState and TF runtime received successfully"
        )

        record_camera_data = (
            controller_node
            ._get_seedo_record_camera_data(
                timeout_sec=10.0
            )
        )

        if set(record_camera_data) != EXPECTED_ROLLOUT_KEYS:
            raise AssertionError(
                "Interactive ROS simulator did not provide the "
                "complete four-camera rollout data: "
                f"{sorted(record_camera_data)}"
            )

        print(
            "[PASS] four-camera RGB/depth recording callbacks"
        )

        print(
            "\n=== VALIDATING REAL ROBOT-STATE CAPTURE ==="
        )

        robot_state = (
            controller_node
            ._capture_robot_state()
        )

        eef_position = robot_state.get(
            EEF_POS_NAME
        )

        eef_orientation = robot_state.get(
            EEF_QUAT_NAME
        )

        if eef_position is None:
            raise AssertionError(
                "_capture_robot_state() did not "
                "produce the EEF position."
            )

        if eef_orientation is None:
            raise AssertionError(
                "_capture_robot_state() did not "
                "produce the EEF orientation."
            )

        if not np.allclose(
            eef_position,
            INITIAL_EEF_POSITION,
            atol=1e-9,
        ):
            raise AssertionError(
                "Captured EEF position does not "
                "match the simulated TCP pose."
            )

        if not np.allclose(
            eef_orientation,
            INITIAL_EEF_ORIENTATION,
            atol=1e-9,
        ):
            raise AssertionError(
                "Captured EEF orientation does not "
                "match the simulated TCP pose."
            )

        print(
            "Real _capture_robot_state() validated"
        )

        #
        # AIControllerNode appends:
        #
        #   seedo_controller/pick_place
        #
        # to the save_rollout_path parameter.
        #
        actual_rollout_root = Path(
            controller_node.save_rollout_path
        )

        before_pkl, before_json = (
            _snapshot_rollouts(
                actual_rollout_root
            )
        )

        print(
            "\n=== INTERACTIVE DRY-RUN READY ==="
        )

        print(
            "The real AIControllerNode.control_loop() "
            "will now start."
        )

        print(
            "move_robot=False and seedo_execute_gripper=False: "
            "no physical robot or gripper command will be sent."
        )

        print()
        print(
            "Complete ONE trajectory normally."
        )

        print(
            "After the trajectory is saved and you answer "
            "the rollout outcome questions, wait until the "
            "control loop asks again:"
        )

        print()
        print(
            "  Press Enter to start the control loop..."
        )

        print()
        print(
            "At that point press Ctrl+C to finish the test."
        )

        print(
            "\n=== STARTING REAL CONTROL LOOP ==="
        )

        interrupted_by_user = False

        try:
            controller_node.control_loop()

        except KeyboardInterrupt:
            interrupted_by_user = True

            print(
                "\nCtrl+C received after interactive "
                "control-loop execution."
            )

        if not interrupted_by_user:
            raise AssertionError(
                "Interactive test ended without the expected "
                "manual Ctrl+C."
            )

        print(
            "\n=== VALIDATING SAVED ROLLOUT ==="
        )

        after_pkl, after_json = (
            _snapshot_rollouts(
                actual_rollout_root
            )
        )

        new_pkl = sorted(
            after_pkl - before_pkl
        )

        new_json = sorted(
            after_json - before_json
        )

        if not new_pkl:
            raise AssertionError(
                "No new trajectory .pkl file was saved."
            )

        if not new_json:
            raise AssertionError(
                "No new trajectory outcome .json "
                "file was saved."
            )

        if len(new_pkl) != 1:
            raise AssertionError(
                "Expected exactly one new trajectory "
                f".pkl file, found {len(new_pkl)}: "
                f"{new_pkl}"
            )

        if len(new_json) != 1:
            raise AssertionError(
                "Expected exactly one new trajectory "
                f".json file, found {len(new_json)}: "
                f"{new_json}"
            )

        if (
            new_pkl[0].stem
            != new_json[0].stem
        ):
            raise AssertionError(
                "Saved trajectory and outcome metadata "
                "do not refer to the same trajectory."
            )

        print(
            "[PASS] Trajectory saved: "
            f"{new_pkl[0]}"
        )

        print(
            "[PASS] Outcome metadata saved: "
            f"{new_json[0]}"
        )

        if new_pkl[0].stat().st_size == 0:
            raise AssertionError(
                f"Saved trajectory is empty: {new_pkl[0]}"
            )

        _validate_outcome_json(
            new_json[0]
        )

        print(
            "[PASS] outcome metadata schema/program_status"
        )

        print(
            "\n=== VALIDATING GENERALIZED PIPELINE STATE ==="
        )

        _validate_generalized_state(
            controller_node
        )

        resolved = (
            controller_node
            .controller
            .replicability_result
            .resolved_targets[0]
        )

        print(
            "Resolved runtime targets: "
            f"pick={resolved.runtime_pick_object_id}, "
            f"place={resolved.runtime_place_object_id}"
        )

        print(
            "\n=== VALIDATING SEEDO ARTIFACTS ==="
        )

        _validate_artifacts(
            artifacts_dir
        )

        print(
            "[PASS] complete generalized artifact tree"
        )
        print(
            "[PASS] timings.json"
        )

        if (
            controller_node
            .controller
            .execution_status
            != "completed"
        ):
            raise AssertionError(
                "SeeDoController is not in the "
                "completed state after the trajectory. "
                "Current status: "
                f"{controller_node.controller.execution_status}"
            )

        print(
            "[PASS] SeeDo execution state: completed"
        )

        print(
            "\nINTERACTIVE ROS DRY-RUN TEST PASSED"
        )

        return 0

    finally:
        if publisher_executor is not None:
            publisher_executor.shutdown()

        if (
            publisher_thread is not None
            and publisher_thread.is_alive()
        ):
            publisher_thread.join(
                timeout=2.0
            )

        if publisher_node is not None:
            publisher_node.destroy_node()

        if controller_node is not None:
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
            "/seedo_tests/"
            "ai_controller_node_interactive"
        ),
    )

    parser.add_argument(
        "--rollouts-dir",
        default=(
            "/seedo_tests/"
            "ai_controller_node_interactive/"
            "rollouts"
        ),
    )

    return parser


def main() -> int:
    parser = build_parser()

    args = parser.parse_args()

    return run_test(args)


if __name__ == "__main__":
    raise SystemExit(main())