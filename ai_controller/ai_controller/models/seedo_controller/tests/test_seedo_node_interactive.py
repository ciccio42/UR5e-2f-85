from __future__ import annotations

import argparse
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
            and joint_state_ready
            and table_tf_ready
            and eef_tf_ready
        ):
            return

    raise TimeoutError(
        "Timed out waiting for the complete simulated ROS "
        "runtime: RGB-D, CameraInfo, /joint_states, "
        "base->table TF and base->EEF TF."
    )


def run_test(
    args: argparse.Namespace,
) -> int:
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

    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "ai_controller_target:=seedo_controller",
            "-p",
            f"model_config_path:={args.model_config}",
            "-p",
            f"demo_path:={args.video}",
            "-p",
            "move_robot:=False",
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

        print(
            "AIControllerNode initialized successfully"
        )

        print(
            "\n=== INITIALIZING COMPLETE ROS SIMULATOR ==="
        )

        publisher_node = (
            SeeDoInteractiveRuntimePublisher(
                scene_dir=args.scene_dir,
                transform_path=(
                    args.base_to_table_transform
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
            "RGB-D, CameraInfo, JointState and TF "
            "runtime received successfully"
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
            "move_robot=False: no GoToPose or gripper "
            "command will be sent."
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

        print(
            "\n=== VALIDATING SEEDO ARTIFACTS ==="
        )

        motion_artifact_path = (
            artifacts_dir
            / "motion_layer"
            / "motion_plan.json"
        )

        if not motion_artifact_path.is_file():
            raise AssertionError(
                "Motion Layer artifact was not "
                f"generated: {motion_artifact_path}"
            )

        print(
            "[PASS] Motion Layer artifact: "
            f"{motion_artifact_path}"
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