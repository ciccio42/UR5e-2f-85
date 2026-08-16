from __future__ import annotations

import argparse
import threading

from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from unittest.mock import patch

import ai_controller.ai_controller_node as ai_controller_node_module

from ai_controller.utils.utils import (
    EEF_POS_NAME,
    EEF_QUAT_NAME,
)

from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as RosImage
from tf2_ros import StaticTransformBroadcaster

from ai_controller.ai_controller_node import AIControllerNode


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


class SeeDoRuntimePublisher(Node):
    """Publish an offline scene as ROS runtime inputs."""

    def __init__(
        self,
        scene_dir: str | Path,
        transform_path: str | Path,
        rgb_topic: str,
        depth_topic: str,
        camera_info_topic: str,
        base_frame: str,
        table_frame: str,
    ) -> None:
        super().__init__(
            "seedo_runtime_test_publisher"
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
                    "Required test input does not exist: "
                    f"{path}"
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

        self.rgb_publisher = (
            self.create_publisher(
                RosImage,
                rgb_topic,
                10,
            )
        )

        self.depth_publisher = (
            self.create_publisher(
                RosImage,
                depth_topic,
                10,
            )
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

        self.base_frame = base_frame
        self.table_frame = table_frame

        self._publish_transform()

        self.timer = self.create_timer(
            0.25,
            self._publish_scene,
        )

    def _publish_transform(self) -> None:
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

        transform.transform.translation.x = (
            float(translation[0])
        )
        transform.transform.translation.y = (
            float(translation[1])
        )
        transform.transform.translation.z = (
            float(translation[2])
        )

        transform.transform.rotation.x = (
            float(quaternion[0])
        )
        transform.transform.rotation.y = (
            float(quaternion[1])
        )
        transform.transform.rotation.z = (
            float(quaternion[2])
        )
        transform.transform.rotation.w = (
            float(quaternion[3])
        )

        self.tf_broadcaster.sendTransform(
            transform
        )

    def _publish_scene(self) -> None:
        stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        rgb_msg = self.bridge.cv2_to_imgmsg(
            self.rgb,
            encoding="rgb8",
        )

        rgb_msg.header.stamp = stamp

        depth_msg = self.bridge.cv2_to_imgmsg(
            self.depth,
            encoding="passthrough",
        )

        depth_msg.header.stamp = stamp

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

        self.rgb_publisher.publish(
            rgb_msg
        )

        self.depth_publisher.publish(
            depth_msg
        )

        self.camera_info_publisher.publish(
            camera_info_msg
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


class _ControlLoopFinished(Exception):
    """Stop the control loop after one ROS test trajectory."""


class _RosTestTrajectory:
    """Minimal trajectory used by the ROS integration test."""

    def __init__(self) -> None:
        self.entries: list[dict] = []

    def append(
        self,
        obs,
        action,
        done,
        reward,
    ) -> None:
        self.entries.append(
            {
                "obs": obs,
                "action": action,
                "done": done,
                "reward": reward,
            }
        )
        
def run_test(
    args: argparse.Namespace,
) -> int:
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
            f"seedo_artifacts_dir:={args.artifacts_dir}",
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

        controller_node = AIControllerNode()

        if (
            controller_node.ai_controller_target
            != "seedo_controller"
        ):
            raise AssertionError(
                "AIControllerNode is not using "
                "seedo_controller."
            )

        if controller_node.move_robot:
            raise AssertionError(
                "ROS SeeDo test must run with "
                "move_robot=False."
            )

        controller_node.demo_path = args.video

        print(
            "AIControllerNode initialized successfully"
        )

        print(
            "\n=== INITIALIZING ROS TEST PUBLISHER ==="
        )

        publisher_node = SeeDoRuntimePublisher(
            scene_dir=args.scene_dir,
            transform_path=(
                args.base_to_table_transform
            ),
            rgb_topic=(
                controller_node.seedo_rgb_topic
            ),
            depth_topic=(
                controller_node.seedo_depth_topic
            ),
            camera_info_topic=(
                controller_node
                .seedo_camera_info_topic
            ),
            base_frame=controller_node.frame_id,
            table_frame=(
                controller_node.seedo_table_frame
            ),
        )

        executor = MultiThreadedExecutor(
            num_threads=2
        )

        executor.add_node(
            controller_node
        )

        executor.add_node(
            publisher_node
        )

        executor_thread = threading.Thread(
            target=executor.spin,
            daemon=True,
        )

        executor_thread.start()

        print(
            "\n=== WAITING FOR ROS RUNTIME DATA ==="
        )

        controller_node._wait_for_seedo_runtime_data(
            timeout=10.0
        )

        if controller_node.seedo_rgb_msg is None:
            raise AssertionError(
                "RGB callback was not triggered."
            )

        if controller_node.seedo_depth_msg is None:
            raise AssertionError(
                "Depth callback was not triggered."
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
            "RGB-D and CameraInfo received successfully"
        )

        print(
            "\n=== READING TABLE TF ==="
        )

        base_to_table_transform = (
            controller_node
            ._get_seedo_base_to_table_transform()
        )

        expected_transform = (
            publisher_node.transform_data
        )

        if not np.allclose(
            base_to_table_transform[
                "rotation"
            ],
            np.asarray(
                expected_transform[
                    "rotation"
                ],
                dtype=np.float64,
            ),
            atol=1e-6,
        ):
            raise AssertionError(
                "TF rotation does not match "
                "the offline transform."
            )

        if not np.allclose(
            base_to_table_transform[
                "translation"
            ],
            np.asarray(
                expected_transform[
                    "translation"
                ],
                dtype=np.float64,
            ),
            atol=1e-6,
        ):
            raise AssertionError(
                "TF translation does not match "
                "the offline transform."
            )

        print(
            "base_link <- table_0 TF received successfully"
        )

        print(
            "\n=== BUILDING SEEDO RUNTIME INPUT ==="
        )

        runtime_input = (
            controller_node
            ._get_seedo_runtime_input(
                base_to_table_transform
            )
        )

        if runtime_input["rgb"].shape != (
            publisher_node.rgb.shape
        ):
            raise AssertionError(
                "Received RGB shape does not "
                "match published RGB shape."
            )

        if runtime_input["depth"].shape != (
            publisher_node.depth.shape
        ):
            raise AssertionError(
                "Received depth shape does not "
                "match published depth shape."
            )

        if not np.array_equal(
            runtime_input["rgb"],
            publisher_node.rgb,
        ):
            raise AssertionError(
                "Received RGB data differs "
                "from published RGB data."
            )

        if not np.allclose(
            runtime_input["depth"],
            publisher_node.depth,
            equal_nan=True,
        ):
            raise AssertionError(
                "Received depth data differs "
                "from published depth data."
            )

        print(
            "ROS runtime input matches offline scene"
        )

        print(
            "\n=== PREPARING CONTROL LOOP TEST ==="
        )

        robot_state = {
            EEF_POS_NAME: INITIAL_EEF_POSITION.copy(),
            EEF_QUAT_NAME: INITIAL_EEF_ORIENTATION.copy(),
        }

        original_inference = (
            controller_node.controller.inference
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

            inference_results[t] = result

            return result

        saved_trajectory = None

        def fake_save_rollout(
            traj,
            save_path,
            task_id,
            traj_number,
        ):
            nonlocal saved_trajectory

            saved_trajectory = traj

            print(
                "\n=== CONTROL LOOP REACHED "
                "SAVE_ROLLOUT ==="
            )

            raise _ControlLoopFinished()

        input_values = iter(
            [
                "0",
                "",
                str(args.task_id),
            ]
        )

        def fake_input(prompt=""):
            try:
                value = next(input_values)
            except StopIteration as exc:
                raise AssertionError(
                    "control_loop() requested more "
                    "interactive inputs than expected."
                ) from exc

            print(
                f"[ROS test input] {prompt}{value}"
            )

            return value

        def fake_get_synced_images():
            return [
                publisher_node.rgb
            ]

        def fake_capture_robot_state():
            return {
                EEF_POS_NAME: (
                    robot_state[
                        EEF_POS_NAME
                    ].copy()
                ),
                EEF_QUAT_NAME: (
                    robot_state[
                        EEF_QUAT_NAME
                    ].copy()
                ),
            }

        print(
            "\n=== RUNNING REAL CONTROL LOOP "
            "WITH ROS INPUT ==="
        )

        with (
            patch(
                "builtins.input",
                side_effect=fake_input,
            ),
            patch.object(
                ai_controller_node_module,
                "_get_trajectory_cls",
                return_value=_RosTestTrajectory,
            ),
            patch.object(
                controller_node,
                "get_synced_images",
                side_effect=fake_get_synced_images,
            ),
            patch.object(
                controller_node,
                "_capture_robot_state",
                side_effect=fake_capture_robot_state,
            ),
            patch.object(
                controller_node,
                "save_rollout",
                side_effect=fake_save_rollout,
            ),
            patch.object(
                controller_node.controller,
                "inference",
                side_effect=tracked_inference,
            ),
        ):
            try:
                controller_node.control_loop()
            except _ControlLoopFinished:
                pass

        print(
            "\n=== VALIDATING ROS CONTROL LOOP ==="
        )

        if (
            controller_node.controller.action_plan
            is None
        ):
            raise AssertionError(
                "control_loop() did not generate "
                "an ActionPlan."
            )

        if (
            controller_node.controller.scene_state
            is None
        ):
            raise AssertionError(
                "control_loop() did not generate "
                "a SceneState."
            )

        primitive_plan = (
            controller_node.controller.primitive_plan
        )

        if primitive_plan is None:
            raise AssertionError(
                "control_loop() did not generate "
                "a PrimitivePlan."
            )

        primitive_count = len(
            primitive_plan.steps
        )

        if primitive_count == 0:
            raise AssertionError(
                "SeeDo generated an empty "
                "PrimitivePlan."
            )

        print(
            f"Primitive count: {primitive_count}"
        )

        if 0 not in inference_results:
            raise AssertionError(
                "control_loop() did not execute "
                "inference(t=0)."
            )

        if inference_results[0] is not None:
            raise AssertionError(
                "SeeDo inference(t=0) "
                "must return None."
            )

        print(
            "\n=== VALIDATING LOW-LEVEL ACTIONS ==="
        )

        total_low_level_actions = 0

        for t in range(
            1,
            primitive_count + 1,
        ):
            if t not in inference_results:
                raise AssertionError(
                    "control_loop() did not execute "
                    f"inference(t={t})."
                )

            actions = inference_results[t]

            if actions is None:
                raise AssertionError(
                    "SeeDo returned None before "
                    f"completion at t={t}."
                )

            if not isinstance(
                actions,
                list,
            ):
                raise AssertionError(
                    "SeeDo inference() must return "
                    f"a list at t={t}."
                )

            if not actions:
                raise AssertionError(
                    "SeeDo returned an empty action "
                    f"list at t={t}."
                )

            primitive_step = (
                primitive_plan.steps[t - 1]
            )

            for action_index, action in enumerate(
                actions
            ):
                if not isinstance(
                    action,
                    np.ndarray,
                ):
                    raise AssertionError(
                        "Invalid action type at "
                        f"t={t}, index={action_index}."
                    )

                if action.shape != (8,):
                    raise AssertionError(
                        "Invalid action shape at "
                        f"t={t}, index={action_index}: "
                        f"{action.shape}."
                    )

                if not np.all(
                    np.isfinite(action)
                ):
                    raise AssertionError(
                        "Non-finite action at "
                        f"t={t}, index={action_index}."
                    )

            total_low_level_actions += len(
                actions
            )

            print(
                f"[PASS] t={t}: "
                f"{primitive_step.name}"
                f"({primitive_step.arguments}) "
                f"-> {len(actions)} action(s)"
            )

        print(
            "Total low-level actions processed: "
            f"{total_low_level_actions}"
        )

        completion_t = primitive_count + 1

        if completion_t not in inference_results:
            raise AssertionError(
                "control_loop() did not execute "
                "the completion inference."
            )

        if (
            inference_results[completion_t]
            is not None
        ):
            raise AssertionError(
                "Completion inference must "
                "return None."
            )

        if (
            controller_node
            .controller
            .execution_status
            != "completed"
        ):
            raise AssertionError(
                "SeeDoController did not enter "
                "completed state."
            )

        print("[PASS] completion state")

        if saved_trajectory is None:
            raise AssertionError(
                "control_loop() did not reach "
                "save_rollout()."
            )

        if len(
            saved_trajectory.entries
        ) != total_low_level_actions:
            raise AssertionError(
                "Expected one trajectory entry "
                "per SeeDo low-level action. "
                f"Expected {total_low_level_actions}, "
                f"found {len(saved_trajectory.entries)}."
            )

        if not saved_trajectory.entries[-1][
            "done"
        ]:
            raise AssertionError(
                "Final trajectory entry is not "
                "marked done."
            )

        print(
            "[PASS] control_loop reached "
            "trajectory completion"
        )

        motion_artifact_path = (
            Path(args.artifacts_dir)
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

        print(
            "\nROS CONTROL LOOP TEST PASSED"
        )

        return 0

    finally:
        if executor is not None:
            executor.shutdown()

        if (
            executor_thread is not None
            and executor_thread.is_alive()
        ):
            executor_thread.join(
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
        default="/seedo_tests/ai_controller_node_ros",
    )

    parser.add_argument(
        "--task-id",
        default="1",
    )

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    return run_test(args)


if __name__ == "__main__":
    raise SystemExit(main())