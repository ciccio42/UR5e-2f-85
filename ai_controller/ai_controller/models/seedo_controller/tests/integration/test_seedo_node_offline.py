from __future__ import annotations

import argparse
import json
from pathlib import Path
from unittest.mock import patch

import numpy as np
import rclpy

import ai_controller.ai_controller_node as ai_controller_node_module

from ai_controller.ai_controller_node import AIControllerNode
from ai_controller.models.seedo_controller.tests.common import (
    load_scene_runtime_input,
)
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


class _ControlLoopFinished(Exception):
    """Stop the offline control loop immediately after one completed rollout."""


class _OfflineTrajectory:
    """Minimal Trajectory compatible with the current SeeDo recording path."""

    def __init__(self) -> None:
        self.entries: list[dict] = []

    def __len__(self) -> int:
        return len(self.entries)

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
                "obs": obs,
                "action": action,
                "done": done,
                "reward": reward,
                "info": info,
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
            f"Expected artifact was not created: {path}"
        )

    if path.stat().st_size == 0:
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


def _validate_canonical_inputs(
    args: argparse.Namespace,
) -> tuple[
    Path,
    Path,
    Path,
    Path,
]:
    if args.video is None:
        raise ValueError(
            "--video is required for the seedo_node_offline test."
        )

    if args.scene_dir is None:
        raise ValueError(
            "--scene-dir is required for the seedo_node_offline test."
        )

    if args.base_to_table_transform is None:
        raise ValueError(
            "--base-to-table-transform is required for the "
            "seedo_node_offline test."
        )

    if not args.model_config:
        raise ValueError(
            "--model-config is required for the seedo_node_offline test."
        )

    if args.artifacts_dir is None:
        raise ValueError(
            "--artifacts-dir is required for the seedo_node_offline "
            "integration test."
        )

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
            "This integration test is pinned to the canonical "
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
            "This integration test is pinned to the canonical "
            "no-distractors runtime scene. "
            f"Expected {expected_scene_dir}, received {scene_dir}."
        )

    expected_transform = (
        expected_scene_dir
        / "base_to_table_transform.yaml"
    ).resolve()

    transform_path = (
        Path(
            args.base_to_table_transform
        )
        .expanduser()
        .resolve()
    )

    if transform_path != expected_transform:
        raise ValueError(
            "The runtime transform must belong to the canonical "
            "runtime scene. "
            f"Expected {expected_transform}, received {transform_path}."
        )

    model_config_path = (
        Path(args.model_config)
        .expanduser()
        .resolve()
    )

    if not model_config_path.is_file():
        raise FileNotFoundError(
            f"SeeDo model configuration does not exist: {model_config_path}"
        )

    artifacts_dir = (
        Path(args.artifacts_dir)
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


def _validate_generalized_controller_state(
    node: AIControllerNode,
) -> None:
    controller = (
        node.controller
    )

    if controller.perception_mode != "generalized":
        raise AssertionError(
            "Offline node integration requires generalized mode, "
            f"received {controller.perception_mode!r}."
        )

    if controller.action_plan is None:
        raise AssertionError(
            "control_loop() did not generate an ActionPlanningResult."
        )

    if controller.demo_structured_scene is None:
        raise AssertionError(
            "control_loop() did not generate a demonstration "
            "StructuredScene."
        )

    if controller.perception_result is None:
        raise AssertionError(
            "control_loop() did not generate a ScenePerceptionResult."
        )

    if controller.scene_state is None:
        raise AssertionError(
            "control_loop() did not generate a SceneState."
        )

    if controller.runtime_structured_scene is None:
        raise AssertionError(
            "control_loop() did not generate a runtime StructuredScene."
        )

    if controller.structural_matching_result is None:
        raise AssertionError(
            "control_loop() did not generate a StructuralMatchingResult."
        )

    if controller.replicability_result is None:
        raise AssertionError(
            "control_loop() did not generate a ReplicabilityResult."
        )

    if controller.primitive_plan is None:
        raise AssertionError(
            "control_loop() did not generate a PrimitivePlan."
        )

    action_plan = (
        controller.action_plan
    )

    if action_plan.status != "completed":
        raise AssertionError(
            "Expected a completed action plan, "
            f"received {action_plan.status!r}."
        )

    if _normalize_task_type(
        action_plan.task_type
    ) != "pick_and_place":
        raise AssertionError(
            "Unexpected task type: "
            f"{action_plan.task_type!r}"
        )

    if len(
        action_plan.steps
    ) != 1:
        raise AssertionError(
            "Expected exactly one demonstration step, "
            f"received {len(action_plan.steps)}."
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
            "Unexpected demonstration destination track: "
            f"{action_step.destination_track_id}"
        )

    demo_object_ids = {
        obj.object_id
        for obj
        in controller
        .demo_structured_scene
        .objects
    }

    if (
        demo_object_ids
        != EXPECTED_DEMO_PLACE_IDS
    ):
        raise AssertionError(
            "Unexpected demonstration structured-scene IDs: "
            f"{sorted(demo_object_ids)}"
        )

    runtime_object_ids = {
        obj.object_id
        for obj
        in controller
        .scene_state
        .objects
    }

    if (
        runtime_object_ids
        != EXPECTED_RUNTIME_OBJECT_IDS
    ):
        raise AssertionError(
            "Unexpected runtime SceneState IDs: "
            f"expected={sorted(EXPECTED_RUNTIME_OBJECT_IDS)}, "
            f"received={sorted(runtime_object_ids)}"
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

    if not matching_result.is_valid:
        raise AssertionError(
            "Structural matching returned no valid mapping."
        )

    if not matching_result.is_unique:
        raise AssertionError(
            "Expected one unique structural mapping."
        )

    if len(
        matching_result.valid_mappings
    ) != 1:
        raise AssertionError(
            "Expected exactly one structural mapping, "
            f"received {len(matching_result.valid_mappings)}."
        )

    returned_mapping = {
        match.demo_object_id:
            match.runtime_object_id
        for match
        in matching_result
        .valid_mappings[0]
        .matches
    }

    if (
        returned_mapping
        != EXPECTED_STRUCTURAL_MAPPING
    ):
        raise AssertionError(
            "Unexpected structural mapping: "
            f"expected={EXPECTED_STRUCTURAL_MAPPING}, "
            f"received={returned_mapping}"
        )

    replicability_result = (
        controller
        .replicability_result
    )

    if not replicability_result.replicable:
        raise AssertionError(
            "Canonical task was classified as non-replicable: "
            f"{replicability_result.failure_reasons}"
        )

    if replicability_result.failure_reasons:
        raise AssertionError(
            "Replicable result contains failure reasons: "
            f"{replicability_result.failure_reasons}"
        )

    if len(
        replicability_result.resolved_targets
    ) != 1:
        raise AssertionError(
            "Expected exactly one resolved runtime target pair."
        )

    resolved = (
        replicability_result
        .resolved_targets[0]
    )

    if (
        resolved.runtime_pick_object_id
        != "green_block_0"
    ):
        raise AssertionError(
            "Unexpected runtime pick target: "
            f"{resolved.runtime_pick_object_id!r}"
        )

    if (
        resolved.runtime_place_object_id
        != "storage_bin_0"
    ):
        raise AssertionError(
            "Unexpected runtime place target: "
            f"{resolved.runtime_place_object_id!r}"
        )

    primitive_plan = (
        controller
        .primitive_plan
    )

    if len(
        primitive_plan.steps
    ) != len(
        EXPECTED_PRIMITIVES
    ):
        raise AssertionError(
            "Unexpected PrimitivePlan length: "
            f"expected={len(EXPECTED_PRIMITIVES)}, "
            f"received={len(primitive_plan.steps)}"
        )

    for index, (
        primitive_step,
        (
            expected_name,
            expected_target,
        ),
    ) in enumerate(
        zip(
            primitive_plan.steps,
            EXPECTED_PRIMITIVES,
            strict=True,
        ),
        start=1,
    ):
        if (
            primitive_step.name
            != expected_name
        ):
            raise AssertionError(
                f"Primitive {index}: expected {expected_name!r}, "
                f"received {primitive_step.name!r}."
            )

        if (
            primitive_step.arguments
            != {
                "target":
                    expected_target,
            }
        ):
            raise AssertionError(
                f"Primitive {index}: expected runtime target "
                f"{expected_target!r}, received "
                f"{primitive_step.arguments!r}."
            )


def _validate_pipeline_artifacts(
    artifacts_dir: Path,
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


def _validate_saved_trajectory(
    *,
    saved_trajectory: _OfflineTrajectory,
    total_low_level_actions: int,
    runtime_depth: np.ndarray,
) -> None:
    if len(
        saved_trajectory.entries
    ) != total_low_level_actions:
        raise AssertionError(
            "Expected one trajectory entry per SeeDo low-level "
            "action. "
            f"Expected {total_low_level_actions}, "
            f"found {len(saved_trajectory.entries)}."
        )

    if not saved_trajectory.entries:
        raise AssertionError(
            "Offline trajectory contains no entries."
        )

    done_indices = [
        index
        for index, entry
        in enumerate(
            saved_trajectory.entries
        )
        if bool(
            entry["done"]
        )
    ]

    if done_indices != [
        len(
            saved_trajectory.entries
        )
        - 1
    ]:
        raise AssertionError(
            "Exactly the final trajectory entry must be marked done. "
            f"done_indices={done_indices}"
        )

    reward_indices = [
        index
        for index, entry
        in enumerate(
            saved_trajectory.entries
        )
        if entry["reward"] == 1
    ]

    if reward_indices != done_indices:
        raise AssertionError(
            "Trajectory reward=1 must occur exactly on the final "
            f"done entry. reward_indices={reward_indices}, "
            f"done_indices={done_indices}"
        )

    observed_statuses = []

    for index, entry in enumerate(
        saved_trajectory.entries
    ):
        action = np.asarray(
            entry["action"]
        )

        if action.shape != (
            8,
        ):
            raise AssertionError(
                "Dataset trajectory action has invalid shape at "
                f"entry {index}: {action.shape}."
            )

        if not np.all(
            np.isfinite(
                action
            )
        ):
            raise AssertionError(
                f"Dataset trajectory action contains non-finite "
                f"values at entry {index}: {action}"
            )

        if float(
            action[7]
        ) not in {
            0.0,
            1.0,
        }:
            raise AssertionError(
                "Dataset gripper action must be binary, received "
                f"{action[7]} at entry {index}."
            )

        obs = entry["obs"]

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
                    f"Trajectory observation {index} is missing {key!r}."
                )

            encoded = np.asarray(
                obs[key]
            )

            if (
                encoded.ndim != 1
                or encoded.size == 0
            ):
                raise AssertionError(
                    "SeeDo rollout RGB image was not JPEG-compressed "
                    f"for {key!r} at entry {index}: "
                    f"shape={encoded.shape}"
                )

        for key in (
            ROLLOUT_DEPTH_KEYS
        ):
            if key not in obs:
                raise AssertionError(
                    f"Trajectory observation {index} is missing {key!r}."
                )

            depth = np.asarray(
                obs[key]
            )

            if (
                depth.shape
                != runtime_depth.shape
            ):
                raise AssertionError(
                    f"Unexpected depth shape for {key!r} at "
                    f"entry {index}: expected={runtime_depth.shape}, "
                    f"received={depth.shape}"
                )

        obj_bb = obs.get(
            "obj_bb"
        )

        if not isinstance(
            obj_bb,
            dict,
        ):
            raise AssertionError(
                f"Trajectory observation {index} has no obj_bb dictionary."
            )

        front_bb = obj_bb.get(
            "camera_front"
        )

        if not isinstance(
            front_bb,
            dict,
        ):
            raise AssertionError(
                f"Trajectory observation {index} has no "
                "camera_front bounding-box dictionary."
            )

        if len(
            front_bb
        ) != 8:
            raise AssertionError(
                "Expected bounding boxes for all 8 runtime objects, "
                f"received {len(front_bb)} at entry {index}."
            )

        info = entry.get(
            "info"
        )

        if not isinstance(
            info,
            dict,
        ):
            raise AssertionError(
                f"Trajectory entry {index} is missing the SeeDo info field."
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

        observed_statuses.append(
            status
        )

    required_statuses = {
        "start",
        "approaching",
        "picking",
        "moving",
        "placing",
        "end",
    }

    missing_statuses = (
        required_statuses
        - set(
            observed_statuses
        )
    )

    if missing_statuses:
        raise AssertionError(
            "Trajectory did not contain every expected SeeDo dataset "
            f"status: missing={sorted(missing_statuses)}"
        )

    if (
        observed_statuses[-1]
        != "end"
    ):
        raise AssertionError(
            "Final SeeDo dataset status must be 'end', received "
            f"{observed_statuses[-1]!r}."
        )


def run_test(
    args: argparse.Namespace,
) -> int:
    """Run AIControllerNode.control_loop() offline with the real SeeDo pipeline."""

    (
        video_path,
        _scene_dir,
        _transform_path,
        artifacts_dir,
    ) = _validate_canonical_inputs(
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
        ]
    )

    node = None

    try:
        print(
            "=== INITIALIZING AI CONTROLLER NODE ==="
        )

        node = AIControllerNode()

        if (
            node.ai_controller_target
            != "seedo_controller"
        ):
            raise AssertionError(
                "AIControllerNode is not configured to use "
                "seedo_controller."
            )

        if node.move_robot:
            raise AssertionError(
                "Offline SeeDo node test must run with move_robot=False."
            )

        if node.seedo_execute_gripper:
            raise AssertionError(
                "Offline SeeDo node test must run with "
                "seedo_execute_gripper=False."
            )

        if (
            node.controller.perception_mode
            != "generalized"
        ):
            raise AssertionError(
                "Offline node integration requires "
                "perception_mode='generalized'."
            )

        # Force the full demonstration path: no precomputed handoffs.
        node.demo_path = str(
            video_path
        )

        node.seedo_artifacts_dir = str(
            artifacts_dir
        )

        node.seedo_precomputed_action_plan_path = ""
        node.seedo_precomputed_demo_structured_scene_path = ""

        print(
            "AIControllerNode initialized successfully"
        )
        print(
            "Persistent artifact directory: "
            f"{artifacts_dir}"
        )

        print()
        print(
            "=== LOADING OFFLINE RUNTIME SCENE ==="
        )

        scene_runtime_input = (
            load_scene_runtime_input(
                args
            )
        )

        required_runtime_keys = {
            "rgb",
            "depth",
            "camera_info",
            "base_to_table_transform",
        }

        missing_runtime_keys = (
            required_runtime_keys
            - set(
                scene_runtime_input
            )
        )

        if missing_runtime_keys:
            raise AssertionError(
                "Offline runtime scene is incomplete: "
                f"{sorted(missing_runtime_keys)}"
            )

        seedo_runtime_input = dict(
            scene_runtime_input
        )

        runtime_rgb = np.asarray(
            seedo_runtime_input[
                "rgb"
            ]
        )

        runtime_depth = np.asarray(
            seedo_runtime_input[
                "depth"
            ],
            dtype=np.float32,
        )

        if (
            runtime_rgb.ndim != 3
            or runtime_rgb.shape[2] != 3
        ):
            raise AssertionError(
                "Offline runtime RGB must have shape HxWx3, "
                f"received {runtime_rgb.shape}."
            )

        if runtime_depth.ndim != 2:
            raise AssertionError(
                "Offline runtime depth must be two-dimensional, "
                f"received {runtime_depth.shape}."
            )

        print(
            "Offline runtime scene loaded successfully"
        )

        print()
        print(
            "=== PREPARING OFFLINE ROBOT / CAMERA STATE ==="
        )

        robot_state = {
            EEF_POS_NAME:
                INITIAL_EEF_POSITION.copy(),
            EEF_QUAT_NAME:
                INITIAL_EEF_ORIENTATION.copy(),
        }

        def fake_get_synced_images():
            return [
                runtime_rgb.copy()
            ]

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

        def fake_get_record_camera_data():
            return {
                "camera_front_image":
                    runtime_rgb.copy(),
                "camera_front_depth":
                    runtime_depth.copy(),
                "camera_lateral_left_image":
                    runtime_rgb.copy(),
                "camera_lateral_left_depth":
                    runtime_depth.copy(),
                "camera_lateral_right_image":
                    runtime_rgb.copy(),
                "camera_lateral_right_depth":
                    runtime_depth.copy(),
                "eye_in_hand_image":
                    runtime_rgb.copy(),
                "eye_in_hand_depth":
                    runtime_depth.copy(),
            }

        def fake_wait_for_seedo_runtime_data(
            *args,
            **kwargs,
        ):
            return None

        def fake_get_seedo_base_to_table_transform(
            *args,
            **kwargs,
        ):
            return seedo_runtime_input[
                "base_to_table_transform"
            ]

        def fake_get_seedo_runtime_input(
            *args,
            **kwargs,
        ):
            return seedo_runtime_input

        # Capture the real controller.inference() outputs. The control
        # loop remains responsible for deciding when each inference call
        # is made.
        original_inference = (
            node.controller.inference
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

        # control_loop() asks for:
        #   1. trajectory count
        #   2. Enter before starting
        #   3. task ID
        #
        # save_rollout() itself is intercepted, so no manual outcome
        # questions are requested in this offline integration test.
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
                f"[offline input] "
                f"{prompt}{value}"
            )

            return value

        print()
        print(
            "=== RUNNING REAL CONTROL LOOP / FULL SEEDO PIPELINE ==="
        )

        with (
            patch(
                "builtins.input",
                side_effect=fake_input,
            ),
            patch.object(
                ai_controller_node_module,
                "_get_trajectory_cls",
                return_value=(
                    _OfflineTrajectory
                ),
            ),
            patch.object(
                ai_controller_node_module,
                "wait_for_seedo_runtime_data",
                side_effect=(
                    fake_wait_for_seedo_runtime_data
                ),
            ),
            patch.object(
                ai_controller_node_module,
                "get_seedo_base_to_table_transform",
                side_effect=(
                    fake_get_seedo_base_to_table_transform
                ),
            ),
            patch.object(
                ai_controller_node_module,
                "get_seedo_runtime_input",
                side_effect=(
                    fake_get_seedo_runtime_input
                ),
            ),
            patch.object(
                node,
                "get_synced_images",
                side_effect=(
                    fake_get_synced_images
                ),
            ),
            patch.object(
                node,
                "_capture_robot_state",
                side_effect=(
                    fake_capture_robot_state
                ),
            ),
            patch.object(
                node,
                "_get_seedo_record_camera_data",
                side_effect=(
                    fake_get_record_camera_data
                ),
            ),
            patch.object(
                node.controller,
                "inference",
                side_effect=(
                    tracked_inference
                ),
            ),
            patch.object(
                node,
                "save_rollout",
                side_effect=(
                    fake_save_rollout
                ),
            ),
        ):
            try:
                node.control_loop()
            except _ControlLoopFinished:
                pass

        print()
        print(
            "=== VALIDATING GENERALIZED PIPELINE STATE ==="
        )

        _validate_generalized_controller_state(
            node
        )

        controller = (
            node.controller
        )

        primitive_count = len(
            controller
            .primitive_plan
            .steps
        )

        print(
            f"Primitive count: "
            f"{primitive_count}"
        )

        resolved = (
            controller
            .replicability_result
            .resolved_targets[0]
        )

        print(
            "Resolved runtime targets: "
            f"pick={resolved.runtime_pick_object_id}, "
            f"place={resolved.runtime_place_object_id}"
        )

        print()
        print(
            "=== VALIDATING CONTROL-LOOP INFERENCE SEQUENCE ==="
        )

        if 0 not in inference_results:
            raise AssertionError(
                "control_loop() did not execute SeeDo inference(t=0)."
            )

        if inference_results[
            0
        ] is not None:
            raise AssertionError(
                "SeeDo inference(t=0) must return None."
            )

        print(
            "[PASS] t=0 runtime perception/planning"
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

            actions = (
                inference_results[
                    t
                ]
            )

            if actions is None:
                raise AssertionError(
                    "SeeDo returned None before the "
                    f"PrimitivePlan was completed at t={t}."
                )

            if not isinstance(
                actions,
                list,
            ):
                raise AssertionError(
                    "SeeDo inference() did not return "
                    f"a list at t={t}: "
                    f"{type(actions).__name__}."
                )

            if not actions:
                raise AssertionError(
                    "SeeDo returned an empty low-level "
                    f"action list at t={t}."
                )

            (
                expected_name,
                expected_target,
            ) = EXPECTED_PRIMITIVES[
                t - 1
            ]

            primitive_step = (
                controller
                .primitive_plan
                .steps[
                    t - 1
                ]
            )

            if (
                primitive_step.name
                != expected_name
            ):
                raise AssertionError(
                    f"Unexpected primitive at t={t}: "
                    f"{primitive_step.name!r}"
                )

            if (
                primitive_step.arguments
                != {
                    "target":
                        expected_target,
                }
            ):
                raise AssertionError(
                    f"Unexpected runtime target at t={t}: "
                    f"{primitive_step.arguments!r}"
                )

            for (
                action_index,
                action,
            ) in enumerate(
                actions
            ):
                if not isinstance(
                    action,
                    np.ndarray,
                ):
                    raise AssertionError(
                        "Invalid low-level action type "
                        f"at t={t}, index={action_index}: "
                        f"{type(action).__name__}."
                    )

                if action.shape != (
                    8,
                ):
                    raise AssertionError(
                        "Invalid low-level action shape "
                        f"at t={t}, index={action_index}: "
                        f"{action.shape}."
                    )

                if not np.all(
                    np.isfinite(
                        action
                    )
                ):
                    raise AssertionError(
                        "Non-finite low-level action "
                        f"at t={t}, index={action_index}: "
                        f"{action}"
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

        completion_t = (
            primitive_count
            + 1
        )

        if (
            completion_t
            not in inference_results
        ):
            raise AssertionError(
                "control_loop() did not perform the final "
                "SeeDo completion inference."
            )

        if (
            inference_results[
                completion_t
            ]
            is not None
        ):
            raise AssertionError(
                "SeeDo completion inference must return None."
            )

        if (
            controller.execution_status
            != "completed"
        ):
            raise AssertionError(
                "SeeDoController did not enter completed state. "
                f"Current status: "
                f"{controller.execution_status}"
            )

        print(
            "[PASS] final completion inference"
        )

        print()
        print(
            "Total low-level actions processed by control_loop(): "
            f"{total_low_level_actions}"
        )

        print()
        print(
            "=== VALIDATING RECORDED OFFLINE TRAJECTORY ==="
        )

        if saved_trajectory is None:
            raise AssertionError(
                "control_loop() never reached save_rollout()."
            )

        if not isinstance(
            saved_trajectory,
            _OfflineTrajectory,
        ):
            raise AssertionError(
                "Unexpected trajectory type captured by "
                "the offline test."
            )

        _validate_saved_trajectory(
            saved_trajectory=(
                saved_trajectory
            ),
            total_low_level_actions=(
                total_low_level_actions
            ),
            runtime_depth=(
                runtime_depth
            ),
        )

        if saved_rollout_call is None:
            raise AssertionError(
                "Offline save_rollout call metadata was not captured."
            )

        expected_task_id = str(
            args.task_id
        ).zfill(
            2
        )

        if (
            saved_rollout_call[
                "task_id"
            ]
            != expected_task_id
        ):
            raise AssertionError(
                "control_loop() passed an unexpected task ID "
                "to save_rollout(): "
                f"{saved_rollout_call['task_id']!r}"
            )

        if (
            saved_rollout_call[
                "traj_number"
            ]
            != 0
        ):
            raise AssertionError(
                "Expected first offline trajectory number 0, "
                f"received {saved_rollout_call['traj_number']}."
            )

        print(
            "[PASS] one dataset entry per low-level action"
        )
        print(
            "[PASS] four-camera RGB/depth rollout structure"
        )
        print(
            "[PASS] final done/reward/status=end"
        )

        print()
        print(
            "=== VALIDATING CONTROLLER / NODE ARTIFACTS ==="
        )

        if (
            controller.artifacts_dir
            is None
        ):
            raise AssertionError(
                "Controller artifact directory was cleared "
                "before rollout completion."
            )

        controller_artifacts_dir = (
            Path(
                controller
                .artifacts_dir
            )
            .expanduser()
            .resolve()
        )

        if (
            controller_artifacts_dir
            != artifacts_dir
        ):
            raise AssertionError(
                "Controller used an unexpected artifact directory: "
                f"expected={artifacts_dir}, "
                f"received={controller_artifacts_dir}"
            )

        _validate_pipeline_artifacts(
            controller_artifacts_dir
        )

        motion_plan = _load_json(
            controller_artifacts_dir
            / "motion_layer"
            / "motion_plan.json"
        )

        if (
            motion_plan.get(
                "total_actions"
            )
            != total_low_level_actions
        ):
            raise AssertionError(
                "motion_plan.json total_actions does not match "
                "the control-loop action count. "
                f"artifact={motion_plan.get('total_actions')}, "
                f"control_loop={total_low_level_actions}"
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

        if node.pause_executor.is_set():
            raise AssertionError(
                "pause_executor remained set after demonstration "
                "processing."
            )

        print()
        print(
            "SEEDO NODE OFFLINE END-TO-END TEST PASSED"
        )

        return 0

    finally:
        if node is not None:
            node.destroy_node()

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
        "--artifacts-dir",
        required=True,
    )

    parser.add_argument(
        "--task-id",
        default="1",
    )

    parser.add_argument(
        "--model-config",
        required=True,
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
