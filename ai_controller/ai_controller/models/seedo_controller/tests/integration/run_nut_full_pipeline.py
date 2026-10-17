from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from ai_controller.models.seedo_controller.seedo_controller import (
    SeeDoController,
)

from ai_controller.utils.utils import (
    EEF_POS_NAME,
    EEF_QUAT_NAME,
)


EXPECTED_PRIMITIVES = (
    "reach",
    "approaching",
    "pick",
    "lift_up",
    "moving",
    "aligning",
    "inserting",
)


# =====================================================================
# IO helpers
# =====================================================================


def _require_file(
    path: Path,
    description: str,
) -> Path:

    path = (
        Path(path)
        .expanduser()
        .resolve()
    )

    if not path.is_file():
        raise FileNotFoundError(
            f"Missing {description}: {path}"
        )

    return path


def _load_json(
    path: Path,
) -> dict[str, Any]:

    path = _require_file(
        path,
        "JSON file",
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
        raise RuntimeError(
            f"Expected a JSON object in {path}."
        )

    return payload


def _jsonable(
    value: Any,
) -> Any:

    if isinstance(
        value,
        np.ndarray,
    ):
        return value.tolist()

    if isinstance(
        value,
        (
            np.integer,
            np.floating,
            np.bool_,
        ),
    ):
        return value.item()

    if isinstance(
        value,
        Path,
    ):
        return str(
            value
        )

    if isinstance(
        value,
        dict,
    ):
        return {
            str(key): _jsonable(
                item
            )
            for key, item
            in value.items()
        }

    if isinstance(
        value,
        (
            list,
            tuple,
        ),
    ):
        return [
            _jsonable(
                item
            )
            for item in value
        ]

    return value


def _write_json(
    path: Path,
    payload: Any,
) -> None:

    path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    with path.open(
        "w",
        encoding="utf-8",
    ) as stream:
        json.dump(
            _jsonable(
                payload
            ),
            stream,
            indent=2,
            ensure_ascii=False,
            allow_nan=False,
        )


def _load_rgb(
    path: Path,
) -> np.ndarray:

    path = _require_file(
        path,
        "RGB image",
    )

    image_bgr = cv2.imread(
        str(
            path
        ),
        cv2.IMREAD_COLOR,
    )

    if image_bgr is None:
        raise RuntimeError(
            f"Could not read RGB image: {path}"
        )

    return cv2.cvtColor(
        image_bgr,
        cv2.COLOR_BGR2RGB,
    )


def _load_depth(
    path: Path,
) -> np.ndarray:

    path = _require_file(
        path,
        "depth image",
    )

    depth = np.asarray(
        np.load(
            path
        ),
        dtype=np.float32,
    )

    if depth.ndim != 2:
        raise RuntimeError(
            f"Depth image must be 2D: {path}"
        )

    if not np.isfinite(
        depth
    ).any():
        raise RuntimeError(
            f"Depth image contains no finite values: {path}"
        )

    return depth


def _load_camera_info(
    path: Path,
) -> dict[str, Any]:

    payload = _load_json(
        path
    )

    if "k" not in payload:
        raise RuntimeError(
            f"CameraInfo does not contain 'k': {path}"
        )

    K_flat = np.asarray(
        payload[
            "k"
        ],
        dtype=np.float64,
    )

    if K_flat.size != 9:
        raise RuntimeError(
            f"CameraInfo 'k' must contain 9 values: {path}"
        )

    if not np.isfinite(
        K_flat
    ).all():
        raise RuntimeError(
            f"CameraInfo contains non-finite values: {path}"
        )

    return payload


def _camera_matrix_from_info(
    camera_info: dict[str, Any],
) -> np.ndarray:

    return np.asarray(
        camera_info[
            "k"
        ],
        dtype=np.float64,
    ).reshape(
        3,
        3,
    )


def _load_transform(
    path: Path,
) -> dict[str, Any]:

    payload = _load_json(
        path
    )

    if "rotation" not in payload:
        raise RuntimeError(
            f"Transform is missing 'rotation': {path}"
        )

    if "translation" not in payload:
        raise RuntimeError(
            f"Transform is missing 'translation': {path}"
        )

    rotation = np.asarray(
        payload[
            "rotation"
        ],
        dtype=np.float64,
    )

    translation = np.asarray(
        payload[
            "translation"
        ],
        dtype=np.float64,
    )

    if rotation.shape != (
        3,
        3,
    ):
        raise RuntimeError(
            f"Transform rotation must be 3x3: {path}"
        )

    if translation.shape != (3,):
        raise RuntimeError(
            f"Transform translation must be length 3: {path}"
        )

    return {
        "rotation": rotation,
        "translation": translation,
    }


def _load_matrix4(
    path: Path,
) -> np.ndarray:

    path = _require_file(
        path,
        "4x4 transform",
    )

    matrix = np.asarray(
        np.load(
            path
        ),
        dtype=np.float64,
    )

    if matrix.shape != (
        4,
        4,
    ):
        raise RuntimeError(
            f"Expected 4x4 matrix in {path}, "
            f"received {matrix.shape}."
        )

    if not np.isfinite(
        matrix
    ).all():
        raise RuntimeError(
            f"Transform contains non-finite values: {path}"
        )

    return matrix


def _load_robot_state(
    path: Path,
) -> dict[str, np.ndarray]:

    payload = _load_json(
        path
    )

    position = np.asarray(
        payload[
            EEF_POS_NAME
        ],
        dtype=np.float64,
    )

    orientation = np.asarray(
        payload[
            EEF_QUAT_NAME
        ],
        dtype=np.float64,
    )

    if position.shape != (3,):
        raise RuntimeError(
            f"{EEF_POS_NAME} must have shape (3,) in {path}."
        )

    if orientation.shape != (4,):
        raise RuntimeError(
            f"{EEF_QUAT_NAME} must have shape (4,) in {path}."
        )

    quaternion_norm = float(
        np.linalg.norm(
            orientation
        )
    )

    if quaternion_norm <= 1e-12:
        raise RuntimeError(
            f"Invalid zero quaternion in {path}."
        )

    orientation = (
        orientation
        / quaternion_norm
    )

    return {
        EEF_POS_NAME: position,
        EEF_QUAT_NAME: orientation,
    }


# =====================================================================
# Case loader
# =====================================================================


def _load_case(
    case_dir: Path,
) -> dict[str, Any]:

    case_dir = (
        Path(case_dir)
        .expanduser()
        .resolve()
    )

    demo_path = _require_file(
        case_dir
        / "demo.mp4",
        "demonstration video",
    )

    front_dir = (
        case_dir
        / "front"
    )

    gripper_dir = (
        case_dir
        / "gripper"
    )

    front_rgb = _load_rgb(
        front_dir
        / "rgb.png"
    )

    front_depth = _load_depth(
        front_dir
        / "depth.npy"
    )

    if (
        front_rgb.shape[:2]
        != front_depth.shape
    ):
        raise RuntimeError(
            "Front RGB/depth dimensions differ: "
            f"rgb={front_rgb.shape[:2]}, "
            f"depth={front_depth.shape}."
        )

    front_camera_info = (
        _load_camera_info(
            front_dir
            / "camera_info.json"
        )
    )

    base_to_table_transform = (
        _load_transform(
            front_dir
            / "base_to_table_transform.json"
        )
    )

    gripper_rgb = _load_rgb(
        gripper_dir
        / "rgb.png"
    )

    gripper_depth = _load_depth(
        gripper_dir
        / "depth.npy"
    )

    if (
        gripper_rgb.shape[:2]
        != gripper_depth.shape
    ):
        raise RuntimeError(
            "Gripper RGB/depth dimensions differ: "
            f"rgb={gripper_rgb.shape[:2]}, "
            f"depth={gripper_depth.shape}."
        )

    gripper_camera_info = (
        _load_camera_info(
            gripper_dir
            / "camera_info.json"
        )
    )

    gripper_K = (
        _camera_matrix_from_info(
            gripper_camera_info
        )
    )

    T_base_camera = (
        _load_matrix4(
            gripper_dir
            / "T_base_camera.npy"
        )
    )

    T_base_table = (
        _load_matrix4(
            gripper_dir
            / "T_base_table.npy"
        )
    )

    initial_robot_state = (
        _load_robot_state(
            case_dir
            / "initial_robot_state.json"
        )
    )

    grasp_robot_state = (
        _load_robot_state(
            gripper_dir
            / "robot_state.json"
        )
    )

    return {
        "case_dir": case_dir,

        "demo_path": (
            demo_path
        ),

        "front_rgb": (
            front_rgb
        ),

        "front_depth": (
            front_depth
        ),

        "front_camera_info": (
            front_camera_info
        ),

        "base_to_table_transform": (
            base_to_table_transform
        ),

        "initial_robot_state": (
            initial_robot_state
        ),

        "grasp_robot_state": (
            grasp_robot_state
        ),

        "grasp_input": {
            "rgb_image": (
                gripper_rgb
            ),

            "depth_image": (
                gripper_depth
            ),

            "camera_matrix": (
                gripper_K
            ),

            "T_base_camera": (
                T_base_camera
            ),

            "T_base_table": (
                T_base_table
            ),
        },
    }


# =====================================================================
# Full pipeline
# =====================================================================


def run_full_pipeline(
    *,
    model_config: Path,
    case_dir: Path,
    artifacts_dir: Path,
    task_id: str,
) -> dict[str, Any]:

    case = _load_case(
        case_dir
    )

    artifacts_dir = (
        Path(artifacts_dir)
        .expanduser()
        .resolve()
    )

    artifacts_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    print()
    print("=" * 90)
    print("FULL NUT-ASSEMBLY PIPELINE")
    print("=" * 90)

    print(
        "Case:",
        case[
            "case_dir"
        ],
    )

    print(
        "Artifacts:",
        artifacts_dir,
    )

    print()

    # -------------------------------------------------------------
    # Build complete SeeDo pipeline.
    # -------------------------------------------------------------

    controller = SeeDoController(
        model_config=str(
            Path(model_config)
            .expanduser()
            .resolve()
        )
    )

    # -------------------------------------------------------------
    # DEMONSTRATION PIPELINE
    #
    # video ->
    # keyframes ->
    # visual prompting ->
    # action planning
    # -------------------------------------------------------------

    print(
        "[FULL TEST] Processing demonstration..."
    )

    controller.load_command(
        demo_path=str(
            case[
                "demo_path"
            ]
        ),

        task_id=str(
            task_id
        ),

        artifacts_dir=(
            artifacts_dir
        ),
    )

    if controller.action_plan is None:
        raise RuntimeError(
            "ActionPlanner produced no ActionPlanningResult."
        )

    print()
    print(
        "[FULL TEST] Action-plan task type:",
        controller.action_plan.task_type,
    )

    print(
        "[FULL TEST] Natural-language plan:",
        controller.action_plan.natural_language_plan,
    )

    if (
        str(
            controller.action_plan.task_type
        )
        .strip()
        .lower()
        != "nut_assembly"
    ):
        raise RuntimeError(
            "The demonstration was not classified as "
            f"nut_assembly: {controller.action_plan.task_type!r}"
        )

    # -------------------------------------------------------------
    # RUNTIME PERCEPTION + CAP/LMP
    # -------------------------------------------------------------

    print()
    print(
        "[FULL TEST] Running initial runtime perception..."
    )

    controller.inference(
        input_data={
            "rgb": (
                case[
                    "front_rgb"
                ]
            ),

            "depth": (
                case[
                    "front_depth"
                ]
            ),

            "camera_info": (
                case[
                    "front_camera_info"
                ]
            ),

            "base_to_table_transform": (
                case[
                    "base_to_table_transform"
                ]
            ),
        },

        t=0,
    )

    if controller.primitive_plan is None:
        raise RuntimeError(
            "LMPGenerator produced no PrimitivePlan."
        )

    primitive_names = tuple(
        str(
            step.name
        )
        .strip()
        .lower()
        for step in controller.primitive_plan.steps
    )

    print()
    print(
        "[FULL TEST] Primitive plan:",
        primitive_names,
    )

    # For the first complete regression case we deliberately require
    # exactly one Nut-Assembly cycle.
    if primitive_names != EXPECTED_PRIMITIVES:
        raise RuntimeError(
            "Unexpected primitive sequence.\n"
            f"Expected: {EXPECTED_PRIMITIVES}\n"
            f"Actual:   {primitive_names}"
        )

    # -------------------------------------------------------------
    # MOTION PIPELINE
    # -------------------------------------------------------------

    action_dir = (
        artifacts_dir
        / "full_pipeline_actions"
    )

    action_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    action_counts: dict[
        str,
        int,
    ] = {}

    reach_endpoint = None

    for t, primitive_step in enumerate(
        controller.primitive_plan.steps,
        start=1,
    ):

        primitive_name = (
            str(
                primitive_step.name
            )
            .strip()
            .lower()
        )

        print()
        print(
            f"[FULL TEST] t={t} | "
            f"primitive={primitive_name}"
        )

        input_data: dict[
            str,
            Any,
        ] = {}

        # MotionLayer initialization.
        if t == 1:
            input_data[
                "robot_state"
            ] = (
                case[
                    "initial_robot_state"
                ]
            )

        # GraspGenX uses a separately captured eye-in-hand
        # image and the robot state associated with that image.
        if primitive_name == "approaching":

            input_data[
                "robot_state"
            ] = (
                case[
                    "grasp_robot_state"
                ]
            )

            input_data[
                "grasp_input"
            ] = (
                case[
                    "grasp_input"
                ]
            )

            # Diagnostic only:
            # compare the offline gripper capture pose with
            # the endpoint predicted by REACH.
            if reach_endpoint is not None:

                grasp_position = np.asarray(
                    case[
                        "grasp_robot_state"
                    ][
                        EEF_POS_NAME
                    ],
                    dtype=np.float64,
                )

                reach_grasp_position_error = float(
                    np.linalg.norm(
                        reach_endpoint[
                            :3
                        ]
                        - grasp_position
                    )
                )

                print(
                    "[FULL TEST] REACH endpoint vs "
                    "gripper-capture TCP position error: "
                    f"{reach_grasp_position_error * 1000.0:.2f} mm"
                )

        actions = controller.inference(
            input_data=(
                input_data
            ),
            t=t,
        )

        if actions is None:
            raise RuntimeError(
                f"Primitive {primitive_name!r} "
                "produced no actions."
            )

        actions_array = np.asarray(
            actions,
            dtype=np.float64,
        )

        if actions_array.ndim != 2:
            raise RuntimeError(
                f"Unexpected action shape for "
                f"{primitive_name}: "
                f"{actions_array.shape}"
            )

        if not np.isfinite(
            actions_array
        ).all():
            raise RuntimeError(
                f"Primitive {primitive_name} generated "
                "non-finite actions."
            )

        action_counts[
            primitive_name
        ] = int(
            len(
                actions_array
            )
        )

        np.save(
            action_dir
            / (
                f"{t:02d}_"
                f"{primitive_name}.npy"
            ),
            actions_array,
        )

        if primitive_name == "reach":
            reach_endpoint = (
                actions_array[
                    -1
                ].copy()
            )

    # One extra inference call marks controller execution complete.
    controller.inference(
        input_data={},
        t=(
            len(
                controller.primitive_plan.steps
            )
            + 1
        ),
    )

    # -------------------------------------------------------------
    # Validate automatically generated Nut visual diagnostics.
    # -------------------------------------------------------------

    visual_debug_dir = (
        artifacts_dir
        / "nut_visual_debug"
        / "action_00"
    )

    expected_visual_artifacts = (
        "nut_visual_debug_points.json",
        "nut_place_targets_front_overlay.png",
        "nut_place_targets_topdown.png",
        "nut_place_targets_sideview.png",
        "predicted_final_assembly_object_points_base.npy",
    )

    missing_visual_artifacts = [
        name
        for name in expected_visual_artifacts
        if not (
            visual_debug_dir
            / name
        ).is_file()
    ]

    if missing_visual_artifacts:
        raise RuntimeError(
            "Full pipeline completed but Nut-Assembly "
            "visual diagnostics are incomplete: "
            + ", ".join(
                missing_visual_artifacts
            )
        )

    # -------------------------------------------------------------
    # Summary
    # -------------------------------------------------------------

    raw_object_ids = []

    if controller.perception_result is not None:
        raw_object_ids = [
            obj.object_id
            for obj in (
                controller
                .perception_result
                .raw_scene
                .objects
            )
        ]

    summary = {
        "case_dir": (
            case[
                "case_dir"
            ]
        ),

        "demo_path": (
            case[
                "demo_path"
            ]
        ),

        "task_id": (
            task_id
        ),

        "action_plan": {
            "status": (
                controller.action_plan.status
            ),

            "task_type": (
                controller.action_plan.task_type
            ),

            "natural_language_plan": (
                controller
                .action_plan
                .natural_language_plan
            ),
        },

        "raw_scene_object_ids": (
            raw_object_ids
        ),

        "primitive_plan": [
            {
                "name": (
                    str(
                        step.name
                    )
                ),

                "arguments": (
                    dict(
                        step.arguments
                    )
                ),
            }
            for step in (
                controller
                .primitive_plan
                .steps
            )
        ],

        "nut_pick_refinement_targets": (
            list(
                controller
                .nut_pick_refinements
                .keys()
            )
        ),

        "nut_place_refinement_targets": (
            list(
                controller
                .nut_place_refinements
                .keys()
            )
        ),

        "grasp": (
            None
            if controller.grasp_plan is None
            else {
                "target": (
                    controller
                    .grasp_plan_target
                ),

                "tcp_position_base": np.asarray(
                    controller
                    .grasp_plan
                    .tcp_position_base,
                    dtype=np.float64,
                ),

                "tcp_orientation_base": np.asarray(
                    controller
                    .grasp_plan
                    .tcp_orientation_base,
                    dtype=np.float64,
                ),

                "approach_direction_base": np.asarray(
                    controller
                    .grasp_plan
                    .approach_direction_base,
                    dtype=np.float64,
                ),

                "confidence": float(
                    controller
                    .grasp_plan
                    .confidence
                ),
            }
        ),

        "action_counts": (
            action_counts
        ),

        "execution_status": (
            controller.execution_status
        ),

        "visual_debug_dir": (
            visual_debug_dir
        ),
    }

    summary_path = (
        artifacts_dir
        / "full_pipeline_summary.json"
    )

    _write_json(
        summary_path,
        summary,
    )

    print()
    print("=" * 90)
    print("FULL PIPELINE COMPLETED")
    print("=" * 90)

    print(
        "Execution status:",
        controller.execution_status,
    )

    print(
        "Detected objects:",
        raw_object_ids,
    )

    print(
        "Pick refinements:",
        list(
            controller
            .nut_pick_refinements
            .keys()
        ),
    )

    print(
        "Place refinements:",
        list(
            controller
            .nut_place_refinements
            .keys()
        ),
    )

    print(
        "Grasp target:",
        controller.grasp_plan_target,
    )

    print(
        "Primitive action counts:",
        action_counts,
    )

    print()

    print(
        "Visual debug:",
        visual_debug_dir,
    )

    print(
        "Summary:",
        summary_path,
    )

    print("=" * 90)

    return summary


# =====================================================================
# CLI
# =====================================================================


def main() -> None:

    parser = argparse.ArgumentParser(
        description=(
            "Run one complete SeeDo Nut-Assembly pipeline "
            "from raw demonstration/runtime inputs."
        )
    )

    parser.add_argument(
        "--model-config",
        type=Path,
        required=True,
        help="Path to seedo_controller.yaml.",
    )

    parser.add_argument(
        "--case-dir",
        type=Path,
        required=True,
        help=(
            "Input directory containing demo.mp4, "
            "front/, gripper/, and initial_robot_state.json."
        ),
    )

    parser.add_argument(
        "--artifacts-dir",
        type=Path,
        required=True,
        help="Directory where the complete run is stored.",
    )

    parser.add_argument(
        "--task-id",
        type=str,
        default="00",
    )

    args = parser.parse_args()

    run_full_pipeline(
        model_config=(
            args.model_config
        ),

        case_dir=(
            args.case_dir
        ),

        artifacts_dir=(
            args.artifacts_dir
        ),

        task_id=(
            args.task_id
        ),
    )


if __name__ == "__main__":
    main()