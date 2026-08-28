#!/usr/bin/env python3
"""Diagnose UR5e rotation targets and terminal action padding.

The script reads action safetensors without loading visual embeddings and checks:

1. the exact training encoder and inference decoder for R -> 6D -> R;
2. the TCP z axis represented by the third column of the absolute rotation;
3. real delta-rotation magnitudes, axes, and composition convention;
4. terminal non-zero actions and their amplification by padded action chunks;
5. matrix -> quaternion -> matrix serialization before MoveIt.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Any, Callable

import numpy as np
from safetensors import safe_open
from scipy.spatial.transform import Rotation


REPO_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = REPO_ROOT / "mimic_video_workspace"
DEFAULT_DATA_DIR = WORKSPACE_ROOT / "processed_data" / "ur5e_pick_place_action_no_extension"
DEFAULT_OUTPUT_DIR = WORKSPACE_ROOT / "outputs" / "action_rotation_diagnostics"

ROTATION_KEY = "eef_rot_ref_delta_lowdim"
STATE_ROTATION_KEY = "eef_rot_lowdim"
POSITION_KEY = "eef_pos_ref_delta_lowdim"
EMBEDDING_TIMESTAMPS_KEY = "workspace_rgb_embedding_timestamps"
NS_PER_SECOND = 1_000_000_000

LOWDIM_DUPLICATE_KEYS = (
    "eef_pos_ref_delta_lowdim",
    "eef_rot_ref_delta_lowdim",
    "gripper_action_lowdim",
    "eef_pos_lowdim",
    "eef_rot_lowdim",
    "gripper_lowdim",
)

PERCENTILES = (0, 1, 5, 25, 50, 75, 95, 99, 100)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA_DIR)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--action-horizon", type=int, default=15)
    parser.add_argument("--target-frequency", type=float, default=10.0)
    parser.add_argument("--terminal-position-threshold-mm", type=float, default=0.5)
    parser.add_argument("--terminal-rotation-threshold-deg", type=float, default=0.5)
    parser.add_argument("--roundtrip-tolerance-deg", type=float, default=1.0e-3)
    parser.add_argument("--limit", type=int)
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Exit with status 1 if an exact rotation conversion check fails.",
    )
    return parser.parse_args()


def load_project_rotation_operations() -> dict[str, Any]:
    ai_controller_source = REPO_ROOT / "ai_controller"
    mimic_model_source = WORKSPACE_ROOT / "external" / "mimic-video" / "model"
    sys.path.insert(0, str(ai_controller_source))
    sys.path.insert(0, str(mimic_model_source))

    try:
        from ai_controller.models.mimic_video_controller.mimic_video_utils import (
            quaternion_to_rotation_matrix,
            rotation_6d_to_matrix,
            rotation_matrix_to_6d,
            rotation_matrix_to_quaternion,
        )
        from cosmos_predict2.data.action.data_transforms import RotationMatrixTo6D
    except ImportError as error:
        raise RuntimeError(
            "Cannot import the training and inference rotation functions. "
            "Run this script in the Mimic Video training container."
        ) from error

    training_transform = RotationMatrixTo6D(targets=["rotation"], metas={})
    return {
        "training_transform": training_transform,
        "quaternion_to_matrix": quaternion_to_rotation_matrix,
        "matrix_to_6d": rotation_matrix_to_6d,
        "sixd_to_matrix": rotation_6d_to_matrix,
        "matrix_to_quaternion": rotation_matrix_to_quaternion,
    }


def describe(values: np.ndarray) -> dict[str, float | int | None]:
    values = np.asarray(values, dtype=np.float64).reshape(-1)
    values = values[np.isfinite(values)]
    if not len(values):
        return {"count": 0, "mean": None, "std": None}

    result: dict[str, float | int | None] = {
        "count": int(len(values)),
        "mean": float(np.mean(values)),
        "std": float(np.std(values)),
    }
    for percentile, value in zip(PERCENTILES, np.percentile(values, PERCENTILES), strict=True):
        result[f"p{percentile:02d}"] = float(value)
    return result


def describe_components(values: np.ndarray) -> dict[str, dict[str, float | int | None]]:
    return {
        axis: describe(values[:, index])
        for index, axis in enumerate(("x", "y", "z"))
    }


def percentage(mask: np.ndarray) -> float:
    mask = np.asarray(mask, dtype=bool)
    return float(100.0 * np.mean(mask)) if len(mask) else 0.0


def rotation_error_deg(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    first = np.asarray(first, dtype=np.float64)
    second = np.asarray(second, dtype=np.float64)
    leading_shape = first.shape[:-2]
    first_rotation = Rotation.from_matrix(first.reshape(-1, 3, 3))
    second_rotation = Rotation.from_matrix(second.reshape(-1, 3, 3))
    error = (first_rotation * second_rotation.inv()).magnitude()
    return np.degrees(error).reshape(leading_shape)


def rotation_angles_deg(rotations: np.ndarray) -> np.ndarray:
    rotations = np.asarray(rotations, dtype=np.float64)
    leading_shape = rotations.shape[:-2]
    angles = Rotation.from_matrix(rotations.reshape(-1, 3, 3)).magnitude()
    return np.degrees(angles).reshape(leading_shape)


def matrix_health(rotations: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    determinants = np.linalg.det(rotations)
    identity = np.eye(3, dtype=np.float64)
    orthogonality = np.linalg.norm(
        rotations @ np.swapaxes(rotations, -1, -2) - identity,
        axis=(-2, -1),
    )
    return determinants, orthogonality


def equal_consecutive_rows(values: np.ndarray) -> np.ndarray:
    equal = np.equal(values[:-1], values[1:])
    return equal.reshape(len(equal), -1).all(axis=1)


def training_matrix_to_6d(transform: Any, rotations: np.ndarray) -> np.ndarray:
    transformed = list(transform([("rotation", rotations)]))
    if len(transformed) != 1 or transformed[0][0] != "rotation":
        raise RuntimeError("Unexpected RotationMatrixTo6D output.")
    return np.asarray(transformed[0][1], dtype=np.float32)


def map_rows(function: Callable[[np.ndarray], np.ndarray], values: np.ndarray) -> np.ndarray:
    return np.stack([function(value) for value in values])


def repeat_rotation_angles(rotations: np.ndarray, repetitions: int) -> np.ndarray:
    return np.asarray(
        [rotation_angles_deg(np.linalg.matrix_power(rotation, repetitions)[None])[0] for rotation in rotations],
        dtype=np.float64,
    )


def load_episode_arrays(root: Any, path: Path) -> dict[str, np.ndarray]:
    required = {
        ROTATION_KEY,
        STATE_ROTATION_KEY,
        POSITION_KEY,
        EMBEDDING_TIMESTAMPS_KEY,
        f"{ROTATION_KEY}_timestamps",
        *LOWDIM_DUPLICATE_KEYS,
    }
    available = set(root.keys())
    missing = sorted(required - available)
    if missing:
        raise KeyError(f"{path.name}: missing required tensors: {missing}")
    return {key: root.get_tensor(key) for key in required}


def analyze_episode(
    path: Path,
    operations: dict[str, Any],
    action_horizon: int,
    target_frequency: float,
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    with safe_open(path, framework="np") as root:
        data = load_episode_arrays(root, path)
        lengths = {key: len(data[key]) for key in LOWDIM_DUPLICATE_KEYS}
        if len(set(lengths.values())) != 1:
            raise ValueError(f"{path.name}: inconsistent low-dimensional lengths: {lengths}")

        num_steps = lengths[ROTATION_KEY]
        if num_steps < 2:
            raise ValueError(f"{path.name}: at least two steps are required.")

        lowdim_equal = np.ones(num_steps - 1, dtype=bool)
        for key in LOWDIM_DUPLICATE_KEYS:
            lowdim_equal &= equal_consecutive_rows(data[key])

        full_duplicate_indices: list[int] = []
        if "workspace_rgb" in root.keys():
            workspace_rgb = root.get_slice("workspace_rgb")
            for first_index in np.flatnonzero(lowdim_equal):
                if np.array_equal(workspace_rgb[first_index], workspace_rgb[first_index + 1]):
                    full_duplicate_indices.append(int(first_index + 1))
            terminal_rgb_repeated = bool(
                np.array_equal(workspace_rgb[num_steps - 2], workspace_rgb[num_steps - 1])
            )
        else:
            terminal_rgb_repeated = False

    delta_rotations = np.asarray(data[ROTATION_KEY], dtype=np.float64)
    delta_positions = np.asarray(data[POSITION_KEY], dtype=np.float64)
    state_quaternions = np.asarray(data[STATE_ROTATION_KEY], dtype=np.float64)
    if delta_rotations.shape != (num_steps, 3, 3):
        raise ValueError(f"{path.name}: invalid delta rotation shape {delta_rotations.shape}")
    if delta_positions.shape != (num_steps, 3):
        raise ValueError(f"{path.name}: invalid delta position shape {delta_positions.shape}")
    if state_quaternions.shape != (num_steps, 4):
        raise ValueError(f"{path.name}: invalid quaternion shape {state_quaternions.shape}")

    state_rotations = map_rows(operations["quaternion_to_matrix"], state_quaternions)

    training_delta_6d = training_matrix_to_6d(operations["training_transform"], delta_rotations)
    controller_delta_6d = map_rows(operations["matrix_to_6d"], delta_rotations)
    decoded_delta_rotations = map_rows(operations["sixd_to_matrix"], training_delta_6d)

    training_state_6d = training_matrix_to_6d(operations["training_transform"], state_rotations)
    controller_state_6d = map_rows(operations["matrix_to_6d"], state_rotations)
    decoded_state_rotations = map_rows(operations["sixd_to_matrix"], training_state_6d)

    delta_roundtrip_errors = rotation_error_deg(decoded_delta_rotations, delta_rotations)
    state_roundtrip_errors = rotation_error_deg(decoded_state_rotations, state_rotations)
    delta_roundtrip_frobenius = np.linalg.norm(
        decoded_delta_rotations - delta_rotations,
        axis=(-2, -1),
    )
    state_roundtrip_frobenius = np.linalg.norm(
        decoded_state_rotations - state_rotations,
        axis=(-2, -1),
    )
    encoder_delta_differences = np.max(np.abs(training_delta_6d - controller_delta_6d), axis=1)
    encoder_state_differences = np.max(np.abs(training_state_6d - controller_state_6d), axis=1)
    delta_6d_first_norm = np.linalg.norm(training_delta_6d[:, :3], axis=1)
    delta_6d_second_norm = np.linalg.norm(training_delta_6d[:, 3:], axis=1)
    delta_6d_row_dot = np.sum(training_delta_6d[:, :3] * training_delta_6d[:, 3:], axis=1)
    state_6d_first_norm = np.linalg.norm(training_state_6d[:, :3], axis=1)
    state_6d_second_norm = np.linalg.norm(training_state_6d[:, 3:], axis=1)
    state_6d_row_dot = np.sum(training_state_6d[:, :3] * training_state_6d[:, 3:], axis=1)

    left_targets = decoded_delta_rotations[:-1] @ state_rotations[:-1]
    right_targets = state_rotations[:-1] @ decoded_delta_rotations[:-1]
    next_states = state_rotations[1:]
    left_composition_errors = rotation_error_deg(left_targets, next_states)
    right_composition_errors = rotation_error_deg(right_targets, next_states)

    moveit_quaternions = np.stack(
        [
            operations["matrix_to_quaternion"](target, reference_quaternion=reference)
            for target, reference in zip(left_targets, state_quaternions[:-1], strict=True)
        ]
    )
    moveit_roundtrip = map_rows(operations["quaternion_to_matrix"], moveit_quaternions)
    moveit_serialization_errors = rotation_error_deg(moveit_roundtrip, left_targets)

    sequential_errors: list[float] = []
    integrated_rotation = state_rotations[0]
    for index in range(num_steps - 1):
        integrated_rotation = decoded_delta_rotations[index] @ integrated_rotation
        sequential_errors.append(
            float(rotation_error_deg(integrated_rotation[None], state_rotations[index + 1][None])[0])
        )
    sequential_errors_array = np.asarray(sequential_errors, dtype=np.float64)

    tcp_z = state_rotations[:, :, 2]
    downward_alignment = np.clip(-tcp_z[:, 2], -1.0, 1.0)
    tcp_z_tilt_deg = np.degrees(np.arccos(downward_alignment))

    # This is intentionally reported as a transpose diagnostic. The TCP z axis
    # is the third column for a local-to-base active rotation matrix.
    third_row = state_rotations[:, 2, :]
    third_row_downward_alignment = np.clip(-third_row[:, 2], -1.0, 1.0)
    third_row_tilt_deg = np.degrees(np.arccos(third_row_downward_alignment))

    delta_angles = rotation_angles_deg(delta_rotations)
    delta_rotation_vectors_deg = np.degrees(Rotation.from_matrix(delta_rotations).as_rotvec())
    delta_euler_xyz_deg = Rotation.from_matrix(delta_rotations).as_euler("xyz", degrees=True)

    raw_delta_determinants, raw_delta_orthogonality = matrix_health(delta_rotations)
    state_determinants, state_orthogonality = matrix_health(state_rotations)

    terminal_position_mm = float(np.linalg.norm(delta_positions[-1]) * 1000.0)
    terminal_rotation_deg = float(delta_angles[-1])

    action_timestamps = np.asarray(data[f"{ROTATION_KEY}_timestamps"], dtype=np.uint64)
    embedding_timestamps = np.asarray(data[EMBEDDING_TIMESTAMPS_KEY], dtype=np.uint64)
    if len(action_timestamps) != num_steps:
        raise ValueError(f"{path.name}: action timestamp length mismatch.")
    if np.any(np.diff(action_timestamps.astype(np.int64)) <= 0):
        raise ValueError(f"{path.name}: action timestamps are not strictly increasing.")

    duplicate_embedding_timestamps = int(len(embedding_timestamps) - len(np.unique(embedding_timestamps)))
    first_action_timestamp = int(action_timestamps[0])
    last_action_timestamp = int(action_timestamps[-1])
    usable_anchor_timestamps = embedding_timestamps[
        (embedding_timestamps >= first_action_timestamp) & (embedding_timestamps <= last_action_timestamp)
    ].astype(np.float64)
    terminal_embedding_anchors = int(np.sum(embedding_timestamps > last_action_timestamp))

    requested_offsets = np.linspace(
        0.0,
        (action_horizon - 1) / target_frequency * NS_PER_SECOND,
        action_horizon,
        dtype=np.float64,
    )
    requested_timestamps = usable_anchor_timestamps[:, None] + requested_offsets[None, :]
    padded_mask = requested_timestamps > float(last_action_timestamp)
    terminal_use_mask = requested_timestamps >= float(last_action_timestamp)
    padded_slots = int(np.sum(padded_mask))
    terminal_action_uses = int(np.sum(terminal_use_mask))
    padded_chunks = int(np.sum(np.any(padded_mask, axis=1)))

    worst_tcp_index = int(np.argmax(tcp_z_tilt_deg))
    record = {
        "episode": path.stem,
        "steps": num_steps,
        "usable_visual_anchors": int(len(usable_anchor_timestamps)),
        "terminal_embedding_anchors_excluded_by_reader": terminal_embedding_anchors,
        "duplicate_embedding_timestamps": duplicate_embedding_timestamps,
        "exact_full_step_duplicates": len(full_duplicate_indices),
        "terminal_rgb_equals_previous": terminal_rgb_repeated,
        "delta_roundtrip_max_deg": float(np.max(delta_roundtrip_errors)),
        "state_roundtrip_max_deg": float(np.max(state_roundtrip_errors)),
        "moveit_serialization_max_deg": float(np.max(moveit_serialization_errors)),
        "encoder_max_abs_difference": float(
            max(np.max(encoder_delta_differences), np.max(encoder_state_differences))
        ),
        "tcp_z_downward_percent": percentage(tcp_z[:, 2] < 0.0),
        "tcp_z_tilt_mean_deg": float(np.mean(tcp_z_tilt_deg)),
        "tcp_z_tilt_max_deg": float(np.max(tcp_z_tilt_deg)),
        "worst_tcp_z_step": worst_tcp_index,
        "worst_tcp_z_x": float(tcp_z[worst_tcp_index, 0]),
        "worst_tcp_z_y": float(tcp_z[worst_tcp_index, 1]),
        "worst_tcp_z_z": float(tcp_z[worst_tcp_index, 2]),
        "delta_angle_mean_deg": float(np.mean(delta_angles)),
        "delta_angle_max_deg": float(np.max(delta_angles)),
        "left_composition_mean_error_deg": float(np.mean(left_composition_errors)),
        "left_composition_max_error_deg": float(np.max(left_composition_errors)),
        "right_composition_mean_error_deg": float(np.mean(right_composition_errors)),
        "right_composition_max_error_deg": float(np.max(right_composition_errors)),
        "sequential_integration_final_error_deg": float(sequential_errors_array[-1]),
        "sequential_integration_max_error_deg": float(np.max(sequential_errors_array)),
        "terminal_position_delta_mm": terminal_position_mm,
        "terminal_rotation_delta_deg": terminal_rotation_deg,
        "terminal_gripper_action": float(np.asarray(data["gripper_action_lowdim"][-1]).reshape(-1)[0]),
        "padded_chunks": padded_chunks,
        "padded_target_slots": padded_slots,
        "terminal_action_target_uses": terminal_action_uses,
        "terminal_position_extra_padding_mm": terminal_position_mm * (action_horizon - 1),
        "terminal_position_full_chunk_mm": terminal_position_mm * action_horizon,
        "terminal_rotation_extra_padding_deg": float(
            repeat_rotation_angles(delta_rotations[-1:], action_horizon - 1)[0]
        ),
        "terminal_rotation_full_chunk_deg": float(
            repeat_rotation_angles(delta_rotations[-1:], action_horizon)[0]
        ),
    }

    arrays = {
        "delta_roundtrip_errors": delta_roundtrip_errors,
        "state_roundtrip_errors": state_roundtrip_errors,
        "delta_roundtrip_frobenius": delta_roundtrip_frobenius,
        "state_roundtrip_frobenius": state_roundtrip_frobenius,
        "moveit_serialization_errors": moveit_serialization_errors,
        "encoder_differences": np.concatenate((encoder_delta_differences, encoder_state_differences)),
        "delta_6d_first_norm": delta_6d_first_norm,
        "delta_6d_second_norm": delta_6d_second_norm,
        "delta_6d_row_dot": delta_6d_row_dot,
        "state_6d_first_norm": state_6d_first_norm,
        "state_6d_second_norm": state_6d_second_norm,
        "state_6d_row_dot": state_6d_row_dot,
        "tcp_z": tcp_z,
        "tcp_z_tilt_deg": tcp_z_tilt_deg,
        "third_row_tilt_deg": third_row_tilt_deg,
        "delta_angles_deg": delta_angles,
        "delta_rotation_vectors_deg": delta_rotation_vectors_deg,
        "delta_euler_xyz_deg": delta_euler_xyz_deg,
        "raw_delta_determinants": raw_delta_determinants,
        "raw_delta_orthogonality": raw_delta_orthogonality,
        "state_determinants": state_determinants,
        "state_orthogonality": state_orthogonality,
        "left_composition_errors": left_composition_errors,
        "right_composition_errors": right_composition_errors,
        "sequential_errors": sequential_errors_array,
        "terminal_position_mm": np.asarray([terminal_position_mm]),
        "terminal_rotation_deg": np.asarray([terminal_rotation_deg]),
        "terminal_position_extra_padding_mm": np.asarray(
            [record["terminal_position_extra_padding_mm"]]
        ),
        "terminal_position_full_chunk_mm": np.asarray([record["terminal_position_full_chunk_mm"]]),
        "terminal_rotation_extra_padding_deg": np.asarray(
            [record["terminal_rotation_extra_padding_deg"]]
        ),
        "terminal_rotation_full_chunk_deg": np.asarray([record["terminal_rotation_full_chunk_deg"]]),
    }
    return record, arrays


def concatenate(results: list[dict[str, np.ndarray]], key: str) -> np.ndarray:
    return np.concatenate([result[key] for result in results], axis=0)


def counts_at_thresholds(values: np.ndarray, thresholds: tuple[float, ...]) -> dict[str, Any]:
    return {
        str(threshold): {
            "count": int(np.sum(values <= threshold)),
            "percent": percentage(values <= threshold),
        }
        for threshold in thresholds
    }


def write_csv(path: Path, records: list[dict[str, Any]]) -> None:
    if not records:
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(records[0].keys()))
        writer.writeheader()
        writer.writerows(records)


def build_report(
    args: argparse.Namespace,
    paths: list[Path],
    records: list[dict[str, Any]],
    arrays: list[dict[str, np.ndarray]],
) -> dict[str, Any]:
    delta_roundtrip = concatenate(arrays, "delta_roundtrip_errors")
    state_roundtrip = concatenate(arrays, "state_roundtrip_errors")
    moveit_serialization = concatenate(arrays, "moveit_serialization_errors")
    encoder_differences = concatenate(arrays, "encoder_differences")
    tcp_z = concatenate(arrays, "tcp_z")
    tcp_z_tilt = concatenate(arrays, "tcp_z_tilt_deg")
    third_row_tilt = concatenate(arrays, "third_row_tilt_deg")
    delta_angles = concatenate(arrays, "delta_angles_deg")
    delta_rotation_vectors = concatenate(arrays, "delta_rotation_vectors_deg")
    delta_euler_xyz = concatenate(arrays, "delta_euler_xyz_deg")
    terminal_position = concatenate(arrays, "terminal_position_mm")
    terminal_rotation = concatenate(arrays, "terminal_rotation_deg")

    tolerance = args.roundtrip_tolerance_deg
    conversion_checks = {
        "training_encoder_vs_inference_encoder": {
            "max_abs_difference": float(np.max(encoder_differences)),
            "pass": bool(np.max(encoder_differences) == 0.0),
        },
        "absolute_R_to_6D_to_R_error_deg": describe(state_roundtrip),
        "delta_R_to_6D_to_R_error_deg": describe(delta_roundtrip),
        "absolute_R_to_6D_to_R_frobenius_error": describe(
            concatenate(arrays, "state_roundtrip_frobenius")
        ),
        "delta_R_to_6D_to_R_frobenius_error": describe(
            concatenate(arrays, "delta_roundtrip_frobenius")
        ),
        "matrix_to_MoveIt_quaternion_to_matrix_error_deg": describe(moveit_serialization),
        "tolerance_deg": tolerance,
    }
    conversion_checks["pass"] = bool(
        conversion_checks["training_encoder_vs_inference_encoder"]["pass"]
        and np.max(state_roundtrip) <= tolerance
        and np.max(delta_roundtrip) <= tolerance
        and np.max(moveit_serialization) <= tolerance
    )

    near_zero = (terminal_position <= args.terminal_position_threshold_mm) & (
        terminal_rotation <= args.terminal_rotation_threshold_deg
    )

    total_anchors = sum(int(record["usable_visual_anchors"]) for record in records)
    total_target_slots = total_anchors * args.action_horizon
    total_padded_slots = sum(int(record["padded_target_slots"]) for record in records)
    total_padded_chunks = sum(int(record["padded_chunks"]) for record in records)

    report = {
        "input": {
            "data_dir": str(args.data_dir.resolve()),
            "episodes": len(paths),
            "steps": int(len(delta_angles)),
            "action_horizon": args.action_horizon,
            "target_frequency_hz": args.target_frequency,
            "padded_tails_assumption": True,
        },
        "conventions": {
            "training_6d": "first two rows of R, flattened",
            "inference_6d": "Gram-Schmidt on two rows, third row = cross(row0, row1)",
            "tcp_z_in_base_link": "R[:, 2], the third column of the absolute local-to-base rotation",
            "base_link_axis_assumption": "+Z is up, therefore TCP z points down when R[:, 2].z < 0",
            "inference_delta_composition": "R_target = R_delta @ R_current",
            "quaternion_order": "XYZW",
        },
        "conversion_checks": conversion_checks,
        "training_6d_target_geometry": {
            "delta_first_row_norm": describe(concatenate(arrays, "delta_6d_first_norm")),
            "delta_second_row_norm": describe(concatenate(arrays, "delta_6d_second_norm")),
            "delta_row_dot_product": describe(concatenate(arrays, "delta_6d_row_dot")),
            "absolute_first_row_norm": describe(concatenate(arrays, "state_6d_first_norm")),
            "absolute_second_row_norm": describe(concatenate(arrays, "state_6d_second_norm")),
            "absolute_row_dot_product": describe(concatenate(arrays, "state_6d_row_dot")),
            "expected": "row norms near 1 and row dot product near 0",
        },
        "tcp_z_direction": {
            "column_R_col_2": {
                "z_component": describe(tcp_z[:, 2]),
                "tilt_from_base_minus_z_deg": describe(tcp_z_tilt),
                "points_into_negative_base_z_percent": percentage(tcp_z[:, 2] < 0.0),
                "within_15_deg_of_down_percent": percentage(tcp_z_tilt <= 15.0),
                "within_30_deg_of_down_percent": percentage(tcp_z_tilt <= 30.0),
                "within_45_deg_of_down_percent": percentage(tcp_z_tilt <= 45.0),
            },
            "third_row_transpose_diagnostic": {
                "tilt_from_base_minus_z_deg": describe(third_row_tilt),
                "note": "This is not the TCP z axis; it is included to expose row/column confusion.",
            },
        },
        "delta_rotation_distribution": {
            "angle_deg": describe(delta_angles),
            "rotation_vector_components_deg": describe_components(delta_rotation_vectors),
            "euler_xyz_components_deg": describe_components(delta_euler_xyz),
            "angle_thresholds_deg": {
                str(threshold): {
                    "count_above": int(np.sum(delta_angles > threshold)),
                    "percent_above": percentage(delta_angles > threshold),
                }
                for threshold in (0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 45.0)
            },
            "stored_delta_matrix_determinant": describe(
                concatenate(arrays, "raw_delta_determinants")
            ),
            "stored_delta_matrix_orthogonality_frobenius": describe(
                concatenate(arrays, "raw_delta_orthogonality")
            ),
            "absolute_state_matrix_determinant": describe(concatenate(arrays, "state_determinants")),
            "absolute_state_matrix_orthogonality_frobenius": describe(
                concatenate(arrays, "state_orthogonality")
            ),
        },
        "delta_composition_against_next_state": {
            "R_delta_at_R_current_error_deg": describe(
                concatenate(arrays, "left_composition_errors")
            ),
            "R_current_at_R_delta_error_deg": describe(
                concatenate(arrays, "right_composition_errors")
            ),
            "sequential_integration_error_deg": describe(concatenate(arrays, "sequential_errors")),
            "note": "The final action has no next state in the same episode and is excluded from this check.",
        },
        "terminal_actions": {
            "position_delta_norm_mm": describe(terminal_position),
            "rotation_delta_angle_deg": describe(terminal_rotation),
            "position_near_zero_thresholds_mm": counts_at_thresholds(
                terminal_position, (0.1, 0.5, 1.0, 2.0, 5.0)
            ),
            "rotation_near_zero_thresholds_deg": counts_at_thresholds(
                terminal_rotation, (0.1, 0.5, 1.0, 2.0, 5.0)
            ),
            "combined_near_zero": {
                "position_threshold_mm": args.terminal_position_threshold_mm,
                "rotation_threshold_deg": args.terminal_rotation_threshold_deg,
                "count": int(np.sum(near_zero)),
                "percent": percentage(near_zero),
            },
            "warning": "The final delta cannot be checked against a following state, but padded tails can repeat it.",
        },
        "padded_tail_effect": {
            "usable_anchor_chunks": total_anchors,
            "chunks_with_at_least_one_padded_slot": total_padded_chunks,
            "chunks_with_padding_percent": 100.0 * total_padded_chunks / max(1, total_anchors),
            "target_slots": total_target_slots,
            "padded_target_slots": total_padded_slots,
            "padded_target_slots_percent": 100.0 * total_padded_slots / max(1, total_target_slots),
            "terminal_action_target_uses": sum(
                int(record["terminal_action_target_uses"]) for record in records
            ),
            "extra_padding_if_terminal_delta_were_executed": {
                "position_norm_mm": describe(
                    concatenate(arrays, "terminal_position_extra_padding_mm")
                ),
                "rotation_angle_deg": describe(
                    concatenate(arrays, "terminal_rotation_extra_padding_deg")
                ),
                "repetitions": args.action_horizon - 1,
            },
            "full_target_at_last_anchor_if_executed": {
                "position_norm_mm": describe(concatenate(arrays, "terminal_position_full_chunk_mm")),
                "rotation_angle_deg": describe(
                    concatenate(arrays, "terminal_rotation_full_chunk_deg")
                ),
                "repetitions": args.action_horizon,
            },
        },
        "duplicate_checks": {
            "exact_full_step_duplicates": sum(
                int(record["exact_full_step_duplicates"]) for record in records
            ),
            "episodes_with_terminal_rgb_equal_to_previous": sum(
                bool(record["terminal_rgb_equals_previous"]) for record in records
            ),
            "duplicate_visual_embedding_timestamps": sum(
                int(record["duplicate_embedding_timestamps"]) for record in records
            ),
            "terminal_embedding_anchors_excluded_by_reader": sum(
                int(record["terminal_embedding_anchors_excluded_by_reader"]) for record in records
            ),
        },
    }
    return report


def print_summary(report: dict[str, Any], output_dir: Path) -> None:
    conversion = report["conversion_checks"]
    tcp = report["tcp_z_direction"]["column_R_col_2"]
    delta = report["delta_rotation_distribution"]["angle_deg"]
    composition = report["delta_composition_against_next_state"]
    terminal = report["terminal_actions"]
    padding = report["padded_tail_effect"]
    duplicates = report["duplicate_checks"]

    print(f"Episodes: {report['input']['episodes']}, steps: {report['input']['steps']}")
    print(
        "R -> 6D -> R: "
        f"{'PASS' if conversion['pass'] else 'FAIL'}, "
        f"absolute max={conversion['absolute_R_to_6D_to_R_error_deg']['p100']:.6g} deg, "
        f"delta max={conversion['delta_R_to_6D_to_R_error_deg']['p100']:.6g} deg"
    )
    print(
        "R -> quaternion -> R before MoveIt: "
        f"max={conversion['matrix_to_MoveIt_quaternion_to_matrix_error_deg']['p100']:.6g} deg"
    )
    print(
        "TCP z from R[:,2]: "
        f"negative base z={tcp['points_into_negative_base_z_percent']:.2f}%, "
        f"tilt p50={tcp['tilt_from_base_minus_z_deg']['p50']:.3f} deg, "
        f"p95={tcp['tilt_from_base_minus_z_deg']['p95']:.3f} deg, "
        f"max={tcp['tilt_from_base_minus_z_deg']['p100']:.3f} deg"
    )
    print(
        "Real delta rotation angle: "
        f"p50={delta['p50']:.4f} deg, p95={delta['p95']:.4f} deg, "
        f"p99={delta['p99']:.4f} deg, max={delta['p100']:.4f} deg"
    )
    left = composition["R_delta_at_R_current_error_deg"]
    right = composition["R_current_at_R_delta_error_deg"]
    print(
        "Composition vs next state: "
        f"delta@current p50={left['p50']:.4f} deg, p95={left['p95']:.4f}; "
        f"current@delta p50={right['p50']:.4f} deg, p95={right['p95']:.4f}"
    )
    print(
        "Terminal action: "
        f"position p50={terminal['position_delta_norm_mm']['p50']:.4f} mm, "
        f"p95={terminal['position_delta_norm_mm']['p95']:.4f} mm; "
        f"rotation p50={terminal['rotation_delta_angle_deg']['p50']:.4f} deg, "
        f"p95={terminal['rotation_delta_angle_deg']['p95']:.4f} deg; "
        f"combined near zero={terminal['combined_near_zero']['percent']:.2f}%"
    )
    print(
        "Padded tails: "
        f"chunks affected={padding['chunks_with_padding_percent']:.2f}%, "
        f"target slots padded={padding['padded_target_slots_percent']:.2f}%"
    )
    print(
        "Duplicates: "
        f"full steps={duplicates['exact_full_step_duplicates']}, "
        f"embedding timestamps={duplicates['duplicate_visual_embedding_timestamps']}, "
        f"episodes with repeated terminal RGB={duplicates['episodes_with_terminal_rgb_equal_to_previous']}"
    )
    print(f"JSON report: {output_dir / 'rotation_diagnostics.json'}")
    print(f"Episode CSV: {output_dir / 'episode_diagnostics.csv'}")


def main() -> None:
    args = parse_args()
    if args.action_horizon <= 0:
        raise ValueError("--action-horizon must be positive.")
    if args.target_frequency <= 0:
        raise ValueError("--target-frequency must be positive.")
    if args.limit is not None and args.limit <= 0:
        raise ValueError("--limit must be positive.")

    data_dir = args.data_dir.resolve()
    if not data_dir.is_dir():
        raise FileNotFoundError(data_dir)
    paths = sorted(data_dir.glob("*.safetensors"))
    if args.limit is not None:
        paths = paths[: args.limit]
    if not paths:
        raise FileNotFoundError(f"No safetensors files found in {data_dir}")

    operations = load_project_rotation_operations()
    records: list[dict[str, Any]] = []
    arrays: list[dict[str, np.ndarray]] = []
    for index, path in enumerate(paths, start=1):
        record, episode_arrays = analyze_episode(
            path,
            operations,
            action_horizon=args.action_horizon,
            target_frequency=args.target_frequency,
        )
        records.append(record)
        arrays.append(episode_arrays)
        print(f"[{index}/{len(paths)}] checked {path.name}")

    report = build_report(args, paths, records, arrays)
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    report_path = output_dir / "rotation_diagnostics.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    write_csv(output_dir / "episode_diagnostics.csv", records)
    print_summary(report, output_dir)

    if args.strict and not report["conversion_checks"]["pass"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
