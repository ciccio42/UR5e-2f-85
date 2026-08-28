#!/usr/bin/env python3
"""Remove artificial episode extension from UR5e action safetensors.

The script removes bit-identical duplicated robot steps, restores the compact
10 Hz state/action timeline, and remaps the already-computed visual embedding
anchors accordingly. Terminal padded embedding anchors are preserved with
their original offset relative to the end of the episode.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np
from safetensors import safe_open
from safetensors.numpy import load_file, save_file


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT_DIR = WORKSPACE_ROOT / "processed_data" / "ur5e_pick_place_action"
DEFAULT_OUTPUT_DIR = WORKSPACE_ROOT / "processed_data" / "ur5e_pick_place_action_no_extension"

DEFAULT_FPS = 10
DEFAULT_TARGET_LENGTH = 61
NS_PER_SECOND = 1_000_000_000

VISUAL_EMBEDDING_KEY = "workspace_rgb_embedding"
VISUAL_EMBEDDING_TIMESTAMPS_KEY = f"{VISUAL_EMBEDDING_KEY}_timestamps"

REQUIRED_STEP_KEYS = (
    "workspace_rgb",
    "eef_pos_ref_delta_lowdim",
    "eef_rot_ref_delta_lowdim",
    "gripper_action_lowdim",
    "eef_pos_lowdim",
    "eef_rot_lowdim",
    "gripper_lowdim",
)

# Comparing low-dimensional data first avoids comparing every pair of large
# RGB frames. RGB is checked bit-for-bit only for candidate duplicate pairs.
DUPLICATE_LOW_DIM_KEYS = (
    "eef_pos_ref_delta_lowdim",
    "eef_rot_ref_delta_lowdim",
    "gripper_action_lowdim",
    "eef_pos_lowdim",
    "eef_rot_lowdim",
    "gripper_lowdim",
)


@dataclass(frozen=True)
class EpisodeReport:
    episode: str
    old_steps: int
    new_steps: int
    removed_step_indices: list[int]
    old_embeddings: int
    new_embeddings: int
    dropped_nonfinal_embedding_anchors: int
    dropped_terminal_embedding_anchors: int
    position_error_median_m: float | None
    position_error_max_m: float | None
    rotation_error_median_deg: float | None
    rotation_error_max_deg: float | None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, default=DEFAULT_INPUT_DIR)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS)
    parser.add_argument("--target-length", type=int, default=DEFAULT_TARGET_LENGTH)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate and report the conversion without writing output files.",
    )
    parser.add_argument(
        "--limit",
        type=int,
        help="Process only the first N episodes. Useful for a smoke test.",
    )
    return parser.parse_args()


def exact_step_timestamps(length: int, dt_ns: int) -> np.ndarray:
    return np.arange(length, dtype=np.uint64) * np.uint64(dt_ns)


def require_keys(data: dict[str, np.ndarray], path: Path) -> None:
    required = {
        VISUAL_EMBEDDING_KEY,
        VISUAL_EMBEDDING_TIMESTAMPS_KEY,
        *(key for step_key in REQUIRED_STEP_KEYS for key in (step_key, f"{step_key}_timestamps")),
    }
    missing = sorted(required - data.keys())
    if missing:
        raise KeyError(f"{path.name}: missing required keys: {missing}")


def validate_original_timeline(
    data: dict[str, np.ndarray],
    path: Path,
    dt_ns: int,
) -> int:
    old_steps = len(data["workspace_rgb"])
    expected_timestamps = exact_step_timestamps(old_steps, dt_ns)

    for key in REQUIRED_STEP_KEYS:
        values = data[key]
        timestamps = data[f"{key}_timestamps"]
        if values.shape[0] != old_steps:
            raise ValueError(
                f"{path.name}: {key} has {values.shape[0]} rows, expected {old_steps}."
            )
        if timestamps.shape != (old_steps,):
            raise ValueError(
                f"{path.name}: {key}_timestamps has shape {timestamps.shape}, "
                f"expected {(old_steps,)}."
            )
        if not np.array_equal(timestamps, expected_timestamps):
            raise ValueError(
                f"{path.name}: {key}_timestamps is not an exact {NS_PER_SECOND // dt_ns} Hz timeline."
            )

    embedding_values = data[VISUAL_EMBEDDING_KEY]
    embedding_timestamps = data[VISUAL_EMBEDDING_TIMESTAMPS_KEY]
    if embedding_values.shape[0] != embedding_timestamps.shape[0]:
        raise ValueError(
            f"{path.name}: embedding/timestamp length mismatch: "
            f"{embedding_values.shape[0]} != {embedding_timestamps.shape[0]}."
        )
    if embedding_timestamps.ndim != 1:
        raise ValueError(
            f"{path.name}: {VISUAL_EMBEDDING_TIMESTAMPS_KEY} must be one-dimensional."
        )
    if len(embedding_timestamps) and np.any(np.diff(embedding_timestamps.astype(np.int64)) < 0):
        raise ValueError(f"{path.name}: visual embedding timestamps are not sorted.")

    return old_steps


def find_step_keys(data: dict[str, np.ndarray], old_steps: int) -> list[str]:
    """Find all per-step values that share the workspace timeline."""
    workspace_timestamps = data["workspace_rgb_timestamps"]
    step_keys: list[str] = []

    for key, values in data.items():
        if key.endswith("_timestamps") or key == VISUAL_EMBEDDING_KEY:
            continue
        timestamp_key = f"{key}_timestamps"
        if timestamp_key not in data or values.ndim == 0 or values.shape[0] != old_steps:
            continue
        timestamps = data[timestamp_key]
        if timestamps.shape == (old_steps,) and np.array_equal(timestamps, workspace_timestamps):
            step_keys.append(key)

    missing = sorted(set(REQUIRED_STEP_KEYS) - set(step_keys))
    if missing:
        raise ValueError(f"Required per-step keys were not detected: {missing}")
    return sorted(step_keys)


def find_duplicate_second_indices(data: dict[str, np.ndarray]) -> np.ndarray:
    old_steps = len(data["workspace_rgb"])
    duplicates: list[int] = []

    for first_idx in range(old_steps - 1):
        second_idx = first_idx + 1
        if not all(
            np.array_equal(data[key][first_idx], data[key][second_idx])
            for key in DUPLICATE_LOW_DIM_KEYS
        ):
            continue
        if np.array_equal(data["workspace_rgb"][first_idx], data["workspace_rgb"][second_idx]):
            duplicates.append(second_idx)

    return np.asarray(duplicates, dtype=np.int64)


def expected_extension_duplicate_indices(original_steps: int, duplicates: int) -> np.ndarray:
    if duplicates == 0:
        return np.empty(0, dtype=np.int64)

    duplicated_source_indices = set(
        np.round(np.linspace(0, original_steps - 1, duplicates)).astype(np.int64).tolist()
    )
    if len(duplicated_source_indices) != duplicates:
        raise ValueError(
            "The extension rule would select the same source step more than once; "
            "the episode cannot be reconstructed unambiguously."
        )

    expanded_idx = 0
    duplicate_indices: list[int] = []
    for source_idx in range(original_steps):
        expanded_idx += 1
        if source_idx in duplicated_source_indices:
            duplicate_indices.append(expanded_idx)
            expanded_idx += 1

    return np.asarray(duplicate_indices, dtype=np.int64)


def validate_extension_pattern(
    path: Path,
    old_steps: int,
    target_length: int,
    duplicate_indices: np.ndarray,
) -> None:
    if old_steps > target_length:
        if len(duplicate_indices):
            raise ValueError(
                f"{path.name}: found bit-identical consecutive steps in an episode of "
                f"length {old_steps}. extension() only modifies episodes shorter than "
                f"{target_length}, so these may be real stationary samples."
            )
        return

    if old_steps < target_length:
        raise ValueError(
            f"{path.name}: episode has {old_steps} steps. The current extension pipeline "
            f"should have produced exactly {target_length}."
        )

    if not len(duplicate_indices):
        return

    original_steps = old_steps - len(duplicate_indices)
    expected = expected_extension_duplicate_indices(original_steps, len(duplicate_indices))
    if not np.array_equal(duplicate_indices, expected):
        raise ValueError(
            f"{path.name}: detected duplicate indices {duplicate_indices.tolist()}, but "
            f"extension() predicts {expected.tolist()} for an original episode of "
            f"{original_steps} steps. Refusing to remove ambiguous samples."
        )


def make_compaction_map(old_steps: int, duplicate_indices: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    keep = np.ones(old_steps, dtype=bool)
    keep[duplicate_indices] = False
    old_to_new = np.cumsum(keep, dtype=np.int64) - 1
    if old_to_new[0] < 0:
        raise ValueError("The first step cannot be removed.")
    return keep, old_to_new


def compact_step_data(
    result: dict[str, np.ndarray],
    step_keys: list[str],
    keep: np.ndarray,
    dt_ns: int,
) -> None:
    new_timestamps = exact_step_timestamps(int(keep.sum()), dt_ns)
    for key in step_keys:
        result[key] = np.ascontiguousarray(result[key][keep])
        result[f"{key}_timestamps"] = new_timestamps.copy()


def remap_visual_embeddings(
    result: dict[str, np.ndarray],
    old_to_new: np.ndarray,
    dt_ns: int,
    path: Path,
) -> tuple[int, int]:
    embeddings = result[VISUAL_EMBEDDING_KEY]
    timestamps = result[VISUAL_EMBEDDING_TIMESTAMPS_KEY].astype(np.uint64, copy=False)
    old_steps = len(old_to_new)

    old_anchor_indices, remainders = np.divmod(timestamps, np.uint64(dt_ns))
    if np.any(remainders != 0):
        invalid = timestamps[remainders != 0][:5].tolist()
        raise ValueError(
            f"{path.name}: embedding timestamps are not aligned to {dt_ns} ns: {invalid}"
        )
    old_anchor_indices = old_anchor_indices.astype(np.int64)

    new_steps = int(old_to_new[-1]) + 1
    removed_steps = old_steps - new_steps

    # Costruisce il nuovo indice temporale di ogni embedding.
    mapped_indices = np.empty_like(old_anchor_indices)

    # Anchor che appartengono alla timeline reale:
    # usano la vera mappa di compattazione.
    in_episode = old_anchor_indices < old_steps

    mapped_indices[in_episode] = old_to_new[
        old_anchor_indices[in_episode]
    ]

    # Anchor oltre la fine dell'episodio:
    # vengono semplicemente traslati del numero totale di step rimossi.
    #
    # In questo modo mantengono lo stesso offset rispetto
    # all'ultimo frame reale.
    mapped_indices[~in_episode] = (
        old_anchor_indices[~in_episode] - removed_steps
    )

    # Non eliminiamo più gli anchor terminali.
    terminal_dropped = 0

    # Tutti gli embedding rimangono candidati.
    selected_rows = np.arange(len(old_anchor_indices), dtype=np.int64)
    selected_new_indices = mapped_indices

    nonfinal_dropped = 0

    # video_embeddings_idxs is normally unique, but enforce one visual anchor
    # per compact timestamp even if an input file contains duplicate anchors.
    best_row_by_new_idx: dict[int, int] = {}
    for row, new_idx in zip(selected_rows.tolist(), selected_new_indices.tolist(), strict=True):
        previous_row = best_row_by_new_idx.get(new_idx)
        if previous_row is None or old_anchor_indices[row] > old_anchor_indices[previous_row]:
            best_row_by_new_idx[new_idx] = row

    ordered = sorted(best_row_by_new_idx.items())
    if not ordered:
        raise ValueError(f"{path.name}: no visual embedding anchors remain after compaction.")

    kept_new_indices = np.asarray([new_idx for new_idx, _row in ordered], dtype=np.int64)
    kept_rows = np.asarray([row for _new_idx, row in ordered], dtype=np.int64)
    result[VISUAL_EMBEDDING_KEY] = np.ascontiguousarray(embeddings[kept_rows])
    result[VISUAL_EMBEDDING_TIMESTAMPS_KEY] = (
        kept_new_indices.astype(np.uint64) * np.uint64(dt_ns)
    )
    return nonfinal_dropped, terminal_dropped


def quaternion_xyzw_to_matrix(quaternions: np.ndarray) -> np.ndarray:
    quaternions = np.asarray(quaternions, dtype=np.float64)
    norms = np.linalg.norm(quaternions, axis=-1, keepdims=True)
    if np.any(norms == 0):
        raise ValueError("A zero-norm end-effector quaternion was found.")
    x, y, z, w = np.moveaxis(quaternions / norms, -1, 0)

    matrices = np.empty((*quaternions.shape[:-1], 3, 3), dtype=np.float64)
    matrices[..., 0, 0] = 1 - 2 * (y * y + z * z)
    matrices[..., 0, 1] = 2 * (x * y - z * w)
    matrices[..., 0, 2] = 2 * (x * z + y * w)
    matrices[..., 1, 0] = 2 * (x * y + z * w)
    matrices[..., 1, 1] = 1 - 2 * (x * x + z * z)
    matrices[..., 1, 2] = 2 * (y * z - x * w)
    matrices[..., 2, 0] = 2 * (x * z - y * w)
    matrices[..., 2, 1] = 2 * (y * z + x * w)
    matrices[..., 2, 2] = 1 - 2 * (x * x + y * y)
    return matrices


def kinematic_errors(data: dict[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    positions = data["eef_pos_lowdim"].astype(np.float64)
    delta_positions = data["eef_pos_ref_delta_lowdim"].astype(np.float64)
    position_errors = np.linalg.norm(
        positions[:-1] + delta_positions[:-1] - positions[1:], axis=-1
    )

    rotations = quaternion_xyzw_to_matrix(data["eef_rot_lowdim"])
    delta_rotations = data["eef_rot_ref_delta_lowdim"].astype(np.float64)
    predicted_rotations = delta_rotations[:-1] @ rotations[:-1]
    relative = rotations[1:] @ np.swapaxes(predicted_rotations, -1, -2)
    cos_angles = np.clip((np.trace(relative, axis1=-2, axis2=-1) - 1) / 2, -1, 1)
    rotation_errors_deg = np.degrees(np.arccos(cos_angles))
    return position_errors, rotation_errors_deg


def optional_error_stat(values: np.ndarray, operation: str) -> float | None:
    if not len(values):
        return None
    if operation == "median":
        return float(np.median(values))
    if operation == "max":
        return float(np.max(values))
    raise ValueError(operation)


def validate_compacted_data(
    data: dict[str, np.ndarray],
    step_keys: list[str],
    dt_ns: int,
    path: Path,
) -> None:
    new_steps = len(data["workspace_rgb"])
    expected = exact_step_timestamps(new_steps, dt_ns)
    for key in step_keys:
        if data[key].shape[0] != new_steps:
            raise ValueError(f"{path.name}: compacted {key} has the wrong length.")
        if not np.array_equal(data[f"{key}_timestamps"], expected):
            raise ValueError(f"{path.name}: compacted {key} has invalid timestamps.")

    embedding_timestamps = data[VISUAL_EMBEDDING_TIMESTAMPS_KEY]
    if len(embedding_timestamps) != len(np.unique(embedding_timestamps)):
        raise ValueError(f"{path.name}: duplicate visual embedding timestamps remain.")
    if np.any(np.diff(embedding_timestamps.astype(np.int64)) <= 0):
        raise ValueError(f"{path.name}: visual embedding timestamps are not strictly increasing.")
    

    remaining_duplicates = find_duplicate_second_indices(data)
    if len(remaining_duplicates):
        raise ValueError(
            f"{path.name}: full-step duplicates remain at indices "
            f"{remaining_duplicates.tolist()}."
        )


def load_metadata(path: Path) -> dict[str, str] | None:
    with safe_open(path, framework="np") as handle:
        return handle.metadata()


def write_safetensor(
    path: Path,
    data: dict[str, np.ndarray],
    metadata: dict[str, str] | None,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = path.with_name(f".{path.name}.tmp")
    try:
        save_file(data, temporary_path, metadata=metadata)
        os.replace(temporary_path, path)
    finally:
        temporary_path.unlink(missing_ok=True)


def process_episode(
    input_path: Path,
    output_path: Path,
    dt_ns: int,
    target_length: int,
    dry_run: bool,
) -> EpisodeReport:
    data = load_file(input_path)
    require_keys(data, input_path)
    old_steps = validate_original_timeline(data, input_path, dt_ns)
    step_keys = find_step_keys(data, old_steps)
    duplicate_indices = find_duplicate_second_indices(data)
    validate_extension_pattern(input_path, old_steps, target_length, duplicate_indices)

    keep, old_to_new = make_compaction_map(old_steps, duplicate_indices)
    result = dict(data)
    compact_step_data(result, step_keys, keep, dt_ns)
    nonfinal_dropped, terminal_dropped = remap_visual_embeddings(
        result, old_to_new, dt_ns, input_path
    )
    validate_compacted_data(result, step_keys, dt_ns, input_path)

    position_errors, rotation_errors = kinematic_errors(result)
    if not dry_run:
        write_safetensor(output_path, result, load_metadata(input_path))

    return EpisodeReport(
        episode=input_path.name,
        old_steps=old_steps,
        new_steps=int(keep.sum()),
        removed_step_indices=duplicate_indices.tolist(),
        old_embeddings=len(data[VISUAL_EMBEDDING_KEY]),
        new_embeddings=len(result[VISUAL_EMBEDDING_KEY]),
        dropped_nonfinal_embedding_anchors=nonfinal_dropped,
        dropped_terminal_embedding_anchors=terminal_dropped,
        position_error_median_m=optional_error_stat(position_errors, "median"),
        position_error_max_m=optional_error_stat(position_errors, "max"),
        rotation_error_median_deg=optional_error_stat(rotation_errors, "median"),
        rotation_error_max_deg=optional_error_stat(rotation_errors, "max"),
    )


def invalidate_statistics_cache(output_dir: Path, dry_run: bool) -> None:
    cache_dir = output_dir / ".statistics_cache"
    if cache_dir.exists():
        if dry_run:
            print(f"Would remove stale statistics cache: {cache_dir}")
        else:
            shutil.rmtree(cache_dir)
            print(f"Removed stale statistics cache: {cache_dir}")


def aggregate_report(
    input_dir: Path,
    output_dir: Path,
    fps: int,
    reports: list[EpisodeReport],
) -> dict[str, Any]:
    return {
        "input_dir": str(input_dir),
        "output_dir": str(output_dir),
        "fps": fps,
        "episodes": len(reports),
        "old_steps": sum(report.old_steps for report in reports),
        "new_steps": sum(report.new_steps for report in reports),
        "removed_steps": sum(report.old_steps - report.new_steps for report in reports),
        "old_embeddings": sum(report.old_embeddings for report in reports),
        "new_embeddings": sum(report.new_embeddings for report in reports),
        "episode_reports": [asdict(report) for report in reports],
    }


def main() -> None:
    args = parse_args()
    if args.fps <= 0 or NS_PER_SECOND % args.fps != 0:
        raise ValueError("--fps must be a positive integer divisor of 1,000,000,000.")
    if args.target_length <= 0:
        raise ValueError("--target-length must be positive.")
    if args.limit is not None and args.limit <= 0:
        raise ValueError("--limit must be positive.")

    input_dir = args.input_dir.resolve()
    output_dir = args.output_dir.resolve()
    if input_dir == output_dir:
        raise ValueError("Input and output directories must be different.")
    if not input_dir.is_dir():
        raise FileNotFoundError(input_dir)

    input_paths = sorted(input_dir.glob("*.safetensors"))
    if args.limit is not None:
        input_paths = input_paths[: args.limit]
    if not input_paths:
        raise FileNotFoundError(f"No .safetensors files found in {input_dir}")

    dt_ns = NS_PER_SECOND // args.fps
    reports: list[EpisodeReport] = []
    skipped = 0

    if not args.dry_run:
        output_dir.mkdir(parents=True, exist_ok=True)
    invalidate_statistics_cache(output_dir, args.dry_run)

    for episode_number, input_path in enumerate(input_paths, start=1):
        output_path = output_dir / input_path.name
        if output_path.exists() and not args.overwrite and not args.dry_run:
            print(f"[{episode_number}/{len(input_paths)}] skip {input_path.name}: output exists")
            skipped += 1
            continue

        report = process_episode(
            input_path=input_path,
            output_path=output_path,
            dt_ns=dt_ns,
            target_length=args.target_length,
            dry_run=args.dry_run,
        )
        reports.append(report)
        mode = "check" if args.dry_run else "wrote"
        print(
            f"[{episode_number}/{len(input_paths)}] {mode} {input_path.name}: "
            f"steps {report.old_steps}->{report.new_steps}, "
            f"embeddings {report.old_embeddings}->{report.new_embeddings}"
        )

    summary = aggregate_report(input_dir, output_dir, args.fps, reports)
    summary["skipped_existing"] = skipped
    report_path = output_dir / "action_preprocess_report.json"
    if args.dry_run:
        print(json.dumps(summary, indent=2))
    else:
        report_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        print(f"Report: {report_path}")
        print("Dataset statistics will be recomputed automatically by the next Action Head training run.")


if __name__ == "__main__":
    main()
