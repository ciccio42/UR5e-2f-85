#!/usr/bin/env python3
"""
Inspect SeeDo / dataset .pkl trajectory files without modifying them.

Examples:
    python3 inspect_pkl.py /path/to/traj000.pkl
    python3 inspect_pkl.py /path/to/traj000.pkl --step 0
    python3 inspect_pkl.py /path/to/traj000.pkl --step 0 --full
    python3 inspect_pkl.py /path/to/folder --recursive

NOTE:
    Pickle files can execute code while being loaded. Use this script only
    with trusted .pkl files (e.g. your own dataset / rollout files).
"""

from __future__ import annotations

import argparse
import pickle
from pathlib import Path
from typing import Any

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import torch
except ImportError:
    torch = None


class TrajectoryProxy:
    """Compatibility class for pickled Trajectory objects."""

    def __init__(self, *args, **kwargs):
        self._data = []
        self._raw_state = []
        self._config_str = None

    @property
    def T(self) -> int:
        return len(getattr(self, "_data", []))

    def __len__(self) -> int:
        return self.T

    def get(self, t: int, decompress: bool = True) -> dict[str, Any]:
        obs_t, reward_t, done_t, info_t, action_t = self._data[t]
        obs_t = dict(obs_t) if isinstance(obs_t, dict) else obs_t

        if decompress and isinstance(obs_t, dict) and cv2 is not None:
            obs_t = _best_effort_decompress_obs(obs_t)

        result = {
            "obs": obs_t,
            "reward": reward_t,
            "done": done_t,
            "info": info_t,
            "action": action_t,
        }
        return {k: v for k, v in result.items() if v is not None}

    def __getitem__(self, t: int):
        return self.get(t)


class InspectUnpickler(pickle.Unpickler):
    def find_class(self, module: str, name: str):
        if name == "Trajectory":
            return TrajectoryProxy
        return super().find_class(module, name)


def load_pickle(path: Path) -> Any:
    with path.open("rb") as stream:
        return InspectUnpickler(stream).load()


def _looks_like_encoded_image(value):
    if not isinstance(value, np.ndarray):
        return False

    if value.dtype != np.uint8:
        return False

    if value.ndim == 1:
        flat = value
    elif value.ndim == 2 and value.shape[1] == 1:
        flat = value.reshape(-1)
    else:
        return False

    return (
        flat.size > 2
        and flat[0] == 0xFF
        and flat[1] == 0xD8
    )

def _best_effort_decompress_obs(obs: dict[str, Any]) -> dict[str, Any]:
    result = dict(obs)

    for key in (
        "image",
        "depth",
        "camera_front_image",
        "camera_lateral_left_image",
        "camera_lateral_right_image",
        "eye_in_hand_image",
    ):
        value = result.get(key)

        if _looks_like_encoded_image(value):
            encoded = value.reshape(-1)

            decoded = cv2.imdecode(
                encoded,
                cv2.IMREAD_COLOR,
            )

            if decoded is not None:
                result[key] = decoded

    return result


def scalar_repr(value: Any, max_length: int = 160) -> str:
    text = repr(value)
    if len(text) > max_length:
        text = text[: max_length - 3] + "..."
    return text


def summarize_array(array: np.ndarray, full: bool = False) -> str:
    base = f"ndarray shape={array.shape} dtype={array.dtype}"

    if array.size == 0:
        return base + " empty"

    if np.issubdtype(array.dtype, np.number):
        try:
            finite = array[np.isfinite(array)]
            if finite.size:
                base += (
                    f" min={finite.min():.6g}"
                    f" max={finite.max():.6g}"
                    f" mean={finite.mean():.6g}"
                )
        except Exception:
            pass

    if _looks_like_encoded_image(array):
        base += " [JPEG-compressed image]"

    if full and array.size <= 200:
        base += f"\n      values={array}"

    return base


def summarize_value(value: Any, full: bool = False) -> str:
    if isinstance(value, np.ndarray):
        return summarize_array(value, full=full)

    if torch is not None and isinstance(value, torch.Tensor):
        tensor = value.detach().cpu()
        text = f"torch.Tensor shape={tuple(tensor.shape)} dtype={tensor.dtype}"
        if tensor.numel() > 0:
            try:
                text += (
                    f" min={tensor.min().item():.6g}"
                    f" max={tensor.max().item():.6g}"
                )
            except Exception:
                pass
        if full and tensor.numel() <= 200:
            text += f"\n      values={tensor}"
        return text

    if isinstance(value, dict):
        return f"dict keys={list(value.keys())}"

    if isinstance(value, (list, tuple)):
        return f"{type(value).__name__} len={len(value)}"

    if isinstance(value, (bytes, bytearray)):
        return f"{type(value).__name__} len={len(value)}"

    if isinstance(value, (str, int, float, bool, type(None))):
        return scalar_repr(value)

    return f"{type(value).__module__}.{type(value).__name__}: {scalar_repr(value)}"

CAMERA_KEYWORDS = (
    "camera",
    "image",
    "rgb",
    "depth",
)

BBOX_KEYWORDS = (
    "bbox",
    "bboxes",
    "bounding_box",
    "bounding_boxes",
    "box",
    "boxes",
)


def collect_key_paths(
    obj,
    prefix="",
    paths=None,
):
    """
    Recursively collect all dictionary keys and their complete paths.
    """

    if paths is None:
        paths = {}

    if isinstance(obj, dict):
        for key, value in obj.items():

            key = str(key)

            current_path = (
                f"{prefix}.{key}"
                if prefix
                else key
            )

            if current_path not in paths:
                paths[current_path] = {
                    "type": type(value).__name__,
                    "shapes": set(),
                }

            if isinstance(value, np.ndarray):
                paths[current_path]["shapes"].add(
                    tuple(value.shape)
                )

            elif torch is not None and isinstance(
                value,
                torch.Tensor,
            ):
                paths[current_path]["shapes"].add(
                    tuple(value.shape)
                )

            collect_key_paths(
                value,
                prefix=current_path,
                paths=paths,
            )

    elif isinstance(obj, (list, tuple)):

        for value in obj:
            collect_key_paths(
                value,
                prefix=prefix,
                paths=paths,
            )

    return paths


def is_camera_key(path):
    path_lower = path.lower()

    return any(
        keyword in path_lower
        for keyword in CAMERA_KEYWORDS
    )


def is_bbox_key(path):
    path_lower = path.lower()

    return any(
        keyword in path_lower
        for keyword in BBOX_KEYWORDS
    )


def print_dataset_keys(traj):
    """
    Scan the complete trajectory and report all saved keys,
    with dedicated sections for cameras and bounding boxes.
    """

    all_paths = {}

    for step_index in range(len(traj._data)):

        try:
            step = get_trajectory_step(
                traj,
                step_index,
            )
        except Exception as exc:
            print(
                f"Warning: cannot inspect step "
                f"{step_index}: {exc}"
            )
            continue

        collect_key_paths(
            step,
            paths=all_paths,
        )

        raw_states = getattr(
            traj,
            "_raw_state",
            None,
        )

        if (
            isinstance(raw_states, list)
            and step_index < len(raw_states)
            and raw_states[step_index] is not None
        ):
            collect_key_paths(
                raw_states[step_index],
                prefix="raw_state",
                paths=all_paths,
            )

    print()
    print("=" * 80)
    print("ALL SAVED KEYS")
    print("=" * 80)

    for path in sorted(all_paths):

        info = all_paths[path]

        shapes = sorted(
            info["shapes"]
        )

        if shapes:
            print(
                f"{path:<60} "
                f"type={info['type']:<15} "
                f"shape={shapes}"
            )
        else:
            print(
                f"{path:<60} "
                f"type={info['type']}"
            )

    camera_paths = [
        path
        for path in all_paths
        if is_camera_key(path)
    ]

    bbox_paths = [
        path
        for path in all_paths
        if is_bbox_key(path)
    ]

    print()
    print("=" * 80)
    print("CAMERA / IMAGE KEYS")
    print("=" * 80)

    if camera_paths:
        for path in sorted(camera_paths):

            info = all_paths[path]

            print(
                f"{path} "
                f"type={info['type']} "
                f"shape={sorted(info['shapes'])}"
            )
    else:
        print("No camera/image keys detected.")

    print()
    print("=" * 80)
    print("BOUNDING BOX KEYS")
    print("=" * 80)

    if bbox_paths:
        for path in sorted(bbox_paths):

            info = all_paths[path]

            print(
                f"{path} "
                f"type={info['type']} "
                f"shape={sorted(info['shapes'])}"
            )
    else:
        print("No bounding-box keys detected.")

def print_mapping(mapping: dict[str, Any], indent: str = "", full: bool = False):
    for key, value in mapping.items():
        print(f"{indent}{key}: {summarize_value(value, full=full)}")

        if full and isinstance(value, dict):
            print_mapping(value, indent=indent + "  ", full=full)


def is_trajectory(obj: Any) -> bool:
    return hasattr(obj, "_data") and isinstance(getattr(obj, "_data", None), list)


def get_trajectory_step(traj: Any, index: int) -> dict[str, Any]:
    if hasattr(traj, "get"):
        try:
            return traj.get(index, decompress=True)
        except TypeError:
            return traj.get(index)

    raw = traj._data[index]
    obs, reward, done, info, action = raw
    result = {
        "obs": obs,
        "reward": reward,
        "done": done,
        "info": info,
        "action": action,
    }
    return {k: v for k, v in result.items() if v is not None}


def inspect_step(traj: Any, index: int, full: bool = False):
    length = len(traj._data)

    if index < 0:
        index += length

    if index < 0 or index >= length:
        raise IndexError(
            f"Step {index} is out of range for trajectory length {length}."
        )

    step = get_trajectory_step(traj, index)

    print()
    print(f"--- STEP {index} ---")

    for key, value in step.items():
        if key == "obs" and isinstance(value, dict):
            print("obs:")
            for obs_key, obs_value in value.items():
                print(f"  {obs_key}: {summarize_value(obs_value, full=full)}")
        else:
            print(f"{key}: {summarize_value(value, full=full)}")

    raw_states = getattr(traj, "_raw_state", None)
    if isinstance(raw_states, list) and index < len(raw_states):
        raw_state = raw_states[index]
        if raw_state is not None:
            print("raw_state: " + summarize_value(raw_state, full=full))
            if full and isinstance(raw_state, dict):
                print_mapping(raw_state, indent="  ", full=True)

def inspect_gripper_and_status(traj):
    print()
    print("=" * 80)
    print("GRIPPER / ACTION / STATUS")
    print("=" * 80)

    for i in range(len(traj._data)):
        step = get_trajectory_step(
            traj,
            i,
        )

        obs = step.get("obs", {})
        action = step.get("action")
        info = step.get("info", {})

        gripper_qpos = obs.get(
            "gripper_qpos"
        )

        gripper_action = None

        if action is not None:
            try:
                gripper_action = action[-1]
            except Exception:
                pass

        status = None

        if isinstance(info, dict):
            status = info.get("status")

        print(
            f"step={i:03d} | "
            f"gripper_qpos={gripper_qpos!r} | "
            f"action[-1]={gripper_action!r} | "
            f"status={status!r}"
        )

def inspect_trajectory(
    traj: Any,
    *,
    step: int | None,
    full: bool,
    all_steps: bool,
):
    data = getattr(traj, "_data", [])
    print(f"Trajectory length: {len(data)}")

    config = getattr(traj, "_config_str", None)
    if config is not None:
        print(f"Trajectory config_str: {scalar_repr(config, 300)}")

    if not data:
        return

    if step is not None:
        inspect_step(traj, step, full=full)
        return

    indexes = range(len(data)) if all_steps else [0] + ([len(data) - 1] if len(data) > 1 else [])

    for index in indexes:
        inspect_step(traj, index, full=full)

    if not all_steps and len(data) > 2:
        print()
        print(
            f"(Showing first and last step only. "
            f"Use --all-steps to inspect all {len(data)} steps.)"
        )


def inspect_object(
    obj: Any,
    *,
    step: int | None,
    full: bool,
    all_steps: bool,
):
    if isinstance(obj, dict):
        print("Top-level type: dict")
        print(f"Top-level keys: {list(obj.keys())}")

        metadata = {key: value for key, value in obj.items() if key != "traj"}

        if metadata:
            print()
            print("--- METADATA ---")
            print_mapping(metadata, full=full)

        traj = obj.get("traj")
        if traj is not None and is_trajectory(traj):
            print()
            print("--- TRAJECTORY ---")
            inspect_trajectory(
                traj,
                step=step,
                full=full,
                all_steps=all_steps,
            )
            inspect_gripper_and_status(
                traj
            )
            print_dataset_keys(traj)
            # Temporary inspection
            inspect_bbox_values(
                traj,
                step_index=14,
            )
        elif traj is not None:
            print()
            print(f"traj: {summarize_value(traj, full=full)}")

        if "traj" not in obj:
            print()
            print("--- CONTENT ---")
            print_mapping(obj, full=full)

        return

    if is_trajectory(obj):
        print(f"Top-level type: {type(obj).__name__}")
        inspect_trajectory(
            obj,
            step=step,
            full=full,
            all_steps=all_steps,
        )
        print_dataset_keys(traj)
        return

    print(f"Top-level type: {type(obj).__module__}.{type(obj).__name__}")
    print(summarize_value(obj, full=full))

    if full and hasattr(obj, "__dict__"):
        print()
        print("--- __dict__ ---")
        print_mapping(vars(obj), full=True)


def inspect_file(
    path: Path,
    *,
    step: int | None,
    full: bool,
    all_steps: bool,
):
    print("=" * 80)
    print(f"FILE: {path}")
    print(f"SIZE: {path.stat().st_size / (1024 * 1024):.3f} MiB")
    print("=" * 80)

    try:
        obj = load_pickle(path)
    except Exception as exc:
        print(f"ERROR while loading pickle: {type(exc).__name__}: {exc}")
        return False

    inspect_object(
        obj,
        step=step,
        full=full,
        all_steps=all_steps,
    )

    return True

def inspect_bbox_values(traj, step_index=0):
    step = get_trajectory_step(
        traj,
        step_index,
    )

    obs = step.get("obs", {})
    obj_bb = obs.get("obj_bb", {})

    print()
    print("=" * 80)
    print(f"BOUNDING BOX VALUES - STEP {step_index}")
    print("=" * 80)

    camera_front = obj_bb.get(
        "camera_front",
        {}
    )

    for object_name, bbox in camera_front.items():
        print()
        print(object_name)

        if isinstance(bbox, dict):
            print(
                "  upper_left_corner:",
                bbox.get("upper_left_corner"),
            )
            print(
                "  bottom_right_corner:",
                bbox.get("bottom_right_corner"),
            )
            print(
                "  center:",
                bbox.get("center"),
            )

def collect_files(path: Path, recursive: bool) -> list[Path]:
    if path.is_file():
        return [path]

    pattern = "**/*.pkl" if recursive else "*.pkl"
    return sorted(file for file in path.glob(pattern) if file.is_file())


def parse_args():
    parser = argparse.ArgumentParser(
        description="Inspect SeeDo/dataset pickle files and trajectory contents."
    )

    parser.add_argument(
        "path",
        type=Path,
        help="Path to a .pkl file or a directory containing .pkl files.",
    )

    parser.add_argument(
        "--step",
        type=int,
        default=None,
        help="Inspect one trajectory step in detail (e.g. --step 0).",
    )

    parser.add_argument(
        "--all-steps",
        action="store_true",
        help="Print a summary for every trajectory step.",
    )

    parser.add_argument(
        "--full",
        action="store_true",
        help=(
            "Print small arrays and nested dictionaries in more detail. "
            "Large images/arrays are still summarized by shape/dtype."
        ),
    )

    parser.add_argument(
        "--recursive",
        action="store_true",
        help="When PATH is a directory, search recursively for .pkl files.",
    )

    return parser.parse_args()


def main() -> int:
    args = parse_args()
    path = args.path.expanduser().resolve()

    if not path.exists():
        print(f"ERROR: path does not exist: {path}")
        return 1

    files = collect_files(path, recursive=args.recursive)

    if not files:
        print(f"No .pkl files found in: {path}")
        return 1

    failures = 0

    for index, file in enumerate(files):
        if index:
            print()

        okay = inspect_file(
            file,
            step=args.step,
            full=args.full,
            all_steps=args.all_steps,
        )

        if not okay:
            failures += 1

    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())