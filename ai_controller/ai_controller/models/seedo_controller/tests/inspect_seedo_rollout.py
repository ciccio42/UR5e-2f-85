from __future__ import annotations

import argparse
import importlib
import pickle
import sys
from pathlib import Path

import contextlib
import io

import numpy as np

from ament_index_python.packages import get_package_share_directory

class TeeStream:
    def __init__(self, *streams):
        self.streams = streams

    def write(self, data):
        for stream in self.streams:
            stream.write(data)
        return len(data)

    def flush(self):
        for stream in self.streams:
            stream.flush()

class DebugTrajectoryUnpickler(pickle.Unpickler):
    """Resolve the Trajectory class used by dataset_collector_pkg."""

    def find_class(self, module, name):
        if module.startswith("multi_task_il") and name == "Trajectory":
            return importlib.import_module("savers").Trajectory

        return super().find_class(module, name)


def _add_dataset_collector_scripts_to_path() -> Path:
    scripts_dir = (
        Path(
            get_package_share_directory(
                "dataset_collector_pkg"
            )
        )
        / "scripts"
    )

    if not scripts_dir.is_dir():
        raise FileNotFoundError(
            "dataset_collector_pkg scripts directory "
            f"does not exist: {scripts_dir}"
        )

    if str(scripts_dir) not in sys.path:
        sys.path.append(
            str(scripts_dir)
        )

    return scripts_dir


def _describe_value(
    value,
) -> str:
    if isinstance(value, np.ndarray):
        return (
            f"ndarray shape={value.shape}, "
            f"dtype={value.dtype}"
        )

    if isinstance(value, list):
        return f"list len={len(value)}"

    if isinstance(value, tuple):
        return f"tuple len={len(value)}"

    if isinstance(value, dict):
        return f"dict keys={list(value.keys())}"

    return (
        f"{type(value).__name__}: "
        f"{value!r}"
    )


def _print_observation(
    obs: dict,
) -> None:
    print("  obs:")

    for key, value in obs.items():
        if key == "camera_front_image":
            if isinstance(value, np.ndarray):
                print(
                    f"    {key}: ndarray "
                    f"shape={value.shape}, "
                    f"dtype={value.dtype}"
                )
            else:
                print(
                    f"    {key}: "
                    f"{_describe_value(value)}"
                )

            continue

        print(
            f"    {key}: "
            f"{_describe_value(value)}"
        )

        if isinstance(value, np.ndarray):
            print(
                f"      value={value}"
            )


def _inspect_step(
    trajectory,
    index: int,
) -> tuple[bool, int]:
    step = trajectory[index]

    print()
    print(
        "=" * 80
    )
    print(
        f"STEP {index}"
    )
    print(
        "=" * 80
    )

    if not isinstance(step, dict):
        print(
            "Step is not a dict:"
        )
        print(step)

        return False, 0

    print(
        f"Step keys: {list(step.keys())}"
    )

    obs = step.get(
        "obs"
    )

    action = step.get(
        "action"
    )

    done = step.get(
        "done"
    )

    reward = step.get(
        "reward"
    )

    if isinstance(obs, dict):
        _print_observation(
            obs
        )
    else:
        print(
            f"  obs: {_describe_value(obs)}"
        )

    print(
        "  action:"
    )

    if isinstance(action, np.ndarray):
        print(
            f"    shape={action.shape}"
        )
        print(
            f"    dtype={action.dtype}"
        )
        print(
            f"    value={action}"
        )

        action_size = int(
            action.size
        )

        action_valid = (
            action.shape == (8,)
            and np.all(
                np.isfinite(action)
            )
        )

    else:
        print(
            f"    {_describe_value(action)}"
        )

        action_size = 0
        action_valid = False

    print(
        f"  done: {done!r}"
    )

    print(
        f"  reward: {reward!r}"
    )

    return (
        action_valid,
        action_size,
    )


def inspect_rollout(
    trajectory_path: str | Path,
    expected_steps: int | None,
) -> int:
    trajectory_path = (
        Path(trajectory_path)
        .expanduser()
        .resolve()
    )

    if not trajectory_path.is_file():
        raise FileNotFoundError(
            "Trajectory file does not exist: "
            f"{trajectory_path}"
        )

    scripts_dir = (
        _add_dataset_collector_scripts_to_path()
    )

    print(
        "=== SEEDO SAVED ROLLOUT INSPECTION ==="
    )

    print(
        f"Trajectory file: {trajectory_path}"
    )

    print(
        "dataset_collector_pkg scripts: "
        f"{scripts_dir}"
    )

    print(
        "\n=== LOADING PICKLE ==="
    )

    with trajectory_path.open(
        "rb"
    ) as stream:
        data = DebugTrajectoryUnpickler(
            stream
        ).load()

    print(
        f"Top-level type: {type(data)}"
    )

    if isinstance(data, dict):
        print(
            f"Top-level keys: {list(data.keys())}"
        )

        for key, value in data.items():
            if key == "traj":
                continue

            print(
                f"  {key}: "
                f"{_describe_value(value)}"
            )

    if not isinstance(data, dict):
        raise TypeError(
            "Expected the saved pickle to contain "
            "a top-level dictionary."
        )

    if "traj" not in data:
        raise KeyError(
            "Saved pickle does not contain "
            "the 'traj' key."
        )

    trajectory = data["traj"]

    print(
        "\n=== TRAJECTORY ==="
    )

    print(
        f"Trajectory type: {type(trajectory)}"
    )

    trajectory_length = len(
        trajectory
    )

    print(
        f"Number of recorded steps: "
        f"{trajectory_length}"
    )

    if (
        expected_steps is not None
        and trajectory_length
        != expected_steps
    ):
        raise AssertionError(
            "Unexpected trajectory length: "
            f"expected {expected_steps}, "
            f"found {trajectory_length}."
        )

    valid_actions = 0

    for index in range(
        trajectory_length
    ):
        action_valid, _ = (
            _inspect_step(
                trajectory,
                index,
            )
        )

        if action_valid:
            valid_actions += 1

    print()
    print(
        "=" * 80
    )

    print(
        "SUMMARY"
    )

    print(
        "=" * 80
    )

    print(
        f"Recorded trajectory entries: "
        f"{trajectory_length}"
    )

    print(
        f"Valid 8D actions: "
        f"{valid_actions}/{trajectory_length}"
    )

    if trajectory_length > 0:
        first_step = trajectory[0]
        last_step = trajectory[
            trajectory_length - 1
        ]

        print(
            f"First step done: "
            f"{first_step.get('done')}"
        )

        print(
            f"First step reward: "
            f"{first_step.get('reward')}"
        )

        print(
            f"Last step done: "
            f"{last_step.get('done')}"
        )

        print(
            f"Last step reward: "
            f"{last_step.get('reward')}"
        )

        if (
            last_step.get("done")
            is not True
        ):
            raise AssertionError(
                "Final trajectory step is not "
                "marked done=True."
            )

        if (
            last_step.get("reward")
            != 1
        ):
            raise AssertionError(
                "Final trajectory step does not "
                "have reward=1."
            )

        for index in range(
            trajectory_length - 1
        ):
            step = trajectory[index]

            if step.get("done"):
                raise AssertionError(
                    f"Step {index} is unexpectedly "
                    "marked done=True."
                )

    if valid_actions != trajectory_length:
        raise AssertionError(
            "At least one saved action does not "
            "have the expected finite 8D format."
        )

    print()
    print(
        "ROLLOUT INSPECTION PASSED"
    )

    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Inspect a SeeDo rollout Trajectory "
            "saved by AIControllerNode."
        )
    )

    parser.add_argument(
        "--trajectory",
        default=(
            "/seedo_tests/"
            "ai_controller_node_interactive/"
            "rollouts/"
            "seedo_controller/"
            "pick_place/"
            "task_00/"
            "traj_000.pkl"
        ),
    )

    parser.add_argument(
        "--expected-steps",
        type=int,
        default=58,
        help=(
            "Expected number of recorded "
            "Trajectory entries. "
            "Use a negative value to disable "
            "this check."
        ),
    )

    return parser


def main() -> int:
    parser = build_parser()

    parser.add_argument(
        "--output",
        default=(
            "/seedo_tests/"
            "ai_controller_node_interactive/"
            "rollout_inspection.txt"
        ),
    )

    args = parser.parse_args()

    expected_steps = (
        None
        if args.expected_steps < 0
        else args.expected_steps
    )

    output_path = (
        Path(args.output)
        .expanduser()
        .resolve()
    )

    output_path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    buffer = io.StringIO()

    tee = TeeStream(
        sys.stdout,
        buffer,
    )

    exit_code = 0

    try:
        with contextlib.redirect_stdout(tee):
            inspect_rollout(
                trajectory_path=args.trajectory,
                expected_steps=expected_steps,
            )

    except Exception:
        exit_code = 1

        import traceback

        traceback_text = traceback.format_exc()

        print(
            traceback_text,
            file=tee,
        )

    finally:
        output_path.write_text(
            buffer.getvalue(),
            encoding="utf-8",
        )

        print(
            f"\nInspection artifact saved to: "
            f"{output_path}"
        )

    return exit_code


if __name__ == "__main__":
    raise SystemExit(
        main()
    )