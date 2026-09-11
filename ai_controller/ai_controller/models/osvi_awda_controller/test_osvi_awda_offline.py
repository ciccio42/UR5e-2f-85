"""Offline OSVI-AWDA smoke test: no ROS node and no robot commands."""

import argparse
import importlib
import os
import tempfile
from pathlib import Path


REQUIRED_MODULES = {
    "cv2": "opencv-python (or the ROS python3-opencv package)",
    "einops": "einops",
    "numpy": "numpy",
    "PIL": "Pillow",
    "torch": "torch",
    "torchvision": "torchvision compatible with the installed torch",
    "yaml": "PyYAML",
}


def check_dependencies():
    errors = []
    for module_name, package_hint in REQUIRED_MODULES.items():
        try:
            importlib.import_module(module_name)
        except Exception as exc:
            errors.append(f"  - {module_name}: {exc} (install {package_hint})")
    if errors:
        raise RuntimeError(
            "Missing or incompatible OSVI-AWDA inference dependencies:\n"
            + "\n".join(errors)
        )


def resolve_config_path(value, config_dir):
    path = Path(os.path.expandvars(os.path.expanduser(str(value))))
    if not path.is_absolute():
        path = config_dir / path
    return path.resolve()


def parse_args():
    default_config = Path(__file__).resolve().with_name("osvi_awda_config.yaml")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=default_config)
    parser.add_argument(
        "--demo-root",
        type=Path,
        default=Path("/dataset/pick_place/human_rgb_pick_place"),
    )
    parser.add_argument("--task-id", default="01")
    parser.add_argument("--checkpoint-dir", type=Path)
    parser.add_argument("--checkpoint-step", type=int)
    parser.add_argument("--device", choices=("cpu", "cuda"))
    return parser.parse_args()


def main():
    args = parse_args()
    check_dependencies()

    import numpy as np
    import yaml

    from ai_controller.models.osvi_awda_controller import OSVIAWDAController

    config_path = args.config.expanduser().resolve()
    if not config_path.is_file():
        raise FileNotFoundError(f"Runtime config not found: {config_path}")

    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}

    if args.checkpoint_dir is not None:
        config["checkpoint_dir"] = str(args.checkpoint_dir.expanduser().resolve())
    if args.checkpoint_step is not None:
        config["checkpoint_step"] = args.checkpoint_step
    if args.device is not None:
        config["device"] = args.device

    # The temporary YAML lives outside the package, so make its relative paths
    # absolute before writing it.
    for key in ("checkpoint_dir", "projection_matrix_path"):
        if not config.get(key):
            raise ValueError(f"Missing required runtime setting: {key}")
        config[key] = str(resolve_config_path(config[key], config_path.parent))

    checkpoint_dir = Path(config["checkpoint_dir"])
    checkpoint_step = int(config["checkpoint_step"])
    checkpoint_path = checkpoint_dir / f"model_save-{checkpoint_step}.pt"
    training_config_path = checkpoint_dir / "config.yaml"
    if not training_config_path.is_file():
        raise FileNotFoundError(f"AWDA training config not found: {training_config_path}")
    if not checkpoint_path.is_file():
        raise FileNotFoundError(f"AWDA checkpoint not found: {checkpoint_path}")

    task_folder = f"task_{str(args.task_id).removeprefix('task_').zfill(2)}"
    demo_directory = args.demo_root.expanduser().resolve() / task_folder
    if not any(demo_directory.glob("*.pkl")):
        raise FileNotFoundError(f"No demo .pkl files found in {demo_directory}")

    # Depth refinement requires a real post-hover image; GUI debug is disabled
    # because this test is intended to work in a headless container.
    config.setdefault("grasp_refinement", {})["enabled"] = False
    config.setdefault("debug", {})["show_waypoint_overlay"] = False
    config["debug"]["save_waypoint_overlay"] = False

    temporary_config = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".yaml", encoding="utf-8", delete=False
        ) as stream:
            yaml.safe_dump(config, stream, sort_keys=False)
            temporary_config = Path(stream.name)

        controller = OSVIAWDAController(
            str(temporary_config),
            task_name="pick_place",
        )
        controller.load_command(
            str(args.demo_root.expanduser().resolve()),
            str(args.task_id),
            save_demo_frames=False,
        )

        front_image = np.zeros((720, 1280, 3), dtype=np.uint8)
        robot_state = {
            "eef_pos": np.array([0.0, 0.0, 0.30], dtype=np.float64),
        }
        actions = controller.inference(
            input_data=[[front_image], robot_state],
            t=0,
            save_path=None,
        )

        assert isinstance(actions, list)
        assert actions
        assert all(np.asarray(action).shape == (8,) for action in actions)
        assert all(np.isfinite(action).all() for action in actions)

        for index, action in enumerate(actions):
            print(index, action)
        print(f"Test superato: {len(actions)} azioni eseguibili")
    finally:
        if temporary_config is not None:
            temporary_config.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
