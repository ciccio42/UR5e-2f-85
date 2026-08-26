from __future__ import annotations

import argparse
from pathlib import Path

from .test_action_planning import run_action_planning_test
from .test_keyframe import run_keyframe_test
from .test_lmp_generator import run_lmp_generator_test
from .test_scene_interpreter import run_scene_interpreter_test
from .test_scene_perceiver import run_scene_perceiver_test
from .test_seedo_controller import run_seedo_controller_test
from .test_visual_prompting import run_visual_prompting_test

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Standalone test for the SeeDo keyframe selector."
    )
    parser.add_argument(
        "--video",
        type=Path,
        default=None,
        help="Path to the input demonstration video.",
    )
    parser.add_argument(
        "--artifacts-dir",
        type=Path,
        default=None,
        help=(
            "Optional directory for persistent debug artifacts. "
            "If omitted, temporary artifact storage is used."
        ),
    )
    parser.add_argument(
        "--expected-keyframes",
        type=int,
        nargs="*",
        default=None,
        help="Optional expected keyframe indexes.",
    )

    parser.add_argument(
        "--stage",
        choices=[
            "keyframe",
            "visual_prompting",
            "action_planning",
            "scene_perceiver",
            "scene_interpreter",
            "lmp_generator",
            "seedo_controller",
        ],
        default="keyframe",
        help="Pipeline stage to test.",
    )

    parser.add_argument(
        "--grounding-config",
        type=Path,
        default=Path(
            "/opt/checkpoints/seedo/groundingdino/"
            "GroundingDINO_SwinB.cfg.py"
        ),
    )

    parser.add_argument(
        "--objects",
        type=str,
        default=None,
        help="Comma-separated object labels for visual prompting.",
    )

    parser.add_argument(
        "--grounding-checkpoint",
        type=Path,
        default=Path(
            "/opt/checkpoints/seedo/groundingdino/"
            "groundingdino_swinb_cogcoor.pth"
        ),
    )

    parser.add_argument(
        "--bert-model",
        type=Path,
        default=Path(
            "/opt/checkpoints/seedo/bert-base-uncased"
        ),
    )

    parser.add_argument(
        "--sam-checkpoint",
        type=Path,
        default=Path(
            "/opt/checkpoints/seedo/sam/sam_vit_h_4b8939.pth"
        ),
    )

    parser.add_argument(
        "--sam2-checkpoint",
        type=Path,
        default=Path(
            "/opt/checkpoints/seedo/sam2/sam2_hiera_large.pt"
        ),
    )
    
    parser.add_argument(
        "--model-config",
        type=str,
        help="Path to the SeeDo controller YAML configuration.",
    )

    parser.add_argument(
        "--scene-dir",
        type=Path,
        default=None,
        help=(
            "Directory containing the offline runtime scene "
            "(rgb.png, depth.npy, camera_info.yaml)."
        ),
    )

    parser.add_argument(
        "--base-to-table-transform",
        type=Path,
        default=None,
        help=(
            "YAML file containing the base_link/table_0 "
            "transform used for offline runtime tests."
        ),
    )

    return parser.parse_args()

def main() -> int:
    args = parse_args()

    if args.stage == "keyframe":
        return run_keyframe_test(args)

    if args.stage == "visual_prompting":
        return run_visual_prompting_test(args)

    if args.stage == "action_planning":
        return run_action_planning_test(args)

    if args.stage == "scene_perceiver":
        return run_scene_perceiver_test(args)

    if args.stage == "scene_interpreter":
        return run_scene_interpreter_test(args)

    if args.stage == "lmp_generator":
        return run_lmp_generator_test(args)

    if args.stage == "seedo_controller":
        return run_seedo_controller_test(args)

    raise ValueError(f"Unsupported stage: {args.stage}")
