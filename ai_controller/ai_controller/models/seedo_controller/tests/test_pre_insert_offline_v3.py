#!/usr/bin/env python3
from __future__ import annotations

import argparse
import gc
import inspect
import json
import math
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml


REPO_ROOT = Path("/home/ros2_ws/src/UR5e-2f-85")
SEEDO_DIR = (
    REPO_ROOT
    / "ai_controller"
    / "ai_controller"
    / "models"
    / "seedo_controller"
)
AI_CONTROLLER_ROOT = REPO_ROOT / "ai_controller"

# Make both package-style and local SeeDo imports available.
sys.path.insert(0, str(AI_CONTROLLER_ROOT))
sys.path.insert(0, str(SEEDO_DIR))

from inspect_pkl import load_pickle  # noqa: E402
from scene_perceiver import ScenePerceiver  # noqa: E402
from ai_controller.utils.utils import _quat2mat  # noqa: E402


DEFAULT_CONFIG = (
    SEEDO_DIR
    / "config"
    / "seedo_controller.yaml"
)


def find_unique(root: Path, name: str) -> Path:
    matches = sorted(root.rglob(name))

    if len(matches) != 1:
        formatted = "\n".join(
            f"  - {path}"
            for path in matches
        )

        raise RuntimeError(
            f"Expected exactly one {name!r} below {root}, "
            f"found {len(matches)}.\n{formatted}"
        )

    return matches[0]


def pose_matrix(
    position: np.ndarray,
    quaternion_xyzw: np.ndarray,
) -> np.ndarray:
    T = np.eye(
        4,
        dtype=np.float64,
    )

    T[:3, :3] = _quat2mat(
        np.asarray(
            quaternion_xyzw,
            dtype=np.float64,
        )
    )

    T[:3, 3] = np.asarray(
        position,
        dtype=np.float64,
    )

    return T


def project_base_point(
    point_base: np.ndarray,
    camera_matrix: np.ndarray,
    T_base_camera: np.ndarray,
) -> tuple[float, float]:
    point_base = np.asarray(
        point_base,
        dtype=np.float64,
    )

    K = np.asarray(
        camera_matrix,
        dtype=np.float64,
    )

    T_camera_base = np.linalg.inv(
        np.asarray(
            T_base_camera,
            dtype=np.float64,
        )
    )

    point_camera = (
        T_camera_base
        @ np.array(
            [
                point_base[0],
                point_base[1],
                point_base[2],
                1.0,
            ],
            dtype=np.float64,
        )
    )[:3]

    if point_camera[2] <= 0.0:
        raise RuntimeError(
            "Cannot project point behind the eye-in-hand camera: "
            f"{point_base}"
        )

    u = (
        K[0, 0]
        * point_camera[0]
        / point_camera[2]
        + K[0, 2]
    )

    v = (
        K[1, 1]
        * point_camera[1]
        / point_camera[2]
        + K[1, 2]
    )

    return (
        float(u),
        float(v),
    )


def to_jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()

    if isinstance(value, np.generic):
        return value.item()

    if isinstance(value, Path):
        return str(value)

    if isinstance(value, dict):
        return {
            str(key): to_jsonable(item)
            for key, item in value.items()
        }

    if isinstance(value, (list, tuple)):
        return [
            to_jsonable(item)
            for item in value
        ]

    return value


def save_mask(
    mask: np.ndarray,
    path: Path,
) -> None:
    image = (
        np.asarray(
            mask,
            dtype=np.uint8,
        )
        * 255
    )

    cv2.imwrite(
        str(path),
        image,
    )


def draw_point(
    image: np.ndarray,
    pixel: tuple[float, float],
    color: tuple[int, int, int],
    label: str,
    radius: int = 7,
) -> None:
    u = int(round(pixel[0]))
    v = int(round(pixel[1]))

    cv2.circle(
        image,
        (u, v),
        radius,
        color,
        thickness=-1,
        lineType=cv2.LINE_AA,
    )

    cv2.putText(
        image,
        label,
        (u + 10, v - 8),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        color,
        2,
        cv2.LINE_AA,
    )


def build_hole_aperture_mask(
    *,
    hole_refinement: dict[str, Any],
    shape: tuple[int, int],
    scale: float = 0.90,
) -> np.ndarray:
    """
    Build a conservative image-space aperture mask from the fresh hole fit.

    The ellipse is shrunk slightly so ring-edge depth does not leak into the
    local peg-top segmentation.
    """

    if not (
        0.50 <= scale <= 1.00
    ):
        raise ValueError(
            "Hole-aperture scale must be in [0.50, 1.00]."
        )

    center = np.asarray(
        hole_refinement[
            "hole_center_px"
        ],
        dtype=np.float64,
    )

    axes = np.asarray(
        hole_refinement[
            "hole_ellipse_axes_px"
        ],
        dtype=np.float64,
    )

    angle_deg = float(
        hole_refinement[
            "hole_ellipse_angle_deg"
        ]
    )

    if (
        center.shape != (2,)
        or axes.shape != (2,)
        or not np.all(
            np.isfinite(
                np.concatenate(
                    (
                        center,
                        axes,
                        [angle_deg],
                    )
                )
            )
        )
    ):
        raise ValueError(
            "Invalid fresh-hole ellipse geometry."
        )

    half_width = max(
        2,
        int(
            round(
                0.5
                * axes[0]
                * scale
            )
        ),
    )

    half_height = max(
        2,
        int(
            round(
                0.5
                * axes[1]
                * scale
            )
        ),
    )

    aperture_u8 = np.zeros(
        shape,
        dtype=np.uint8,
    )

    cv2.ellipse(
        aperture_u8,
        (
            int(
                round(
                    center[0]
                )
            ),
            int(
                round(
                    center[1]
                )
            ),
        ),
        (
            half_width,
            half_height,
        ),
        angle_deg,
        0.0,
        360.0,
        255,
        thickness=cv2.FILLED,
    )

    return aperture_u8 > 0


def estimate_pre_insert_peg_top(
    *,
    depth_image: np.ndarray,
    camera_matrix: np.ndarray,
    T_base_camera: np.ndarray,
    aperture_mask: np.ndarray,
    expected_peg_pixel: tuple[float, float],
    expected_axis_direction_base: np.ndarray,
    expected_radius_m: float,
    min_depth_gap_m: float = 0.015,
    min_component_area_px: int = 150,
    max_expected_distance_px: float = 100.0,
    min_cluster_fraction: float = 0.03,
    max_top_depth_mad_m: float = 0.004,
    min_radius_ratio: float = 0.50,
    max_radius_ratio: float = 1.50,
) -> dict[str, Any]:
    """
    Estimate the fresh peg axis XY from the visible peg top face.

    This deliberately does NOT run GroundingDINO/SAM for the peg. The ring
    hole defines a local ROI. Inside that ROI, the peg top is the coherent
    near-depth surface, while the table/background lies substantially farther
    from the eye-in-hand camera.

    The original peg-axis direction is preserved. Only the fresh lateral axis
    position is estimated from the new RGB-D frame.
    """

    depth = np.asarray(
        depth_image,
        dtype=np.float64,
    )

    K = np.asarray(
        camera_matrix,
        dtype=np.float64,
    )

    T_base_camera = np.asarray(
        T_base_camera,
        dtype=np.float64,
    )

    aperture = np.asarray(
        aperture_mask,
        dtype=bool,
    )

    axis_direction_base = np.asarray(
        expected_axis_direction_base,
        dtype=np.float64,
    )

    if depth.ndim != 2:
        raise ValueError(
            "depth_image must be 2D."
        )

    if aperture.shape != depth.shape:
        raise ValueError(
            "aperture_mask and depth_image must have identical shape."
        )

    if K.shape != (3, 3):
        raise ValueError(
            "camera_matrix must have shape (3, 3)."
        )

    if T_base_camera.shape != (4, 4):
        raise ValueError(
            "T_base_camera must have shape (4, 4)."
        )

    axis_norm = float(
        np.linalg.norm(
            axis_direction_base
        )
    )

    if axis_norm <= 1e-12:
        raise ValueError(
            "expected_axis_direction_base has near-zero norm."
        )

    axis_direction_base = (
        axis_direction_base
        / axis_norm
    )

    expected_radius_m = float(
        expected_radius_m
    )

    if expected_radius_m <= 0.0:
        raise ValueError(
            "expected_radius_m must be positive."
        )

    valid = (
        aperture
        & np.isfinite(
            depth
        )
        & (
            depth > 0.0
        )
    )

    depth_values = depth[
        valid
    ]

    if depth_values.size < 300:
        raise RuntimeError(
            "Too few valid depth samples inside the fresh hole aperture: "
            f"{depth_values.size}."
        )

    # Ignore extreme depth outliers before looking for the dominant gap
    # separating the close peg top from the far table/background.
    lower_clip = float(
        np.percentile(
            depth_values,
            1.0,
        )
    )

    upper_clip = float(
        np.percentile(
            depth_values,
            99.0,
        )
    )

    clipped_values = depth_values[
        (
            depth_values
            >= lower_clip
        )
        & (
            depth_values
            <= upper_clip
        )
    ]

    sorted_depth = np.sort(
        clipped_values
    )

    min_cluster_size = max(
        50,
        int(
            round(
                min_cluster_fraction
                * len(
                    sorted_depth
                )
            )
        ),
    )

    if (
        len(
            sorted_depth
        )
        < 2
        * min_cluster_size
        + 1
    ):
        raise RuntimeError(
            "Not enough aperture depth samples for two-surface separation."
        )

    gaps = np.diff(
        sorted_depth
    )

    valid_split_start = (
        min_cluster_size
        - 1
    )

    valid_split_end = (
        len(
            sorted_depth
        )
        - min_cluster_size
    )

    candidate_gaps = gaps[
        valid_split_start:
        valid_split_end
    ]

    if candidate_gaps.size == 0:
        raise RuntimeError(
            "Could not form valid peg/background depth split candidates."
        )

    relative_index = int(
        np.argmax(
            candidate_gaps
        )
    )

    split_index = (
        valid_split_start
        + relative_index
    )

    near_depth_max = float(
        sorted_depth[
            split_index
        ]
    )

    far_depth_min = float(
        sorted_depth[
            split_index
            + 1
        ]
    )

    depth_gap_m = (
        far_depth_min
        - near_depth_max
    )

    if depth_gap_m < min_depth_gap_m:
        raise RuntimeError(
            "No sufficiently strong depth gap separates a peg top "
            "from the background inside the ring hole: "
            f"gap={depth_gap_m * 1000.0:.2f} mm."
        )

    split_depth_m = 0.5 * (
        near_depth_max
        + far_depth_min
    )

    near_mask_u8 = (
        valid
        & (
            depth
            <= split_depth_m
        )
    ).astype(
        np.uint8
    )

    open_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (3, 3),
    )

    close_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (5, 5),
    )

    near_mask_u8 = cv2.morphologyEx(
        near_mask_u8,
        cv2.MORPH_OPEN,
        open_kernel,
    )

    near_mask_u8 = cv2.morphologyEx(
        near_mask_u8,
        cv2.MORPH_CLOSE,
        close_kernel,
    )

    (
        component_count,
        component_labels,
        component_stats,
        component_centroids,
    ) = cv2.connectedComponentsWithStats(
        near_mask_u8,
        connectivity=8,
    )

    expected_u = float(
        expected_peg_pixel[0]
    )

    expected_v = float(
        expected_peg_pixel[1]
    )

    candidates: list[
        dict[str, Any]
    ] = []

    for label_index in range(
        1,
        component_count,
    ):
        area_px = int(
            component_stats[
                label_index,
                cv2.CC_STAT_AREA,
            ]
        )

        if area_px < min_component_area_px:
            continue

        center_u = float(
            component_centroids[
                label_index,
                0,
            ]
        )

        center_v = float(
            component_centroids[
                label_index,
                1,
            ]
        )

        expected_distance_px = float(
            np.hypot(
                center_u
                - expected_u,
                center_v
                - expected_v,
            )
        )

        if (
            expected_distance_px
            > max_expected_distance_px
        ):
            continue

        candidates.append(
            {
                "label": int(
                    label_index
                ),
                "area_px": area_px,
                "center_pixel": (
                    center_u,
                    center_v,
                ),
                "expected_distance_px": (
                    expected_distance_px
                ),
            }
        )

    if not candidates:
        raise RuntimeError(
            "No coherent near-depth component inside the ring hole "
            "is compatible with the expected peg location."
        )

    selected_candidate = min(
        candidates,
        key=lambda candidate: (
            candidate[
                "expected_distance_px"
            ],
            -candidate[
                "area_px"
            ],
        ),
    )

    peg_mask = (
        component_labels
        == selected_candidate[
            "label"
        ]
    )

    peg_valid = (
        peg_mask
        & np.isfinite(
            depth
        )
        & (
            depth > 0.0
        )
    )

    peg_v, peg_u = np.nonzero(
        peg_valid
    )

    peg_depth = depth[
        peg_v,
        peg_u,
    ]

    if peg_depth.size < min_component_area_px:
        raise RuntimeError(
            "Fresh peg-top component contains too few valid depth samples."
        )

    peg_depth_median = float(
        np.median(
            peg_depth
        )
    )

    peg_depth_mad_m = float(
        np.median(
            np.abs(
                peg_depth
                - peg_depth_median
            )
        )
    )

    fx = float(
        K[0, 0]
    )

    fy = float(
        K[1, 1]
    )

    cx = float(
        K[0, 2]
    )

    cy = float(
        K[1, 2]
    )

    point_x_camera = (
        (
            peg_u.astype(
                np.float64
            )
            - cx
        )
        * peg_depth
        / fx
    )

    point_y_camera = (
        (
            peg_v.astype(
                np.float64
            )
            - cy
        )
        * peg_depth
        / fy
    )

    points_camera = np.column_stack(
        (
            point_x_camera,
            point_y_camera,
            peg_depth,
            np.ones(
                len(
                    peg_depth
                ),
                dtype=np.float64,
            ),
        )
    )

    points_base = (
        T_base_camera
        @ points_camera.T
    ).T[
        :,
        :3
    ]

    # The filled top disk is approximately symmetric. Its robust 3D median
    # gives a stable lateral axis estimate without needing the side wall.
    center_base = np.median(
        points_base,
        axis=0,
    )

    centered = (
        points_base
        - center_base.reshape(
            1,
            3,
        )
    )

    axial_component = (
        centered
        @ axis_direction_base
    )

    radial_vectors = (
        centered
        - axial_component.reshape(
            -1,
            1,
        )
        * axis_direction_base.reshape(
            1,
            3,
        )
    )

    radial_distance = np.linalg.norm(
        radial_vectors,
        axis=1,
    )

    observed_radius_m = float(
        np.percentile(
            radial_distance,
            95.0,
        )
    )

    radius_ratio = (
        observed_radius_m
        / expected_radius_m
    )

    rejection_reasons: list[str] = []

    if (
        peg_depth_mad_m
        > max_top_depth_mad_m
    ):
        rejection_reasons.append(
            "top_face_depth_too_noisy"
        )

    if not (
        min_radius_ratio
        <= radius_ratio
        <= max_radius_ratio
    ):
        rejection_reasons.append(
            "implausible_top_face_radius"
        )

    accepted = (
        len(
            rejection_reasons
        )
        == 0
    )

    return {
        "accepted": accepted,
        "rejection_reasons": tuple(
            rejection_reasons
        ),
        "axis_xy_base": (
            float(
                center_base[0]
            ),
            float(
                center_base[1]
            ),
        ),
        "axis_point_base": (
            float(
                center_base[0]
            ),
            float(
                center_base[1]
            ),
            float(
                center_base[2]
            ),
        ),
        "axis_direction_base": tuple(
            float(
                value
            )
            for value
            in axis_direction_base
        ),
        "peg_tip_point_base": (
            float(
                center_base[0]
            ),
            float(
                center_base[1]
            ),
            float(
                center_base[2]
            ),
        ),
        "center_pixel": (
            float(
                selected_candidate[
                    "center_pixel"
                ][0]
            ),
            float(
                selected_candidate[
                    "center_pixel"
                ][1]
            ),
        ),
        "expected_distance_px": float(
            selected_candidate[
                "expected_distance_px"
            ]
        ),
        "component_area_px": int(
            selected_candidate[
                "area_px"
            ]
        ),
        "depth_split_m": float(
            split_depth_m
        ),
        "depth_gap_m": float(
            depth_gap_m
        ),
        "near_depth_max_m": float(
            near_depth_max
        ),
        "far_depth_min_m": float(
            far_depth_min
        ),
        "top_depth_median_m": float(
            peg_depth_median
        ),
        "top_depth_mad_m": float(
            peg_depth_mad_m
        ),
        "observed_radius_m": float(
            observed_radius_m
        ),
        "expected_radius_m": float(
            expected_radius_m
        ),
        "radius_ratio": float(
            radius_ratio
        ),
        "aperture_mask": aperture,
        "near_depth_mask": (
            near_mask_u8 > 0
        ),
        "mask": peg_mask,
    }


def load_perceiver(
    config_path: Path,
) -> ScenePerceiver:
    with config_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        config = yaml.safe_load(
            stream
        )

    scene_cfg = config[
        "scene_perceiver"
    ]

    visual_cfg = config[
        "visual_prompter"
    ]

    crop_cfg = (
        config
        .get(
            "grounding_dino",
            {},
        )
        .get(
            "crop",
            {},
        )
    )

    perceiver = ScenePerceiver(
        camera_calibration_path=(
            scene_cfg[
                "camera_calibration_path"
            ]
        ),
        camera_name=(
            scene_cfg.get(
                "camera_name",
                "zed_front",
            )
        ),
        grounding_config=(
            visual_cfg[
                "grounding_config"
            ]
        ),
        grounding_checkpoint=(
            visual_cfg[
                "grounding_checkpoint"
            ]
        ),
        bert_model=(
            visual_cfg[
                "bert_model"
            ]
        ),
        sam_checkpoint=(
            visual_cfg[
                "sam_checkpoint"
            ]
        ),
        # These labels are irrelevant to the target-specific
        # detector below, but ScenePerceiver requires a non-empty list.
        detector_labels=(
            "gray ring",
            "wooden peg",
        ),
        perception_mode="generalized",
        grounding_dino_crop=(
            crop_cfg
        ),
    )

    # Fail immediately if the local code is not the version
    # containing the new pre-insertion functionality.
    detect_params = inspect.signature(
        perceiver.detect_post_grasp_target_mask
    ).parameters

    for required in (
        "expected_pixel",
        "max_expected_distance_px",
    ):
        if required not in detect_params:
            raise RuntimeError(
                "Local scene_perceiver.py does not contain "
                f"the new {required!r} target-selection parameter."
            )

    peg_params = inspect.signature(
        perceiver.refine_peg_axis
    ).parameters

    for required in (
        "T_base_camera",
        "T_base_table",
    ):
        if required not in peg_params:
            raise RuntimeError(
                "Local scene_perceiver.py does not contain "
                f"the new live-transform parameter {required!r}."
            )

    if not hasattr(
        perceiver,
        "refine_pre_insert_peg_top",
    ):
        raise RuntimeError(
            "Local scene_perceiver.py does not contain "
            "refine_pre_insert_peg_top()."
        )

    hole_params = inspect.signature(
        perceiver.refine_post_grasp_nut_hole
    ).parameters

    if (
        "allow_deep_hole_background"
        not in hole_params
    ):
        raise RuntimeError(
            "Local scene_perceiver.py does not contain "
            "allow_deep_hole_background."
        )

    return perceiver


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Offline replay of the Nut-Assembly pre-insert "
            "eye-in-hand refinement."
        )
    )

    parser.add_argument(
        "--run",
        type=Path,
        default=Path(
            "/test_real/test_task00_6"
        ),
    )

    parser.add_argument(
        "--step",
        type=int,
        default=60,
        help=(
            "Rollout step captured immediately after ALIGNING. "
            "For test_task00_6 this is step 60."
        ),
    )

    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
    )

    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
    )

    args = parser.parse_args()

    run_root = (
        args.run
        .expanduser()
        .resolve()
    )

    config_path = (
        args.config
        .expanduser()
        .resolve()
    )

    output_dir = (
        args.output_dir
        if args.output_dir is not None
        else run_root
        / "pre_insert_offline_replay"
    )

    output_dir = (
        output_dir
        .expanduser()
        .resolve()
    )

    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    pkl_path = (
        run_root
        / "rollouts"
        / "seedo_controller"
        / "nut_assembly"
        / "task_00"
        / "traj_000.pkl"
    )

    if not pkl_path.is_file():
        raise FileNotFoundError(
            f"Rollout not found: {pkl_path}"
        )

    post_path = find_unique(
        run_root,
        "post_grasp_raw_input.npz",
    )

    pick_path = find_unique(
        run_root,
        "nut_pick_refinements.json",
    )

    place_path = find_unique(
        run_root,
        "nut_place_refinements.json",
    )

    debug_path = find_unique(
        run_root,
        "nut_visual_debug_points.json",
    )

    print(
        "============================================================"
    )
    print(
        "OFFLINE PRE-INSERT REPLAY"
    )
    print(
        "============================================================"
    )
    print(
        "run:",
        run_root,
    )
    print(
        "rollout:",
        pkl_path,
    )
    print(
        "step:",
        args.step,
    )
    print(
        "post-grasp:",
        post_path,
    )
    print(
        "visual debug:",
        debug_path,
    )

    # -------------------------------------------------------------
    # 1. Extract exactly one rollout frame, then release the
    #    ~250 MiB trajectory before loading vision models.
    # -------------------------------------------------------------

    payload = load_pickle(
        pkl_path
    )

    trajectory = payload[
        "traj"
    ]

    if not (
        0
        <= args.step
        < len(
            trajectory._data
        )
    ):
        raise IndexError(
            f"Step {args.step} outside trajectory "
            f"length {len(trajectory._data)}."
        )

    rollout_step = trajectory.get(
        args.step,
        decompress=True,
    )

    obs = rollout_step[
        "obs"
    ]

    status = (
        rollout_step
        .get(
            "info",
            {},
        )
        .get(
            "status"
        )
    )

    rgb_bgr = np.asarray(
        obs[
            "eye_in_hand_image"
        ],
        dtype=np.uint8,
    ).copy()

    # Runtime get_seedo_grasp_runtime_input() uses rgb8.
    # The saved rollout image is OpenCV/BGR, therefore convert it.
    rgb = cv2.cvtColor(
        rgb_bgr,
        cv2.COLOR_BGR2RGB,
    )

    depth = np.asarray(
        obs[
            "eye_in_hand_depth"
        ],
        dtype=np.float32,
    ).copy()

    current_tcp_position = np.asarray(
        obs[
            "eef_pos"
        ],
        dtype=np.float64,
    ).copy()

    current_tcp_orientation = np.asarray(
        obs[
            "eef_quat"
        ],
        dtype=np.float64,
    ).copy()

    print(
        "rollout status:",
        status,
    )
    print(
        "TCP position:",
        current_tcp_position,
    )
    print(
        "TCP quaternion [x,y,z,w]:",
        current_tcp_orientation,
    )

    del payload
    del trajectory
    del rollout_step
    gc.collect()

    # -------------------------------------------------------------
    # 2. Load saved geometry and reconstruct the live camera pose.
    # -------------------------------------------------------------

    post = np.load(
        post_path
    )

    camera_matrix = np.asarray(
        post[
            "camera_matrix"
        ],
        dtype=np.float64,
    )

    T_base_camera_post = np.asarray(
        post[
            "T_base_camera"
        ],
        dtype=np.float64,
    )

    T_base_table = np.asarray(
        post[
            "T_base_table"
        ],
        dtype=np.float64,
    )

    T_base_tcp_post = pose_matrix(
        post[
            "measured_tcp_position"
        ],
        post[
            "measured_tcp_orientation"
        ],
    )

    # Fixed rigid transform measured during the real run.
    T_tcp_camera = (
        np.linalg.inv(
            T_base_tcp_post
        )
        @ T_base_camera_post
    )

    T_base_tcp_current = pose_matrix(
        current_tcp_position,
        current_tcp_orientation,
    )

    T_base_camera = (
        T_base_tcp_current
        @ T_tcp_camera
    )

    print()
    print(
        "Recovered T_tcp_camera translation [m]:",
        T_tcp_camera[
            :3,
            3,
        ],
    )
    print(
        "Reconstructed current camera position [m]:",
        T_base_camera[
            :3,
            3,
        ],
    )

    with pick_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        pick_refinements = json.load(
            stream
        )

    with place_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        place_refinements = json.load(
            stream
        )

    with debug_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        visual_debug = json.load(
            stream
        )

    pick_target = str(
        visual_debug[
            "pick_target"
        ]
    )

    place_target = str(
        visual_debug[
            "place_target"
        ]
    )

    pick_refinement = (
        pick_refinements[
            pick_target
        ]
    )

    old_place_refinement = (
        place_refinements[
            place_target
        ]
    )

    known_thickness_m = float(
        pick_refinement[
            "nut_thickness_estimate_m"
        ]
    )

    # ---------------------------------------------------------
    # Reconstruct the SAME grasp-relative hole geometry used by
    # MotionLayer.predict_nut_hole_geometry_at_tcp().
    #
    # The predicted final pose in nut_visual_debug was generated
    # from the current hole<->TCP geometry after the post-grasp
    # update/fallback. Recover that local geometry from the saved
    # final TCP pose, then apply it to the measured TCP at step 60.
    # ---------------------------------------------------------

    predicted_final = visual_debug[
        "predicted_final_assembly_object_pose"
    ]

    T_base_tcp_reference = pose_matrix(
        np.asarray(
            predicted_final[
                "tcp_position_base"
            ],
            dtype=np.float64,
        ),
        np.asarray(
            predicted_final[
                "tcp_orientation_base"
            ],
            dtype=np.float64,
        ),
    )

    reference_hole_center_base = np.asarray(
        predicted_final[
            "hole_top_base"
        ],
        dtype=np.float64,
    )

    reference_hole_bottom_base = np.asarray(
        predicted_final[
            "hole_bottom_base"
        ],
        dtype=np.float64,
    )

    reference_hole_axis_base = np.asarray(
        predicted_final[
            "hole_axis_base"
        ],
        dtype=np.float64,
    )

    T_tcp_reference_base = np.linalg.inv(
        T_base_tcp_reference
    )

    hole_center_local = (
        T_tcp_reference_base
        @ np.array(
            [
                reference_hole_center_base[0],
                reference_hole_center_base[1],
                reference_hole_center_base[2],
                1.0,
            ],
            dtype=np.float64,
        )
    )[:3]

    hole_bottom_local = (
        T_tcp_reference_base
        @ np.array(
            [
                reference_hole_bottom_base[0],
                reference_hole_bottom_base[1],
                reference_hole_bottom_base[2],
                1.0,
            ],
            dtype=np.float64,
        )
    )[:3]

    hole_axis_local = (
        T_base_tcp_reference[
            :3,
            :3,
        ].T
        @ reference_hole_axis_base
    )

    expected_hole_center_base = (
        T_base_tcp_current
        @ np.array(
            [
                hole_center_local[0],
                hole_center_local[1],
                hole_center_local[2],
                1.0,
            ],
            dtype=np.float64,
        )
    )[:3]

    expected_hole_bottom_base = (
        T_base_tcp_current
        @ np.array(
            [
                hole_bottom_local[0],
                hole_bottom_local[1],
                hole_bottom_local[2],
                1.0,
            ],
            dtype=np.float64,
        )
    )[:3]

    expected_hole_axis_base = (
        T_base_tcp_current[
            :3,
            :3,
        ]
        @ hole_axis_local
    )

    expected_hole_axis_base /= (
        np.linalg.norm(
            expected_hole_axis_base
        )
    )

    expected_hole_pixel = (
        project_base_point(
            expected_hole_center_base,
            camera_matrix,
            T_base_camera,
        )
    )

    print()
    print(
        "Rigid-model expected hole pixel:",
        expected_hole_pixel,
    )

    if (
        "axis_point_base"
        in old_place_refinement
    ):
        expected_peg_point_base = np.asarray(
            old_place_refinement[
                "axis_point_base"
            ],
            dtype=np.float64,
        )

    else:
        expected_peg_point_base = 0.5 * (
            np.asarray(
                old_place_refinement[
                    "peg_tip_point_base"
                ],
                dtype=np.float64,
            )
            + np.asarray(
                old_place_refinement[
                    "peg_support_point_base"
                ],
                dtype=np.float64,
            )
        )

    expected_peg_pixel = (
        project_base_point(
            expected_peg_point_base,
            camera_matrix,
            T_base_camera,
        )
    )

    old_tip_pixel = (
        project_base_point(
            np.asarray(
                old_place_refinement[
                    "peg_tip_point_base"
                ],
                dtype=np.float64,
            ),
            camera_matrix,
            T_base_camera,
        )
    )

    print()
    print(
        "Old peg expected pixel:",
        expected_peg_pixel,
    )
    print(
        "Old peg tip pixel:",
        old_tip_pixel,
    )

    # -------------------------------------------------------------
    # 3. Load GroundingDINO + SAM lazily and rerun the fresh
    #    pre-insert observation exactly on the saved RGB-D frame.
    # -------------------------------------------------------------

    perceiver = load_perceiver(
        config_path
    )

    ring_artifacts_dir = (
        output_dir
        / "ring"
    )

    peg_artifacts_dir = (
        output_dir
        / "peg"
    )

    ring_artifacts_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    peg_artifacts_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    print()
    print(
        "============================================================"
    )
    print(
        "FRESH RING / HOLE REFINEMENT"
    )
    print(
        "============================================================"
    )

    ring_detection = None
    hole_refinement = None
    ring_mask = np.zeros(
        depth.shape,
        dtype=bool,
    )

    try:
        ring_detection = (
            perceiver
            .detect_post_grasp_target_mask(
                rgb_image=rgb,
                target_label=pick_target,
                artifacts_dir=(
                    ring_artifacts_dir
                ),
            )
        )

        ring_mask = np.asarray(
            ring_detection[
                "mask"
            ],
            dtype=bool,
        )

        save_mask(
            ring_mask,
            output_dir
            / "ring_mask.png",
        )

        hole_refinement = (
            perceiver
            .refine_post_grasp_nut_hole(
                mask=ring_mask,
                depth_image=depth,
                camera_matrix=(
                    camera_matrix
                ),
                T_base_camera=(
                    T_base_camera
                ),
                T_base_table=(
                    T_base_table
                ),
                expected_hole_center_base=(
                    expected_hole_center_base
                ),
                expected_hole_axis_base=(
                    expected_hole_axis_base
                ),
                known_thickness_m=(
                    known_thickness_m
                ),
                allow_deep_hole_background=True,

                # Pre-insert observation is intentionally allowed to
                # disagree more strongly with the rigid grasp model than
                # the immediate post-grasp refinement. This is diagnostic:
                # the final motion correction is gated separately below.
                max_lateral_correction_m=0.050,

                artifacts_dir=(
                    ring_artifacts_dir
                ),
            )
        )

        print(
            "hole accepted:",
            hole_refinement[
                "accepted"
            ],
        )
        print(
            "hole rejection reasons:",
            hole_refinement.get(
                "rejection_reasons"
            ),
        )
        print(
            "hole center pixel:",
            hole_refinement.get(
                "hole_center_px"
            ),
        )
        print(
            "hole correction from rigid expectation [mm]:",
            1000.0
            * float(
                hole_refinement.get(
                    "lateral_correction_m",
                    float("nan"),
                )
            ),
        )

    except RuntimeError as exc:
        print(
            "[OFFLINE][WARN] Fresh hole refinement failed:",
            exc,
        )

    print()
    print()
    print(
        "============================================================"
    )
    print(
        "FRESH PEG TOP-FACE REFINEMENT"
    )
    print(
        "============================================================"
    )

    peg_detection = None
    peg_refinement = None
    peg_mask = np.zeros(
        depth.shape,
        dtype=bool,
    )
    peg_aperture_mask = np.zeros(
        depth.shape,
        dtype=bool,
    )
    peg_near_depth_mask = np.zeros(
        depth.shape,
        dtype=bool,
    )

    old_peg_xy = np.asarray(
        old_place_refinement[
            "axis_xy_base"
        ],
        dtype=np.float64,
    )

    peg_xy_shift_m = None
    peg_consistency_ok = False

    try:
        if (
            hole_refinement is None
            or hole_refinement.get(
                "accepted"
            ) is not True
        ):
            raise RuntimeError(
                "Fresh peg-top refinement requires an accepted "
                "fresh hole refinement."
            )

        peg_refinement = (
            perceiver
            .refine_pre_insert_peg_top(
                hole_refinement=(
                    hole_refinement
                ),
                depth_image=depth,
                camera_matrix=(
                    camera_matrix
                ),
                T_base_camera=(
                    T_base_camera
                ),
                T_base_table=(
                    T_base_table
                ),
                expected_peg_pixel=(
                    expected_peg_pixel
                ),
                expected_axis_direction_base=np.asarray(
                    old_place_refinement[
                        "axis_direction_base"
                    ],
                    dtype=np.float64,
                ),
                expected_radius_m=float(
                    old_place_refinement.get(
                        "peg_radius_m",
                        0.015,
                    )
                ),
                artifacts_dir=(
                    output_dir
                ),
            )
        )

        peg_mask = np.asarray(
            peg_refinement[
                "mask"
            ],
            dtype=bool,
        )
        peg_aperture_mask = np.asarray(
            peg_refinement[
                "aperture_mask"
            ],
            dtype=bool,
        )
        peg_near_depth_mask = np.asarray(
            peg_refinement[
                "near_depth_mask"
            ],
            dtype=bool,
        )

        print(
            "peg accepted:",
            peg_refinement[
                "accepted"
            ],
        )
        print(
            "peg rejection reasons:",
            peg_refinement.get(
                "rejection_reasons"
            ),
        )
        print(
            "fresh peg top pixel:",
            peg_refinement.get(
                "center_pixel"
            ),
        )
        print(
            "fresh peg axis XY:",
            peg_refinement.get(
                "axis_xy_base"
            ),
        )
        print(
            "peg/background depth gap:",
            f"{peg_refinement['depth_gap_m'] * 1000.0:.2f} mm",
        )
        print(
            "peg top depth MAD:",
            f"{peg_refinement['top_depth_mad_m'] * 1000.0:.2f} mm",
        )
        print(
            "observed peg radius:",
            f"{peg_refinement['observed_radius_m'] * 1000.0:.2f} mm",
            "| expected:",
            f"{peg_refinement['expected_radius_m'] * 1000.0:.2f} mm",
        )

        new_peg_xy = np.asarray(
            peg_refinement[
                "axis_xy_base"
            ],
            dtype=np.float64,
        )

        peg_xy_shift_m = float(
            np.linalg.norm(
                new_peg_xy
                - old_peg_xy
            )
        )

        peg_consistency_ok = (
            bool(
                peg_refinement[
                    "accepted"
                ]
            )
            and peg_xy_shift_m
            <= 0.030
        )

        print(
            "old -> fresh peg XY shift:",
            f"{peg_xy_shift_m * 1000.0:.2f} mm",
        )
        print(
            "peg consistency gate <=30 mm:",
            peg_consistency_ok,
        )

    except RuntimeError as exc:
        print(
            "[OFFLINE][WARN] Fresh peg-top refinement failed:",
            exc,
        )


    # -------------------------------------------------------------
    # 4. Reproduce the controller fallback policy.
    # -------------------------------------------------------------

    hole_updated = bool(
        hole_refinement is not None
        and hole_refinement.get(
            "accepted"
        ) is True
    )

    peg_updated = (
        peg_consistency_ok
    )

    if hole_updated:
        current_hole_bottom = np.asarray(
            hole_refinement[
                "hole_bottom_center_base"
            ],
            dtype=np.float64,
        )

        current_hole_axis = np.asarray(
            hole_refinement[
                "hole_axis_base"
            ],
            dtype=np.float64,
        )

    else:
        current_hole_bottom = (
            expected_hole_bottom_base.copy()
        )

        current_hole_axis = (
            expected_hole_axis_base.copy()
        )

    current_hole_axis /= (
        np.linalg.norm(
            current_hole_axis
        )
    )

    if peg_updated:
        peg_axis = np.asarray(
            peg_refinement[
                "axis_direction_base"
            ],
            dtype=np.float64,
        )

        peg_tip = np.asarray(
            peg_refinement[
                "peg_tip_point_base"
            ],
            dtype=np.float64,
        )

    else:
        peg_axis = np.asarray(
            old_place_refinement[
                "axis_direction_base"
            ],
            dtype=np.float64,
        )

        peg_tip = np.asarray(
            old_place_refinement[
                "peg_tip_point_base"
            ],
            dtype=np.float64,
        )

    peg_axis /= np.linalg.norm(
        peg_axis
    )

    axis_alignment = float(
        np.dot(
            current_hole_axis,
            peg_axis,
        )
    )

    axis_coordinate = float(
        np.dot(
            current_hole_bottom
            - peg_tip,
            peg_axis,
        )
    )

    closest_peg_axis_point = (
        peg_tip
        + axis_coordinate
        * peg_axis
    )

    lateral_correction = (
        closest_peg_axis_point
        - current_hole_bottom
    )

    lateral_correction = (
        lateral_correction
        - np.dot(
            lateral_correction,
            peg_axis,
        )
        * peg_axis
    )

    lateral_correction_m = float(
        np.linalg.norm(
            lateral_correction
        )
    )

    axis_gate_ok = (
        axis_alignment
        >= 0.999
    )

    correction_gate_ok = (
        lateral_correction_m
        <= 0.040
    )

    print()
    print(
        "============================================================"
    )
    print(
        "PROPOSED PRE-INSERT V2 RESULT"
    )
    print(
        "============================================================"
    )
    print(
        "hole_updated:",
        hole_updated,
    )
    print(
        "peg_updated:",
        peg_updated,
    )
    print(
        "axis alignment dot:",
        f"{axis_alignment:.6f}",
        "| gate >=0.999:",
        axis_gate_ok,
    )
    print(
        "lateral correction [mm]:",
        1000.0
        * lateral_correction,
    )
    print(
        "lateral correction norm:",
        f"{lateral_correction_m * 1000.0:.2f} mm",
    )
    print(
        "40 mm safety gate:",
        correction_gate_ok,
    )

    if not axis_gate_ok:
        print(
            "[RESULT] Current controller would STOP before "
            "insertion because the axes are not aligned."
        )

    elif not correction_gate_ok:
        print(
            "[RESULT] Current 40 mm motion gate would STOP before "
            "insertion because the lateral correction exceeds 40 mm."
        )

    else:
        print(
            "[RESULT] Current 40 mm motion gate would allow this "
            "fresh relative correction before PRE-ENTRY."
        )

    # -------------------------------------------------------------
    # 5. Visual overlay.
    # -------------------------------------------------------------

    overlay = rgb_bgr.copy()

    # Transparent mask overlays.
    ring_layer = overlay.copy()
    ring_layer[
        ring_mask
    ] = (
        255,
        180,
        0,
    )

    overlay = cv2.addWeighted(
        overlay,
        0.78,
        ring_layer,
        0.22,
        0.0,
    )

    peg_layer = overlay.copy()
    peg_layer[
        peg_mask
    ] = (
        0,
        255,
        255,
    )

    overlay = cv2.addWeighted(
        overlay,
        0.82,
        peg_layer,
        0.18,
        0.0,
    )

    expected_hole_bottom_px = (
        project_base_point(
            expected_hole_bottom_base,
            camera_matrix,
            T_base_camera,
        )
    )

    current_hole_bottom_px = (
        project_base_point(
            current_hole_bottom,
            camera_matrix,
            T_base_camera,
        )
    )

    target_axis_px = (
        project_base_point(
            closest_peg_axis_point,
            camera_matrix,
            T_base_camera,
        )
    )

    draw_point(
        overlay,
        expected_peg_pixel,
        (
            0,
            0,
            255,
        ),
        "old peg",
        radius=6,
    )

    if (
        peg_refinement is not None
        and "axis_point_base"
        in peg_refinement
    ):
        fresh_peg_axis_point = np.asarray(
            peg_refinement[
                "axis_point_base"
            ],
            dtype=np.float64,
        )

        fresh_peg_axis_px = (
            project_base_point(
                fresh_peg_axis_point,
                camera_matrix,
                T_base_camera,
            )
        )

        draw_point(
            overlay,
            fresh_peg_axis_px,
            (
                0,
                255,
                255,
            ),
            "fresh peg axis",
            radius=7,
        )

    draw_point(
        overlay,
        current_hole_bottom_px,
        (
            255,
            255,
            0,
        ),
        "hole",
        radius=7,
    )

    draw_point(
        overlay,
        target_axis_px,
        (
            0,
            255,
            0,
        ),
        "corrected target",
        radius=7,
    )

    hole_u = int(
        round(
            current_hole_bottom_px[0]
        )
    )

    hole_v = int(
        round(
            current_hole_bottom_px[1]
        )
    )

    target_u = int(
        round(
            target_axis_px[0]
        )
    )

    target_v = int(
        round(
            target_axis_px[1]
        )
    )

    cv2.arrowedLine(
        overlay,
        (
            hole_u,
            hole_v,
        ),
        (
            target_u,
            target_v,
        ),
        (
            0,
            255,
            0,
        ),
        thickness=3,
        tipLength=0.18,
        line_type=cv2.LINE_AA,
    )

    text_lines = [
        (
            f"hole fresh={hole_updated} | "
            f"peg fresh={peg_updated}"
        ),
        (
            "dXYZ = "
            f"[{lateral_correction[0] * 1000.0:+.1f}, "
            f"{lateral_correction[1] * 1000.0:+.1f}, "
            f"{lateral_correction[2] * 1000.0:+.1f}] mm"
        ),
        (
            f"norm={lateral_correction_m * 1000.0:.1f} mm | "
            f"axis dot={axis_alignment:.5f}"
        ),
        (
            f"axis gate={'PASS' if axis_gate_ok else 'FAIL'} | "
            f"40mm gate={'PASS' if correction_gate_ok else 'FAIL'}"
        ),
    ]

    y = 24

    for line in text_lines:
        cv2.putText(
            overlay,
            line,
            (
                12,
                y,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (
                255,
                255,
                255,
            ),
            3,
            cv2.LINE_AA,
        )

        cv2.putText(
            overlay,
            line,
            (
                12,
                y,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (
                0,
                0,
                0,
            ),
            1,
            cv2.LINE_AA,
        )

        y += 22

    overlay_path = (
        output_dir
        / "pre_insert_alignment_overlay.png"
    )

    cv2.imwrite(
        str(
            overlay_path
        ),
        overlay,
    )

    cv2.imwrite(
        str(
            output_dir
            / "input_eye_in_hand.png"
        ),
        rgb_bgr,
    )

    result = {
        "run": run_root,
        "step": args.step,
        "rollout_status": status,
        "pick_target": pick_target,
        "place_target": place_target,
        "T_tcp_camera": T_tcp_camera,
        "T_base_camera_step": T_base_camera,
        "expected_hole_bottom_base": (
            expected_hole_bottom_base
        ),
        "expected_hole_center_base": (
            expected_hole_center_base
        ),
        "old_peg_expected_pixel": (
            expected_peg_pixel
        ),
        "old_peg_tip_pixel": (
            old_tip_pixel
        ),
        "ring_detection": (
            None
            if ring_detection is None
            else {
                key: value
                for key, value
                in ring_detection.items()
                if key != "mask"
            }
        ),
        "hole_refinement": (
            hole_refinement
        ),
        "peg_detection": (
            None
            if peg_detection is None
            else {
                key: value
                for key, value
                in peg_detection.items()
                if key != "mask"
            }
        ),
        "peg_refinement": (
            None
            if peg_refinement is None
            else {
                key: value
                for key, value
                in peg_refinement.items()
                if key not in {
                    "mask",
                    "aperture_mask",
                    "near_depth_mask",
                }
            }
        ),
        "old_to_fresh_peg_xy_shift_m": (
            peg_xy_shift_m
        ),
        "hole_updated": (
            hole_updated
        ),
        "peg_updated": (
            peg_updated
        ),
        "axis_alignment_dot": (
            axis_alignment
        ),
        "axis_gate_ok": (
            axis_gate_ok
        ),
        "lateral_correction_base_m": (
            lateral_correction
        ),
        "lateral_correction_norm_m": (
            lateral_correction_m
        ),
        "lateral_correction_gate_ok": (
            correction_gate_ok
        ),
        "overlay_path": (
            overlay_path
        ),
    }

    result_path = (
        output_dir
        / "pre_insert_offline_result.json"
    )

    with result_path.open(
        "w",
        encoding="utf-8",
    ) as stream:
        json.dump(
            to_jsonable(
                result
            ),
            stream,
            indent=2,
            ensure_ascii=False,
        )

    print()
    print(
        "Overlay:",
        overlay_path,
    )
    print(
        "Result JSON:",
        result_path,
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(
        main()
    )
