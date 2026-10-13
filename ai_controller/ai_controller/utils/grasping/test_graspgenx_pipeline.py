#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import torch
from PIL import Image

# =====================================================================
# Paths
# =====================================================================

REPO_ROOT = Path(
    "/home/ros2_ws/src/UR5e-2f-85"
)

SEEDO_DIR = (
    REPO_ROOT
    / "ai_controller"
    / "ai_controller"
    / "models"
    / "seedo_controller"
)

if str(SEEDO_DIR) not in sys.path:
    sys.path.insert(
        0,
        str(SEEDO_DIR),
    )

from inspect_pkl import (  # noqa: E402
    load_pickle,
    get_trajectory_step,
)

from segment_anything import (  # noqa: E402
    SamPredictor,
    build_sam,
)

from ai_controller.utils.grasping.grasp_planner_factory import (  # noqa: E402
    build_grasp_planner,
)


OUTPUT_DIR = (
    REPO_ROOT
    / ".runtime"
    / "graspgenx_pipeline_test"
)


DEFAULT_SAM_CHECKPOINT = (
    "/opt/checkpoints/seedo/sam/"
    "sam_vit_h_4b8939.pth"
)


# =====================================================================
# Eye-in-hand intrinsics
#
# Same calibration currently used by the existing grasp pipeline test.
# =====================================================================

DEFAULT_K = np.array(
    [
        [
            363.8071594238281,
            0.0,
            335.86553955078125,
        ],
        [
            0.0,
            363.8071594238281,
            183.81210327148438,
        ],
        [
            0.0,
            0.0,
            1.0,
        ],
    ],
    dtype=np.float64,
)


# =====================================================================
# Rollout RGB-D
# =====================================================================

def get_gripper_rgbd(
    obs: dict,
) -> tuple[
    np.ndarray,
    np.ndarray,
    str,
    str,
]:
    """
    Support both rollout formats.

    SeeDo rollout:
        eye_in_hand_image
        eye_in_hand_depth

    Other rollout format:
        camera_gripper_image
        camera_gripper_depth
    """

    candidates = [
        (
            "eye_in_hand_image",
            "eye_in_hand_depth",
        ),
        (
            "camera_gripper_image",
            "camera_gripper_depth",
        ),
    ]

    for image_key, depth_key in candidates:

        if (
            image_key in obs
            and depth_key in obs
        ):
            image = np.asarray(
                obs[image_key]
            )

            depth = np.asarray(
                obs[depth_key],
                dtype=np.float32,
            )

            return (
                image,
                depth,
                image_key,
                depth_key,
            )

    raise RuntimeError(
        "No supported gripper RGB-D pair found. "
        "Expected either "
        "'eye_in_hand_image' + 'eye_in_hand_depth' "
        "or "
        "'camera_gripper_image' + 'camera_gripper_depth'."
    )


# =====================================================================
# SAM
# =====================================================================

def build_sam_predictor(
    checkpoint_path: str,
) -> SamPredictor:

    device = torch.device(
        "cuda:0"
        if torch.cuda.is_available()
        else "cpu"
    )

    print(
        "[TEST] SAM device:",
        device,
    )

    sam = build_sam(
        checkpoint=checkpoint_path,
    )

    sam.to(
        device=device
    )

    return SamPredictor(
        sam
    )


def select_best_mask(
    predictor: SamPredictor,
    image_rgb: np.ndarray,
    point_xy: np.ndarray,
) -> tuple[
    np.ndarray,
    float,
    np.ndarray,
    np.ndarray,
    int,
]:

    predictor.set_image(
        image_rgb
    )

    input_point = np.asarray(
        [
            [
                float(point_xy[0]),
                float(point_xy[1]),
            ]
        ],
        dtype=np.float32,
    )

    input_label = np.asarray(
        [1],
        dtype=np.int32,
    )

    masks, scores, logits = (
        predictor.predict(
            point_coords=input_point,
            point_labels=input_label,
            multimask_output=True,
        )
    )

    best_idx = int(
        np.argmax(
            scores
        )
    )

    best_mask = (
        masks[
            best_idx
        ]
        .astype(bool)
    )

    return (
        best_mask,
        float(
            scores[
                best_idx
            ]
        ),
        masks,
        scores,
        best_idx,
    )


# =====================================================================
# Target point cloud
# =====================================================================

def mask_to_camera_point_cloud(
    mask: np.ndarray,
    depth: np.ndarray,
    K: np.ndarray,
    min_depth: float = 0.15,
    max_depth: float = 0.60,
) -> np.ndarray:

    depth = np.asarray(
        depth,
        dtype=np.float32,
    )

    mask = np.asarray(
        mask,
        dtype=bool,
    )

    K = np.asarray(
        K,
        dtype=np.float64,
    )

    if mask.shape != depth.shape:
        raise ValueError(
            "SAM mask and depth image have different shapes: "
            f"{mask.shape} vs {depth.shape}"
        )

    valid = (
        mask
        & np.isfinite(
            depth
        )
        & (
            depth >= min_depth
        )
        & (
            depth <= max_depth
        )
    )

    v, u = np.where(
        valid
    )

    if len(u) == 0:
        raise RuntimeError(
            "SAM mask contains no valid depth points."
        )

    z = depth[
        v,
        u,
    ].astype(
        np.float32
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

    x = (
        (
            u.astype(
                np.float32
            )
            - cx
        )
        * z
        / fx
    )

    y = (
        (
            v.astype(
                np.float32
            )
            - cy
        )
        * z
        / fy
    )

    point_cloud = np.stack(
        [
            x,
            y,
            z,
        ],
        axis=1,
    ).astype(
        np.float32
    )

    return point_cloud


# =====================================================================
# Projection utilities
# =====================================================================

def project_camera_point(
    point_camera: np.ndarray,
    K: np.ndarray,
) -> tuple[int, int] | None:

    point_camera = np.asarray(
        point_camera,
        dtype=np.float64,
    )

    if point_camera.shape != (3,):
        return None

    if not np.isfinite(
        point_camera
    ).all():
        return None

    x, y, z = (
        point_camera
    )

    if z <= 0.0:
        return None

    u = (
        K[0, 0]
        * x
        / z
        + K[0, 2]
    )

    v = (
        K[1, 1]
        * y
        / z
        + K[1, 2]
    )

    if (
        not np.isfinite(u)
        or not np.isfinite(v)
    ):
        return None

    return (
        int(
            round(
                float(u)
            )
        ),
        int(
            round(
                float(v)
            )
        ),
    )


# =====================================================================
# Gripper visualization
# =====================================================================

def draw_gripper_projection(
    grasp: np.ndarray,
    K: np.ndarray,
    out_image: np.ndarray,
) -> None:
    """
    Project the GraspGen-X Robotiq 2F-85 grasp geometry.

    Official GraspGen-X convention:
        local +X = closing direction
        local +Z = direction from gripper base toward fingertips

    Official Robotiq 2F-85 parameters:
        fingertip depth = 0.136 m
        opening width   = 0.085 m
    """

    grasp = np.asarray(
        grasp,
        dtype=np.float64,
    )

    origin = grasp[:3, 3]
    x_axis = grasp[:3, 0]
    z_axis = grasp[:3, 2]

    fingertip_depth = 0.136
    half_opening = 0.085 / 2.0

    # ---------------------------------------------------------
    # Official grasp geometry
    # ---------------------------------------------------------

    fingertip_center = (
        origin
        + fingertip_depth
        * z_axis
    )

    left_tip = (
        fingertip_center
        - half_opening
        * x_axis
    )

    right_tip = (
        fingertip_center
        + half_opening
        * x_axis
    )

    # Mid finger points only for clearer visualization.
    mid_depth = (
        fingertip_depth
        * 0.60
    )

    left_mid = (
        origin
        + mid_depth
        * z_axis
        - half_opening
        * x_axis
    )

    right_mid = (
        origin
        + mid_depth
        * z_axis
        + half_opening
        * x_axis
    )

    def proj(point):
        return project_camera_point(
            point,
            K,
        )

    p_origin = proj(origin)
    p_center = proj(
        fingertip_center
    )
    p_left_tip = proj(
        left_tip
    )
    p_right_tip = proj(
        right_tip
    )
    p_left_mid = proj(
        left_mid
    )
    p_right_mid = proj(
        right_mid
    )

    required = [
        p_origin,
        p_center,
        p_left_tip,
        p_right_tip,
        p_left_mid,
        p_right_mid,
    ]

    if any(
        point is None
        for point in required
    ):
        print(
            "[TEST] Warning: gripper geometry "
            "cannot be fully projected."
        )
        return

    # ---------------------------------------------------------
    # Base -> fingertip center
    # ---------------------------------------------------------

    cv2.line(
        out_image,
        p_origin,
        p_center,
        (255, 255, 0),
        3,
    )

    # ---------------------------------------------------------
    # Left finger
    # ---------------------------------------------------------

    cv2.line(
        out_image,
        p_origin,
        p_left_mid,
        (255, 165, 0),
        3,
    )

    cv2.line(
        out_image,
        p_left_mid,
        p_left_tip,
        (255, 165, 0),
        5,
    )

    # ---------------------------------------------------------
    # Right finger
    # ---------------------------------------------------------

    cv2.line(
        out_image,
        p_origin,
        p_right_mid,
        (255, 165, 0),
        3,
    )

    cv2.line(
        out_image,
        p_right_mid,
        p_right_tip,
        (255, 165, 0),
        5,
    )

    # ---------------------------------------------------------
    # Opening line at fingertips
    # ---------------------------------------------------------

    cv2.line(
        out_image,
        p_left_tip,
        p_right_tip,
        (0, 255, 255),
        3,
    )

    # ---------------------------------------------------------
    # Key points
    # ---------------------------------------------------------

    # Gripper base.
    cv2.circle(
        out_image,
        p_origin,
        6,
        (255, 0, 0),
        -1,
    )

    # Fingertip center / actual grasp region.
    cv2.circle(
        out_image,
        p_center,
        7,
        (0, 255, 0),
        -1,
    )

    # Fingertips.
    cv2.circle(
        out_image,
        p_left_tip,
        5,
        (255, 165, 0),
        -1,
    )

    cv2.circle(
        out_image,
        p_right_tip,
        5,
        (255, 165, 0),
        -1,
    )


# =====================================================================
# Semantic + SAM overlay
# =====================================================================

def draw_semantic_sam_overlay(
    image_rgb: np.ndarray,
    mask: np.ndarray,
    semantic_point: np.ndarray,
    sam_score: float,
    instruction: str,
    out_path: Path,
) -> None:

    output = image_rgb.copy()

    # ---------------------------------------------------------
    # SAM mask
    # ---------------------------------------------------------

    mask_color = np.zeros_like(
        output
    )

    mask_color[
        ...,
        1,
    ] = 255

    output = np.where(
        mask[
            ...,
            None,
        ],
        (
            0.65
            * output
            + 0.35
            * mask_color
        ).astype(
            np.uint8
        ),
        output,
    )

    # ---------------------------------------------------------
    # Mask bounding box
    # ---------------------------------------------------------

    ys, xs = np.where(
        mask
    )

    if len(xs) > 0:

        u_min = int(
            xs.min()
        )

        u_max = int(
            xs.max()
        )

        v_min = int(
            ys.min()
        )

        v_max = int(
            ys.max()
        )

        cv2.rectangle(
            output,
            (
                u_min,
                v_min,
            ),
            (
                u_max,
                v_max,
            ),
            (
                255,
                0,
                0,
            ),
            2,
        )

    else:

        u_min = -1
        u_max = -1
        v_min = -1
        v_max = -1

    # ---------------------------------------------------------
    # GraspMolmo point
    # ---------------------------------------------------------

    u = int(
        round(
            float(
                semantic_point[0]
            )
        )
    )

    v = int(
        round(
            float(
                semantic_point[1]
            )
        )
    )

    cv2.circle(
        output,
        (
            u,
            v,
        ),
        7,
        (
            255,
            0,
            255,
        ),
        -1,
    )

    cv2.circle(
        output,
        (
            u,
            v,
        ),
        12,
        (
            255,
            255,
            255,
        ),
        2,
    )

    # ---------------------------------------------------------
    # Text
    # ---------------------------------------------------------

    text_lines = [
        "GraspGenX pre-test",
        f"SAM score: {sam_score:.4f}",
        f"Point: ({u}, {v})",
        (
            "Mask bbox: "
            f"({u_min}, {v_min}) - "
            f"({u_max}, {v_max})"
        ),
        f"Instruction: {instruction}",
    ]

    text_y = 30

    for line in text_lines:

        cv2.putText(
            output,
            line,
            (
                20,
                text_y,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (
                255,
                255,
                255,
            ),
            3,
            cv2.LINE_AA,
        )

        cv2.putText(
            output,
            line,
            (
                20,
                text_y,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (
                20,
                20,
                20,
            ),
            1,
            cv2.LINE_AA,
        )

        text_y += 28

    Image.fromarray(
        output
    ).save(
        out_path
    )


# =====================================================================
# GraspGen-X overlay
# =====================================================================

def draw_graspgenx_overlay(
    image_rgb: np.ndarray,
    mask: np.ndarray,
    semantic_point: np.ndarray,
    grasps: np.ndarray,
    confidences: np.ndarray,
    K: np.ndarray,
    out_path: Path,
    num_visualized: int = 10,
) -> None:

    output = image_rgb.copy()

    height, width = (
        output.shape[
            :2
        ]
    )

    # ---------------------------------------------------------
    # SAM mask
    # ---------------------------------------------------------

    mask_color = np.zeros_like(
        output
    )

    mask_color[
        ...,
        1,
    ] = 255

    output = np.where(
        mask[
            ...,
            None,
        ],
        (
            0.70
            * output
            + 0.30
            * mask_color
        ).astype(
            np.uint8
        ),
        output,
    )

    # ---------------------------------------------------------
    # GraspMolmo semantic point
    # ---------------------------------------------------------

    semantic_u = int(
        round(
            float(
                semantic_point[0]
            )
        )
    )

    semantic_v = int(
        round(
            float(
                semantic_point[1]
            )
        )
    )

    cv2.circle(
        output,
        (
            semantic_u,
            semantic_v,
        ),
        7,
        (
            255,
            0,
            255,
        ),
        -1,
    )

    cv2.circle(
        output,
        (
            semantic_u,
            semantic_v,
        ),
        12,
        (
            255,
            255,
            255,
        ),
        2,
    )

    # ---------------------------------------------------------
    # Rank candidates globally by confidence
    # ---------------------------------------------------------

    order = np.argsort(
        -confidences
    )

    order = order[
        :min(
            num_visualized,
            len(order),
        )
    ]

    for rank, index in enumerate(
        order
    ):

        grasp = grasps[
            index
        ]

        origin = grasp[
            :3,
            3,
        ]

        pixel_origin = (
            project_camera_point(
                origin,
                K,
            )
        )

        if pixel_origin is None:
            continue

        u0, v0 = (
            pixel_origin
        )

        # Do not draw candidate labels completely outside
        # the image.
        if (
            u0 < -100
            or u0 > width + 100
            or v0 < -100
            or v0 > height + 100
        ):
            continue

        # -----------------------------------------------------
        # Other top candidates
        # -----------------------------------------------------

        if rank > 0:

            cv2.circle(
                output,
                (
                    u0,
                    v0,
                ),
                4,
                (
                    0,
                    255,
                    255,
                ),
                -1,
            )

            cv2.putText(
                output,
                str(
                    rank + 1
                ),
                (
                    u0 + 5,
                    v0 - 5,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (
                    255,
                    255,
                    255,
                ),
                1,
                cv2.LINE_AA,
            )

            continue

        # -----------------------------------------------------
        # Best grasp: draw approximate complete gripper
        # -----------------------------------------------------

        draw_gripper_projection(
            grasp=grasp,
            K=K,
            out_image=output,
        )

        cv2.putText(
            output,
            (
                "BEST "
                f"{float(confidences[index]):.3f}"
            ),
            (
                u0 + 10,
                v0 + 20,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (
                255,
                255,
                255,
            ),
            2,
            cv2.LINE_AA,
        )

    Image.fromarray(
        output
    ).save(
        out_path
    )


# =====================================================================
# Main
# =====================================================================

def main() -> None:

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--pkl",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--step",
        type=int,
        default=19,
    )

    parser.add_argument(
        "--instruction",
        type=str,
        required=True,
    )

    parser.add_argument(
        "--sam-checkpoint",
        type=str,
        default=DEFAULT_SAM_CHECKPOINT,
    )

    parser.add_argument(
        "--depth-min",
        type=float,
        default=0.15,
    )

    parser.add_argument(
        "--depth-max",
        type=float,
        default=0.60,
    )

    parser.add_argument(
        "--num-visualized",
        type=int,
        default=10,
    )

    args = parser.parse_args()

    # ---------------------------------------------------------
    # Output directory
    # ---------------------------------------------------------

    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    print(
        "=" * 78
    )

    print(
        "GRASPGENX PIPELINE TEST"
    )

    print(
        "=" * 78
    )

    print(
        "[TEST] PKL:",
        args.pkl,
    )

    print(
        "[TEST] Step:",
        args.step,
    )

    print(
        "[TEST] Instruction:",
        args.instruction,
    )

    # =============================================================
    # Load rollout
    # =============================================================

    data = load_pickle(
        args.pkl
    )

    trajectory = data[
        "traj"
    ]

    print(
        "[TEST] Trajectory length:",
        len(
            trajectory
        ),
    )

    step = get_trajectory_step(
        trajectory,
        args.step,
    )

    obs = step[
        "obs"
    ]

    # =============================================================
    # Extract eye-in-hand RGB-D
    # =============================================================

    (
        image_bgr,
        depth,
        rgb_key,
        depth_key,
    ) = get_gripper_rgbd(
        obs
    )

    if (
        image_bgr.ndim != 3
        or image_bgr.shape[
            2
        ] != 3
    ):
        raise RuntimeError(
            "Invalid RGB image shape: "
            f"{image_bgr.shape}"
        )

    if depth.ndim != 2:
        raise RuntimeError(
            "Invalid depth image shape: "
            f"{depth.shape}"
        )

    # Dataset camera images are stored as OpenCV/BGR.
    image_rgb = cv2.cvtColor(
        image_bgr,
        cv2.COLOR_BGR2RGB,
    )

    print(
        "[TEST] RGB key:",
        rgb_key,
    )

    print(
        "[TEST] Depth key:",
        depth_key,
    )

    print(
        "[TEST] RGB shape:",
        image_rgb.shape,
    )

    print(
        "[TEST] Depth shape:",
        depth.shape,
    )

    # Save original inputs.
    Image.fromarray(
        image_rgb
    ).save(
        OUTPUT_DIR
        / "input_rgb.png"
    )

    np.save(
        OUTPUT_DIR
        / "input_depth.npy",
        depth,
    )

    # =============================================================
    # Build GraspGen-X planner abstraction
    # =============================================================

    planner = build_grasp_planner(
        {
            "backend": "graspgenx",
        }
    )

    # =============================================================
    # GraspMolmo
    # =============================================================

    semantic_point = (
        planner._predict_semantic_point(
            image_rgb,
            args.instruction,
        )
    )

    print(
        "[TEST] Semantic point:",
        semantic_point,
    )

    # =============================================================
    # SAM point-prompt segmentation
    # =============================================================

    predictor = build_sam_predictor(
        args.sam_checkpoint
    )

    (
        mask,
        sam_score,
        masks,
        scores,
        best_mask_index,
    ) = select_best_mask(
        predictor=predictor,
        image_rgb=image_rgb,
        point_xy=semantic_point,
    )

    print(
        "[TEST] SAM masks:",
        len(
            masks
        ),
    )

    print(
        "[TEST] SAM scores:",
        np.asarray(
            scores
        ),
    )

    print(
        "[TEST] Selected SAM mask:",
        best_mask_index,
    )

    print(
        "[TEST] SAM score:",
        sam_score,
    )

    print(
        "[TEST] Mask pixels:",
        int(
            mask.sum()
        ),
    )

    # Save semantic/SAM-only overlay.
    draw_semantic_sam_overlay(
        image_rgb=image_rgb,
        mask=mask,
        semantic_point=semantic_point,
        sam_score=sam_score,
        instruction=args.instruction,
        out_path=(
            OUTPUT_DIR
            / "semantic_sam_overlay.png"
        ),
    )

    # =============================================================
    # SAM mask + depth -> target point cloud in camera frame
    # =============================================================

    target_pc_camera = (
        mask_to_camera_point_cloud(
            mask=mask,
            depth=depth,
            K=DEFAULT_K,
            min_depth=(
                args.depth_min
            ),
            max_depth=(
                args.depth_max
            ),
        )
    )

    target_centroid = (
        target_pc_camera.mean(
            axis=0
        )
    )

    print()

    print(
        "[TEST] Target point cloud:",
        target_pc_camera.shape,
    )

    print(
        "[TEST] Target centroid CAMERA:",
        target_centroid,
    )

    print(
        "[TEST] Target min CAMERA:",
        target_pc_camera.min(
            axis=0
        ),
    )

    print(
        "[TEST] Target max CAMERA:",
        target_pc_camera.max(
            axis=0
        ),
    )

    # =============================================================
    # GraspGen-X
    # =============================================================

    print()

    print(
        "[TEST] Running GraspGen-X..."
    )

    (
        grasps,
        confidences,
    ) = (
        planner.client.predict_grasps(
            point_cloud=target_pc_camera,
            num_grasps=(
                planner.num_grasps
            ),
            grasp_threshold=(
                planner.grasp_threshold
            ),
            topk_num_grasps=(
                planner.topk_num_grasps
            ),
            timeout=120.0,
        )
    )

    if len(
        grasps
    ) == 0:
        raise RuntimeError(
            "GraspGen-X returned zero grasps."
        )

    best_index = int(
        np.argmax(
            confidences
        )
    )

    best_grasp = grasps[
        best_index
    ]

    best_confidence = float(
        confidences[
            best_index
        ]
    )

    best_origin = best_grasp[
        :3,
        3,
    ]

    print(
        "[TEST] Grasps returned:",
        len(
            grasps
        ),
    )

    print(
        "[TEST] Confidence range:",
        float(
            confidences.min()
        ),
        "->",
        float(
            confidences.max()
        ),
    )

    print(
        "[TEST] Best index:",
        best_index,
    )

    print(
        "[TEST] Best confidence:",
        best_confidence,
    )

    print(
        "[TEST] Best grasp CAMERA:"
    )

    print(
        best_grasp
    )

    print(
        "[TEST] Best grasp origin CAMERA:",
        best_origin,
    )

    best_fingertip_center = (
        best_origin
        + 0.136
        * best_grasp[:3, 2]
    )

    print(
        "[TEST] Fingertip center CAMERA:",
        best_fingertip_center,
    )

    print(
        "[TEST] Fingertip center - target centroid:",
        (
            best_fingertip_center
            - target_centroid
        ),
    )

    print(
        "[TEST] Distance fingertip center -> target centroid [m]:",
        float(
            np.linalg.norm(
                best_fingertip_center
                - target_centroid
            )
        ),
    )

    print(
        "[TEST] Best origin - target centroid:",
        (
            best_origin
            - target_centroid
        ),
    )

    print(
        "[TEST] Distance origin -> target centroid [m]:",
        float(
            np.linalg.norm(
                best_origin
                - target_centroid
            )
        ),
    )

    print(
        "[TEST] Best closing axis (+X):",
        best_grasp[
            :3,
            0,
        ],
    )

    print(
        "[TEST] Best approach axis (+Z):",
        best_grasp[
            :3,
            2,
        ],
    )

    # =============================================================
    # Visual GraspGen-X result
    # =============================================================

    draw_graspgenx_overlay(
        image_rgb=image_rgb,
        mask=mask,
        semantic_point=semantic_point,
        grasps=grasps,
        confidences=confidences,
        K=DEFAULT_K,
        out_path=(
            OUTPUT_DIR
            / "graspgenx_overlay.png"
        ),
        num_visualized=(
            args.num_visualized
        ),
    )

    # =============================================================
    # Save numerical result
    # =============================================================

    np.savez_compressed(
        OUTPUT_DIR
        / "graspgenx_result.npz",
        semantic_point=(
            semantic_point
        ),
        sam_mask=(
            mask.astype(
                np.uint8
            )
        ),
        target_point_cloud_camera=(
            target_pc_camera
        ),
        grasps_camera=(
            grasps
        ),
        confidences=(
            confidences
        ),
        best_index=np.asarray(
            best_index,
            dtype=np.int32,
        ),
        best_grasp_camera=(
            best_grasp
        ),
    )

    # =============================================================
    # JSON report
    # =============================================================

    result_data = {
        "pkl": str(
            args.pkl
        ),
        "step": int(
            args.step
        ),
        "instruction": (
            args.instruction
        ),
        "rgb_key": (
            rgb_key
        ),
        "depth_key": (
            depth_key
        ),
        "rgb_shape": list(
            image_rgb.shape
        ),
        "depth_shape": list(
            depth.shape
        ),
        "semantic_point_px": [
            float(
                semantic_point[0]
            ),
            float(
                semantic_point[1]
            ),
        ],
        "sam_mask_index": int(
            best_mask_index
        ),
        "sam_score": float(
            sam_score
        ),
        "sam_mask_pixels": int(
            mask.sum()
        ),
        "target_point_cloud_points": int(
            len(
                target_pc_camera
            )
        ),
        "target_centroid_camera": (
            target_centroid.tolist()
        ),
        "graspgenx_candidates": int(
            len(
                grasps
            )
        ),
        "best_grasp_index": int(
            best_index
        ),
        "best_confidence": float(
            best_confidence
        ),
        "best_grasp_camera": (
            best_grasp.tolist()
        ),
        "best_grasp_origin_camera": (
            best_origin.tolist()
        ),
        "origin_minus_target_centroid": (
            (
                best_origin
                - target_centroid
            ).tolist()
        ),
        "origin_target_distance_m": float(
            np.linalg.norm(
                best_origin
                - target_centroid
            )
        ),
        "closing_axis_camera": (
            best_grasp[
                :3,
                0,
            ].tolist()
        ),
        "approach_axis_camera": (
            best_grasp[
                :3,
                2,
            ].tolist()
        ),
    }

    with (
        OUTPUT_DIR
        / "result.json"
    ).open(
        "w",
        encoding="utf-8",
    ) as stream:

        json.dump(
            result_data,
            stream,
            indent=2,
        )

    # =============================================================
    # Done
    # =============================================================

    print()

    print(
        "=" * 78
    )

    print(
        "[PASS] GraspGen-X pipeline completed."
    )

    print(
        "[TEST] Output directory:",
        OUTPUT_DIR,
    )

    print()

    print(
        "[TEST] Files:"
    )

    print(
        "       input_rgb.png"
    )

    print(
        "       input_depth.npy"
    )

    print(
        "       semantic_sam_overlay.png"
    )

    print(
        "       graspgenx_overlay.png"
    )

    print(
        "       graspgenx_result.npz"
    )

    print(
        "       result.json"
    )

    print(
        "=" * 78
    )


if __name__ == "__main__":
    main()