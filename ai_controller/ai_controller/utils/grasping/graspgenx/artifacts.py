from __future__ import annotations

import csv
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation


def _save_depth_preview(
    depth: np.ndarray,
    path: Path,
) -> None:
    depth = np.asarray(
        depth,
        dtype=np.float32,
    )

    valid = (
        np.isfinite(depth)
        & (depth > 0.0)
    )

    preview = np.zeros(
        depth.shape,
        dtype=np.uint8,
    )

    if valid.any():
        values = depth[valid]

        low = float(
            np.percentile(
                values,
                2.0,
            )
        )

        high = float(
            np.percentile(
                values,
                98.0,
            )
        )

        if high > low:
            normalized = (
                (depth - low)
                / (high - low)
            )

            normalized = np.clip(
                normalized,
                0.0,
                1.0,
            )

            preview[valid] = (
                255.0
                * (
                    1.0
                    - normalized[valid]
                )
            ).astype(
                np.uint8
            )

    Image.fromarray(
        preview
    ).save(
        path
    )


def _project_camera_point(
    point_camera: np.ndarray,
    K: np.ndarray,
) -> tuple[int, int] | None:
    point_camera = np.asarray(
        point_camera,
        dtype=np.float64,
    )

    if (
        point_camera.shape != (3,)
        or not np.isfinite(
            point_camera
        ).all()
        or point_camera[2] <= 0.0
    ):
        return None

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

    if not np.isfinite(
        [u, v]
    ).all():
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


def _draw_gripper_projection(
    grasp_camera: np.ndarray,
    K: np.ndarray,
    out_image: np.ndarray,
    *,
    fingertip_depth: float,
    gripper_opening: float,
) -> None:
    grasp_camera = np.asarray(
        grasp_camera,
        dtype=np.float64,
    )

    origin = grasp_camera[
        :3,
        3,
    ]

    closing_axis = grasp_camera[
        :3,
        0,
    ]

    approach_axis = grasp_camera[
        :3,
        2,
    ]

    half_opening = (
        float(gripper_opening)
        / 2.0
    )

    fingertip_center = (
        origin
        + float(
            fingertip_depth
        )
        * approach_axis
    )

    left_tip = (
        fingertip_center
        - half_opening
        * closing_axis
    )

    right_tip = (
        fingertip_center
        + half_opening
        * closing_axis
    )

    mid_depth = (
        float(
            fingertip_depth
        )
        * 0.60
    )

    left_mid = (
        origin
        + mid_depth
        * approach_axis
        - half_opening
        * closing_axis
    )

    right_mid = (
        origin
        + mid_depth
        * approach_axis
        + half_opening
        * closing_axis
    )

    points = [
        _project_camera_point(
            point,
            K,
        )
        for point in (
            origin,
            fingertip_center,
            left_tip,
            right_tip,
            left_mid,
            right_mid,
        )
    ]

    if any(
        point is None
        for point in points
    ):
        return

    (
        p_origin,
        p_center,
        p_left_tip,
        p_right_tip,
        p_left_mid,
        p_right_mid,
    ) = points

    cv2.line(
        out_image,
        p_origin,
        p_center,
        (255, 255, 0),
        3,
    )

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

    cv2.line(
        out_image,
        p_left_tip,
        p_right_tip,
        (0, 255, 255),
        3,
    )

    cv2.circle(
        out_image,
        p_origin,
        5,
        (255, 0, 0),
        -1,
    )

    cv2.circle(
        out_image,
        p_center,
        6,
        (0, 255, 0),
        -1,
    )


def _bbox_corners(
    bbox_min: np.ndarray,
    bbox_max: np.ndarray,
) -> np.ndarray:
    return np.asarray(
        [
            [x, y, z]
            for x in (
                bbox_min[0],
                bbox_max[0],
            )
            for y in (
                bbox_min[1],
                bbox_max[1],
            )
            for z in (
                bbox_min[2],
                bbox_max[2],
            )
        ],
        dtype=np.float64,
    )


def save_graspgenx_artifacts(
    *,
    artifacts_dir: str | Path,
    rgb_image: np.ndarray,
    depth_image: np.ndarray,
    camera_matrix: np.ndarray,
    T_base_camera: np.ndarray,
    T_base_table: np.ndarray,
    semantic_point_px: np.ndarray,
    target_mask: np.ndarray,
    target_point_cloud_camera: np.ndarray,
    target_point_cloud_base: np.ndarray,
    grasps_base: np.ndarray,
    confidences: np.ndarray,
    branch_tags: np.ndarray,
    selected_index: int,
    tcp_position_base: np.ndarray,
    tcp_orientation_base: np.ndarray,
    approach_direction_base: np.ndarray,
    fingertip_depth: float,
    gripper_opening: float,
    bbox_min: np.ndarray,
    bbox_max: np.ndarray,
    sam_masks: np.ndarray | None = None,
    sam_scores: np.ndarray | None = None,
    sam_logits: np.ndarray | None = None,
    sam_best_index: int | None = None,
) -> None:
    output_dir = (
        Path(
            artifacts_dir
        )
        .expanduser()
        .resolve()
    )

    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    rgb_image = np.asarray(
        rgb_image,
        dtype=np.uint8,
    )

    depth_image = np.asarray(
        depth_image,
        dtype=np.float32,
    )

    K = np.asarray(
        camera_matrix,
        dtype=np.float64,
    )

    T_base_camera = np.asarray(
        T_base_camera,
        dtype=np.float64,
    )

    T_base_table = np.asarray(
        T_base_table,
        dtype=np.float64,
    )

    grasps_base = np.asarray(
        grasps_base,
        dtype=np.float64,
    )

    confidences = np.asarray(
        confidences,
        dtype=np.float64,
    )

    branch_tags = np.asarray(
        branch_tags,
        dtype=str,
    )

    selected_index = int(
        selected_index
    )

    selected_grasp_base = (
        grasps_base[
            selected_index
        ]
    )

    # -------------------------------------------------------------
    # Raw input
    # -------------------------------------------------------------

    Image.fromarray(
        rgb_image
    ).save(
        output_dir
        / "input_rgb.png"
    )

    np.save(
        output_dir
        / "input_depth.npy",
        depth_image,
    )

    _save_depth_preview(
        depth_image,
        output_dir
        / "input_depth_preview.png",
    )

    # -------------------------------------------------------------
    # Target mask
    # -------------------------------------------------------------

    mask_image = (
        np.asarray(
            target_mask,
            dtype=np.uint8,
        )
        * 255
    )

    Image.fromarray(
        mask_image
    ).save(
        output_dir
        / "target_mask.png"
    )

    # -------------------------------------------------------------
    # Semantic + SAM overlay
    # -------------------------------------------------------------

    semantic_overlay = (
        rgb_image.copy()
    )

    mask_color = np.zeros_like(
        semantic_overlay
    )

    mask_color[
        ...,
        1,
    ] = 255

    semantic_overlay = np.where(
        target_mask[
            ...,
            None,
        ],
        (
            0.70
            * semantic_overlay
            + 0.30
            * mask_color
        ).astype(
            np.uint8
        ),
        semantic_overlay,
    )

    semantic_u = int(
        round(
            float(
                semantic_point_px[
                    0
                ]
            )
        )
    )

    semantic_v = int(
        round(
            float(
                semantic_point_px[
                    1
                ]
            )
        )
    )

    cv2.circle(
        semantic_overlay,
        (
            semantic_u,
            semantic_v,
        ),
        8,
        (
            255,
            0,
            255,
        ),
        -1,
    )

    cv2.circle(
        semantic_overlay,
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

    Image.fromarray(
        semantic_overlay
    ).save(
        output_dir
        / "semantic_sam_overlay.png"
    )

    # -------------------------------------------------------------
    # Selected grasp overlay
    # -------------------------------------------------------------

    T_camera_base = np.linalg.inv(
        T_base_camera
    )

    selected_grasp_camera = (
        T_camera_base
        @ selected_grasp_base
    )

    table_normal = (
        T_base_table[
            :3,
            2,
        ].copy()
    )

    table_normal /= np.linalg.norm(
        table_normal
    )

    if table_normal[2] < 0.0:
        table_normal = (
            -table_normal
        )

    table_origin = (
        T_base_table[
            :3,
            3,
        ]
    )

    plane_d = -float(
        np.dot(
            table_normal,
            table_origin,
        )
    )

    local_bbox = _bbox_corners(
        np.asarray(
            bbox_min,
            dtype=np.float64,
        ),
        np.asarray(
            bbox_max,
            dtype=np.float64,
        ),
    )

    selected_bbox_base = (
        local_bbox
        @ selected_grasp_base[
            :3,
            :3,
        ].T
        + selected_grasp_base[
            :3,
            3,
        ]
    )

    table_clearances = (
        selected_bbox_base
        @ table_normal
        + plane_d
    )

    selected_clearance = float(
        table_clearances.min()
    )

    target_centroid_base = (
        np.asarray(
            target_point_cloud_base,
            dtype=np.float64,
        ).mean(
            axis=0
        )
    )

    target_distance = float(
        np.linalg.norm(
            np.asarray(
                tcp_position_base,
                dtype=np.float64,
            )
            - target_centroid_base
        )
    )

    selected_overlay = (
        semantic_overlay.copy()
    )

    _draw_gripper_projection(
        selected_grasp_camera,
        K,
        selected_overlay,
        fingertip_depth=(
            fingertip_depth
        ),
        gripper_opening=(
            gripper_opening
        ),
    )

    selected_confidence = float(
        confidences[
            selected_index
        ]
    )

    selected_branch = str(
        branch_tags[
            selected_index
        ]
    )

    lines = [
        (
            f"selected="
            f"{selected_index} "
            f"({selected_branch})"
        ),
        (
            f"confidence="
            f"{selected_confidence:.4f}"
        ),
        (
            f"table clearance="
            f"{selected_clearance * 1000.0:.2f} mm"
        ),
        (
            f"target distance="
            f"{target_distance * 1000.0:.2f} mm"
        ),
    ]

    y = 28

    for line in lines:
        cv2.putText(
            selected_overlay,
            line,
            (
                20,
                y,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (
                255,
                255,
                255,
            ),
            3,
            cv2.LINE_AA,
        )

        cv2.putText(
            selected_overlay,
            line,
            (
                20,
                y,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (
                20,
                20,
                20,
            ),
            1,
            cv2.LINE_AA,
        )

        y += 25

    Image.fromarray(
        selected_overlay
    ).save(
        output_dir
        / "selected_grasp_overlay.png"
    )

    # -------------------------------------------------------------
    # Numeric diagnostics
    # -------------------------------------------------------------

    npz_payload = {
        "semantic_point_px": (
            np.asarray(
                semantic_point_px,
                dtype=np.float32,
            )
        ),
        "target_mask": (
            np.asarray(
                target_mask,
                dtype=np.uint8,
            )
        ),
        "target_point_cloud_camera": (
            np.asarray(
                target_point_cloud_camera,
                dtype=np.float32,
            )
        ),
        "target_point_cloud_base": (
            np.asarray(
                target_point_cloud_base,
                dtype=np.float32,
            )
        ),
        "grasps_base": (
            grasps_base.astype(
                np.float32
            )
        ),
        "confidences": (
            confidences.astype(
                np.float32
            )
        ),
        "branch_tags": (
            branch_tags
        ),
        "selected_index": np.asarray(
            selected_index,
            dtype=np.int32,
        ),
        "selected_grasp_base": (
            selected_grasp_base
        ),
        "selected_grasp_camera": (
            selected_grasp_camera
        ),
        "tcp_position_base": (
            np.asarray(
                tcp_position_base,
                dtype=np.float64,
            )
        ),
        "tcp_orientation_base": (
            np.asarray(
                tcp_orientation_base,
                dtype=np.float64,
            )
        ),
        "approach_direction_base": (
            np.asarray(
                approach_direction_base,
                dtype=np.float64,
            )
        ),
        "T_base_camera": (
            T_base_camera
        ),
        "T_base_table": (
            T_base_table
        ),
        "selected_table_clearance_m": (
            np.asarray(
                selected_clearance,
                dtype=np.float64,
            )
        ),
        "selected_target_distance_m": (
            np.asarray(
                target_distance,
                dtype=np.float64,
            )
        ),
    }

    if sam_masks is not None:
        npz_payload[
            "sam_masks"
        ] = np.asarray(
            sam_masks,
            dtype=np.uint8,
        )

    if sam_scores is not None:
        npz_payload[
            "sam_scores"
        ] = np.asarray(
            sam_scores,
            dtype=np.float32,
        )

    if sam_logits is not None:
        npz_payload[
            "sam_logits"
        ] = np.asarray(
            sam_logits,
            dtype=np.float32,
        )

    if sam_best_index is not None:
        npz_payload[
            "sam_best_index"
        ] = np.asarray(
            sam_best_index,
            dtype=np.int32,
        )

    np.savez_compressed(
        output_dir
        / "graspgenx_diagnostics.npz",
        **npz_payload,
    )

    # -------------------------------------------------------------
    # Human-readable candidate list
    # -------------------------------------------------------------

    with (
        output_dir
        / "candidates.csv"
    ).open(
        "w",
        newline="",
        encoding="utf-8",
    ) as stream:
        writer = csv.writer(
            stream
        )

        writer.writerow(
            [
                "index",
                "branch",
                "confidence",
                "selected",
                "x",
                "y",
                "z",
            ]
        )

        for index, (
            grasp,
            confidence,
            branch,
        ) in enumerate(
            zip(
                grasps_base,
                confidences,
                branch_tags,
                strict=True,
            )
        ):
            writer.writerow(
                [
                    index,
                    str(branch),
                    float(
                        confidence
                    ),
                    int(
                        index
                        == selected_index
                    ),
                    float(
                        grasp[
                            0,
                            3,
                        ]
                    ),
                    float(
                        grasp[
                            1,
                            3,
                        ]
                    ),
                    float(
                        grasp[
                            2,
                            3,
                        ]
                    ),
                ]
            )