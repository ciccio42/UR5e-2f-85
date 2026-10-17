from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

from ai_controller.models.seedo_controller.utils import (
    ARUCO_TO_TABLE0_ROTATION,
    ARUCO_TO_TABLE0_TRANSLATION,
)

def _normalized(
    vector: np.ndarray,
) -> np.ndarray:

    vector = np.asarray(
        vector,
        dtype=np.float64,
    )

    norm = float(
        np.linalg.norm(
            vector
        )
    )

    if norm <= 1e-12:
        raise RuntimeError(
            "Cannot normalize zero vector."
        )

    return (
        vector
        / norm
    )

def _camera_points_to_base(
    points_camera: np.ndarray,
    *,
    camera_calibration: dict[str, np.ndarray],
    T_base_table: np.ndarray,
) -> np.ndarray:

    points_camera = np.asarray(
        points_camera,
        dtype=np.float64,
    )

    R_camera_aruco = np.asarray(
        camera_calibration[
            "orientation_matrix"
        ],
        dtype=np.float64,
    )

    t_camera_aruco = np.asarray(
        camera_calibration[
            "position"
        ],
        dtype=np.float64,
    )

    points_aruco = (
        points_camera
        @ R_camera_aruco.T
        + t_camera_aruco
    )

    points_table = (
        points_aruco
        @ ARUCO_TO_TABLE0_ROTATION.T
        + ARUCO_TO_TABLE0_TRANSLATION
    )

    points_base = (
        points_table
        @ T_base_table[
            :3,
            :3,
        ].T
        + T_base_table[
            :3,
            3,
        ]
    )

    return points_base

def _base_points_to_camera(
    points_base: np.ndarray,
    *,
    camera_calibration: dict[str, np.ndarray],
    T_base_table: np.ndarray,
) -> np.ndarray:

    points_base = np.asarray(
        points_base,
        dtype=np.float64,
    )

    R_table_base = np.asarray(
        T_base_table[
            :3,
            :3,
        ],
        dtype=np.float64,
    )

    t_table_base = np.asarray(
        T_base_table[
            :3,
            3,
        ],
        dtype=np.float64,
    )

    points_table = (
        points_base
        - t_table_base
    ) @ R_table_base

    points_aruco = (
        points_table
        - ARUCO_TO_TABLE0_TRANSLATION
    ) @ ARUCO_TO_TABLE0_ROTATION

    R_camera_aruco = np.asarray(
        camera_calibration[
            "orientation_matrix"
        ],
        dtype=np.float64,
    )

    t_camera_aruco = np.asarray(
        camera_calibration[
            "position"
        ],
        dtype=np.float64,
    )

    points_camera = (
        points_aruco
        - t_camera_aruco
    ) @ R_camera_aruco

    return points_camera

def _project_base_points(
    points_base: np.ndarray,
    *,
    K: np.ndarray,
    camera_calibration: dict[str, np.ndarray],
    T_base_table: np.ndarray,
) -> tuple[
    np.ndarray,
    np.ndarray,
]:

    points_camera = _base_points_to_camera(
        points_base,

        camera_calibration=(
            camera_calibration
        ),

        T_base_table=(
            T_base_table
        ),
    )

    z = points_camera[
        :,
        2
    ]

    valid = (
        np.isfinite(
            points_camera
        ).all(
            axis=1
        )
        & (
            z
            > 1e-6
        )
    )

    pixels = np.full(
        (
            len(
                points_camera
            ),
            2,
        ),
        np.nan,
        dtype=np.float64,
    )

    valid_points = points_camera[
        valid
    ]

    pixels[
        valid,
        0,
    ] = (
        K[
            0,
            0
        ]
        * valid_points[
            :,
            0
        ]
        / valid_points[
            :,
            2
        ]
        + K[
            0,
            2
        ]
    )

    pixels[
        valid,
        1,
    ] = (
        K[
            1,
            1
        ]
        * valid_points[
            :,
            1
        ]
        / valid_points[
            :,
            2
        ]
        + K[
            1,
            2
        ]
    )

    return (
        pixels,
        valid,
    )

def _mask_depth_to_base_cloud(
    *,
    mask: np.ndarray,
    depth: np.ndarray,
    K: np.ndarray,
    camera_calibration: dict[str, np.ndarray],
    T_base_table: np.ndarray,
    max_points: int = 6000,
) -> np.ndarray:

    mask = np.asarray(
        mask,
        dtype=bool,
    )

    depth = np.asarray(
        depth,
        dtype=np.float64,
    )

    if mask.shape != depth.shape:
        raise RuntimeError(
            "Front mask and depth dimensions differ."
        )

    valid = (
        mask
        & np.isfinite(
            depth
        )
        & (
            depth > 0.05
        )
        & (
            depth < 2.0
        )
    )

    v, u = np.where(
        valid
    )

    if len(
        u
    ) == 0:
        raise RuntimeError(
            "Assembly-object mask contains no valid depth pixels."
        )

    if len(
        u
    ) > max_points:

        indices = np.linspace(
            0,
            len(
                u
            )
            - 1,
            max_points,
            dtype=int,
        )

        u = u[
            indices
        ]

        v = v[
            indices
        ]

    z = depth[
        v,
        u
    ]

    x = (
        (
            u
            - K[
                0,
                2
            ]
        )
        * z
        / K[
            0,
            0
        ]
    )

    y = (
        (
            v
            - K[
                1,
                2
            ]
        )
        * z
        / K[
            1,
            1
        ]
    )

    points_camera = np.column_stack(
        (
            x,
            y,
            z,
        )
    )

    return _camera_points_to_base(
        points_camera,

        camera_calibration=(
            camera_calibration
        ),

        T_base_table=(
            T_base_table
        ),
    )

def _transform_object_via_tcp(
    points_base: np.ndarray,
    *,
    source_tcp_position: np.ndarray,
    source_tcp_orientation: np.ndarray,
    destination_tcp_position: np.ndarray,
    destination_tcp_orientation: np.ndarray,
) -> np.ndarray:

    points_base = np.asarray(
        points_base,
        dtype=np.float64,
    )

    source_rotation = Rotation.from_quat(
        source_tcp_orientation
    )

    destination_rotation = Rotation.from_quat(
        destination_tcp_orientation
    )

    points_tcp_local = (
        source_rotation
        .inv()
        .apply(
            points_base
            - source_tcp_position
        )
    )

    predicted_points_base = (
        destination_tcp_position
        + destination_rotation.apply(
            points_tcp_local
        )
    )

    return predicted_points_base

def _draw_projected_point(
    image_bgr: np.ndarray,
    *,
    point_base: np.ndarray,
    label: str,
    color: tuple[int, int, int],
    K: np.ndarray,
    camera_calibration: dict[str, np.ndarray],
    T_base_table: np.ndarray,
    radius: int = 7,
) -> tuple[int, int] | None:

    pixels, valid = _project_base_points(
        np.asarray(
            point_base,
            dtype=np.float64,
        ).reshape(
            1,
            3,
        ),

        K=K,

        camera_calibration=(
            camera_calibration
        ),

        T_base_table=(
            T_base_table
        ),
    )

    if not bool(
        valid[
            0
        ]
    ):
        return None

    u = int(
        round(
            float(
                pixels[
                    0,
                    0
                ]
            )
        )
    )

    v = int(
        round(
            float(
                pixels[
                    0,
                    1
                ]
            )
        )
    )

    height, width = (
        image_bgr.shape[
            :2
        ]
    )

    if (
        u < 0
        or u >= width
        or v < 0
        or v >= height
    ):
        return None

    cv2.circle(
        image_bgr,
        (
            u,
            v,
        ),
        radius,
        color,
        -1,
    )

    cv2.putText(
        image_bgr,
        label,
        (
            u + 8,
            v - 8,
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        color,
        1,
        cv2.LINE_AA,
    )

    return (
        u,
        v,
    )

def _draw_projected_segment(
    image_bgr: np.ndarray,
    *,
    point_a: np.ndarray,
    point_b: np.ndarray,
    color: tuple[int, int, int],
    K: np.ndarray,
    camera_calibration: dict[str, np.ndarray],
    T_base_table: np.ndarray,
    thickness: int = 2,
) -> None:

    points = np.asarray(
        [
            point_a,
            point_b,
        ],
        dtype=np.float64,
    )

    pixels, valid = _project_base_points(
        points,

        K=K,

        camera_calibration=(
            camera_calibration
        ),

        T_base_table=(
            T_base_table
        ),
    )

    if not bool(
        np.all(
            valid
        )
    ):
        return

    p1 = tuple(
        np.round(
            pixels[
                0
            ]
        )
        .astype(int)
        .tolist()
    )

    p2 = tuple(
        np.round(
            pixels[
                1
            ]
        )
        .astype(int)
        .tolist()
    )

    cv2.line(
        image_bgr,
        p1,
        p2,
        color,
        thickness,
        cv2.LINE_AA,
    )

def _hole_from_pose(
    *,
    position: np.ndarray,
    orientation: np.ndarray,
    hole_top_local: np.ndarray,
    hole_bottom_local: np.ndarray,
    hole_axis_local: np.ndarray,
):

    rotation = Rotation.from_quat(
        orientation
    )

    top = (
        position
        + rotation.apply(
            hole_top_local
        )
    )

    bottom = (
        position
        + rotation.apply(
            hole_bottom_local
        )
    )

    axis = _normalized(
        rotation.apply(
            hole_axis_local
        )
    )

    return (
        top,
        bottom,
        axis,
    )

def _hole_from_action(
    action: np.ndarray,
    *,
    hole_top_local: np.ndarray,
    hole_bottom_local: np.ndarray,
    hole_axis_local: np.ndarray,
):

    action = np.asarray(
        action,
        dtype=np.float64,
    )

    return _hole_from_pose(
        position=(
            action[
                :3
            ]
        ),

        orientation=(
            action[
                3:7
            ]
        ),

        hole_top_local=(
            hole_top_local
        ),

        hole_bottom_local=(
            hole_bottom_local
        ),

        hole_axis_local=(
            hole_axis_local
        ),
    )

def _collect_hole_bottom_trajectory(
    action_groups: dict[str, list],
    *,
    hole_top_local: np.ndarray,
    hole_bottom_local: np.ndarray,
    hole_axis_local: np.ndarray,
) -> dict[str, np.ndarray]:

    output = {}

    for primitive, actions in action_groups.items():

        bottoms = []

        for action in actions:

            action = np.asarray(
                action,
                dtype=np.float64,
            )

            if action.size < 7:
                continue

            (
                _,
                bottom,
                _,
            ) = _hole_from_action(
                action,

                hole_top_local=(
                    hole_top_local
                ),

                hole_bottom_local=(
                    hole_bottom_local
                ),

                hole_axis_local=(
                    hole_axis_local
                ),
            )

            bottoms.append(
                bottom
            )

        output[
            primitive
        ] = np.asarray(
            bottoms,
            dtype=np.float64,
        )

    return output

def _write_json(
    path: Path,
    payload: Any,
) -> None:

    def jsonable(
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
                str(key): jsonable(
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
                jsonable(
                    item
                )
                for item in value
            ]

        return value

    path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    with path.open(
        "w",
        encoding="utf-8",
    ) as stream:
        json.dump(
            jsonable(
                payload
            ),
            stream,
            indent=2,
            ensure_ascii=False,
            allow_nan=False,
        )

def export_nut_visual_debug_artifacts(
    *,
    output_dir: Path,
    front_rgb: np.ndarray,
    front_depth: np.ndarray,
    front_K: np.ndarray,
    front_camera_calibration: dict[str, np.ndarray],
    T_base_table: np.ndarray,
    assembly_object_mask: np.ndarray,
    pick_target: str,
    place_target: str,
    real_grasp_tcp: np.ndarray,
    real_grasp_quaternion: np.ndarray,
    final_tcp_position: np.ndarray,
    final_tcp_orientation: np.ndarray,
    hole_top_initial: np.ndarray,
    hole_bottom_initial: np.ndarray,
    hole_axis_initial: np.ndarray,
    peg_support: np.ndarray,
    peg_tip: np.ndarray,
    peg_axis: np.ndarray,
    align_target_hole_bottom: np.ndarray,
    initial_entry_hole_bottom: np.ndarray,
    final_release_hole_bottom: np.ndarray,
    final_hole_top: np.ndarray,
    final_hole_bottom: np.ndarray,
    final_hole_axis: np.ndarray,
    lift_tcp: np.ndarray,
    object_sweep_radius_m: float,
    object_thickness_m: float,
    peg_height_m: float,
    action_groups: dict[str, list[np.ndarray]],
    hole_top_local: np.ndarray,
    hole_bottom_local: np.ndarray,
    hole_axis_local: np.ndarray,
    auto_release: bool,
) -> dict[str, Path]:
    """
    Export geometric diagnostics for one Nut-Assembly execution.

    This function performs no perception, segmentation, grasp planning,
    or motion planning.

    Every artifact is derived only from:
      - the RGB-D and SAM mask already obtained at runtime;
      - the already-refined hole and peg geometry;
      - the real grasp pose selected by the grasp planner;
      - the actions already generated by the Motion Layer.
    """

    output_dir = (
        Path(output_dir)
        .expanduser()
        .resolve()
    )

    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    # -------------------------------------------------------------
    # Normalize and validate inputs.
    # -------------------------------------------------------------

    front_rgb = np.asarray(
        front_rgb,
        dtype=np.uint8,
    )

    front_depth = np.asarray(
        front_depth,
        dtype=np.float64,
    )

    front_K = np.asarray(
        front_K,
        dtype=np.float64,
    )

    T_base_table = np.asarray(
        T_base_table,
        dtype=np.float64,
    )

    assembly_object_mask = np.asarray(
        assembly_object_mask,
        dtype=bool,
    )

    if (
        front_rgb.ndim != 3
        or front_rgb.shape[2] != 3
    ):
        raise ValueError(
            "front_rgb must have shape (H, W, 3)."
        )

    if front_depth.ndim != 2:
        raise ValueError(
            "front_depth must be a 2D array."
        )

    if (
        front_rgb.shape[:2]
        != front_depth.shape
    ):
        raise ValueError(
            "front_rgb and front_depth must have "
            "identical spatial shape."
        )

    if (
        assembly_object_mask.shape
        != front_depth.shape
    ):
        raise ValueError(
            "assembly_object_mask and front_depth must "
            "have identical shape."
        )

    if front_K.shape != (3, 3):
        raise ValueError(
            "front_K must have shape (3, 3)."
        )

    if T_base_table.shape != (4, 4):
        raise ValueError(
            "T_base_table must have shape (4, 4)."
        )

    real_grasp_tcp = np.asarray(
        real_grasp_tcp,
        dtype=np.float64,
    )

    real_grasp_quaternion = _normalized(
        np.asarray(
            real_grasp_quaternion,
            dtype=np.float64,
        )
    )

    final_tcp_position = np.asarray(
        final_tcp_position,
        dtype=np.float64,
    )

    final_tcp_orientation = _normalized(
        np.asarray(
            final_tcp_orientation,
            dtype=np.float64,
        )
    )

    hole_top_initial = np.asarray(
        hole_top_initial,
        dtype=np.float64,
    )

    hole_bottom_initial = np.asarray(
        hole_bottom_initial,
        dtype=np.float64,
    )

    hole_axis_initial = _normalized(
        hole_axis_initial
    )

    peg_support = np.asarray(
        peg_support,
        dtype=np.float64,
    )

    peg_tip = np.asarray(
        peg_tip,
        dtype=np.float64,
    )

    peg_axis = _normalized(
        peg_axis
    )

    align_target_hole_bottom = np.asarray(
        align_target_hole_bottom,
        dtype=np.float64,
    )

    initial_entry_hole_bottom = np.asarray(
        initial_entry_hole_bottom,
        dtype=np.float64,
    )

    final_release_hole_bottom = np.asarray(
        final_release_hole_bottom,
        dtype=np.float64,
    )

    final_hole_top = np.asarray(
        final_hole_top,
        dtype=np.float64,
    )

    final_hole_bottom = np.asarray(
        final_hole_bottom,
        dtype=np.float64,
    )

    final_hole_axis = _normalized(
        final_hole_axis
    )

    lift_tcp = np.asarray(
        lift_tcp,
        dtype=np.float64,
    )

    hole_top_local = np.asarray(
        hole_top_local,
        dtype=np.float64,
    )

    hole_bottom_local = np.asarray(
        hole_bottom_local,
        dtype=np.float64,
    )

    hole_axis_local = _normalized(
        hole_axis_local
    )

    object_sweep_radius_m = float(
        object_sweep_radius_m
    )

    object_thickness_m = float(
        object_thickness_m
    )

    peg_height_m = float(
        peg_height_m
    )

    # -------------------------------------------------------------
    # Reconstruct the actual observed assembly object.
    #
    # This uses the already-existing t=0 SAM mask + front RGB-D.
    # No segmentation or perception inference is repeated.
    # -------------------------------------------------------------

    object_points_initial_base = (
        _mask_depth_to_base_cloud(
            mask=assembly_object_mask,
            depth=front_depth,
            K=front_K,
            camera_calibration=(
                front_camera_calibration
            ),
            T_base_table=(
                T_base_table
            ),
        )
    )

    # -------------------------------------------------------------
    # Predict the final 3D placement of the whole observed object.
    #
    # 1. Express every observed object point in the TCP frame of
    #    the real planned grasp.
    # 2. Apply the final TCP pose produced by MotionLayer.
    #
    # No object-shape prior is introduced here.
    # -------------------------------------------------------------

    object_points_final_base = (
        _transform_object_via_tcp(
            object_points_initial_base,

            source_tcp_position=(
                real_grasp_tcp
            ),

            source_tcp_orientation=(
                real_grasp_quaternion
            ),

            destination_tcp_position=(
                final_tcp_position
            ),

            destination_tcp_orientation=(
                final_tcp_orientation
            ),
        )
    )

    predicted_object_cloud_path = (
        output_dir
        / "predicted_final_assembly_object_points_base.npy"
    )

    np.save(
        predicted_object_cloud_path,
        object_points_final_base,
    )

    # -------------------------------------------------------------
    # Recover the hole-bottom trajectory generated by MotionLayer.
    # -------------------------------------------------------------

    hole_trajectories = (
        _collect_hole_bottom_trajectory(
            action_groups,

            hole_top_local=(
                hole_top_local
            ),

            hole_bottom_local=(
                hole_bottom_local
            ),

            hole_axis_local=(
                hole_axis_local
            ),
        )
    )

    predicted_bbox_min = np.min(
        object_points_final_base,
        axis=0,
    )

    predicted_bbox_max = np.max(
        object_points_final_base,
        axis=0,
    )

    # -------------------------------------------------------------
    # Numerical JSON artifact.
    # -------------------------------------------------------------

    points_json_path = (
        output_dir
        / "nut_visual_debug_points.json"
    )

    _write_json(
        points_json_path,
        {
            "frame": "base_link",

            "pick_target": (
                pick_target
            ),

            "place_target": (
                place_target
            ),

            "auto_release": bool(
                auto_release
            ),

            "important_note": (
                "final_release_hole_bottom_base is the planned "
                "final placement target. "
                "final_hole_bottom_base is the hole position "
                "actually produced by the final MotionLayer pose. "
                "The point is an actual gripper-release location "
                "only when auto_release is enabled."
            ),

            "initial_assembly_object_geometry": {
                "hole_top_base": (
                    hole_top_initial
                ),

                "hole_bottom_base": (
                    hole_bottom_initial
                ),

                "hole_axis_base": (
                    hole_axis_initial
                ),

                "object_thickness_m": (
                    object_thickness_m
                ),
            },

            "peg_geometry": {
                "support_base": (
                    peg_support
                ),

                "tip_base": (
                    peg_tip
                ),

                "axis_base": (
                    peg_axis
                ),

                "height_m": (
                    peg_height_m
                ),
            },

            "planned_targets": {
                "align_hole_bottom_base": (
                    align_target_hole_bottom
                ),

                "initial_entry_hole_bottom_base": (
                    initial_entry_hole_bottom
                ),

                "final_release_hole_bottom_base": (
                    final_release_hole_bottom
                ),
            },

            "predicted_final_assembly_object_pose": {
                "hole_top_base": (
                    final_hole_top
                ),

                "hole_bottom_base": (
                    final_hole_bottom
                ),

                "hole_axis_base": (
                    final_hole_axis
                ),

                "tcp_position_base": (
                    final_tcp_position
                ),

                "tcp_orientation_base": (
                    final_tcp_orientation
                ),

                "point_cloud_bbox_min_base": (
                    predicted_bbox_min
                ),

                "point_cloud_bbox_max_base": (
                    predicted_bbox_max
                ),

                "point_count": int(
                    len(
                        object_points_final_base
                    )
                ),
            },

            "real_grasp": {
                "tcp_position_base": (
                    real_grasp_tcp
                ),

                "tcp_orientation_base": (
                    real_grasp_quaternion
                ),
            },

            "lift_tcp_base": (
                lift_tcp
            ),

            "object_sweep_radius_m": (
                object_sweep_radius_m
            ),

            "hole_bottom_trajectories": {
                primitive: points
                for primitive, points
                in hole_trajectories.items()
            },
        },
    )

    # -------------------------------------------------------------
    # Front-camera overlay.
    #
    # front_rgb is RGB.
    # OpenCV drawing/imwrite uses BGR here.
    # -------------------------------------------------------------

    overlay = cv2.cvtColor(
        front_rgb,
        cv2.COLOR_RGB2BGR,
    ).copy()

    initial_pixels, initial_valid = (
        _project_base_points(
            object_points_initial_base,

            K=front_K,

            camera_calibration=(
                front_camera_calibration
            ),

            T_base_table=(
                T_base_table
            ),
        )
    )

    for pixel, valid in zip(
        initial_pixels,
        initial_valid,
    ):

        if not valid:
            continue

        u, v = (
            np.round(
                pixel
            )
            .astype(int)
        )

        if (
            0 <= u < overlay.shape[1]
            and
            0 <= v < overlay.shape[0]
        ):
            cv2.circle(
                overlay,
                (
                    int(u),
                    int(v),
                ),
                1,
                (
                    255,
                    160,
                    0,
                ),
                -1,
            )

    final_pixels, final_valid = (
        _project_base_points(
            object_points_final_base,

            K=front_K,

            camera_calibration=(
                front_camera_calibration
            ),

            T_base_table=(
                T_base_table
            ),
        )
    )

    for pixel, valid in zip(
        final_pixels,
        final_valid,
    ):

        if not valid:
            continue

        u, v = (
            np.round(
                pixel
            )
            .astype(int)
        )

        if (
            0 <= u < overlay.shape[1]
            and
            0 <= v < overlay.shape[0]
        ):
            cv2.circle(
                overlay,
                (
                    int(u),
                    int(v),
                ),
                1,
                (
                    0,
                    255,
                    255,
                ),
                -1,
            )

    # Peg axis.
    _draw_projected_segment(
        overlay,

        point_a=(
            peg_support
        ),

        point_b=(
            peg_tip
        ),

        color=(
            0,
            255,
            0,
        ),

        K=front_K,

        camera_calibration=(
            front_camera_calibration
        ),

        T_base_table=(
            T_base_table
        ),

        thickness=3,
    )

    # Align -> initial entry.
    _draw_projected_segment(
        overlay,

        point_a=(
            align_target_hole_bottom
        ),

        point_b=(
            initial_entry_hole_bottom
        ),

        color=(
            255,
            255,
            0,
        ),

        K=front_K,

        camera_calibration=(
            front_camera_calibration
        ),

        T_base_table=(
            T_base_table
        ),
    )

    # Initial entry -> final target.
    _draw_projected_segment(
        overlay,

        point_a=(
            initial_entry_hole_bottom
        ),

        point_b=(
            final_release_hole_bottom
        ),

        color=(
            255,
            0,
            255,
        ),

        K=front_K,

        camera_calibration=(
            front_camera_calibration
        ),

        T_base_table=(
            T_base_table
        ),
    )

    _draw_projected_point(
        overlay,

        point_base=(
            hole_bottom_initial
        ),

        label="HOLE initial",

        color=(
            255,
            0,
            0,
        ),

        K=front_K,

        camera_calibration=(
            front_camera_calibration
        ),

        T_base_table=(
            T_base_table
        ),
    )

    _draw_projected_point(
        overlay,

        point_base=(
            peg_support
        ),

        label="PEG support",

        color=(
            0,
            180,
            0,
        ),

        K=front_K,

        camera_calibration=(
            front_camera_calibration
        ),

        T_base_table=(
            T_base_table
        ),
    )

    _draw_projected_point(
        overlay,

        point_base=(
            peg_tip
        ),

        label="PEG tip",

        color=(
            0,
            0,
            255,
        ),

        K=front_K,

        camera_calibration=(
            front_camera_calibration
        ),

        T_base_table=(
            T_base_table
        ),
    )

    _draw_projected_point(
        overlay,

        point_base=(
            align_target_hole_bottom
        ),

        label="ALIGN",

        color=(
            255,
            255,
            0,
        ),

        K=front_K,

        camera_calibration=(
            front_camera_calibration
        ),

        T_base_table=(
            T_base_table
        ),
    )

    _draw_projected_point(
        overlay,

        point_base=(
            initial_entry_hole_bottom
        ),

        label="ENTRY",

        color=(
            0,
            165,
            255,
        ),

        K=front_K,

        camera_calibration=(
            front_camera_calibration
        ),

        T_base_table=(
            T_base_table
        ),
    )

    final_label = (
        "RELEASE"
        if auto_release
        else "FINAL"
    )

    _draw_projected_point(
        overlay,

        point_base=(
            final_hole_bottom
        ),

        label=final_label,

        color=(
            255,
            0,
            255,
        ),

        K=front_K,

        camera_calibration=(
            front_camera_calibration
        ),

        T_base_table=(
            T_base_table
        ),
    )

    cv2.putText(
        overlay,
        "BLUE CLOUD: initial assembly object",
        (
            15,
            22,
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (
            255,
            160,
            0,
        ),
        1,
        cv2.LINE_AA,
    )

    cv2.putText(
        overlay,
        "YELLOW CLOUD: predicted final assembly object",
        (
            15,
            42,
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (
            0,
            255,
            255,
        ),
        1,
        cv2.LINE_AA,
    )

    front_overlay_path = (
        output_dir
        / "nut_place_targets_front_overlay.png"
    )

    if not cv2.imwrite(
        str(
            front_overlay_path
        ),
        overlay,
    ):
        raise RuntimeError(
            "Could not save Nut-Assembly front overlay."
        )

    # -------------------------------------------------------------
    # Top-down X-Y plot.
    # -------------------------------------------------------------

    topdown_path = (
        output_dir
        / "nut_place_targets_topdown.png"
    )

    figure = plt.figure(
        figsize=(
            10,
            8,
        )
    )

    axis = figure.add_subplot(
        111
    )

    initial_plot_points = (
        object_points_initial_base[
            ::max(
                1,
                len(
                    object_points_initial_base
                )
                // 1200,
            )
        ]
    )

    final_plot_points = (
        object_points_final_base[
            ::max(
                1,
                len(
                    object_points_final_base
                )
                // 1200,
            )
        ]
    )

    axis.scatter(
        initial_plot_points[
            :,
            0
        ],
        initial_plot_points[
            :,
            1
        ],
        s=3,
        alpha=0.25,
        label="Initial observed assembly object",
    )

    axis.scatter(
        final_plot_points[
            :,
            0
        ],
        final_plot_points[
            :,
            1
        ],
        s=3,
        alpha=0.35,
        label="Predicted final assembly object",
    )

    axis.scatter(
        hole_bottom_initial[
            0
        ],
        hole_bottom_initial[
            1
        ],
        s=90,
        marker="x",
        label="Initial hole bottom",
    )

    axis.scatter(
        peg_tip[
            0
        ],
        peg_tip[
            1
        ],
        s=100,
        marker="^",
        label="Peg axis XY",
    )

    axis.scatter(
        align_target_hole_bottom[
            0
        ],
        align_target_hole_bottom[
            1
        ],
        s=85,
        marker="o",
        label="Align target",
    )

    axis.scatter(
        final_hole_bottom[
            0
        ],
        final_hole_bottom[
            1
        ],
        s=100,
        marker="*",
        label=(
            "Release position"
            if auto_release
            else "Final placement position"
        ),
    )

    path_xy = np.asarray(
        [
            hole_bottom_initial,
            align_target_hole_bottom,
            initial_entry_hole_bottom,
            final_release_hole_bottom,
        ],
        dtype=np.float64,
    )

    axis.plot(
        path_xy[
            :,
            0
        ],
        path_xy[
            :,
            1
        ],
        "--",
        linewidth=1.5,
        label="Hole target progression",
    )

    axis.set_xlabel(
        "base_link X [m]"
    )

    axis.set_ylabel(
        "base_link Y [m]"
    )

    axis.set_title(
        "Nut Assembly - Top Down (X-Y)"
    )

    axis.axis(
        "equal"
    )

    axis.grid(
        True,
        alpha=0.25,
    )

    axis.legend(
        loc="best"
    )

    figure.tight_layout()

    figure.savefig(
        topdown_path,
        dpi=180,
    )

    plt.close(
        figure
    )

    # -------------------------------------------------------------
    # Side Y-Z plot.
    # -------------------------------------------------------------

    sideview_path = (
        output_dir
        / "nut_place_targets_sideview.png"
    )

    figure = plt.figure(
        figsize=(
            10,
            8,
        )
    )

    axis = figure.add_subplot(
        111
    )

    axis.scatter(
        initial_plot_points[
            :,
            1
        ],
        initial_plot_points[
            :,
            2
        ],
        s=3,
        alpha=0.25,
        label="Initial observed assembly object",
    )

    axis.scatter(
        final_plot_points[
            :,
            1
        ],
        final_plot_points[
            :,
            2
        ],
        s=3,
        alpha=0.35,
        label="Predicted final assembly object",
    )

    axis.plot(
        [
            peg_support[
                1
            ],
            peg_tip[
                1
            ],
        ],
        [
            peg_support[
                2
            ],
            peg_tip[
                2
            ],
        ],
        linewidth=4,
        label="Refined peg axis",
    )

    axis.scatter(
        peg_support[
            1
        ],
        peg_support[
            2
        ],
        s=100,
        marker="s",
        label="Peg support",
    )

    axis.scatter(
        peg_tip[
            1
        ],
        peg_tip[
            2
        ],
        s=100,
        marker="^",
        label="Peg tip",
    )

    axis.scatter(
        align_target_hole_bottom[
            1
        ],
        align_target_hole_bottom[
            2
        ],
        s=90,
        marker="o",
        label="Align target",
    )

    axis.scatter(
        initial_entry_hole_bottom[
            1
        ],
        initial_entry_hole_bottom[
            2
        ],
        s=90,
        marker="v",
        label="Initial entry target",
    )

    axis.scatter(
        final_hole_bottom[
            1
        ],
        final_hole_bottom[
            2
        ],
        s=120,
        marker="*",
        label=(
            "Release position"
            if auto_release
            else "Final placement position"
        ),
    )

    insertion_path = np.asarray(
        [
            align_target_hole_bottom,
            initial_entry_hole_bottom,
            final_release_hole_bottom,
        ],
        dtype=np.float64,
    )

    axis.plot(
        insertion_path[
            :,
            1
        ],
        insertion_path[
            :,
            2
        ],
        "--",
        linewidth=2,
        label="Insertion target path",
    )

    axis.set_xlabel(
        "base_link Y [m]"
    )

    axis.set_ylabel(
        "base_link Z [m]"
    )

    axis.set_title(
        "Nut Assembly - Side View (Y-Z)"
    )

    axis.grid(
        True,
        alpha=0.25,
    )

    axis.legend(
        loc="best"
    )

    figure.tight_layout()

    figure.savefig(
        sideview_path,
        dpi=180,
    )

    plt.close(
        figure
    )

    return {
        "points_json": (
            points_json_path
        ),

        "front_overlay": (
            front_overlay_path
        ),

        "topdown": (
            topdown_path
        ),

        "sideview": (
            sideview_path
        ),

        "predicted_object_cloud": (
            predicted_object_cloud_path
        ),
    }