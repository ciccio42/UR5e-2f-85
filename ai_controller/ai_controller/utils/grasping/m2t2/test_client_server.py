#!/usr/bin/env python3
"""
Integration test for M2T2Client <-> server.py on a real captured RGB-D scene.

Geometry follows the same conventions used by SeeDo on branch
`angelo_soluzione`:

    camera optical frame
        -> raw ArUco frame
        -> table_0 frame
        -> base_link frame

M2T2 receives the point cloud in table_0 coordinates, where the tabletop
is approximately z = 0.

The test saves:
    - visualization of the workspace passed to M2T2;
    - visualization of M2T2 grasp candidates;
    - complete numerical results as .npz.

Requires the M2T2 server to already be running.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw

from ai_controller.utils.grasping.m2t2.client import M2T2Client


THIS_DIR = Path(__file__).resolve().parent

REPO_ROOT = Path(
    "/home/ros2_ws/src/UR5e-2f-85"
)

DEFAULT_SCENE_DIR = Path(
    "/scene_capture/nut/scena_1"
)

DEFAULT_OUTPUT_DIR = (
    REPO_ROOT
    / ".runtime"
    / "m2t2_tests"
)

DEFAULT_CAMERA_CALIBRATION = (
    REPO_ROOT
    / "zed_camera"
    / "zed_camera_calibration"
    / "estimated_camera_positions.yaml"
)


# Same fixed transform used in:
#
# ai_controller/models/seedo_controller/utils.py
#
# on branch angelo_soluzione.
ARUCO_TO_TABLE0_ROTATION = np.array(
    [
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)

ARUCO_TO_TABLE0_TRANSLATION = np.zeros(
    3,
    dtype=np.float64,
)


# Initial table_0 workspace bounds.
#
# These are deliberately configurable from the command line so we can refine
# them after inspecting the saved workspace visualization.
DEFAULT_WORKSPACE_BOUNDS = {
    "x_min": -0.55,
    "x_max": 0.55,
    "y_min": -0.50,
    "y_max": 0.50,
    "z_min": -0.03,
    "z_max": 0.30,
}


def load_camera_matrix(
    camera_info_path: Path,
) -> np.ndarray:

    with open(
        camera_info_path,
        "r",
        encoding="utf-8",
    ) as stream:
        info = yaml.safe_load(
            stream
        )

    K = np.asarray(
        info["k"],
        dtype=np.float64,
    ).reshape(
        3,
        3,
    )

    return K


def load_camera_calibration(
    calibration_path: Path,
    camera_name: str,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Load camera -> raw ArUco transformation.

    Same convention as SeeDo's camera_point_to_aruco():

        p_aruco = R_aruco_camera @ p_camera + t_aruco_camera
    """

    with open(
        calibration_path,
        "r",
        encoding="utf-8",
    ) as stream:
        calibration = yaml.safe_load(
            stream
        )

    if camera_name not in calibration:
        raise KeyError(
            f"Camera {camera_name!r} not found in "
            f"{calibration_path}"
        )

    entry = calibration[
        camera_name
    ]

    rotation = np.asarray(
        entry["orientation_matrix"],
        dtype=np.float64,
    )

    translation = np.asarray(
        entry["position"],
        dtype=np.float64,
    )

    if rotation.shape != (3, 3):
        raise ValueError(
            f"Invalid camera rotation shape: {rotation.shape}"
        )

    if translation.shape != (3,):
        raise ValueError(
            f"Invalid camera translation shape: {translation.shape}"
        )

    return (
        rotation,
        translation,
    )


def load_table_to_base_transform(
    path: Path,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Load the transform stored by SeeDo as base_to_table_transform.yaml.

    Despite the filename, SeeDo defines it as:

        p_base = R_base_table @ p_table + t_base_table

    i.e. table_0 -> base_link.
    """

    with open(
        path,
        "r",
        encoding="utf-8",
    ) as stream:
        transform = yaml.safe_load(
            stream
        )

    rotation = np.asarray(
        transform["rotation"],
        dtype=np.float64,
    )

    translation = np.asarray(
        transform["translation"],
        dtype=np.float64,
    )

    return (
        rotation,
        translation,
    )


def make_transform(
    rotation: np.ndarray,
    translation: np.ndarray,
) -> np.ndarray:

    transform = np.eye(
        4,
        dtype=np.float64,
    )

    transform[
        :3,
        :3,
    ] = rotation

    transform[
        :3,
        3,
    ] = translation

    return transform


def transform_points(
    points: np.ndarray,
    rotation: np.ndarray,
    translation: np.ndarray,
) -> np.ndarray:
    """
    Apply:

        p_out = R @ p_in + t
    """

    points = np.asarray(
        points,
        dtype=np.float64,
    )

    return (
        points
        @ rotation.T
        + translation
    )


def transform_poses(
    poses: np.ndarray,
    transform: np.ndarray,
) -> np.ndarray:

    poses = np.asarray(
        poses,
        dtype=np.float64,
    )

    if poses.shape[0] == 0:
        return np.empty(
            (0, 4, 4),
            dtype=np.float32,
        )

    transformed = (
        transform[None, ...]
        @ poses
    )

    return transformed.astype(
        np.float32
    )


def depth_to_point_cloud(
    depth: np.ndarray,
    K: np.ndarray,
) -> np.ndarray:
    """
    Deproject the complete valid depth map into the camera optical frame.

    Same pinhole model used by SeeDo's deproject_pixel().
    """

    depth = np.asarray(
        depth,
        dtype=np.float64,
    )

    height, width = depth.shape

    fx = K[0, 0]
    fy = K[1, 1]

    cx = K[0, 2]
    cy = K[1, 2]

    u, v = np.meshgrid(
        np.arange(
            width,
            dtype=np.float64,
        ),
        np.arange(
            height,
            dtype=np.float64,
        ),
        indexing="xy",
    )

    valid = (
        np.isfinite(
            depth
        )
        & (
            depth > 0.0
        )
    )

    z = depth[
        valid
    ]

    x = (
        u[valid]
        - cx
    ) * z / fx

    y = (
        v[valid]
        - cy
    ) * z / fy

    point_cloud = np.stack(
        [
            x,
            y,
            z,
        ],
        axis=1,
    )

    return point_cloud.astype(
        np.float32
    )


def project_points(
    points_camera: np.ndarray,
    K: np.ndarray,
) -> np.ndarray:

    points_camera = np.asarray(
        points_camera,
        dtype=np.float64,
    )

    pixels = np.full(
        (
            points_camera.shape[0],
            2,
        ),
        np.nan,
        dtype=np.float64,
    )

    valid = (
        np.isfinite(
            points_camera
        ).all(axis=1)
        & (
            points_camera[:, 2]
            > 0.0
        )
    )

    if not np.any(
        valid
    ):
        return pixels

    projected = (
        points_camera[
            valid
        ]
        @ K.T
    )

    projected = (
        projected[:, :2]
        / projected[:, 2:3]
    )

    pixels[
        valid
    ] = projected

    return pixels


def crop_workspace(
    point_cloud_table: np.ndarray,
    bounds: dict[str, float],
    surface_range: float,
) -> np.ndarray:
    """
    Crop the table workspace and snap tabletop points to z=0.

    The z snapping mirrors the official M2T2 preprocessing:

        xyz_world[abs(z) < surface_range, 2] = 0
    """

    pc = np.asarray(
        point_cloud_table,
        dtype=np.float32,
    )

    inside = (
        (pc[:, 0] >= bounds["x_min"])
        & (pc[:, 0] <= bounds["x_max"])
        & (pc[:, 1] >= bounds["y_min"])
        & (pc[:, 1] <= bounds["y_max"])
        & (pc[:, 2] >= bounds["z_min"])
        & (pc[:, 2] <= bounds["z_max"])
    )

    cropped = pc[
        inside
    ].copy()

    table_mask = (
        np.abs(
            cropped[:, 2]
        )
        < surface_range
    )

    cropped[
        table_mask,
        2,
    ] = 0.0

    return cropped


def draw_workspace(
    image: Image.Image,
    point_cloud_camera: np.ndarray,
    K: np.ndarray,
) -> Image.Image:

    vis = image.copy()

    draw = ImageDraw.Draw(
        vis
    )

    pixels = project_points(
        point_cloud_camera,
        K,
    )

    # Avoid drawing hundreds of thousands of pixels.
    if pixels.shape[0] > 6000:
        indices = np.linspace(
            0,
            pixels.shape[0] - 1,
            6000,
        ).astype(
            np.int64
        )

        pixels = pixels[
            indices
        ]

    width, height = vis.size

    for x, y in pixels:

        if not (
            np.isfinite(x)
            and np.isfinite(y)
        ):
            continue

        if (
            0 <= x < width
            and 0 <= y < height
        ):
            draw.ellipse(
                (
                    x - 1,
                    y - 1,
                    x + 1,
                    y + 1,
                ),
                fill="lime",
            )

    return vis


def draw_candidates(
    image: Image.Image,
    contacts_camera: np.ndarray,
    confidence: np.ndarray,
    K: np.ndarray,
) -> Image.Image:

    vis = image.copy()

    draw = ImageDraw.Draw(
        vis
    )

    pixels = project_points(
        contacts_camera,
        K,
    )

    width, height = vis.size

    for x, y in pixels:

        if not (
            np.isfinite(x)
            and np.isfinite(y)
        ):
            continue

        if not (
            0 <= x < width
            and 0 <= y < height
        ):
            continue

        radius = 3

        draw.ellipse(
            (
                x - radius,
                y - radius,
                x + radius,
                y + radius,
            ),
            fill="red",
            outline="white",
            width=1,
        )

    if confidence.shape[0] > 0:

        best_idx = int(
            np.argmax(
                confidence
            )
        )

        x, y = pixels[
            best_idx
        ]

        if (
            np.isfinite(x)
            and np.isfinite(y)
            and 0 <= x < width
            and 0 <= y < height
        ):

            size = 12

            draw.line(
                (
                    x - size,
                    y,
                    x + size,
                    y,
                ),
                fill="blue",
                width=3,
            )

            draw.line(
                (
                    x,
                    y - size,
                    x,
                    y + size,
                ),
                fill="blue",
                width=3,
            )

    return vis


def main() -> None:

    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        "--scene-dir",
        default=str(
            DEFAULT_SCENE_DIR
        ),
    )

    parser.add_argument(
        "--camera-calibration",
        default=str(
            DEFAULT_CAMERA_CALIBRATION
        ),
    )

    parser.add_argument(
        "--camera-name",
        default="zed_front",
    )

    parser.add_argument(
        "--config",
        default=str(
            THIS_DIR
            / "config"
            / "m2t2_config.yaml"
        ),
    )

    parser.add_argument(
        "--output-dir",
        default=str(
            DEFAULT_OUTPUT_DIR
        ),
    )

    parser.add_argument(
        "--num-runs",
        type=int,
        default=1,
    )

    parser.add_argument(
        "--surface-range",
        type=float,
        default=0.02,
    )

    parser.add_argument(
        "--x-min",
        type=float,
        default=DEFAULT_WORKSPACE_BOUNDS[
            "x_min"
        ],
    )

    parser.add_argument(
        "--x-max",
        type=float,
        default=DEFAULT_WORKSPACE_BOUNDS[
            "x_max"
        ],
    )

    parser.add_argument(
        "--y-min",
        type=float,
        default=DEFAULT_WORKSPACE_BOUNDS[
            "y_min"
        ],
    )

    parser.add_argument(
        "--y-max",
        type=float,
        default=DEFAULT_WORKSPACE_BOUNDS[
            "y_max"
        ],
    )

    parser.add_argument(
        "--z-min",
        type=float,
        default=DEFAULT_WORKSPACE_BOUNDS[
            "z_min"
        ],
    )

    parser.add_argument(
        "--z-max",
        type=float,
        default=DEFAULT_WORKSPACE_BOUNDS[
            "z_max"
        ],
    )

    args = parser.parse_args()

    scene_dir = Path(
        args.scene_dir
    )

    output_dir = Path(
        args.output_dir
    )

    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    # =====================================================================
    # Load capture
    # =====================================================================

    rgb_path = (
        scene_dir
        / "rgb.png"
    )

    depth_path = (
        scene_dir
        / "depth.npy"
    )

    camera_info_path = (
        scene_dir
        / "camera_info.yaml"
    )

    table_to_base_path = (
        scene_dir
        / "base_to_table_transform.yaml"
    )

    print(
        f"[TEST] Loading RGB: {rgb_path}"
    )

    image = Image.open(
        rgb_path
    ).convert(
        "RGB"
    )

    rgb = np.asarray(
        image,
        dtype=np.uint8,
    )

    depth = np.load(
        depth_path
    ).astype(
        np.float32,
        copy=False,
    )

    K = load_camera_matrix(
        camera_info_path
    )

    print(
        "[TEST] RGB shape:",
        rgb.shape,
    )

    print(
        "[TEST] Depth shape:",
        depth.shape,
    )

    print(
        "[TEST] Camera K:"
    )

    print(
        K
    )

    # =====================================================================
    # Camera -> ArUco
    # =====================================================================

    (
        R_aruco_camera,
        t_aruco_camera,
    ) = load_camera_calibration(
        Path(
            args.camera_calibration
        ),
        args.camera_name,
    )

    print()
    print(
        "[TEST] camera -> ArUco translation:",
        t_aruco_camera,
    )

    print(
        "[TEST] camera -> ArUco rotation:"
    )

    print(
        R_aruco_camera
    )

    # =====================================================================
    # Build camera -> table_0
    # =====================================================================

    R_table_aruco = (
        ARUCO_TO_TABLE0_ROTATION
    )

    t_table_aruco = (
        ARUCO_TO_TABLE0_TRANSLATION
    )

    R_table_camera = (
        R_table_aruco
        @ R_aruco_camera
    )

    t_table_camera = (
        R_table_aruco
        @ t_aruco_camera
        + t_table_aruco
    )

    T_table_camera = make_transform(
        R_table_camera,
        t_table_camera,
    )

    T_camera_table = np.linalg.inv(
        T_table_camera
    )

    # =====================================================================
    # table_0 -> base_link
    # =====================================================================

    (
        R_base_table,
        t_base_table,
    ) = load_table_to_base_transform(
        table_to_base_path
    )

    T_base_table = make_transform(
        R_base_table,
        t_base_table,
    )

    # =====================================================================
    # Backprojection
    # =====================================================================

    point_cloud_camera = (
        depth_to_point_cloud(
            depth,
            K,
        )
    )

    print()
    print(
        "[TEST] Raw camera point cloud:",
        point_cloud_camera.shape,
    )

    # =====================================================================
    # Camera -> table_0
    # =====================================================================

    point_cloud_table = (
        transform_points(
            point_cloud_camera,
            R_table_camera,
            t_table_camera,
        )
        .astype(
            np.float32
        )
    )

    print(
        "[TEST] table_0 raw bounds:"
    )

    print(
        "       min:",
        point_cloud_table.min(
            axis=0
        ),
    )

    print(
        "       max:",
        point_cloud_table.max(
            axis=0
        ),
    )

    # =====================================================================
    # Workspace crop
    # =====================================================================

    bounds = {
        "x_min": args.x_min,
        "x_max": args.x_max,
        "y_min": args.y_min,
        "y_max": args.y_max,
        "z_min": args.z_min,
        "z_max": args.z_max,
    }

    point_cloud_table_crop = (
        crop_workspace(
            point_cloud_table,
            bounds,
            args.surface_range,
        )
    )

    print()
    print(
        "[TEST] Workspace bounds:",
        bounds,
    )

    print(
        "[TEST] Cropped point cloud:",
        point_cloud_table_crop.shape,
    )

    if point_cloud_table_crop.shape[0] == 0:
        raise RuntimeError(
            "Workspace crop removed every point."
        )

    print(
        "[TEST] Cropped bounds:"
    )

    print(
        "       min:",
        point_cloud_table_crop.min(
            axis=0
        ),
    )

    print(
        "       max:",
        point_cloud_table_crop.max(
            axis=0
        ),
    )

    # =====================================================================
    # Save workspace visualization
    # =====================================================================

    point_cloud_crop_camera = (
        transform_points(
            point_cloud_table_crop,
            T_camera_table[
                :3,
                :3,
            ],
            T_camera_table[
                :3,
                3,
            ],
        )
    )

    workspace_vis = draw_workspace(
        image,
        point_cloud_crop_camera,
        K,
    )

    workspace_path = (
        output_dir
        / "scena_1_m2t2_table_workspace.png"
    )

    workspace_vis.save(
        workspace_path
    )

    print(
        "[TEST] Workspace visualization saved to:",
        workspace_path,
    )

    # =====================================================================
    # M2T2
    # =====================================================================

    client = M2T2Client(
        config_path=args.config
    )

    print()
    print(
        "[TEST] Running M2T2 in table_0 frame..."
    )

    start = time.perf_counter()

    (
        grasps_table,
        contacts_table,
        confidence,
    ) = client.predict_grasps(
        point_cloud_table_crop,
        num_runs=args.num_runs,
    )

    elapsed = (
        time.perf_counter()
        - start
    )

    print(
        "[TEST] Grasps:",
        grasps_table.shape,
    )

    print(
        "[TEST] Contacts:",
        contacts_table.shape,
    )

    print(
        "[TEST] Confidence:",
        confidence.shape,
    )

    print(
        f"[TEST] HTTP inference time: "
        f"{elapsed:.3f} s"
    )

    if grasps_table.shape[0] == 0:
        raise RuntimeError(
            "M2T2 inference completed successfully, "
            "but no grasp candidates were predicted."
        )

    # =====================================================================
    # table_0 -> camera
    # =====================================================================

    grasps_camera = transform_poses(
        grasps_table,
        T_camera_table,
    )

    contacts_camera = transform_points(
        contacts_table,
        T_camera_table[
            :3,
            :3,
        ],
        T_camera_table[
            :3,
            3,
        ],
    ).astype(
        np.float32
    )

    # =====================================================================
    # table_0 -> base_link
    # =====================================================================

    grasps_base = transform_poses(
        grasps_table,
        T_base_table,
    )

    contacts_base = transform_points(
        contacts_table,
        R_base_table,
        t_base_table,
    ).astype(
        np.float32
    )

    # =====================================================================
    # Save candidate visualization
    # =====================================================================

    candidate_vis = draw_candidates(
        image,
        contacts_camera,
        confidence,
        K,
    )

    candidate_path = (
        output_dir
        / "scena_1_m2t2_table_frame_candidates.png"
    )

    candidate_vis.save(
        candidate_path
    )

    print(
        "[TEST] Candidate visualization saved to:",
        candidate_path,
    )

    # =====================================================================
    # Save complete numerical result
    # =====================================================================

    npz_path = (
        output_dir
        / "scena_1_m2t2_table_frame_candidates.npz"
    )

    np.savez_compressed(
        npz_path,

        K=K,

        point_cloud_camera=point_cloud_camera,
        point_cloud_table=point_cloud_table,
        point_cloud_table_crop=point_cloud_table_crop,

        grasps_table=grasps_table,
        contacts_table=contacts_table,

        grasps_camera=grasps_camera,
        contacts_camera=contacts_camera,

        grasps_base=grasps_base,
        contacts_base=contacts_base,

        confidence=confidence,

        T_table_camera=T_table_camera,
        T_camera_table=T_camera_table,
        T_base_table=T_base_table,

        workspace_bounds=np.asarray(
            [
                bounds["x_min"],
                bounds["x_max"],
                bounds["y_min"],
                bounds["y_max"],
                bounds["z_min"],
                bounds["z_max"],
            ],
            dtype=np.float32,
        ),
    )

    print(
        "[TEST] Numerical results saved to:",
        npz_path,
    )

    # =====================================================================
    # Highest geometric-confidence grasp
    # =====================================================================

    best_idx = int(
        np.argmax(
            confidence
        )
    )

    print()
    print(
        "[TEST] Highest-confidence candidate:"
    )

    print(
        "       index:",
        best_idx,
    )

    print(
        "       confidence:",
        float(
            confidence[
                best_idx
            ]
        ),
    )

    print(
        "       contact table_0:",
        contacts_table[
            best_idx
        ],
    )

    print(
        "       contact camera:",
        contacts_camera[
            best_idx
        ],
    )

    print(
        "       contact base:",
        contacts_base[
            best_idx
        ],
    )

    print()
    print(
        "[PASS] M2T2 real-scene inference "
        "produced grasp candidates."
    )


if __name__ == "__main__":
    main()