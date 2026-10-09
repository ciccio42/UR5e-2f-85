from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


# ---------------------------------------------------------------------
# M2T2 canonical grasp geometry.
#
# From the official M2T2 visualize_grasp() representation:
#
#   X -> closing direction
#   Z -> approach direction
#
# The front control points are approximately 105 mm in front
# of the raw M2T2 grasp-frame origin.
# ---------------------------------------------------------------------

M2T2_FINGER_BACK_Z_M = 0.05900000
M2T2_FINGER_FRONT_Z_M = 0.10527314

M2T2_HALF_WIDTH_M = 0.05268743

# Official M2T2 RLBench adapter uses 0.1034 m as gripper depth.
M2T2_REFERENCE_DEPTH_M = 0.1034


# Legacy SeeDo TCP orientation, used only as a diagnostic reference.
LEGACY_GRASP_QUAT_XYZW = np.array(
    [
        0.9994452044624775,
        0.03161651380119412,
        0.0021438049655468088,
        0.010251021036213035,
    ],
    dtype=np.float64,
)


def quaternion_to_rotation(
    quaternion: np.ndarray,
) -> np.ndarray:
    q = np.asarray(
        quaternion,
        dtype=np.float64,
    )

    if q.shape != (4,):
        raise ValueError(
            f"Quaternion must have shape (4,), got {q.shape}."
        )

    norm = np.linalg.norm(q)

    if norm < 1e-12:
        raise ValueError(
            "Quaternion has near-zero norm."
        )

    x, y, z, w = q / norm

    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def rotation_angle_deg(
    R_a: np.ndarray,
    R_b: np.ndarray,
) -> float:
    """
    Full 3D angular difference between two rotation matrices.
    """

    R_relative = (
        np.asarray(
            R_a,
            dtype=np.float64,
        ).T
        @ np.asarray(
            R_b,
            dtype=np.float64,
        )
    )

    cosine = (
        np.trace(R_relative) - 1.0
    ) / 2.0

    cosine = np.clip(
        cosine,
        -1.0,
        1.0,
    )

    return float(
        np.degrees(
            np.arccos(cosine)
        )
    )


def transform_point(
    transform: np.ndarray,
    point: np.ndarray,
) -> np.ndarray:
    point = np.asarray(
        point,
        dtype=np.float64,
    )

    return (
        transform[:3, :3]
        @ point
        + transform[:3, 3]
    )


def translated_along_local_z(
    transform: np.ndarray,
    distance: float,
) -> np.ndarray:
    """
    Return a transform whose origin is translated along the
    local +Z axis of the supplied transform.

    Orientation remains unchanged.
    """

    result = np.asarray(
        transform,
        dtype=np.float64,
    ).copy()

    result[:3, 3] += (
        float(distance)
        * result[:3, 2]
    )

    return result


def load_grasp_plan(
    path: str | Path,
) -> tuple[dict, np.ndarray]:
    path = (
        Path(path)
        .expanduser()
        .resolve()
    )

    if not path.is_file():
        raise FileNotFoundError(
            f"Grasp plan does not exist: {path}"
        )

    with path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        data = json.load(stream)

    if "grasp_pose_base" not in data:
        raise ValueError(
            "grasp_plan.json does not contain grasp_pose_base."
        )

    grasp_pose = np.asarray(
        data["grasp_pose_base"],
        dtype=np.float64,
    )

    if grasp_pose.shape != (4, 4):
        raise ValueError(
            "grasp_pose_base must have shape (4, 4), "
            f"got {grasp_pose.shape}."
        )

    if not np.all(
        np.isfinite(grasp_pose)
    ):
        raise ValueError(
            "grasp_pose_base contains non-finite values."
        )

    return data, grasp_pose


def validate_rotation(
    rotation: np.ndarray,
) -> tuple[float, float]:
    identity_error = float(
        np.linalg.norm(
            rotation.T
            @ rotation
            - np.eye(3)
        )
    )

    determinant = float(
        np.linalg.det(rotation)
    )

    return (
        identity_error,
        determinant,
    )


def draw_frame(
    ax,
    transform: np.ndarray,
    length: float,
    label: str,
) -> None:
    origin = transform[
        :3,
        3,
    ]

    rotation = transform[
        :3,
        :3,
    ]

    # X
    ax.plot(
        [
            origin[0],
            origin[0]
            + length * rotation[0, 0],
        ],
        [
            origin[1],
            origin[1]
            + length * rotation[1, 0],
        ],
        [
            origin[2],
            origin[2]
            + length * rotation[2, 0],
        ],
        "r-",
    )

    # Y
    ax.plot(
        [
            origin[0],
            origin[0]
            + length * rotation[0, 1],
        ],
        [
            origin[1],
            origin[1]
            + length * rotation[1, 1],
        ],
        [
            origin[2],
            origin[2]
            + length * rotation[2, 1],
        ],
        "g-",
    )

    # Z
    ax.plot(
        [
            origin[0],
            origin[0]
            + length * rotation[0, 2],
        ],
        [
            origin[1],
            origin[1]
            + length * rotation[1, 2],
        ],
        [
            origin[2],
            origin[2]
            + length * rotation[2, 2],
        ],
        "b-",
    )

    ax.scatter(
        origin[0],
        origin[1],
        origin[2],
        marker="o",
        s=35,
        label=label,
    )


def draw_m2t2_gripper(
    ax,
    T_base_grasp: np.ndarray,
) -> None:
    """
    Draw the same simplified Y-shaped gripper geometry used
    by the official M2T2 visualization.
    """

    local_points = np.array(
        [
            [
                M2T2_HALF_WIDTH_M,
                0.0,
                M2T2_FINGER_FRONT_Z_M,
            ],
            [
                M2T2_HALF_WIDTH_M,
                0.0,
                M2T2_FINGER_BACK_Z_M,
            ],
            [
                0.0,
                0.0,
                M2T2_FINGER_BACK_Z_M,
            ],
            [
                0.0,
                0.0,
                0.0,
            ],
            [
                0.0,
                0.0,
                M2T2_FINGER_BACK_Z_M,
            ],
            [
                -M2T2_HALF_WIDTH_M,
                0.0,
                M2T2_FINGER_BACK_Z_M,
            ],
            [
                -M2T2_HALF_WIDTH_M,
                0.0,
                M2T2_FINGER_FRONT_Z_M,
            ],
        ],
        dtype=np.float64,
    )

    world_points = np.asarray(
        [
            transform_point(
                T_base_grasp,
                point,
            )
            for point in local_points
        ],
        dtype=np.float64,
    )

    ax.plot(
        world_points[:, 0],
        world_points[:, 1],
        world_points[:, 2],
        "k-",
        linewidth=2,
        label="M2T2 gripper model",
    )


def set_axes_equal(
    ax,
) -> None:
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(
        x_limits[1]
        - x_limits[0]
    )

    y_range = abs(
        y_limits[1]
        - y_limits[0]
    )

    z_range = abs(
        z_limits[1]
        - z_limits[0]
    )

    max_range = max(
        x_range,
        y_range,
        z_range,
    )

    x_middle = sum(
        x_limits
    ) / 2.0

    y_middle = sum(
        y_limits
    ) / 2.0

    z_middle = sum(
        z_limits
    ) / 2.0

    half = max_range / 2.0

    ax.set_xlim3d(
        x_middle - half,
        x_middle + half,
    )

    ax.set_ylim3d(
        y_middle - half,
        y_middle + half,
    )

    ax.set_zlim3d(
        z_middle - half,
        z_middle + half,
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Offline diagnostic for the M2T2 grasp-frame "
            "convention versus the robot TCP convention."
        )
    )

    parser.add_argument(
        "--grasp-plan",
        required=True,
        help="Path to grasp_plan.json.",
    )

    parser.add_argument(
        "--output-dir",
        default=(
            ".runtime/grasp_frame_test"
        ),
    )

    parser.add_argument(
        "--reference-depth-m",
        type=float,
        default=M2T2_REFERENCE_DEPTH_M,
        help=(
            "Distance along M2T2 local +Z used as a "
            "candidate TCP/contact-plane offset."
        ),
    )

    args = parser.parse_args()

    output_dir = (
        Path(args.output_dir)
        .expanduser()
        .resolve()
    )

    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    (
        grasp_data,
        T_base_grasp,
    ) = load_grasp_plan(
        args.grasp_plan
    )

    R_base_grasp = (
        T_base_grasp[
            :3,
            :3,
        ]
    )

    p_base_grasp = (
        T_base_grasp[
            :3,
            3,
        ]
    )

    (
        orthogonality_error,
        determinant,
    ) = validate_rotation(
        R_base_grasp
    )

    # -------------------------------------------------------------
    # Candidate interpretations
    # -------------------------------------------------------------

    # Hypothesis A:
    # Raw M2T2 origin is directly the robot TCP.
    T_base_tcp_direct = (
        T_base_grasp.copy()
    )

    # Hypothesis B:
    # Robot TCP/contact point lies approximately one canonical
    # M2T2 gripper depth in front of the raw grasp origin.
    T_base_tcp_depth = (
        translated_along_local_z(
            T_base_grasp,
            args.reference_depth_m,
        )
    )

    # Front of the simplified M2T2 fingers.
    T_base_m2t2_front = (
        translated_along_local_z(
            T_base_grasp,
            M2T2_FINGER_FRONT_Z_M,
        )
    )

    direct_to_front_distance = float(
        np.linalg.norm(
            T_base_tcp_direct[
                :3,
                3,
            ]
            - T_base_m2t2_front[
                :3,
                3,
            ]
        )
    )

    depth_to_front_distance = float(
        np.linalg.norm(
            T_base_tcp_depth[
                :3,
                3,
            ]
            - T_base_m2t2_front[
                :3,
                3,
            ]
        )
    )

    # -------------------------------------------------------------
    # Compare orientation with the old SeeDo top-down TCP.
    # This is diagnostic only.
    # -------------------------------------------------------------

    legacy_rotation = (
        quaternion_to_rotation(
            LEGACY_GRASP_QUAT_XYZW
        )
    )

    orientation_difference_deg = (
        rotation_angle_deg(
            legacy_rotation,
            R_base_grasp,
        )
    )

    legacy_approach = (
        legacy_rotation[
            :,
            2,
        ]
    )

    m2t2_approach = (
        R_base_grasp[
            :,
            2,
        ]
    )

    approach_cosine = float(
        np.clip(
            np.dot(
                legacy_approach,
                m2t2_approach,
            ),
            -1.0,
            1.0,
        )
    )

    approach_difference_deg = float(
        np.degrees(
            np.arccos(
                approach_cosine
            )
        )
    )

    # -------------------------------------------------------------
    # Console report
    # -------------------------------------------------------------

    print()
    print(
        "============================================================"
    )
    print(
        "M2T2 GRASP FRAME OFFLINE CHECK"
    )
    print(
        "============================================================"
    )

    print()
    print(
        "[INPUT] grasp_plan:",
        Path(
            args.grasp_plan
        ).resolve(),
    )

    print()
    print(
        "[M2T2] Raw grasp origin [base]:"
    )
    print(
        p_base_grasp
    )

    print()
    print(
        "[M2T2] Local X / closing direction:"
    )
    print(
        R_base_grasp[:, 0]
    )

    print(
        "[M2T2] Local Y:"
    )
    print(
        R_base_grasp[:, 1]
    )

    print(
        "[M2T2] Local Z / approach direction:"
    )
    print(
        R_base_grasp[:, 2]
    )

    print()
    print(
        "[CHECK] Rotation orthogonality error:",
        f"{orthogonality_error:.8e}",
    )

    print(
        "[CHECK] Rotation determinant:",
        f"{determinant:.8f}",
    )

    print()
    print(
        "[LEGACY TCP] Full orientation difference:",
        f"{orientation_difference_deg:.3f} deg",
    )

    print(
        "[LEGACY TCP] Approach-axis difference:",
        f"{approach_difference_deg:.3f} deg",
    )

    print()
    print(
        "[HYPOTHESIS A] raw M2T2 origin == tcp_link"
    )

    print(
        "  candidate TCP:",
        T_base_tcp_direct[
            :3,
            3,
        ],
    )

    print(
        "  distance from M2T2 finger front:",
        f"{direct_to_front_distance * 1000.0:.2f} mm",
    )

    print()
    print(
        "[HYPOTHESIS B] TCP shifted along M2T2 +Z by",
        f"{args.reference_depth_m * 1000.0:.2f} mm",
    )

    print(
        "  candidate TCP:",
        T_base_tcp_depth[
            :3,
            3,
        ],
    )

    print(
        "  distance from M2T2 finger front:",
        f"{depth_to_front_distance * 1000.0:.2f} mm",
    )

    print()
    print(
        "[M2T2] Simplified finger-front center:"
    )
    print(
        T_base_m2t2_front[
            :3,
            3,
        ]
    )

    # -------------------------------------------------------------
    # Save JSON diagnostic
    # -------------------------------------------------------------

    report = {
        "grasp_plan": str(
            Path(
                args.grasp_plan
            ).resolve()
        ),
        "selected_m2t2_index": (
            grasp_data.get(
                "selected_m2t2_index"
            )
        ),
        "raw_grasp_pose_base": (
            T_base_grasp.tolist()
        ),
        "raw_grasp_origin_base": (
            p_base_grasp.tolist()
        ),
        "m2t2_axes_base": {
            "x_closing": (
                R_base_grasp[
                    :,
                    0,
                ].tolist()
            ),
            "y": (
                R_base_grasp[
                    :,
                    1,
                ].tolist()
            ),
            "z_approach": (
                R_base_grasp[
                    :,
                    2,
                ].tolist()
            ),
        },
        "rotation_orthogonality_error": (
            orthogonality_error
        ),
        "rotation_determinant": (
            determinant
        ),
        "legacy_tcp_orientation_difference_deg": (
            orientation_difference_deg
        ),
        "legacy_tcp_approach_difference_deg": (
            approach_difference_deg
        ),
        "m2t2_finger_front_z_m": (
            M2T2_FINGER_FRONT_Z_M
        ),
        "reference_depth_m": (
            args.reference_depth_m
        ),
        "candidate_tcp_direct": (
            T_base_tcp_direct.tolist()
        ),
        "candidate_tcp_depth_shifted": (
            T_base_tcp_depth.tolist()
        ),
        "m2t2_finger_front_pose": (
            T_base_m2t2_front.tolist()
        ),
        "direct_to_finger_front_distance_m": (
            direct_to_front_distance
        ),
        "depth_shift_to_finger_front_distance_m": (
            depth_to_front_distance
        ),
    }

    report_path = (
        output_dir
        / "frame_check.json"
    )

    with report_path.open(
        "w",
        encoding="utf-8",
    ) as stream:
        json.dump(
            report,
            stream,
            indent=2,
        )

    # -------------------------------------------------------------
    # 3D visualization
    # -------------------------------------------------------------

    fig = plt.figure(
        figsize=(
            10,
            8,
        )
    )

    ax = fig.add_subplot(
        111,
        projection="3d",
    )

    draw_m2t2_gripper(
        ax,
        T_base_grasp,
    )

    draw_frame(
        ax,
        T_base_grasp,
        length=0.04,
        label="M2T2 origin",
    )

    direct = (
        T_base_tcp_direct[
            :3,
            3,
        ]
    )

    depth_tcp = (
        T_base_tcp_depth[
            :3,
            3,
        ]
    )

    finger_front = (
        T_base_m2t2_front[
            :3,
            3,
        ]
    )

    ax.scatter(
        direct[0],
        direct[1],
        direct[2],
        marker="x",
        s=90,
        label="TCP if identity",
    )

    ax.scatter(
        depth_tcp[0],
        depth_tcp[1],
        depth_tcp[2],
        marker="^",
        s=70,
        label=(
            "TCP after canonical "
            f"{args.reference_depth_m * 1000:.1f} mm shift"
        ),
    )

    ax.scatter(
        finger_front[0],
        finger_front[1],
        finger_front[2],
        marker="s",
        s=70,
        label="M2T2 finger-front center",
    )

    ax.set_xlabel(
        "base X [m]"
    )

    ax.set_ylabel(
        "base Y [m]"
    )

    ax.set_zlabel(
        "base Z [m]"
    )

    ax.set_title(
        "M2T2 grasp-frame / TCP convention check"
    )

    ax.legend()

    set_axes_equal(
        ax
    )

    fig.tight_layout()

    plot_path = (
        output_dir
        / "frame_check.png"
    )

    fig.savefig(
        plot_path,
        dpi=200,
    )

    plt.close(
        fig
    )

    print()
    print(
        "[OUTPUT]",
        report_path,
    )

    print(
        "[OUTPUT]",
        plot_path,
    )

    print()
    print(
        "PASS: offline grasp-frame diagnostic completed."
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(
        main()
    )