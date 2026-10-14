from __future__ import annotations

import argparse
import json

import numpy as np
import trimesh
from scipy.spatial import cKDTree

from graspgenx.serving.zmq_client import GraspGenXClient
from graspgenx.x_grippers import (
    resolve_gripper_info,
)


def main() -> None:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--action",
        choices=(
            "health",
            "infer",
            "infer_object",
            "collision_filter",
        ),
        required=True,
    )

    parser.add_argument(
        "--host",
        default="127.0.0.1",
    )

    parser.add_argument(
        "--port",
        type=int,
        default=8782,
    )

    parser.add_argument(
        "--input",
        default=None,
    )

    parser.add_argument(
        "--output",
        default=None,
    )

    parser.add_argument(
        "--gripper-name",
        default="robotiq_2f_85",
    )

    parser.add_argument(
        "--planner",
        choices=(
            "graspmoe",
            "diffusion",
        ),
        default="graspmoe",
    )

    parser.add_argument(
        "--num-grasps",
        type=int,
        default=200,
    )

    parser.add_argument(
        "--grasp-threshold",
        type=float,
        default=-1.0,
    )

    parser.add_argument(
        "--topk-num-grasps",
        type=int,
        default=100,
    )

    parser.add_argument(
        "--obb-density",
        choices=(
            "sparse",
            "dense",
            "dense-topandside",
        ),
        default="sparse",
    )

    parser.add_argument(
        "--obb-position-spacing-m",
        type=float,
        default=0.01,
    )

    parser.add_argument(
        "--collision-threshold",
        type=float,
        default=0.002,
    )

    parser.add_argument(
        "--target-collision-threshold",
        type=float,
        default=0.002,
    )

    parser.add_argument(
        "--num-collision-samples",
        type=int,
        default=2000,
    )

    # Legacy compatibility argument. Full-resolution collision mode
    # intentionally ignores this value and keeps every scene point.
    parser.add_argument(
        "--max-scene-points",
        type=int,
        default=8192,
    )

    args = parser.parse_args()

    with GraspGenXClient(
        host=args.host,
        port=args.port,
    ) as client:

        if args.action == "health":
            result = client.health()
            print(json.dumps(result))
            return

        if args.input is None:
            raise ValueError(
                "--input is required for inference."
            )

        if args.output is None:
            raise ValueError(
                "--output is required for inference."
            )

        if args.action == "collision_filter":

            with np.load(
                args.input,
                allow_pickle=False,
            ) as data:

                grasps = np.asarray(
                    data["grasps"],
                    dtype=np.float32,
                )

                background_scene_point_cloud = np.asarray(
                    data[
                        "background_scene_point_cloud"
                    ],
                    dtype=np.float32,
                )

                target_point_cloud = np.asarray(
                    data["target_point_cloud"],
                    dtype=np.float32,
                )

                extents_open = np.asarray(
                    data["extents_open"],
                    dtype=np.float32,
                )

                offset_open = np.asarray(
                    data["offset_open"],
                    dtype=np.float32,
                )

            if (
                grasps.ndim != 3
                or grasps.shape[1:] != (4, 4)
            ):
                raise ValueError(
                    "grasps must have shape (K, 4, 4)."
                )

            # Resolve the real named Robotiq asset.
            gripper = resolve_gripper_info(
                args.gripper_name
            )

            # Prefer the official OPEN gripper point cloud already shipped
            # with the gripper description. This is exactly the geometry
            # state used while approaching an object and avoids depending
            # on coll_mesh.obj being a valid triangle mesh.
            gripper_surface_points = np.asarray(
                gripper.open_pointcloud,
                dtype=np.float32,
            )

            valid_open_pc = (
                gripper_surface_points.ndim == 2
                and gripper_surface_points.shape[1] == 3
                and len(gripper_surface_points) >= 10
                and np.isfinite(
                    gripper_surface_points
                ).all()
                and float(
                    np.max(
                        np.ptp(
                            gripper_surface_points,
                            axis=0,
                        )
                    )
                ) > 1e-4
            )

            if valid_open_pc:
                # Deterministic downsampling to the requested number
                # of collision samples.
                if (
                    len(gripper_surface_points)
                    > int(args.num_collision_samples)
                ):
                    rng = np.random.default_rng(
                        42
                    )

                    keep_indices = rng.choice(
                        len(gripper_surface_points),
                        size=int(
                            args.num_collision_samples
                        ),
                        replace=False,
                    )

                    gripper_surface_points = (
                        gripper_surface_points[
                            keep_indices
                        ]
                    )

                print(
                    "[COLLISION] Using official open gripper "
                    f"point cloud: {len(gripper_surface_points)} points"
                )

            else:
                # Fallback only if points.json is unavailable/invalid.
                collision_mesh = (
                    gripper.collision_mesh
                )

                faces = np.asarray(
                    getattr(
                        collision_mesh,
                        "faces",
                        np.empty(
                            (0, 3),
                            dtype=np.int64,
                        ),
                    )
                )

                if (
                    faces.ndim != 2
                    or len(faces) == 0
                ):
                    raise RuntimeError(
                        "Robotiq collision geometry is unavailable: "
                        "both open_pointcloud and coll_mesh.obj are "
                        "invalid/empty."
                    )

                np.random.seed(
                    42
                )

                gripper_surface_points, _ = (
                    trimesh.sample.sample_surface(
                        collision_mesh,
                        int(
                            args.num_collision_samples
                        ),
                    )
                )

                gripper_surface_points = np.asarray(
                    gripper_surface_points,
                    dtype=np.float32,
                )

                print(
                    "[COLLISION] Falling back to sampled collision mesh: "
                    f"{len(gripper_surface_points)} points"
                )

            # --------------------------------------------------
            # A) Background / other-object collision.
            #
            # Keep the FULL scene point cloud. Building one KD-tree
            # over the complete eye-in-hand depth geometry lets us
            # query every candidate efficiently without randomly
            # dropping thin obstacles such as the ring body.
            # --------------------------------------------------
            background_cloud = np.asarray(
                background_scene_point_cloud,
                dtype=np.float64,
            )

            background_free = np.ones(
                len(grasps),
                dtype=bool,
            )

            background_threshold = float(
                args.collision_threshold
            )

            print(
                "[COLLISION] Full-resolution background cloud: "
                f"{len(background_cloud)} points"
            )

            if len(background_cloud) > 0:
                background_tree = cKDTree(
                    background_cloud
                )

                gripper_surface_points_64 = np.asarray(
                    gripper_surface_points,
                    dtype=np.float64,
                )

                for index, grasp in enumerate(
                    grasps
                ):
                    rotation = np.asarray(
                        grasp[
                            :3,
                            :3,
                        ],
                        dtype=np.float64,
                    )

                    translation = np.asarray(
                        grasp[
                            :3,
                            3,
                        ],
                        dtype=np.float64,
                    )

                    # gripper-local -> base/world
                    gripper_points_base = (
                        gripper_surface_points_64
                        @ rotation.T
                        + translation
                    )

                    nearest_distance, _ = (
                        background_tree.query(
                            gripper_points_base,
                            k=1,
                        )
                    )

                    if np.any(
                        nearest_distance
                        < background_threshold
                    ):
                        background_free[
                            index
                        ] = False

            # --------------------------------------------------
            # B) Collision with another part of the SAME target.
            #
            # Target geometry inside the intended open sweep/contact
            # volume is legitimate. Target points outside that volume
            # remain obstacles.
            # --------------------------------------------------
            sweep_min = (
                offset_open
                - extents_open / 2.0
            )

            sweep_max = (
                offset_open
                + extents_open / 2.0
            )

            target_free = np.ones(
                len(grasps),
                dtype=bool,
            )

            target_threshold = float(
                args.target_collision_threshold
            )

            for index, grasp in enumerate(
                grasps
            ):
                rotation = np.asarray(
                    grasp[
                        :3,
                        :3,
                    ],
                    dtype=np.float64,
                )

                translation = np.asarray(
                    grasp[
                        :3,
                        3,
                    ],
                    dtype=np.float64,
                )

                # base/world -> gripper-local
                target_local = (
                    target_point_cloud.astype(
                        np.float64
                    )
                    - translation
                ) @ rotation

                inside_allowed_sweep = np.all(
                    (
                        target_local
                        >= sweep_min
                    )
                    & (
                        target_local
                        <= sweep_max
                    ),
                    axis=1,
                )

                target_obstacles = (
                    target_local[
                        ~inside_allowed_sweep
                    ]
                )

                if len(target_obstacles) == 0:
                    continue

                obstacle_tree = cKDTree(
                    target_obstacles
                )

                nearest_distance, _ = (
                    obstacle_tree.query(
                        gripper_surface_points,
                        k=1,
                    )
                )

                if np.any(
                    nearest_distance
                    < target_threshold
                ):
                    target_free[
                        index
                    ] = False

            collision_free = (
                background_free
                & target_free
            )

            np.savez_compressed(
                args.output,
                collision_free=(
                    collision_free
                ),
                background_free=(
                    background_free
                ),
                target_free=(
                    target_free
                ),
            )

            print(
                "[COLLISION] background-free: "
                f"{int(background_free.sum())}/"
                f"{len(background_free)}"
            )

            print(
                "[COLLISION] target-self-free: "
                f"{int(target_free.sum())}/"
                f"{len(target_free)}"
            )

            print(
                "[COLLISION] combined-free: "
                f"{int(collision_free.sum())}/"
                f"{len(collision_free)}"
            )

            return

        with np.load(
            args.input,
            allow_pickle=False,
        ) as data:

            point_cloud = np.asarray(
                data["point_cloud"],
                dtype=np.float32,
            )

            sweep_volume = None

            if args.action == "infer_object":

                required_keys = (
                    "extents_open",
                    "offset_open",
                    "extents_mid",
                    "offset_mid",
                )

                missing = [
                    key
                    for key in required_keys
                    if key not in data.files
                ]

                if missing:
                    raise ValueError(
                        "Missing sweep-volume fields: "
                        f"{missing}"
                    )

                sweep_volume = {
                    "extents_open": np.asarray(
                        data["extents_open"],
                        dtype=np.float32,
                    ),
                    "offset_open": np.asarray(
                        data["offset_open"],
                        dtype=np.float32,
                    ),
                    "extents_mid": np.asarray(
                        data["extents_mid"],
                        dtype=np.float32,
                    ),
                    "offset_mid": np.asarray(
                        data["offset_mid"],
                        dtype=np.float32,
                    ),
                }

                if "gripper_type" in data.files:
                    sweep_volume["gripper_type"] = int(
                        np.asarray(
                            data["gripper_type"]
                        ).item()
                    )

                if "fingertip_depth" in data.files:
                    sweep_volume["fingertip_depth"] = float(
                        np.asarray(
                            data["fingertip_depth"]
                        ).item()
                    )

        if args.action == "infer":

            grasps, confidences = client.infer(
                point_cloud,
                gripper_name=args.gripper_name,
                num_grasps=args.num_grasps,
                grasp_threshold=args.grasp_threshold,
                topk_num_grasps=args.topk_num_grasps,
            )

            branch_tags = np.empty(
                (0,),
                dtype="<U8",
            )

        else:

            grasps, confidences, branch_tags = (
                client.infer_object(
                    point_cloud=point_cloud,
                    sweep_volume_params=sweep_volume,
                    planner=args.planner,
                    num_grasps=args.num_grasps,
                    grasp_threshold=args.grasp_threshold,
                    topk_num_grasps=args.topk_num_grasps,
                    return_branch_tags=True,

                    # Official GraspMoE planner kwargs.
                    moe_obb_density=(
                        args.obb_density
                    ),
                    moe_obb_position_spacing_cm=(
                        args.obb_position_spacing_m
                        * 100.0
                    ),
                )
            )

            branch_tags = np.asarray(
                branch_tags,
                dtype="<U8",
            )

        np.savez_compressed(
            args.output,
            grasps=np.asarray(
                grasps,
                dtype=np.float32,
            ),
            confidences=np.asarray(
                confidences,
                dtype=np.float32,
            ),
            branch_tags=branch_tags,
        )


if __name__ == "__main__":
    main()
