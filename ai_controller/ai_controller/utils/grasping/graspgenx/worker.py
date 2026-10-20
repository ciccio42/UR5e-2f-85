from __future__ import annotations

import argparse
import json

import numpy as np

from graspgenx.serving.zmq_client import GraspGenXClient


def main() -> None:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--action",
        choices=(
            "health",
            "infer",
            "infer_object",
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
