from __future__ import annotations

from pathlib import Path

import numpy as np

from ai_controller.utils.grasping.graspgenx.client import (
    GraspGenXRemoteClient,
)


REPO_ROOT = Path(
    "/home/ros2_ws/src/UR5e-2f-85"
)

RESULT_PATH = (
    REPO_ROOT
    / ".runtime"
    / "graspgenx_pipeline_test"
    / "graspgenx_moe_base_result.npz"
)


ROBOTIQ_SWEEP_VOLUME = {
    "extents_open": [
        0.085,
        0.032,
        0.036,
    ],
    "offset_open": [
        0.0,
        0.0,
        0.130,
    ],
    "extents_mid": [
        0.046,
        0.032,
        0.036,
    ],
    "offset_mid": [
        0.0,
        0.0,
        0.143,
    ],
    "gripper_type": 0,
    "fingertip_depth": 0.136,
}


def test_graspgenx_graspmoe_real_server():
    assert RESULT_PATH.exists(), (
        f"Missing integration-test input: "
        f"{RESULT_PATH}"
    )

    with np.load(
        RESULT_PATH,
        allow_pickle=False,
    ) as data:

        point_cloud_base = np.asarray(
            data[
                "target_point_cloud_base"
            ],
            dtype=np.float32,
        )

    assert point_cloud_base.ndim == 2
    assert point_cloud_base.shape[1] == 3
    assert len(point_cloud_base) > 0
    assert np.isfinite(
        point_cloud_base
    ).all()

    client = GraspGenXRemoteClient(
        host="127.0.0.1",
        port=8782,
        gripper_name="robotiq_2f_85",
    )

    assert client.health(
        timeout=20.0
    )

    (
        grasps,
        confidences,
        branch_tags,
    ) = client.predict_object_grasps(
        point_cloud=point_cloud_base,
        sweep_volume_params=(
            ROBOTIQ_SWEEP_VOLUME
        ),
        planner="graspmoe",
        num_grasps=200,
        grasp_threshold=-1.0,

        # Keep everything.
        topk_num_grasps=-1,

        timeout=180.0,
    )

    # --------------------------------------------------------
    # Generic output validation
    # --------------------------------------------------------

    assert grasps.ndim == 3
    assert grasps.shape[1:] == (
        4,
        4,
    )

    assert confidences.ndim == 1
    assert branch_tags.ndim == 1

    assert len(grasps) > 0

    assert (
        len(grasps)
        == len(confidences)
        == len(branch_tags)
    )

    assert np.isfinite(
        grasps
    ).all()

    assert np.isfinite(
        confidences
    ).all()

    # Homogeneous transforms.
    np.testing.assert_allclose(
        grasps[:, 3, :],
        np.tile(
            np.array(
                [
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                ],
                dtype=np.float32,
            ),
            (
                len(grasps),
                1,
            ),
        ),
        atol=1e-5,
    )

    # --------------------------------------------------------
    # GraspMoE must contain both branches
    # --------------------------------------------------------

    tag_set = set(
        branch_tags.tolist()
    )

    assert "diff" in tag_set
    assert "obb" in tag_set

    diffusion_mask = (
        branch_tags == "diff"
    )

    obb_mask = (
        branch_tags == "obb"
    )

    assert diffusion_mask.any()
    assert obb_mask.any()

    # --------------------------------------------------------
    # OBB branch in BASE frame:
    #
    # official GraspMoE top-down convention:
    # local +Z = -world/base Z.
    # --------------------------------------------------------

    obb_approaches = grasps[
        obb_mask,
        :3,
        2,
    ]

    expected_down = np.tile(
        np.array(
            [
                0.0,
                0.0,
                -1.0,
            ],
            dtype=np.float32,
        ),
        (
            len(obb_approaches),
            1,
        ),
    )

    np.testing.assert_allclose(
        obb_approaches,
        expected_down,
        atol=1e-5,
    )

    # --------------------------------------------------------
    # Rotation matrices must be valid
    # --------------------------------------------------------

    rotations = grasps[
        :,
        :3,
        :3,
    ]

    identity = np.eye(
        3,
        dtype=np.float32,
    )

    for rotation in rotations:
        np.testing.assert_allclose(
            rotation.T @ rotation,
            identity,
            atol=1e-4,
        )

        np.testing.assert_allclose(
            np.linalg.det(
                rotation
            ),
            1.0,
            atol=1e-4,
        )

    # --------------------------------------------------------
    # Basic confidence sanity
    # --------------------------------------------------------

    assert np.all(
        confidences >= 0.0
    )

    assert np.all(
        confidences <= 1.0
    )