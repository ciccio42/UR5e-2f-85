from __future__ import annotations

import numpy as np

from ai_controller.utils.grasping.graspgenx.planner import (
    GraspGenXPlanner,
)

import pytest

from scipy.spatial.transform import Rotation


def test_generate_grasp_candidates_base(
    monkeypatch,
):
    planner = GraspGenXPlanner()

    target_camera = np.array(
        [
            [1.0, 2.0, 3.0],
            [4.0, 5.0, 6.0],
        ],
        dtype=np.float32,
    )

    T_base_camera = np.eye(
        4,
        dtype=np.float64,
    )

    T_base_camera[
        :3,
        3,
    ] = [
        10.0,
        20.0,
        30.0,
    ]

    expected_base = np.array(
        [
            [11.0, 22.0, 33.0],
            [14.0, 25.0, 36.0],
        ],
        dtype=np.float32,
    )

    fake_grasps = np.stack(
        [
            np.eye(
                4,
                dtype=np.float32,
            ),
            np.eye(
                4,
                dtype=np.float32,
            ),
        ],
        axis=0,
    )

    fake_confidences = np.array(
        [
            0.8,
            0.6,
        ],
        dtype=np.float32,
    )

    fake_tags = np.array(
        [
            "diff",
            "obb",
        ]
    )

    def fake_predict_object_grasps(
        *,
        point_cloud,
        sweep_volume_params,
        planner,
        num_grasps,
        grasp_threshold,
        topk_num_grasps,
        timeout,
    ):
        np.testing.assert_allclose(
            point_cloud,
            expected_base,
        )

        assert planner == "graspmoe"
        assert num_grasps == 200
        assert grasp_threshold == -1.0
        assert topk_num_grasps == -1
        assert timeout == 180.0

        assert (
            sweep_volume_params[
                "fingertip_depth"
            ]
            == 0.136
        )

        return (
            fake_grasps,
            fake_confidences,
            fake_tags,
        )

    monkeypatch.setattr(
        planner.client,
        "predict_object_grasps",
        fake_predict_object_grasps,
    )

    (
        grasps,
        confidences,
        tags,
        target_base,
    ) = planner._generate_grasp_candidates_base(
        target_point_cloud_camera=(
            target_camera
        ),
        T_base_camera=T_base_camera,
    )

    np.testing.assert_allclose(
        target_base,
        expected_base,
    )

    np.testing.assert_allclose(
        grasps,
        fake_grasps,
    )

    np.testing.assert_allclose(
        confidences,
        fake_confidences,
    )

    np.testing.assert_array_equal(
        tags,
        fake_tags,
    )

def test_plan_end_to_end_unit(
    monkeypatch,
):
    planner = GraspGenXPlanner()

    rgb = np.zeros(
        (4, 4, 3),
        dtype=np.uint8,
    )

    depth = np.full(
        (4, 4),
        0.20,
        dtype=np.float32,
    )

    K = np.array(
        [
            [100.0, 0.0, 1.5],
            [0.0, 100.0, 1.5],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

    T_base_camera = np.eye(
        4,
        dtype=np.float64,
    )

    T_base_table = np.eye(
        4,
        dtype=np.float64,
    )

    semantic_point = np.array(
        [1.0, 1.0],
        dtype=np.float32,
    )

    mask = np.zeros(
        (4, 4),
        dtype=bool,
    )

    mask[
        1:3,
        1:3,
    ] = True

    monkeypatch.setattr(
        planner,
        "_predict_semantic_point",
        lambda **kwargs: (
            semantic_point
        ),
    )

    monkeypatch.setattr(
        planner,
        "_segment_target",
        lambda **kwargs: mask,
    )

    # --------------------------------------------------------
    # Safe top-down OBB grasp.
    #
    # gripper base z = 0.20
    # tcp z = 0.20 - 0.136 = 0.064
    #
    # bbox lowest point:
    # 0.20 - 0.149074 ~= 0.0509 m
    # safely above the z=0 table.
    # --------------------------------------------------------

    grasp_obb = np.eye(
        4,
        dtype=np.float32,
    )

    grasp_obb[
        :3,
        :3,
    ] = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
        ],
        dtype=np.float32,
    )

    grasp_obb[
        :3,
        3,
    ] = [
        0.01,
        0.02,
        0.20,
    ]

    # Unsafe higher-confidence diffusion grasp.
    grasp_diff = grasp_obb.copy()

    grasp_diff[
        2,
        3,
    ] = 0.10

    fake_grasps = np.stack(
        [
            grasp_diff,
            grasp_obb,
        ],
        axis=0,
    )

    fake_confidences = np.array(
        [
            0.95,
            0.70,
        ],
        dtype=np.float32,
    )

    fake_tags = np.array(
        [
            "diff",
            "obb",
        ],
    )

    def fake_generate(
        *,
        target_point_cloud_camera,
        T_base_camera,
    ):
        return (
            fake_grasps,
            fake_confidences,
            fake_tags,
            target_point_cloud_camera,
        )

    monkeypatch.setattr(
        planner,
        "_generate_grasp_candidates_base",
        fake_generate,
    )

    result = planner.plan(
        rgb_image=rgb,
        depth_image=depth,
        camera_matrix=K,
        T_base_camera=T_base_camera,
        T_base_table=T_base_table,
        current_tcp_position=np.zeros(
            3,
            dtype=np.float64,
        ),
        current_tcp_orientation=np.array(
            [
                0.0,
                0.0,
                0.0,
                1.0,
            ],
            dtype=np.float64,
        ),
        grasp_instruction=(
            "grasp the handle"
        ),
    )

    np.testing.assert_allclose(
        result.semantic_point_px,
        semantic_point,
    )

    np.testing.assert_allclose(
        result.tcp_position_base,
        np.array(
            [
                0.01,
                0.02,
                0.064,
            ],
            dtype=np.float64,
        ),
        atol=1e-6,
    )

    np.testing.assert_allclose(
        result.approach_direction_base,
        np.array(
            [
                0.0,
                0.0,
                -1.0,
            ],
            dtype=np.float64,
        ),
        atol=1e-6,
    )

    assert result.confidence == pytest.approx(
        0.70
    )

    assert (
        result.grasp_instruction
        == "grasp the handle"
    )

    rotation = Rotation.from_quat(
        result.tcp_orientation_base
    ).as_matrix()

    np.testing.assert_allclose(
        rotation,
        grasp_obb[
            :3,
            :3,
        ],
        atol=1e-6,
    )