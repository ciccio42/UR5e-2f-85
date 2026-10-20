from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from ai_controller.utils.grasping.grasp_planner_factory import (
    build_grasp_planner,
)
from ai_controller.utils.grasping.graspgenx import (
    inference_core,
)
from ai_controller.utils.grasping.graspgenx.geometry_selector import (
    GraspGenXGeometrySelector,
)
from ai_controller.utils.grasping.graspgenx.geometry_utils import (
    GraspGenXGeometryConfig,
)
from ai_controller.utils.grasping.graspgenx.planner import (
    GraspGenXPlanner,
)


def test_factory_builds_single_graspgenx_backend(
    monkeypatch,
):
    """
    The public factory must construct the final GraspGenXPlanner
    without exposing any of the removed V2/unsafe configuration.
    """

    monkeypatch.setattr(
        inference_core,
        "GraspMolmoClient",
        lambda *args, **kwargs: object(),
    )

    monkeypatch.setattr(
        inference_core,
        "GraspGenXRemoteClient",
        lambda *args, **kwargs: object(),
    )

    planner = build_grasp_planner(
        {
            "backend": "graspgenx",
            "semantic_radius_px": 50.0,
            "graspgenx_table_margin_mm": 0.0,
        }
    )

    assert isinstance(
        planner,
        GraspGenXPlanner,
    )

    assert planner.backend_name == "graspgenx"

    assert (
        planner.semantic_radius_px
        == 50.0
    )

    assert (
        planner.geometry_semantic_radius_px
        == 50.0
    )

    forbidden_attributes = (
        "v2_validation_mode",
        "v2_unsafe_enable_robot_commands",
        "v2_worker_python",
        "v2_table_margin_mm",
        "v2_semantic_radius_px",
    )

    for name in forbidden_attributes:
        assert not hasattr(
            planner,
            name,
        )


def test_geometry_selector_prefers_valid_target_grasp():
    """
    A high-confidence grasp away from the semantic target must
    lose against a lower-confidence grasp that actually surrounds
    the requested target region.

    The scene is completely synthetic:
      - horizontal table at z=0
      - target around z=0.23 m
      - good gripper origin at z=0.10 m
      - second candidate displaced 20 cm along X
    """

    config = GraspGenXGeometryConfig(
        table_margin_m=0.005,
        semantic_radius_px=50.0,
    )

    selector = GraspGenXGeometrySelector(
        cfg=config,
    )

    # ---------------------------------------------------------
    # Synthetic table: 2500 perfectly horizontal observations.
    # ---------------------------------------------------------

    table_x = np.linspace(
        -0.20,
        0.20,
        50,
    )

    table_y = np.linspace(
        -0.20,
        0.20,
        50,
    )

    grid_x, grid_y = np.meshgrid(
        table_x,
        table_y,
    )

    background = np.column_stack(
        (
            grid_x.ravel(),
            grid_y.ravel(),
            np.zeros(
                grid_x.size,
                dtype=np.float64,
            ),
        )
    )

    # ---------------------------------------------------------
    # Synthetic target centered in the Robotiq sweep region.
    #
    # Candidate origin z = 0.10 m
    # sweep center     z = 0.13 m local
    # target center    z = 0.23 m world
    # ---------------------------------------------------------

    target_x = np.linspace(
        -0.010,
        0.010,
        5,
    )

    target_y = np.linspace(
        -0.005,
        0.005,
        5,
    )

    target_grid_x, target_grid_y = (
        np.meshgrid(
            target_x,
            target_y,
        )
    )

    target = np.column_stack(
        (
            target_grid_x.ravel(),
            target_grid_y.ravel(),
            np.full(
                target_grid_x.size,
                0.230,
                dtype=np.float64,
            ),
        )
    )

    good_grasp = np.eye(
        4,
        dtype=np.float64,
    )

    good_grasp[
        :3,
        3,
    ] = [
        0.0,
        0.0,
        0.100,
    ]

    bad_grasp = good_grasp.copy()

    bad_grasp[
        0,
        3,
    ] = 0.200

    grasps = np.stack(
        (
            good_grasp,
            bad_grasp,
        ),
        axis=0,
    )

    confidences = np.array(
        [
            0.60,
            0.99,
        ],
        dtype=np.float64,
    )

    branch_tags = np.array(
        [
            "obb",
            "diff",
        ],
    )

    K = np.array(
        [
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
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

    choice = selector.select(
        grasps_base=grasps,
        confidences=confidences,
        branch_tags=branch_tags,
        semantic_point_px=np.array(
            [
                320.0,
                240.0,
            ],
            dtype=np.float64,
        ),
        T_base_camera=T_base_camera,
        camera_matrix=K,
        T_base_table=T_base_table,
        target_points_base=target,
        background_points_base=background,
        current_tcp_orientation=None,
    )

    assert choice.index == 0
    assert choice.branch == "obb"

    assert choice.confidence == 0.60

    np.testing.assert_allclose(
        choice.T_base_gripper[
            :3,
            3,
        ],
        good_grasp[
            :3,
            3,
        ],
        atol=1e-9,
    )

    assert (
        choice.table_clearance_m
        >= config.table_margin_m
    )

    assert (
        choice.approach_table_clearance_m
        >= config.table_margin_m
    )


def test_planner_uses_geometry_selected_orientation():
    """
    The geometry selector resolves the parallel-jaw symmetry.

    The planner must use exactly that selected orientation and
    must not recompute or alter it during the final handoff.
    """

    # Avoid __init__: this method only depends on the orientation
    # previously supplied by the geometry worker.
    planner = object.__new__(
        GraspGenXPlanner
    )

    raw_rotation = (
        Rotation.from_euler(
            "z",
            10.0,
            degrees=True,
        )
        .as_matrix()
    )

    selected_rotation = (
        Rotation.from_euler(
            "z",
            190.0,
            degrees=True,
        )
        .as_matrix()
    )

    # Both orientations have the same approach axis (+Z), as
    # required for the 180-degree parallel-jaw symmetry.
    np.testing.assert_allclose(
        raw_rotation[:, 2],
        selected_rotation[:, 2],
        atol=1e-10,
    )

    planner._selected_rotation = (
        selected_rotation.copy()
    )

    resolved_rotation = (
        planner
        ._resolve_parallel_gripper_symmetry(
            grasp_rotation_base=(
                raw_rotation
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
        )
    )

    np.testing.assert_allclose(
        resolved_rotation,
        selected_rotation,
        atol=1e-10,
    )

    # The method deliberately returns a copy.
    resolved_rotation[0, 0] = 123.0

    assert (
        planner._selected_rotation[0, 0]
        != 123.0
    )
