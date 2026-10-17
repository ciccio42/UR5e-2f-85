from __future__ import annotations

import cv2
import numpy as np
import pytest

from ai_controller.models.seedo_controller.scene_perceiver import (
    ScenePerceiver,
)


def _make_perceiver_for_geometry_test() -> ScenePerceiver:
    """
    Build ScenePerceiver without loading GroundingDINO/SAM.

    Only the cached runtime geometry required by the refinement
    methods is initialized.
    """

    perceiver = ScenePerceiver.__new__(
        ScenePerceiver
    )

    perceiver._last_runtime_camera_calibration = {
        "position": np.zeros(
            3,
            dtype=np.float64,
        ),
        "orientation_matrix": np.eye(
            3,
            dtype=np.float64,
        ),
    }

    perceiver._last_base_to_table_rotation = (
        np.eye(
            3,
            dtype=np.float64,
        )
    )

    perceiver._last_base_to_table_translation = (
        np.zeros(
            3,
            dtype=np.float64,
        )
    )

    return perceiver


def _synthetic_ring_case():
    height = 101
    width = 101

    center = (
        50,
        50,
    )

    outer_radius_px = 20
    hole_radius_px = 8

    mask = np.zeros(
        (
            height,
            width,
        ),
        dtype=np.uint8,
    )

    cv2.circle(
        mask,
        center,
        outer_radius_px,
        1,
        -1,
    )

    cv2.circle(
        mask,
        center,
        hole_radius_px,
        0,
        -1,
    )

    mask = mask.astype(
        bool
    )

    # Synthetic geometry:
    #
    # support surface: z = 0.48 m
    # ring top:       z = 0.50 m
    #
    # therefore observed thickness = 20 mm.
    depth = np.full(
        (
            height,
            width,
        ),
        0.48,
        dtype=np.float64,
    )

    depth[
        mask
    ] = 0.50

    K = np.array(
        [
            [
                100.0,
                0.0,
                50.0,
            ],
            [
                0.0,
                100.0,
                50.0,
            ],
            [
                0.0,
                0.0,
                1.0,
            ],
        ],
        dtype=np.float64,
    )

    return (
        mask,
        depth,
        K,
    )


def test_refine_nut_hole_recovers_synthetic_geometry():
    perceiver = (
        _make_perceiver_for_geometry_test()
    )

    (
        mask,
        depth,
        K,
    ) = _synthetic_ring_case()

    result = perceiver.refine_nut_hole(
        mask=mask,
        depth_image=depth,
        camera_matrix=K,

        support_annulus_inner_px=2,
        support_annulus_outer_px=8,

        min_plane_points=50,
        min_support_points=50,
    )

    assert result["accepted"] is True

    np.testing.assert_allclose(
        result["hole_center_px"],
        np.array(
            [
                50.0,
                50.0,
            ]
        ),
        atol=0.5,
    )

    np.testing.assert_allclose(
        result["hole_center_base"],
        np.array(
            [
                0.0,
                0.0,
                0.50,
            ]
        ),
        atol=1e-6,
    )

    np.testing.assert_allclose(
        result["hole_bottom_center_base"],
        np.array(
            [
                0.0,
                0.0,
                0.48,
            ]
        ),
        atol=1e-6,
    )

    np.testing.assert_allclose(
        result["hole_axis_base"],
        np.array(
            [
                0.0,
                0.0,
                1.0,
            ]
        ),
        atol=1e-6,
    )

    assert (
        result[
            "nut_thickness_estimate_m"
        ]
        == pytest.approx(
            0.020,
            abs=1e-6,
        )
    )

    assert (
        result["top_plane_rms_m"]
        < 1e-8
    )

    assert (
        result["support_mad_m"]
        < 1e-8
    )


def test_refine_nut_hole_rejects_object_without_internal_cavity():
    perceiver = (
        _make_perceiver_for_geometry_test()
    )

    mask = np.zeros(
        (
            101,
            101,
        ),
        dtype=np.uint8,
    )

    cv2.circle(
        mask,
        (
            50,
            50,
        ),
        20,
        1,
        -1,
    )

    depth = np.full(
        (
            101,
            101,
        ),
        0.50,
        dtype=np.float64,
    )

    K = np.array(
        [
            [
                100.0,
                0.0,
                50.0,
            ],
            [
                0.0,
                100.0,
                50.0,
            ],
            [
                0.0,
                0.0,
                1.0,
            ],
        ],
        dtype=np.float64,
    )

    with pytest.raises(
        RuntimeError,
        match="closed internal cavity",
    ):
        perceiver.refine_nut_hole(
            mask=mask.astype(bool),
            depth_image=depth,
            camera_matrix=K,
        )