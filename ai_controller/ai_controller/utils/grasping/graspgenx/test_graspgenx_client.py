from __future__ import annotations

import subprocess
from pathlib import Path

import numpy as np
import pytest

from ai_controller.utils.grasping.graspgenx.client import (
    GraspGenXRemoteClient,
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


def test_predict_object_grasps_roundtrip(
    monkeypatch,
):
    client = GraspGenXRemoteClient()

    point_cloud = np.array(
        [
            [0.00, 0.00, 0.10],
            [0.01, 0.00, 0.10],
            [0.00, 0.01, 0.10],
        ],
        dtype=np.float32,
    )

    expected_grasps = np.stack(
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

    expected_grasps[
        1,
        0,
        3,
    ] = 0.01

    expected_confidences = np.array(
        [
            0.9,
            0.7,
        ],
        dtype=np.float32,
    )

    expected_tags = np.array(
        [
            "diff",
            "obb",
        ],
        dtype="<U8",
    )

    def fake_run_worker(
        arguments,
        timeout,
    ):
        assert timeout == 180.0

        action_index = arguments.index(
            "--action"
        )

        assert (
            arguments[
                action_index + 1
            ]
            == "infer_object"
        )

        planner_index = arguments.index(
            "--planner"
        )

        assert (
            arguments[
                planner_index + 1
            ]
            == "graspmoe"
        )

        input_index = arguments.index(
            "--input"
        )

        output_index = arguments.index(
            "--output"
        )

        input_path = Path(
            arguments[
                input_index + 1
            ]
        )

        output_path = Path(
            arguments[
                output_index + 1
            ]
        )

        with np.load(
            input_path,
            allow_pickle=False,
        ) as data:

            np.testing.assert_allclose(
                data["point_cloud"],
                point_cloud,
            )

            np.testing.assert_allclose(
                data["extents_open"],
                ROBOTIQ_SWEEP_VOLUME[
                    "extents_open"
                ],
            )

            np.testing.assert_allclose(
                data["offset_open"],
                ROBOTIQ_SWEEP_VOLUME[
                    "offset_open"
                ],
            )

            np.testing.assert_allclose(
                data["extents_mid"],
                ROBOTIQ_SWEEP_VOLUME[
                    "extents_mid"
                ],
            )

            np.testing.assert_allclose(
                data["offset_mid"],
                ROBOTIQ_SWEEP_VOLUME[
                    "offset_mid"
                ],
            )

            assert int(
                data["gripper_type"].item()
            ) == 0

            assert float(
                data[
                    "fingertip_depth"
                ].item()
            ) == pytest.approx(
                0.136
            )

        np.savez_compressed(
            output_path,
            grasps=expected_grasps,
            confidences=(
                expected_confidences
            ),
            branch_tags=expected_tags,
        )

        return subprocess.CompletedProcess(
            args=arguments,
            returncode=0,
            stdout="",
            stderr="",
        )

    monkeypatch.setattr(
        client,
        "_run_worker",
        fake_run_worker,
    )

    (
        grasps,
        confidences,
        tags,
    ) = client.predict_object_grasps(
        point_cloud=point_cloud,
        sweep_volume_params=(
            ROBOTIQ_SWEEP_VOLUME
        ),
        planner="graspmoe",
        num_grasps=200,
        grasp_threshold=-1.0,
        topk_num_grasps=-1,
    )

    np.testing.assert_allclose(
        grasps,
        expected_grasps,
    )

    np.testing.assert_allclose(
        confidences,
        expected_confidences,
    )

    np.testing.assert_array_equal(
        tags,
        expected_tags,
    )


def test_predict_object_grasps_rejects_bad_point_cloud():
    client = GraspGenXRemoteClient()

    with pytest.raises(
        ValueError,
        match="shape",
    ):
        client.predict_object_grasps(
            point_cloud=np.zeros(
                (10, 2),
                dtype=np.float32,
            ),
            sweep_volume_params=(
                ROBOTIQ_SWEEP_VOLUME
            ),
        )