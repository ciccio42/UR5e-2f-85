from __future__ import annotations

import numpy as np
import pytest
import torch

from ai_controller.models.seedo_controller import utils


def test_crop_image_for_grounding_dino_top_only():
    image = np.zeros(
        (376, 672, 3),
        dtype=np.uint8,
    )

    cropped, crop_info = (
        utils.crop_image_for_grounding_dino(
            image,
            top_px=80,
        )
    )

    assert cropped.shape == (
        296,
        672,
        3,
    )

    assert cropped.flags[
        "C_CONTIGUOUS"
    ]

    assert crop_info == {
        "top_px": 80,
        "bottom_px": 0,
        "left_px": 0,
        "right_px": 0,
        "full_height": 376,
        "full_width": 672,
        "crop_height": 296,
        "crop_width": 672,
    }


def test_crop_image_for_grounding_dino_all_margins():
    image = np.arange(
        100 * 200 * 3,
        dtype=np.int32,
    ).reshape(
        100,
        200,
        3,
    )

    cropped, crop_info = (
        utils.crop_image_for_grounding_dino(
            image,
            top_px=10,
            bottom_px=20,
            left_px=30,
            right_px=40,
        )
    )

    assert cropped.shape == (
        70,
        130,
        3,
    )

    np.testing.assert_array_equal(
        cropped,
        image[
            10:80,
            30:160,
        ],
    )

    assert crop_info == {
        "top_px": 10,
        "bottom_px": 20,
        "left_px": 30,
        "right_px": 40,
        "full_height": 100,
        "full_width": 200,
        "crop_height": 70,
        "crop_width": 130,
    }


def test_crop_image_for_grounding_dino_zero_crop():
    image = np.arange(
        12,
        dtype=np.uint8,
    ).reshape(
        2,
        2,
        3,
    )

    cropped, crop_info = (
        utils.crop_image_for_grounding_dino(
            image
        )
    )

    np.testing.assert_array_equal(
        cropped,
        image,
    )

    assert crop_info == {
        "top_px": 0,
        "bottom_px": 0,
        "left_px": 0,
        "right_px": 0,
        "full_height": 2,
        "full_width": 2,
        "crop_height": 2,
        "crop_width": 2,
    }


def test_crop_image_for_grounding_dino_rejects_non_numpy_input():
    with pytest.raises(
        TypeError,
        match="numpy array",
    ):
        utils.crop_image_for_grounding_dino(
            [[1, 2], [3, 4]]
        )


def test_crop_image_for_grounding_dino_rejects_one_dimensional_input():
    image = np.zeros(
        10,
        dtype=np.uint8,
    )

    with pytest.raises(
        ValueError,
        match="at least two dimensions",
    ):
        utils.crop_image_for_grounding_dino(
            image
        )


@pytest.mark.parametrize(
    ("argument", "value"),
    [
        ("top_px", 1.5),
        ("bottom_px", "1"),
        ("left_px", True),
        ("right_px", None),
    ],
)
def test_crop_image_for_grounding_dino_rejects_non_integer_margins(
    argument,
    value,
):
    image = np.zeros(
        (100, 200, 3),
        dtype=np.uint8,
    )

    kwargs = {
        argument: value,
    }

    with pytest.raises(
        TypeError,
        match=f"{argument} must be an integer",
    ):
        utils.crop_image_for_grounding_dino(
            image,
            **kwargs,
        )


@pytest.mark.parametrize(
    "argument",
    [
        "top_px",
        "bottom_px",
        "left_px",
        "right_px",
    ],
)
def test_crop_image_for_grounding_dino_rejects_negative_margins(
    argument,
):
    image = np.zeros(
        (100, 200, 3),
        dtype=np.uint8,
    )

    kwargs = {
        argument: -1,
    }

    with pytest.raises(
        ValueError,
        match="cannot be negative",
    ):
        utils.crop_image_for_grounding_dino(
            image,
            **kwargs,
        )


def test_crop_image_for_grounding_dino_rejects_empty_vertical_crop():
    image = np.zeros(
        (100, 200, 3),
        dtype=np.uint8,
    )

    with pytest.raises(
        ValueError,
        match="Invalid vertical GroundingDINO crop",
    ):
        utils.crop_image_for_grounding_dino(
            image,
            top_px=60,
            bottom_px=40,
        )


def test_crop_image_for_grounding_dino_rejects_empty_horizontal_crop():
    image = np.zeros(
        (100, 200, 3),
        dtype=np.uint8,
    )

    with pytest.raises(
        ValueError,
        match="Invalid horizontal GroundingDINO crop",
    ):
        utils.crop_image_for_grounding_dino(
            image,
            left_px=100,
            right_px=100,
        )


def test_remap_grounding_dino_boxes_top_crop_matches_visual_prompter_logic():
    boxes = torch.tensor(
        [
            [
                0.5,
                0.5,
                0.25,
                0.25,
            ],
        ],
        dtype=torch.float32,
    )

    crop_info = {
        "top_px": 80,
        "bottom_px": 0,
        "left_px": 0,
        "right_px": 0,
        "full_height": 376,
        "full_width": 672,
        "crop_height": 296,
        "crop_width": 672,
    }

    remapped = (
        utils.remap_grounding_dino_boxes_to_full_frame(
            boxes,
            crop_info=crop_info,
        )
    )

    expected = torch.tensor(
        [
            [
                0.5,
                (
                    0.5 * 296.0
                    + 80.0
                ) / 376.0,
                0.25,
                (
                    0.25
                    * 296.0
                    / 376.0
                ),
            ],
        ],
        dtype=torch.float32,
    )

    assert torch.allclose(
        remapped,
        expected,
    )


def test_remap_grounding_dino_boxes_supports_all_crop_margins():
    boxes = torch.tensor(
        [
            [
                0.5,
                0.5,
                0.2,
                0.4,
            ],
        ],
        dtype=torch.float32,
    )

    crop_info = {
        "top_px": 10,
        "bottom_px": 20,
        "left_px": 30,
        "right_px": 40,
        "full_height": 100,
        "full_width": 200,
        "crop_height": 70,
        "crop_width": 130,
    }

    remapped = (
        utils.remap_grounding_dino_boxes_to_full_frame(
            boxes,
            crop_info=crop_info,
        )
    )

    expected = torch.tensor(
        [
            [
                0.475,
                0.45,
                0.13,
                0.28,
            ],
        ],
        dtype=torch.float32,
    )

    assert torch.allclose(
        remapped,
        expected,
    )


def test_remap_grounding_dino_boxes_zero_crop_is_identity():
    boxes = torch.tensor(
        [
            [
                0.3,
                0.4,
                0.2,
                0.1,
            ],
            [
                0.7,
                0.8,
                0.15,
                0.25,
            ],
        ],
        dtype=torch.float32,
    )

    original = boxes.clone()

    crop_info = {
        "top_px": 0,
        "bottom_px": 0,
        "left_px": 0,
        "right_px": 0,
        "full_height": 376,
        "full_width": 672,
        "crop_height": 376,
        "crop_width": 672,
    }

    remapped = (
        utils.remap_grounding_dino_boxes_to_full_frame(
            boxes,
            crop_info=crop_info,
        )
    )

    assert torch.equal(
        remapped,
        original,
    )

    # The helper must not mutate GroundingDINO's original tensor.
    assert torch.equal(
        boxes,
        original,
    )

    assert remapped is not boxes


def test_remap_grounding_dino_boxes_accepts_empty_detection_tensor():
    boxes = torch.empty(
        (0, 4),
        dtype=torch.float32,
    )

    crop_info = {
        "top_px": 80,
        "bottom_px": 0,
        "left_px": 0,
        "right_px": 0,
        "full_height": 376,
        "full_width": 672,
        "crop_height": 296,
        "crop_width": 672,
    }

    remapped = (
        utils.remap_grounding_dino_boxes_to_full_frame(
            boxes,
            crop_info=crop_info,
        )
    )

    assert remapped.shape == (
        0,
        4,
    )


def test_remap_grounding_dino_boxes_rejects_non_tensor_like_input():
    with pytest.raises(
        TypeError,
        match="clone",
    ):
        utils.remap_grounding_dino_boxes_to_full_frame(
            np.zeros(
                (1, 4),
                dtype=np.float32,
            ),
            crop_info={},
        )


def test_remap_grounding_dino_boxes_rejects_invalid_shape():
    boxes = torch.zeros(
        (2, 5),
        dtype=torch.float32,
    )

    with pytest.raises(
        ValueError,
        match=r"shape \(N, 4\)",
    ):
        utils.remap_grounding_dino_boxes_to_full_frame(
            boxes,
            crop_info={},
        )


def test_remap_grounding_dino_boxes_rejects_missing_crop_metadata():
    boxes = torch.zeros(
        (1, 4),
        dtype=torch.float32,
    )

    with pytest.raises(
        ValueError,
        match="missing keys",
    ):
        utils.remap_grounding_dino_boxes_to_full_frame(
            boxes,
            crop_info={
                "top_px": 80,
            },
        )


@pytest.mark.parametrize(
    (
        "dimension_name",
        "invalid_value",
    ),
    [
        ("full_height", 0),
        ("full_width", 0),
        ("crop_height", 0),
        ("crop_width", 0),
    ],
)
def test_remap_grounding_dino_boxes_rejects_non_positive_dimensions(
    dimension_name,
    invalid_value,
):
    boxes = torch.zeros(
        (1, 4),
        dtype=torch.float32,
    )

    crop_info = {
        "top_px": 0,
        "left_px": 0,
        "full_height": 100,
        "full_width": 200,
        "crop_height": 100,
        "crop_width": 200,
    }

    crop_info[
        dimension_name
    ] = invalid_value

    with pytest.raises(
        ValueError,
        match="dimensions must be positive",
    ):
        utils.remap_grounding_dino_boxes_to_full_frame(
            boxes,
            crop_info=crop_info,
        )