from __future__ import annotations

import argparse

from .common import build_scene_perception_result

def run_scene_perceiver_test(
    args: argparse.Namespace,
) -> int:
    perception_result = (
        build_scene_perception_result(
            args
        )
    )

    raw_scene = (
        perception_result.raw_scene
    )

    if not raw_scene.objects:
        raise AssertionError(
            "ScenePerceiver returned no objects."
        )

    raw_ids = [
        obj.object_id
        for obj in raw_scene.objects
    ]

    if len(raw_ids) != len(
        set(raw_ids)
    ):
        raise AssertionError(
            "ScenePerceiver produced duplicate object IDs."
        )

    for obj in raw_scene.objects:
        if not obj.object_id.strip():
            raise AssertionError(
                "Detected object has an empty object ID."
            )

        if not obj.label.strip():
            raise AssertionError(
                f"{obj.object_id} has an empty detector label."
            )

        if len(obj.pixel_coordinates) != 2:
            raise AssertionError(
                f"{obj.object_id} has invalid pixel coordinates."
            )

        if len(obj.position_camera) != 3:
            raise AssertionError(
                f"{obj.object_id} has invalid camera coordinates."
            )

        if len(obj.position_base) != 3:
            raise AssertionError(
                f"{obj.object_id} has invalid base coordinates."
            )

        if obj.mask is None:
            raise AssertionError(
                f"{obj.object_id} has no segmentation mask."
            )

    if args.artifacts_dir is not None:
        if (
            perception_result.overlay_image_path
            is None
        ):
            raise AssertionError(
                "ScenePerceiver did not return an overlay path."
            )

        if (
            not perception_result
            .overlay_image_path
            .is_file()
        ):
            raise AssertionError(
                "Raw scene overlay was not created: "
                f"{perception_result.overlay_image_path}"
            )

        if (
            perception_result.raw_scene_json_path
            is None
        ):
            raise AssertionError(
                "ScenePerceiver did not return "
                "a raw-scene JSON path."
            )

        if (
            not perception_result
            .raw_scene_json_path
            .is_file()
        ):
            raise AssertionError(
                "Raw scene JSON was not created: "
                f"{perception_result.raw_scene_json_path}"
            )

    print("\n=== RAW SCENE STATE ===")
    print(
        f"Detected objects: "
        f"{len(raw_scene.objects)}"
    )

    for obj in raw_scene.objects:
        print()
        print(
            f"Object ID:        "
            f"{obj.object_id}"
        )
        print(
            f"Label:            "
            f"{obj.label}"
        )
        print(
            f"Pixel:            "
            f"{obj.pixel_coordinates}"
        )
        print(
            f"Confidence:       "
            f"{obj.confidence}"
        )
        print(
            f"Camera position:  "
            f"{obj.position_camera}"
        )
        print(
            f"Base position:    "
            f"{obj.position_base}"
        )

    if (
        perception_result.overlay_image_path
        is not None
    ):
        print(
            "\nOverlay: "
            f"{perception_result.overlay_image_path}"
        )

    if (
        perception_result.raw_scene_json_path
        is not None
    ):
        print(
            "Raw scene JSON: "
            f"{perception_result.raw_scene_json_path}"
        )

    print("\nTEST PASSED")

    return 0
