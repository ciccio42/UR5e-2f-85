from __future__ import annotations

import argparse

from ai_controller.models.seedo_controller.keyframe_selector import KeyframeSelector

def run_keyframe_test(args: argparse.Namespace) -> int:

    if args.video is None:
        raise ValueError(
            "--video is required for the keyframe test."
        )

    selector = KeyframeSelector(
        gaussian_sigma=5.0,
        prominence=0.8,
        expected_keyframes=2,
        save_preview=True,
    )

    result = selector.run(
        video_path=args.video,
        artifacts_dir=args.artifacts_dir,
    )

    print("Keyframe selection completed")
    print(f"Video: {result.video_path}")
    print(f"Keyframes: {list(result.keyframes)}")
    print(f"Images returned: {len(result.keyframe_images)}")
    print(f"Artifacts directory: {result.artifacts_dir}")

    for frame_index, image in zip(
        result.keyframes,
        result.keyframe_images,
        strict=True,
    ):
        print(
            f"Frame {frame_index}: "
            f"shape={image.shape}, dtype={image.dtype}"
        )

    if args.expected_keyframes is not None:
        expected = tuple(args.expected_keyframes)

        if result.keyframes != expected:
            raise AssertionError(
                f"Expected keyframes {list(expected)}, "
                f"but received {list(result.keyframes)}"
            )

    preview_dir = result.artifacts_dir / "returned_keyframes"

    if not preview_dir.is_dir():
        raise AssertionError(
            f"Preview directory was not created: {preview_dir}"
        )

    for frame_index in result.keyframes:
        preview_path = (
            preview_dir
            / f"keyframe_{frame_index:06d}.png"
        )

        if not preview_path.is_file():
            raise AssertionError(
                f"Missing keyframe preview: {preview_path}"
            )

    print("TEST PASSED")
    return 0
