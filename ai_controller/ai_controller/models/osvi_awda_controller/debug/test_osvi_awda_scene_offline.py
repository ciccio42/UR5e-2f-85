#!/usr/bin/env python3

"""
Offline test of the real OSVI-AWDA controller using:

- real robot front-camera image;
- real AWDA checkpoint;
- real human demonstration;
- real preprocessing;
- real AWDA forward;
- real camera->base_link projection;
- real primitive expansion.

NO robot command is sent.

The second grasp phase can optionally be simulated with wrist depth disabled,
so that the AWDA WP2 grasp hint itself is used as the grasp target.
"""

import argparse
import importlib
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


# =====================================================================
# Repository / controller discovery
# =====================================================================

def find_repo_root() -> Path:
    current = Path(__file__).resolve()

    candidates = [
        Path.cwd().resolve(),
        current.parent,
        *current.parents,
    ]

    for candidate in candidates:
        if (candidate / "ai_controller").is_dir():
            return candidate

    raise RuntimeError(
        "Cannot find repository root containing ai_controller/. "
        "Run this script from the UR5e-2f-85 repository."
    )


def import_controller(repo_root: Path):
    sys.path.insert(0, str(repo_root))

    module_name = (
        "ai_controller.models.osvi_awda_controller."
        "osvi_awda_controller"
    )

    print(
        "[TEST] Importing controller module:",
        module_name,
    )

    module = importlib.import_module(module_name)

    if not hasattr(module, "OSVIAWDAController"):
        raise RuntimeError(
            f"Module {module_name!r} does not contain "
            "OSVIAWDAController."
        )

    print(
        "[TEST] Controller file:",
        module.__file__,
    )

    return module.OSVIAWDAController


# =====================================================================
# Helpers
# =====================================================================

def load_rgb_image(path: Path) -> np.ndarray:
    image = Image.open(path).convert("RGB")
    return np.asarray(image, dtype=np.uint8)

def _normalized_waypoint_to_model_pixel(
    waypoint,
    width,
    height,
):
    """
    AWDA image waypoint convention:

        u_norm: -1 left  -> +1 right
        v_norm: +1 top   -> -1 bottom

    Returns pixel coordinates (x, y).
    """
    u = float(waypoint[0])
    v = float(waypoint[1])

    x = (u + 1.0) * 0.5 * (width - 1)
    y = (1.0 - v) * 0.5 * (height - 1)

    x = int(np.clip(round(x), 0, width - 1))
    y = int(np.clip(round(y), 0, height - 1))

    return x, y


def _normalized_waypoint_to_raw_pixel(
    waypoint,
    raw_width,
    raw_height,
    crop,
    model_width,
    model_height,
):
    """
    Convert an AWDA normalized waypoint back onto the ORIGINAL
    front-camera image.

    Pipeline being inverted:

        raw image
          -> crop
          -> resize(model_width, model_height)
          -> normalized coordinates
    """

    top, bottom, left, right = [
        int(value)
        for value in crop
    ]

    cropped_width = (
        raw_width - left - right
    )
    cropped_height = (
        raw_height - top - bottom
    )

    if cropped_width <= 0 or cropped_height <= 0:
        raise ValueError(
            f"Invalid crop {crop} for "
            f"{raw_width}x{raw_height} image."
        )

    x_model, y_model = (
        _normalized_waypoint_to_model_pixel(
            waypoint,
            model_width,
            model_height,
        )
    )

    # Undo resize.
    if model_width > 1:
        x_crop = (
            x_model
            / float(model_width - 1)
            * (cropped_width - 1)
        )
    else:
        x_crop = 0.0

    if model_height > 1:
        y_crop = (
            y_model
            / float(model_height - 1)
            * (cropped_height - 1)
        )
    else:
        y_crop = 0.0

    # Undo crop.
    x_raw = left + x_crop
    y_raw = top + y_crop

    x_raw = int(
        np.clip(
            round(x_raw),
            0,
            raw_width - 1,
        )
    )

    y_raw = int(
        np.clip(
            round(y_raw),
            0,
            raw_height - 1,
        )
    )

    return x_raw, y_raw


def _draw_waypoints(
    image_rgb,
    pixel_points,
    waypoints,
    output_path,
):
    """
    Draw the five AWDA selected waypoints.

    WP1 = free_space
    WP2 = grasp
    WP3 = carry
    WP4 = carry
    WP5 = drop
    """

    image = Image.fromarray(
        np.asarray(image_rgb, dtype=np.uint8)
    ).convert("RGB")

    draw = ImageDraw.Draw(image)

    semantics = [
        "FREE",
        "GRASP",
        "CARRY",
        "CARRY",
        "DROP",
    ]

    # RGB colors because the image is handled by PIL.
    colors = [
        (0, 120, 255),    # WP1
        (255, 0, 0),      # WP2 grasp
        (255, 165, 0),    # WP3
        (255, 165, 0),    # WP4
        (0, 200, 0),      # WP5 drop
    ]

    radius = max(
        5,
        int(
            min(
                image.width,
                image.height,
            ) * 0.012
        ),
    )

    # Lines connecting trajectory.
    for i in range(
        len(pixel_points) - 1
    ):
        draw.line(
            [
                pixel_points[i],
                pixel_points[i + 1],
            ],
            fill=(255, 255, 255),
            width=max(2, radius // 3),
        )

    # Points.
    for i, ((x, y), waypoint) in enumerate(
        zip(pixel_points, waypoints),
        start=1,
    ):
        color = colors[i - 1]

        draw.ellipse(
            [
                x - radius,
                y - radius,
                x + radius,
                y + radius,
            ],
            fill=color,
            outline=(255, 255, 255),
            width=2,
        )

        label = (
            f"WP{i} {semantics[i - 1]} "
            f"g={float(waypoint[3]):.3f}"
        )

        text_x = x + radius + 4
        text_y = y - radius - 2

        # Black background so labels remain readable.
        bbox = draw.textbbox(
            (text_x, text_y),
            label,
        )

        draw.rectangle(
            bbox,
            fill=(0, 0, 0),
        )

        draw.text(
            (text_x, text_y),
            label,
            fill=(255, 255, 255),
        )

        # Strong extra marker for grasp waypoint.
        if i == 2:
            cross_radius = radius + 5

            draw.line(
                [
                    x - cross_radius,
                    y,
                    x + cross_radius,
                    y,
                ],
                fill=(255, 0, 0),
                width=2,
            )

            draw.line(
                [
                    x,
                    y - cross_radius,
                    x,
                    y + cross_radius,
                ],
                fill=(255, 0, 0),
                width=2,
            )

    image.save(output_path)

    print(
        "[TEST] Saved waypoint overlay:",
        output_path,
    )


def save_waypoint_overlays(
    controller,
    raw_scene,
    output_dir,
):
    """
    Save two overlays:

    1. Exact 100x180 image seen by AWDA.
    2. Original robot-camera image.
    """

    waypoints = np.asarray(
        controller.last_image_waypoints,
        dtype=np.float64,
    )

    if waypoints.shape != (5, 4):
        raise ValueError(
            f"Expected 5x4 selected waypoints, "
            f"got {waypoints.shape}."
        )

    model_width = int(
        controller.cfg.image.get(
            "width",
            180,
        )
    )

    model_height = int(
        controller.cfg.image.get(
            "height",
            100,
        )
    )

    crop = controller.cfg.image.get(
        "crop",
        [0, 0, 0, 0],
    )

    # ---------------------------------------------------------
    # Exact model-input image
    # ---------------------------------------------------------

    model_chw = controller._preprocess_frame(
        raw_scene,
        source="live",
    )

    model_rgb = controller._chw_to_uint8(
        model_chw
    )

    model_points = [
        _normalized_waypoint_to_model_pixel(
            waypoint,
            model_width,
            model_height,
        )
        for waypoint in waypoints
    ]

    model_overlay_path = (
        Path(output_dir)
        / "osvi_awda_waypoints_overlay_model_t000.png"
    )

    _draw_waypoints(
        model_rgb,
        model_points,
        waypoints,
        model_overlay_path,
    )

    # ---------------------------------------------------------
    # Original front-camera image
    # ---------------------------------------------------------

    raw_height, raw_width = (
        raw_scene.shape[:2]
    )

    raw_points = [
        _normalized_waypoint_to_raw_pixel(
            waypoint,
            raw_width,
            raw_height,
            crop,
            model_width,
            model_height,
        )
        for waypoint in waypoints
    ]

    raw_overlay_path = (
        Path(output_dir)
        / "osvi_awda_waypoints_overlay_raw_t000.png"
    )

    _draw_waypoints(
        raw_scene,
        raw_points,
        waypoints,
        raw_overlay_path,
    )

    print("\nOverlay coordinates:")

    for i, (
        model_point,
        raw_point,
    ) in enumerate(
        zip(model_points, raw_points),
        start=1,
    ):
        print(
            f"  WP{i}: "
            f"model={model_point}, "
            f"raw={raw_point}"
        )


def print_waypoints(controller):
    image_wps = np.asarray(
        controller.last_image_waypoints,
        dtype=np.float64,
    )

    base_wps = np.asarray(
        controller.last_base_waypoints,
        dtype=np.float64,
    )

    print("\n" + "=" * 90)
    print("PREDICTED 5 WAYPOINTS")
    print("=" * 90)

    for i, (image_wp, base_wp) in enumerate(
        zip(image_wps, base_wps),
        start=1,
    ):
        print(
            f"WP{i}\n"
            f"  image: "
            f"u={image_wp[0]:+.6f}, "
            f"v={image_wp[1]:+.6f}, "
            f"depth={image_wp[2]:+.6f}, "
            f"grasp_attr={image_wp[3]:+.6f}\n"
            f"  base:  "
            f"x={base_wp[0]:+.6f}, "
            f"y={base_wp[1]:+.6f}, "
            f"z={base_wp[2]:+.6f}"
        )


def print_decisions(controller):
    decisions = controller.last_gripper_decisions

    print("\n" + "=" * 90)
    print("PRIMITIVE DECISIONS")
    print("=" * 90)

    for item in decisions:
        print(
            f"WP{item['waypoint_index']}: "
            f"raw_grasp_attr={item['raw_grasp_attribute']:+.6f} | "
            f"threshold_closed={item['threshold_closed']} | "
            f"commanded_closed={item['commanded_closed']} | "
            f"primitive={item['primitive']}"
        )


def print_actions(actions, title):
    print("\n" + "=" * 90)
    print(title)
    print("=" * 90)

    for i, action in enumerate(actions):
        action = np.asarray(action, dtype=np.float64)

        state = (
            "CLOSED"
            if action[-1] > 127.5
            else "OPEN"
        )

        print(
            f"action[{i:02d}] "
            f"xyz=["
            f"{action[0]:+.6f}, "
            f"{action[1]:+.6f}, "
            f"{action[2]:+.6f}] "
            f"gripper={action[-1]:.1f} ({state})"
        )


# =====================================================================
# Main
# =====================================================================

def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--config",
        required=True,
        help="Runtime YAML OSVI-AWDA.",
    )

    parser.add_argument(
        "--image",
        required=True,
        help="Real front-camera robot image.",
    )

    parser.add_argument(
        "--demo-root",
        required=True,
        help=(
            "Root containing task_XX folders with human demos. "
            "Example: .../human_rgb_pick_place"
        ),
    )

    parser.add_argument(
        "--task-id",
        required=True,
        help="Task id, e.g. 01.",
    )

    parser.add_argument(
        "--output-dir",
        default="/tmp/osvi_awda_scene_test",
        help="Directory for controller debug outputs.",
    )

    parser.add_argument(
        "--eef-pos",
        nargs=3,
        type=float,
        default=[0.0, 0.35, 0.20],
        metavar=("X", "Y", "Z"),
        help=(
            "Fake current EEF position used only for primitive expansion. "
            "It does not affect AWDA model inference because concat_state=false."
        ),
    )

    parser.add_argument(
        "--simulate-resume",
        action="store_true",
        help=(
            "Also simulate grasp refinement/carry/drop with wrist depth disabled."
        ),
    )

    args = parser.parse_args()

    repo_root = find_repo_root()

    config_path = Path(args.config).expanduser().resolve()
    image_path = Path(args.image).expanduser().resolve()
    demo_root = Path(args.demo_root).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()

    if not config_path.is_file():
        raise FileNotFoundError(config_path)

    if not image_path.is_file():
        raise FileNotFoundError(image_path)

    if not demo_root.is_dir():
        raise FileNotFoundError(demo_root)

    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    print("=" * 90)
    print("OSVI-AWDA REAL-SCENE OFFLINE TEST")
    print("=" * 90)

    print("Config:    ", config_path)
    print("Image:     ", image_path)
    print("Demo root: ", demo_root)
    print("Task:      ", args.task_id)
    print("Output:    ", output_dir)

    # =================================================================
    # 1. Load real controller + checkpoint
    # =================================================================

    OSVIAWDAController = import_controller(repo_root)

    print("\n[TEST] Loading real AWDA controller/checkpoint...")

    controller = OSVIAWDAController(
        str(config_path),
        task_name="pick_place",
    )

    controller.reset()

    # =================================================================
    # 2. Load real human demonstration
    # =================================================================

    print("\n[TEST] Loading human demonstration...")

    controller.load_command(
        demo_path=str(demo_root),
        task_id=str(args.task_id),
        save_demo_frames=True,
        traj_cnt=0,
        save_path=str(output_dir),
    )

    print(
        "[TEST] Context source:",
        controller.context_source,
    )

    print(
        "[TEST] Context tensor:",
        tuple(controller.context_tensor.shape),
    )

    # =================================================================
    # 3. Load real robot scene
    # =================================================================

    scene = load_rgb_image(image_path)

    print("\n[TEST] Scene image:")
    print("       shape =", scene.shape)
    print("       dtype =", scene.dtype)

    # concat_state=false, therefore the AWDA model does not use this state.
    #
    # It IS used later by build_free_space_actions() to decide whether an
    # additional high-Z approach action is required.
    eef_pos = np.asarray(
        args.eef_pos,
        dtype=np.float64,
    )

    robot_state = np.concatenate(
        [
            eef_pos,
            np.asarray(
                [0.0, 0.0, 0.0, 1.0],
                dtype=np.float64,
            ),
        ]
    )

    input_data = (
        [scene],
        robot_state,
    )

    # =================================================================
    # 4. REAL AWDA FORWARD - t=0
    # =================================================================

    print("\n" + "=" * 90)
    print("RUNNING REAL AWDA FORWARD t=0")
    print("=" * 90)

    actions_t0 = controller.inference(
        input_data,
        t=0,
        save_path=str(output_dir),
    )

    # =================================================================
    # 5. Inspect real model output
    # =================================================================

    print_waypoints(controller)
    print_decisions(controller)
    save_waypoint_overlays(
    controller,
    scene,
    output_dir,
)

    # =================================================================
    # 6. Verify fixed primitive schedule
    # =================================================================

    expected_primitives = [
        "free_space",
        "grasp",
        "carry",
        "carry",
        "drop",
    ]

    actual_primitives = [
        item["primitive"]
        for item in controller.last_gripper_decisions
    ]

    print("\nExpected primitives:")
    print("   ", expected_primitives)

    print("Actual primitives:")
    print("   ", actual_primitives)

    if actual_primitives != expected_primitives:
        raise AssertionError(
            "Primitive schedule mismatch.\n"
            f"expected={expected_primitives}\n"
            f"actual={actual_primitives}"
        )

    print(
        "\n[PASS] Fixed primitive schedule:"
        " WP1 free_space, WP2 grasp,"
        " WP3 carry, WP4 carry, WP5 drop"
    )

    # =================================================================
    # 7. First execution plan must stop at WP2 hover
    # =================================================================

    print_actions(
        actions_t0,
        "EXECUTION ACTIONS t=0 - PLAN TO GRASP HOVER",
    )

    if controller.last_execution_phase != "plan_to_grasp_hover":
        raise AssertionError(
            "Expected last_execution_phase='plan_to_grasp_hover', "
            f"got {controller.last_execution_phase!r}"
        )

    print(
        "\n[PASS] execution phase = plan_to_grasp_hover"
    )

    # Last action must be hover over WP2.
    hover_action = np.asarray(
        actions_t0[-1],
        dtype=np.float64,
    )

    wp2_base = np.asarray(
        controller.last_base_waypoints[1, :3],
        dtype=np.float64,
    )

    # Remember: post_process applies workspace clamp before coarse action.
    wp2_base_safe = controller._apply_workspace_safety(
        wp2_base
    )

    hover_height = float(
        controller.cfg.grasp_refinement.get(
            "hover_height_m",
            0.05,
        )
    )

    expected_hover = (
        wp2_base_safe
        + np.asarray(
            [0.0, 0.0, hover_height],
            dtype=np.float64,
        )
    )

    expected_hover = controller._apply_workspace_safety(
        expected_hover
    )

    print("\n" + "=" * 90)
    print("GRASP HOVER CHECK")
    print("=" * 90)

    print(
        "WP2 raw base xyz        =",
        wp2_base.tolist(),
    )

    print(
        "WP2 safe base xyz       =",
        wp2_base_safe.tolist(),
    )

    print(
        "hover_height_m          =",
        hover_height,
    )

    print(
        "expected hover xyz      =",
        expected_hover.tolist(),
    )

    print(
        "actual hover xyz        =",
        hover_action[:3].tolist(),
    )

    print(
        "actual hover gripper    =",
        hover_action[-1],
    )

    if not np.allclose(
        hover_action[:3],
        expected_hover,
        atol=1e-8,
        rtol=0.0,
    ):
        raise AssertionError(
            "The grasp hover is NOT based on WP2."
        )

    if not np.isclose(
        hover_action[-1],
        float(
            controller.cfg.control.get(
                "gripper_open_position",
                0.0,
            )
        ),
    ):
        raise AssertionError(
            "Gripper is not OPEN at grasp hover."
        )

    print(
        "\n[PASS] grasp hover uses exactly WP2 + hover_height"
    )

    print(
        "[PASS] grasp hover has gripper OPEN"
    )

    # =================================================================
    # 8. Check pending plan
    # =================================================================

    pending = controller._pending_grasp_plan

    if pending is None:
        raise AssertionError(
            "No pending grasp plan was created."
        )

    remaining = [
        item["primitive"]
        for item in pending["remaining_decisions"]
    ]

    print("\nRemaining plan after grasp hover:")
    print("   ", remaining)

    if remaining != [
        "carry",
        "carry",
        "drop",
    ]:
        raise AssertionError(
            f"Unexpected remaining plan: {remaining}"
        )

    print(
        "[PASS] after WP2 grasp, remaining plan "
        "= WP3 carry, WP4 carry, WP5 drop"
    )

    # =================================================================
    # 9. Optional second phase
    # =================================================================

    if args.simulate_resume:
        print("\n" + "=" * 90)
        print("SIMULATING t=1 WITH WRIST DEPTH DISABLED")
        print("=" * 90)

        original_use_depth = bool(
            controller.cfg.grasp_refinement.get(
                "use_gripper_depth",
                True,
            )
        )

        # Disable only physical depth access.
        # Refinement logic itself remains enabled.
        controller.cfg.grasp_refinement[
            "use_gripper_depth"
        ] = False

        try:
            actions_t1 = controller.inference(
                input_data,
                t=1,
                save_path=str(output_dir),
            )
        finally:
            controller.cfg.grasp_refinement[
                "use_gripper_depth"
            ] = original_use_depth

        print_actions(
            actions_t1,
            "EXECUTION ACTIONS t=1 - GRASP/CARRY/DROP",
        )

        open_position = float(
            controller.cfg.control.get(
                "gripper_open_position",
                0.0,
            )
        )

        closed_position = float(
            controller.cfg.control.get(
                "gripper_closed_position",
                255.0,
            )
        )

        # First 4 actions = approach, descend, close, lift.
        expected_first_grippers = [
            open_position,
            open_position,
            closed_position,
            closed_position,
        ]

        actual_first_grippers = [
            actions_t1[i][-1]
            for i in range(4)
        ]

        if not np.allclose(
            actual_first_grippers,
            expected_first_grippers,
        ):
            raise AssertionError(
                "Wrong post-hover grasp gripper sequence.\n"
                f"expected={expected_first_grippers}\n"
                f"actual={actual_first_grippers}"
            )

        print(
            "\n[PASS] post-hover sequence = "
            "approach OPEN -> descend OPEN -> "
            "close CLOSED -> lift CLOSED"
        )

        # Find first closed command.
        grippers = np.asarray(
            [
                action[-1]
                for action in actions_t1
            ],
            dtype=np.float64,
        )

        closed_indices = np.where(
            np.isclose(
                grippers,
                closed_position,
            )
        )[0]

        if closed_indices.size == 0:
            raise AssertionError(
                "No gripper CLOSE command found."
            )

        first_close = int(
            closed_indices[0]
        )

        # All actions after the close, except final release,
        # must remain CLOSED.
        for i in range(
            first_close,
            len(actions_t1) - 1,
        ):
            if not np.isclose(
                actions_t1[i][-1],
                closed_position,
            ):
                raise AssertionError(
                    f"Gripper reopened early at action {i}."
                )

        if not np.isclose(
            actions_t1[-1][-1],
            open_position,
        ):
            raise AssertionError(
                "Last action is not OPEN."
            )

        print(
            "[PASS] gripper remains CLOSED "
            "until WP5 final release"
        )

    # =================================================================
    # Done
    # =================================================================

    print("\n" + "=" * 90)
    print("TEST PASSED")
    print("=" * 90)

    print(
        f"""
Debug files saved in:

    {output_dir}

Most useful file:

    osvi_awda_raw_waypoints_t000.json

It contains:
    - all 15 predicted waypoints
    - selected 5 waypoints
    - projected base waypoints
    - coarse actions
    - gripper decisions
    - execution actions
"""
    )


if __name__ == "__main__":
    main()