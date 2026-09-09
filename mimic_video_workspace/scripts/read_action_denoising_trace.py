#!/usr/bin/env python3

import argparse
import numpy as np


def print_action(action):
    dx, dy, dz, dr, dp, dyaw, gripper = action

    return (
        f"dxyz=[{dx:+.7f}, {dy:+.7f}, {dz:+.7f}]  "
        f"dRPY=[{dr:+.7f}, {dp:+.7f}, {dyaw:+.7f}]  "
        f"gripper={gripper:+.7f}"
    )


def main():
    parser = argparse.ArgumentParser(
        description="Inspect Mimic Video action denoising traces."
    )

    parser.add_argument(
        "npz_path",
        help="Path to action_denoising_trace.npz",
    )

    parser.add_argument(
        "--action-index",
        type=int,
        default=0,
        help="Action of the 15-action chunk to track through denoising (0-14).",
    )

    parser.add_argument(
        "--denoise-step",
        type=int,
        default=None,
        help=(
            "If specified, print all 15 actions at this denoising state "
            "(0=initial noise, 10=final)."
        ),
    )

    parser.add_argument(
        "--normalized",
        action="store_true",
        help="Read normalized samples instead of denormalized physical actions.",
    )

    args = parser.parse_args()

    data = np.load(args.npz_path)

    key = (
        "action_samples_normalized"
        if args.normalized
        else "action_samples_denormalized"
    )

    actions = data[key]
    timesteps = data["timesteps"]

    print(f"File: {args.npz_path}")
    print(f"Representation: {key}")
    print(f"actions.shape = {actions.shape}")
    print(f"timesteps      = {timesteps}")
    print()

    if actions.ndim != 3:
        raise ValueError(
            f"Expected actions with shape (D, H, 7), got {actions.shape}"
        )

    num_states, horizon, action_dim = actions.shape

    # ------------------------------------------------------------
    # Modalità 1:
    # stampa tutte le 15 action a un preciso step di denoising
    # ------------------------------------------------------------
    if args.denoise_step is not None:
        d = args.denoise_step

        if not 0 <= d < num_states:
            raise ValueError(
                f"--denoise-step must be in [0, {num_states - 1}]"
            )

        print(
            f"=== DENOISING STATE {d} "
            f"(t={timesteps[d]:.6f}) ==="
        )

        for action_idx in range(horizon):
            print(
                f"action[{action_idx:02d}]  "
                f"{print_action(actions[d, action_idx])}"
            )

        return

    # ------------------------------------------------------------
    # Modalità 2:
    # segue una singola action attraverso tutto il denoising
    # ------------------------------------------------------------
    action_idx = args.action_index

    if not 0 <= action_idx < horizon:
        raise ValueError(
            f"--action-index must be in [0, {horizon - 1}]"
        )

    print(
        f"=== ACTION {action_idx} THROUGH DENOISING ==="
    )

    for denoise_idx, t in enumerate(timesteps):
        print(
            f"denoise={denoise_idx:02d} "
            f"t={t:.6f}  "
            f"{print_action(actions[denoise_idx, action_idx])}"
        )


if __name__ == "__main__":
    main()


# Per vedere evoluzione dell'action 0 di un certo step

# python /workspace/mimic_video_workspace/scripts/read_action_denoising_trace.py \
# /home/ros2_ws/src/ai_controller/saved_images/task_pick_place/task_03/traj_005/mimic_video_input_history/query_step_000010/action_denoising_trace.npz


# Per vedere evoluzione della quinta action

# python /workspace/mimic_video_workspace/scripts/read_action_denoising_trace.py \
# <PERCORSO_NPZ> \
# --action-index 4
# --normalized