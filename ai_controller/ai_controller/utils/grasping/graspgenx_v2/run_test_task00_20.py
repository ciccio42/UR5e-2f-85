"""Offline GraspGen-X V2 test for /test_real/test_task00_20.

Restored V2 geometry controls are NOT bypassed. The user may supply the grasp
step and instruction, or the script will match the saved original grasp RGB
and prompt for an instruction if not provided.

No robot/gripper commands. Requires ROS2 workspace environment sourced.
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
from unittest.mock import patch


def candidate_step(pkl: Path, reference: Path) -> int | None:
    if not reference.is_file():
        print(f"[STEP] Reference image not found: {reference}", flush=True)
        return None

    import cv2
    import numpy as np
    from ai_controller.models.seedo_controller.inspect_pkl import load_pickle

    ref = cv2.imread(str(reference), cv2.IMREAD_COLOR)
    if ref is None:
        print(f"[STEP] Cannot read {reference}", flush=True)
        return None

    data = load_pickle(pkl)  # Trusted local rollout only.
    traj = data['traj']
    differences = []
    for i in range(len(traj)):
        step = traj.get(i, decompress=True)
        obs = step['obs']
        frame = None
        for key in ('eye_in_hand_image', 'camera_gripper_image'):
            if key not in obs:
                continue
            a = np.asarray(obs[key])
            if a.ndim == 3 and a.shape[-1] == 3:
                frame = a
            elif a.dtype == np.uint8 and a.ndim in (1, 2):
                frame = cv2.imdecode(a.reshape(-1), cv2.IMREAD_COLOR)
            if frame is not None:
                break
        if frame is None or frame.shape != ref.shape:
            continue
        error = float(np.abs(frame.astype(np.float32) - ref.astype(np.float32)).mean())
        differences.append((error, i))

    if not differences:
        print('[STEP] No comparable eye-in-hand RGB frames. Specify --step.', flush=True)
        return None

    differences.sort()
    print('[STEP] Closest RGB frames (MAE, step):', differences[:5], flush=True)
    best_error, best_step = differences[0]
    if best_error > 8.0:
        print('[STEP] Best RGB mismatch is high; specify --step manually.', flush=True)
        return None
    print(f'[STEP] Auto-selected step={best_step}, RGB MAE={best_error:.3f}', flush=True)
    return best_step


def main() -> int:
    p = argparse.ArgumentParser(description='Safe offline V2 replay for test_task00_20')
    p.add_argument('--case', type=Path, default=Path('/test_real/test_task00_20'))
    p.add_argument('--pkl', type=Path)
    p.add_argument('--step', type=int)
    p.add_argument('--instruction', type=str)
    p.add_argument('--depth-offset-mm', type=float, default=None,
                   help='Optional diagnostic override. Default: original planner value.')
    args = p.parse_args()

    case = args.case
    pkl = args.pkl or case / 'rollouts/seedo_controller/pick_place/task_00/traj_000.pkl'
    if not pkl.is_file():
        alternatives = sorted(case.rglob('traj_000.pkl')) if case.is_dir() else []
        if len(alternatives) == 1 and args.pkl is None:
            pkl = alternatives[0]
        else:
            raise FileNotFoundError(
                f'Expected PKL {pkl}; alternatives: {alternatives}. Pass --pkl.'
            )

    step = args.step
    if step is None:
        step = candidate_step(pkl, case / 'grasp_planner/action_00/input_rgb.png')
    if step is None:
        step = int(input('Step PKL da usare per la presa: ').strip())
    if step < 0:
        raise ValueError('--step must be nonnegative')

    instruction = (args.instruction or '').strip()
    if not instruction:
        instruction = input('Istruzione originale di presa per test_task00_20: ').strip()
    if not instruction:
        raise ValueError('A precise target instruction is required; not guessing the object.')

    # Explicitly strip all diagnostic overrides from the test and worker.
    for key in (
        'GRASPGENX_V2_OFFLINE_DIAGNOSTIC',
        'GRASPGENX_V2_DIAG_SKIP_APPROACH_TARGET',
        'GRASPGENX_V2_DIAG_APPROACH_TIP_MM',
        'GRASPGENX_V2_DIAG_PREVIEW_FILE',
    ):
        os.environ.pop(key, None)

    root = Path('/test_real/graspgenx_v2_full')
    root.mkdir(parents=True, exist_ok=True)
    output = None
    for i in range(1, 1000):
        candidate = root / f'test_task00_20_reset_{i:03d}'
        if not candidate.exists():
            output = candidate
            break
    if output is None:
        raise RuntimeError('Could not find a new output directory')

    config = Path('/home/ros2_ws/src/UR5e-2f-85/ai_controller/'
                  'ai_controller/models/seedo_controller/config/seedo_controller.yaml')
    if not config.is_file():
        raise FileNotFoundError(config)

    from ai_controller.utils.grasping.graspgenx_v2.backend import GraspGenXV2Planner
    from ai_controller.utils.grasping.graspgenx_v2 import test_v2_full_inference as test

    init_original = GraspGenXV2Planner.__init__

    def checked_init(self, *a, **kw):
        init_original(self, *a, **kw)
        if self.v2_unsafe_enable_robot_commands:
            raise RuntimeError('Set graspgenx_v2_unsafe_enable_robot_commands: false')
        if args.depth_offset_mm is not None:
            self.grasp_depth_offset_m = args.depth_offset_mm / 1000.0
        print('[SAFE OFFLINE] depth offset [mm]:',
              self.grasp_depth_offset_m * 1000, flush=True)

    sys.argv = [
        'test_v2_full_inference.py',
        '--config', str(config),
        '--pkl', str(pkl),
        '--step', str(step),
        '--instruction', instruction,
        '--output-dir', str(output),
        '--skip-visualization',
    ]
    print('[SAFE OFFLINE] case:', case, flush=True)
    print('[SAFE OFFLINE] PKL:', pkl, 'step:', step, flush=True)
    print('[SAFE OFFLINE] instruction:', instruction, flush=True)
    print('[SAFE OFFLINE] artifacts:', output, flush=True)
    print('[SAFE OFFLINE] approach_target and closure constraints ACTIVE', flush=True)
    print('[SAFE OFFLINE] robot/gripper commands DISABLED', flush=True)
    with patch.object(GraspGenXV2Planner, '__init__', checked_init):
        return test.main()


if __name__ == '__main__':
    raise SystemExit(main())
