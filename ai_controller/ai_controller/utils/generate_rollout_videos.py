#!/usr/bin/env python3
"""Generate H.264 (.mp4) videos from saved rollout .pkl trajectories.

For every traj_*.pkl found under a root directory, loads its savers.Trajectory
and, for every per-step obs key that looks like a camera frame
(camera_..._image for RGB, camera_..._depth for depth - i.e. the fields
AIControllerNode.save_rollout() records: camera_front_image and, if present,
camera_lateral_left_image, camera_lateral_right_image, camera_gripper_image,
camera_lateral_left_depth, camera_lateral_right_depth, camera_gripper_depth),
encodes one video per camera key next to the source .pkl.

Depth frames (float32, meters, NaN/Inf where invalid) are colorized using a
FIXED min/max range computed once from all finite values across the whole
trajectory for that camera - not per-frame - so the video doesn't flicker the
way per-frame auto-normalization does with NaN-heavy depth data (see the
RViz "Normalize Range" issue this mirrors).

True H.264 is encoded via an external `ffmpeg -c:v libx264` subprocess rather
than cv2.VideoWriter, since most opencv-python wheels aren't built with a
licensed H.264 encoder. Requires the `ffmpeg` binary (`apt install ffmpeg`).

Usage:
    python3 generate_rollout_videos.py [root_dir] [--fps 5] [--savers-dir DIR]
                                        [--colormap turbo|jet] [--overwrite]

    root_dir defaults to
    /home/ros2_ws/src/ai_controller/saved_rollouts/osvi_awda_controller/pick_place
"""
import argparse
import pickle
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import cv2

DEFAULT_ROOT = '/home/ros2_ws/src/ai_controller/saved_rollouts/osvi_awda_controller/pick_place'
COLORMAPS = {
    'turbo': getattr(cv2, 'COLORMAP_TURBO', cv2.COLORMAP_JET),
    'jet': cv2.COLORMAP_JET,
}


def _add_savers_to_path(savers_dir=None):
    """Make dataset_collector_pkg's savers.Trajectory importable, matching the
    convention used by ai_controller_node.py's _add_dataset_collector_scripts_to_path."""
    if savers_dir is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            savers_dir = str(Path(get_package_share_directory('dataset_collector_pkg')) / 'scripts')
        except Exception as exc:
            raise RuntimeError(
                'Could not locate dataset_collector_pkg via ament_index (not sourced?). '
                'Pass --savers-dir pointing at dataset_collector_pkg/scripts instead.'
            ) from exc
    if savers_dir not in sys.path:
        sys.path.insert(0, savers_dir)
    import savers  # noqa: F401  (import check only)


def _short_camera_name(key):
    """camera_front_image -> front_rgb, camera_gripper_depth -> gripper_depth, ..."""
    name = key[len('camera_'):] if key.startswith('camera_') else key
    if name.endswith('_image'):
        return name[: -len('_image')] + '_rgb'
    return name


def _discover_camera_keys(obs):
    rgb_keys, depth_keys = [], []
    for key in obs:
        if not key.startswith('camera_'):
            continue
        if key.endswith('_image'):
            rgb_keys.append(key)
        elif key.endswith('_depth'):
            depth_keys.append(key)
    return sorted(rgb_keys), sorted(depth_keys)


def _colorize_depth(frame, vmin, vmax, colormap):
    finite = np.isfinite(frame)
    clipped = np.where(finite, frame, vmax)
    clipped = np.clip(clipped, vmin, vmax)
    if vmax > vmin:
        normalized = ((clipped - vmin) / (vmax - vmin) * 255.0).astype(np.uint8)
    else:
        normalized = np.zeros(frame.shape, dtype=np.uint8)
    return cv2.applyColorMap(normalized, colormap)


def _encode_h264(frames, out_path, fps):
    if shutil.which('ffmpeg') is None:
        raise RuntimeError("ffmpeg binary not found on PATH. Install it with 'apt install ffmpeg'.")

    height, width = frames[0].shape[:2]
    cmd = [
        'ffmpeg', '-y', '-loglevel', 'error',
        '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-s', f'{width}x{height}', '-r', str(fps),
        '-i', '-',
        '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-preset', 'medium', '-crf', '23',
        str(out_path),
    ]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    for frame in frames:
        proc.stdin.write(np.ascontiguousarray(frame, dtype=np.uint8).tobytes())
    proc.stdin.close()
    if proc.wait() != 0:
        raise RuntimeError(f'ffmpeg failed encoding {out_path}')


def _gather_frames(traj, key, is_depth, colormap):
    """Collects the frame for `key` at every step that has it. Returns
    (frames, n_missing). For depth, colorizes using a fixed range computed
    once from all finite values across the trajectory."""
    raw_frames = []
    n_missing = 0
    for t in range(traj.T):
        obs = traj.get(t)['obs']
        frame = obs.get(key)
        if frame is None:
            n_missing += 1
            continue
        raw_frames.append(frame)

    if not raw_frames:
        return [], n_missing

    if not is_depth:
        return raw_frames, n_missing

    finite_vals = np.concatenate([f[np.isfinite(f)].ravel() for f in raw_frames])
    if finite_vals.size == 0:
        vmin, vmax = 0.0, 1.0
    else:
        vmin, vmax = float(finite_vals.min()), float(finite_vals.max())
    colorized = [_colorize_depth(f, vmin, vmax, colormap) for f in raw_frames]
    return colorized, n_missing


def generate_videos_for_trajectory(pkl_path, fps, colormap, overwrite):
    with pkl_path.open('rb') as f:
        data = pickle.load(f)
    traj = data['traj']

    if traj.T == 0:
        print(f'  [skip] {pkl_path.name}: empty trajectory')
        return

    rgb_keys, depth_keys = _discover_camera_keys(traj.get(0)['obs'])
    if not rgb_keys and not depth_keys:
        print(f'  [skip] {pkl_path.name}: no camera_* fields found')
        return

    for key in rgb_keys + depth_keys:
        is_depth = key in depth_keys
        out_path = pkl_path.with_name(f'{pkl_path.stem}_{_short_camera_name(key)}.mp4')
        if out_path.exists() and not overwrite:
            print(f'  [skip] {out_path.name} already exists (use --overwrite)')
            continue

        frames, n_missing = _gather_frames(traj, key, is_depth, colormap)
        if not frames:
            print(f'  [warn] {pkl_path.name}: no frames for {key}')
            continue
        if n_missing:
            print(f'  [warn] {pkl_path.name}: {key} missing at {n_missing}/{traj.T} steps, skipped those')

        _encode_h264(frames, out_path, fps)
        print(f'  [ok] {out_path.name} ({len(frames)} frames @ {fps}fps)')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('root_dir', nargs='?', default=DEFAULT_ROOT,
                         help=f'Directory to search for traj_*.pkl (default: {DEFAULT_ROOT})')
    parser.add_argument('--fps', type=float, default=5.0, help='Output video framerate (default: 5)')
    parser.add_argument('--savers-dir', default=None,
                         help="Path to dataset_collector_pkg/scripts (auto-detected via ament_index if omitted)")
    parser.add_argument('--colormap', choices=sorted(COLORMAPS), default='turbo',
                         help='Colormap for depth videos (default: turbo)')
    parser.add_argument('--overwrite', action='store_true', help='Re-encode videos that already exist')
    args = parser.parse_args()

    _add_savers_to_path(args.savers_dir)

    root = Path(args.root_dir)
    pkl_files = sorted(root.rglob('traj_*.pkl'))
    if not pkl_files:
        print(f'No traj_*.pkl files found under {root}')
        return

    print(f'Found {len(pkl_files)} trajectory file(s) under {root}')
    for pkl_path in pkl_files:
        print(f'{pkl_path.relative_to(root)}:')
        try:
            generate_videos_for_trajectory(pkl_path, args.fps, COLORMAPS[args.colormap], args.overwrite)
        except Exception as exc:
            print(f'  [error] {pkl_path.name}: {exc}')


if __name__ == '__main__':
    main()
