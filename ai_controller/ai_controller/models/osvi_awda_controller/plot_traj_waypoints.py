#!/usr/bin/env python3
"""TEMPORARY script: plot the waypoint path recorded in a saved OSVI-AWDA
rollout traj_*.pkl (dataset_collector_pkg's savers.Trajectory format).

IMPORTANT caveat about what "waypoints" means here: ai_controller_node.py's
control loop calls osvi_awda_controller.inference() once per outer `step`,
which internally expands into several *executed* actions on the real robot
(e.g. approach -> grasp-hover -> [depth refine] -> close -> lift -> carry ->
drop), but only appends ONE Trajectory sample per outer `step` (using the
robot_state captured *before* that step's actions, and the *last* action of
that step). So a traj_*.pkl generally does NOT contain the full per-waypoint
AWDA plan (that richer 5-waypoint dump only exists transiently in
saved_images/task_<name>/step_<n>/osvi_awda_raw_waypoints_t*.json, which is
overwritten on every rollout and not tied to a specific trajectory file).

What this script actually plots, per outer step t = 0..T-1:
  - the eef_pos recorded in obs BEFORE that step's actions ran (state_t)
  - the [x, y, z] of the `action` field, i.e. the LAST commanded pose that
    step executed (target_t)
Consecutive state_{t+1} ~= target_t (the robot reached that target), so the
script renders a single connected path: start -> target_0 -> target_1 -> ...
-> target_{T-1}, colored by the commanded gripper state (open/closed) of
each point, next to the front-camera frame captured at step 0 for context.

Usage:
    python3 plot_traj_waypoints.py <path/to/traj_XXX.pkl> [--savers-dir DIR]
                                    [--output out.png] [--show]
"""
import argparse
import pickle
import sys
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers 3D projection)

DEFAULT_SAVERS_DIR = (
    Path(__file__).resolve().parents[4] / 'dataset_collector' / 'dataset_collector_pkg' / 'scripts'
)
GRIPPER_CLOSED_THRESHOLD = 127.0


def _add_savers_to_path(savers_dir=None):
    savers_dir = str(savers_dir or DEFAULT_SAVERS_DIR)
    if savers_dir not in sys.path:
        sys.path.insert(0, savers_dir)
    import savers  # noqa: F401  (import check only)


def _load_trajectory(pkl_path):
    with open(pkl_path, 'rb') as f:
        data = pickle.load(f)
    return data['traj'], data


def _gather_points(traj):
    """Returns (points[N,3], is_closed[N], labels[N]) with N = T+1:
    point 0 is the pre-step-0 eef_pos (rendered as a distinct 'start'
    marker, not part of the gripper-state color coding since gripper_qpos
    is on a different scale than the commanded 0/255 action value), point
    i (1..T) is step (i-1)'s commanded action target, colored by whether
    that action commanded the gripper closed."""
    points, is_closed, labels = [], [], []

    step0 = traj.get(0)
    start_pos = np.asarray(step0['obs']['eef_pos'], dtype=np.float64)
    points.append(start_pos)
    is_closed.append(None)  # start point: no commanded gripper action yet
    labels.append('start')

    for t in range(traj.T):
        step = traj.get(t)
        action = np.asarray(step['action'], dtype=np.float64)
        points.append(action[:3])
        is_closed.append(float(action[-1]) > GRIPPER_CLOSED_THRESHOLD)
        labels.append(f'step {t}')

    return np.stack(points, axis=0), is_closed, labels


def _point_colors(is_closed):
    color_by_state = {None: '#2c3e50', True: '#c0392b', False: '#2980b9'}
    return [color_by_state[state] for state in is_closed]


def plot_trajectory(pkl_path, output_path=None, savers_dir=None, show=False):
    pkl_path = Path(pkl_path)
    _add_savers_to_path(savers_dir)
    traj, meta = _load_trajectory(pkl_path)

    if traj.T == 0:
        raise ValueError(f'{pkl_path} has an empty trajectory (T=0).')

    points, is_closed, labels = _gather_points(traj)
    colors = _point_colors(is_closed)

    front_image = traj.get(0)['obs'].get('camera_front_image')

    fig = plt.figure(figsize=(16, 5.5))
    title = (
        f"OSVI-AWDA rollout waypoints - {pkl_path.name}  "
        f"(task_id={meta.get('task_id')}, traj={meta.get('traj_number')}, "
        f"completed={meta.get('completed')})"
    )
    fig.suptitle(title, fontsize=12)

    # Panel 1: front camera frame at t=0 (the image fed to inference()).
    ax_img = fig.add_subplot(1, 3, 1)
    if front_image is not None:
        ax_img.imshow(front_image[:, :, ::-1])  # stored as BGR -> RGB for display
    ax_img.set_title('camera_front_image (t=0, inference input)')
    ax_img.axis('off')

    # Panel 2: 3D path in base_link frame.
    ax3d = fig.add_subplot(1, 3, 2, projection='3d')
    ax3d.plot(points[:, 0], points[:, 1], points[:, 2], '-', color='#7f8c8d', linewidth=1.5, zorder=1)
    ax3d.scatter(points[:, 0], points[:, 1], points[:, 2], c=colors, s=60, zorder=2, depthshade=False)
    for i, (p, label) in enumerate(zip(points, labels)):
        ax3d.text(p[0], p[1], p[2], f'  {i}:{label}', fontsize=8)
    ax3d.set_xlabel('x [m]')
    ax3d.set_ylabel('y [m]')
    ax3d.set_zlabel('z [m]')
    ax3d.set_title('Waypoint path (base_link), 3D')

    # Panel 3: top-down XY view, easier to read in a slide.
    ax2d = fig.add_subplot(1, 3, 3)
    ax2d.plot(points[:, 0], points[:, 1], '-', color='#7f8c8d', linewidth=1.5, zorder=1)
    ax2d.scatter(points[:, 0], points[:, 1], c=colors, s=80, zorder=2)
    for i, (p, label) in enumerate(zip(points, labels)):
        ax2d.annotate(f'{i}:{label}', (p[0], p[1]), textcoords='offset points', xytext=(6, 6), fontsize=8)
    ax2d.set_xlabel('x [m]')
    ax2d.set_ylabel('y [m]')
    ax2d.set_title('Waypoint path (base_link), top-down XY')
    ax2d.set_aspect('equal', adjustable='datalim')
    ax2d.grid(True, linestyle=':', alpha=0.5)

    handles = [
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='#2c3e50', markersize=9, label='start (eef_pos)'),
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='#2980b9', markersize=9, label='gripper open'),
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='#c0392b', markersize=9, label='gripper closed'),
    ]
    ax2d.legend(handles=handles, loc='best', fontsize=8)

    fig.tight_layout(rect=[0, 0, 1, 0.94])

    output_path = Path(output_path) if output_path else pkl_path.with_name(f'{pkl_path.stem}_waypoints.png')
    fig.savefig(output_path, dpi=200)
    print(f'Saved: {output_path}')
    if show:
        plt.show()
    plt.close(fig)
    return output_path


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('pkl_path', help='Path to a traj_*.pkl saved rollout file')
    parser.add_argument('--savers-dir', default=None, help='Path to dataset_collector_pkg/scripts')
    parser.add_argument('--output', default=None, help='Output PNG path (default: alongside the pkl)')
    parser.add_argument('--show', action='store_true', help='Also display the figure interactively')
    args = parser.parse_args()

    plot_trajectory(args.pkl_path, output_path=args.output, savers_dir=args.savers_dir, show=args.show)


if __name__ == '__main__':
    main()
