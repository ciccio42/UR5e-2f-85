#!/usr/bin/env python3
"""Real-robot pick-and-place results table (TD / OTD) for several models.

Same statistics as Multi-Task-LFD/repo/VLA-Bench/robosuite_test/analyze_results.py:
for Reach / Pick / Success / Succ. W.O. / Succ. W.B. it reports
  1. the overall mean over all trajectories,
  2. the mean of each task variation,
  3. the per-task mean: mean and (population) std across the per-variation means,
separately for the Training-Distribution (TD) tasks and the Out-of-Training-Distribution
(OTD) tasks 0, 5, 10, 15, plus a LaTeX row "P. & S. & W.O. & W.B." for each split.

Metrics (outcome .json written by ai_controller_node.py's save_rollout, or SeeDo's):
  reach   = object_reached
  pick    = object_picked
  success = object_placed
  succ_wo = place_wrong_correct_bin   (a wrong object placed in the commanded bin)
  succ_wb = place_correct_wrong_bin   (the commanded object placed in a wrong bin). Only
            recorded by newer runs; if missing and the trajectory has a Cosmos caption
            (traj_<n>/cosmos/caption.json), it is inferred as picked & not placed & caption
            names the wrong bin, otherwise 0.
Missing / null values (e.g. SeeDo runs that failed before execution) count as 0.

Layouts: <path>/task_<id>/traj_<n>.json (also <path>/task_<id>/traj_<n>/traj_<n>.json);
if <path> has no task_<id>/ children it is searched recursively (e.g. SeeDo's
test_v1/test_task<id>_<k>/rollouts/seedo_controller/pick_place/task_<id>/traj_<n>.json).

Usage:
  python3 real_world_table.py --per_task 4 \\
      --model "COD-Policy=/path/cod_controller/pick_place/epoch_100/wrist" \\
      --model "C.+VLA-JEPA=/path/vla_jepa_controller/pick_place/cosmos" [--out results.json]
"""
import argparse
import collections
import glob
import json
import os
import re

import numpy as np

OTD_TASKS = {0, 5, 10, 15}
BIN_WORD = {0: "first", 1: "second", 2: "third", 3: "fourth"}
METRICS = {
    "reach": "object_reached",
    "pick": "object_picked",
    "success": "object_placed",
    "succ_wo": "place_wrong_correct_bin",
    "succ_wb": "place_correct_wrong_bin",
}
LATEX_METRICS = ("pick", "success", "succ_wo", "succ_wb")


def _natural_key(path):
    return [int(tok) if tok.isdigit() else tok for tok in re.split(r"(\d+)", path)]


def _outcome_files(path):
    patterns = [os.path.join(path, "task_[0-9]*", "traj_*.json"),
                os.path.join(path, "task_[0-9]*", "traj_*", "traj_*.json")]
    files = [f for p in patterns for f in glob.glob(p)]
    if not files:
        patterns = [os.path.join(path, "**", "task_[0-9]*", "traj_*.json")]
        files = [f for p in patterns for f in glob.glob(p, recursive=True)]
    return sorted(set(files), key=_natural_key)


def _cosmos_bin_wrong(outcome_file, task_id):
    stem = outcome_file[:-len(".json")]
    for caption_file in (os.path.join(stem, "cosmos", "caption.json"),
                         os.path.join(os.path.dirname(outcome_file), "cosmos", "caption.json")):
        if os.path.isfile(caption_file):
            caption = json.load(open(caption_file))["caption"].lower()
            return BIN_WORD[task_id % 4] not in set(re.findall(r"[a-z]+", caption))
    return None


def load_episodes(path):
    episodes = []
    for f in _outcome_files(path):
        task_id = int(re.search(r"task_(\d+)", os.path.relpath(f, path)).group(1))
        if task_id > 15:
            continue
        data = json.load(open(f))
        ep = {"task": task_id, "file": f}
        for name, key in METRICS.items():
            value = data.get(key)
            ep[name] = float(value) if value is not None else 0.0
        if "place_correct_wrong_bin" not in data:
            bin_wrong = _cosmos_bin_wrong(f, task_id)
            ep["succ_wb"] = float(bool(bin_wrong) and ep["pick"] == 1.0 and ep["success"] == 0.0)
        episodes.append(ep)
    return episodes


def first_n_per_task(episodes, n):
    kept, count = [], collections.Counter()
    for e in episodes:
        if count[e["task"]] < n:
            kept.append(e)
            count[e["task"]] += 1
    return kept


def compute_stats(episodes):
    by_task = collections.defaultdict(list)
    for e in episodes:
        by_task[e["task"]].append(e)
    stats = {"n_episodes": len(episodes), "n_tasks": len(by_task), "overall": {}, "per_variation": {},
             "per_task": {}}
    for name in METRICS:
        stats["overall"][name] = float(np.mean([e[name] for e in episodes])) if episodes else float("nan")
        means = {t: float(np.mean([e[name] for e in eps])) for t, eps in sorted(by_task.items())}
        for t, m in means.items():
            stats["per_variation"].setdefault(t, {"n": len(by_task[t])})[name] = m
        values = list(means.values())
        stats["per_task"][name] = {"mean": float(np.mean(values)) if values else float("nan"),
                                   "std": float(np.std(values)) if values else float("nan")}
    return stats


def print_stats(label, stats):
    names = list(METRICS)
    print(f"\n=== {label}: {stats['n_episodes']} trajectories over {stats['n_tasks']} tasks")
    print("task    n  " + "  ".join(f"{n:>8s}" for n in names))
    for t, row in stats["per_variation"].items():
        print(f"{t:4d} {row['n']:4d}  " + "  ".join(f"{row[n]:8.2f}" for n in names))
    print("overall   " + "  ".join(f"{stats['overall'][n]:8.3f}" for n in names))
    print("per-task  " + "  ".join(
        f"{stats['per_task'][n]['mean']:.2f}±{stats['per_task'][n]['std']:.2f}" for n in names))


def latex_cells(stats):
    if not stats["n_episodes"]:
        return ["--"] * len(LATEX_METRICS)
    return [f"${stats['per_task'][n]['mean']:.2f} \\pm {stats['per_task'][n]['std']:.2f}$"
            for n in LATEX_METRICS]


def main():
    parser = argparse.ArgumentParser(description="Real-robot TD/OTD results table.")
    parser.add_argument("--model", action="append", required=True, metavar="NAME=PATH",
                        help="Model label and rollout folder; repeat for each model.")
    parser.add_argument("--per_task", type=int, default=None,
                        help="Keep only the first N trajectories of each task (e.g. 4).")
    parser.add_argument("--out", default=None, help="Optional JSON file for all statistics.")
    args = parser.parse_args()

    results, latex = {}, []
    for spec in args.model:
        label, path = spec.split("=", 1)
        episodes = load_episodes(path)
        if args.per_task is not None:
            episodes = first_n_per_task(episodes, args.per_task)
        td = compute_stats([e for e in episodes if e["task"] not in OTD_TASKS])
        otd = compute_stats([e for e in episodes if e["task"] in OTD_TASKS])
        print_stats(f"{label} TD", td)
        print_stats(f"{label} OTD", otd)
        results[label] = {"path": path, "TD": td, "OTD": otd}
        latex.append(f"{label} & " + " & ".join(latex_cells(td) + latex_cells(otd)) + r" \\")

    print("\n% P. & S. & W.O. & W.B. (TD) | P. & S. & W.O. & W.B. (OTD), per-task mean \\pm std")
    print("\n".join(latex))
    if args.out:
        with open(args.out, "w") as stream:
            json.dump(results, stream, indent=4)
        print(f"Saved {args.out}")


if __name__ == "__main__":
    main()
