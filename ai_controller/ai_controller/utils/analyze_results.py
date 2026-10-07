#!/usr/bin/env python3
"""Aggregate real-robot rollout outcomes saved by ai_controller_node.py.

Real-world counterpart of Multi-Task-LFD/repo/VLA-Bench/robosuite_test/
analyze_results.py. Expected layout (save_rollout() + the per-trajectory
intermediate folder):

    <path>/task_<id>/traj_<cnt>.json                 outcome (object_reached, ...)
    <path>/task_<id>/traj_<cnt>/cosmos/caption.json  Cosmos caption (optional)

Unlike the simulator there are no repeated runs, so statistics are computed
over trajectories (overall, per task, per object color, per target bin), plus
a macro average over tasks. When caption.json files exist, Cosmos caption
color/bin accuracy is computed with the same substring check as VLA-Bench.

Usage:
    python3 analyze_results.py --path .../saved_rollouts/vla_jepa_controller/pick_place/cosmos
"""
import argparse
import glob
import json
import os
import re

import numpy as np

VARIATION_OBJ_COLOR = {
    "pick_place": {
        0: "green", 1: "green", 2: "green", 3: "green",
        4: "yellow", 5: "yellow", 6: "yellow", 7: "yellow",
        8: "blue", 9: "blue", 10: "blue", 11: "blue",
        12: "red", 13: "red", 14: "red", 15: "red",
    }
}
BIN_NUMBER_DESCRIPTION = {
    "pick_place": {0: "first", 1: "second", 2: "third", 3: "fourth"}
}
NON_METRIC_KEYS = {"abort_reason"}


def _stats(values):
    arr = np.asarray(values, dtype=np.float64)
    return {"mean": round(float(arr.mean()), 3), "std": round(float(arr.std()), 3), "n": int(arr.size)}


def _aggregate(records):
    metrics = {}
    for rec in records:
        for key, value in rec["outcome"].items():
            if key in NON_METRIC_KEYS or not isinstance(value, (int, float)):
                continue
            metrics.setdefault(key, []).append(value)
    return {key: _stats(values) for key, values in metrics.items()}


def _load_records(path, exclude_aborted):
    records, missing_outcome = [], []
    for task_dir in sorted(glob.glob(os.path.join(path, "task_*"))):
        if not os.path.isdir(task_dir):
            continue
        task_id = int(os.path.basename(task_dir).split("_")[-1])
        for traj_dir in sorted(glob.glob(os.path.join(task_dir, "traj_*"))):
            if os.path.isdir(traj_dir) and not os.path.isfile(traj_dir + ".json"):
                missing_outcome.append(os.path.relpath(traj_dir, path))
        for outcome_file in sorted(glob.glob(os.path.join(task_dir, "traj_*.json"))):
            with open(outcome_file, "r") as stream:
                outcome = json.load(stream)
            if exclude_aborted and outcome.get("aborted", 0):
                continue
            traj_name = os.path.basename(outcome_file)[:-len(".json")]
            caption_file = os.path.join(task_dir, traj_name, "cosmos", "caption.json")
            caption = None
            if os.path.isfile(caption_file):
                with open(caption_file, "r") as stream:
                    caption = json.load(stream)
            records.append({"task_id": task_id, "traj": traj_name, "outcome": outcome, "caption": caption})
    return records, missing_outcome


def _caption_bin_correct(rec, task_name):
    """1/0 whether the Cosmos caption names the ground-truth bin; None if
    there's no caption or no ground truth for this task id."""
    if rec["caption"] is None or rec["task_id"] not in VARIATION_OBJ_COLOR[task_name]:
        return None
    gt_bin = BIN_NUMBER_DESCRIPTION[task_name][rec["task_id"] % 4]
    return int(gt_bin in set(re.findall(r"[a-z]+", rec["caption"]["caption"].lower())))


def _fill_place_correct_wrong_bin(records, task_name):
    """place_correct_wrong_bin (correct object, wrong bin) is only recorded in
    newer outcome files. Where it's missing, infer it: object picked, not
    placed, and the Cosmos caption named the wrong bin - i.e. the policy most
    likely followed the wrong instruction."""
    counts = {"recorded": 0, "inferred": 0}
    for rec in records:
        outcome = rec["outcome"]
        if "place_correct_wrong_bin" in outcome:
            counts["recorded"] += 1
            continue
        bin_correct = _caption_bin_correct(rec, task_name)
        outcome["place_correct_wrong_bin"] = int(
            outcome.get("object_picked", 0) == 1
            and outcome.get("object_placed", 0) == 0
            and bin_correct == 0
        )
        rec["place_correct_wrong_bin_inferred"] = True
        counts["inferred"] += 1
    return counts


def _caption_accuracy(records, task_name):
    color_acc, bin_acc, both_acc, mismatches, per_task = [], [], [], [], {}
    for rec in records:
        if rec["caption"] is None:
            continue
        task_id = rec["task_id"]
        gt_color = VARIATION_OBJ_COLOR[task_name].get(task_id)
        if gt_color is None:
            continue  # task ids outside 00-15 have no color/bin ground truth
        gt_bin = BIN_NUMBER_DESCRIPTION[task_name][task_id % 4]
        caption = rec["caption"]["caption"]
        # same substring check as VLA-Bench, but on whole words so e.g.
        # "red" doesn't match inside another word
        words = set(re.findall(r"[a-z]+", caption.lower()))
        color_ok = int(gt_color in words)
        bin_ok = int(gt_bin in words)
        color_acc.append(color_ok)
        bin_acc.append(bin_ok)
        both_acc.append(color_ok * bin_ok)
        per_task.setdefault(f"task_{task_id:02d}", []).append(color_ok * bin_ok)
        if not (color_ok and bin_ok):
            mismatches.append({
                "task_id": f"{task_id:02d}",
                "traj": rec["traj"],
                "caption": caption,
                "raw_caption": rec["caption"].get("raw_caption"),
                "gt_color": gt_color,
                "gt_bin": gt_bin,
                "color_correct": color_ok,
                "bin_correct": bin_ok,
                "demo_file": rec["caption"].get("demo_file"),
            })
    if not color_acc:
        return None, mismatches
    return {
        "color_accuracy": _stats(color_acc),
        "bin_accuracy": _stats(bin_acc),
        "color_and_bin_accuracy": _stats(both_acc),
        "per_task_color_and_bin_accuracy": {k: _stats(v) for k, v in sorted(per_task.items())},
    }, mismatches


def main():
    parser = argparse.ArgumentParser(description="Analyze real-robot rollout results.")
    parser.add_argument("--path", required=True,
                        help="Folder containing task_<id>/ subfolders, e.g. "
                             ".../saved_rollouts/vla_jepa_controller/pick_place/cosmos")
    parser.add_argument("--task_name", default="pick_place")
    parser.add_argument("--exclude_aborted", action="store_true",
                        help="Drop trajectories saved with aborted=1 (ESC / errors).")
    parser.add_argument("--exclude_tasks", nargs="*", type=int, default=[],
                        help="Task ids to leave out of all statistics, e.g. --exclude_tasks 0 "
                             "for an out-of-distribution task.")
    args = parser.parse_args()

    records, missing_outcome = _load_records(args.path, args.exclude_aborted)
    records = [rec for rec in records if rec["task_id"] not in args.exclude_tasks]
    missing_outcome = [m for m in missing_outcome
                       if int(m.split("/")[0].split("_")[-1]) not in args.exclude_tasks]
    if not records:
        print(f"No task_*/traj_*.json outcome files found under {args.path}")
        raise SystemExit(1)
    pcwb_counts = _fill_place_correct_wrong_bin(records, args.task_name)
    pcwb_trajs = [
        {"task_id": f"{rec['task_id']:02d}", "traj": rec["traj"],
         "source": "inferred" if rec.get("place_correct_wrong_bin_inferred") else "recorded"}
        for rec in records if rec["outcome"]["place_correct_wrong_bin"]
    ]

    by_task, by_color, by_bin = {}, {}, {}
    for rec in records:
        task_id = rec["task_id"]
        by_task.setdefault(f"task_{task_id:02d}", []).append(rec)
        color = VARIATION_OBJ_COLOR[args.task_name].get(task_id)
        if color is not None:
            by_color.setdefault(color, []).append(rec)
            by_bin.setdefault(BIN_NUMBER_DESCRIPTION[args.task_name][task_id % 4], []).append(rec)

    per_task = {task: _aggregate(recs) for task, recs in sorted(by_task.items())}
    macro = {}
    for task_metrics in per_task.values():
        for metric, stats in task_metrics.items():
            macro.setdefault(metric, []).append(stats["mean"])

    caption_results, mismatches = _caption_accuracy(records, args.task_name)

    final_results = {
        "path": os.path.abspath(args.path),
        "num_trajectories": len(records),
        "num_tasks": len(by_task),
        "exclude_aborted": args.exclude_aborted,
        "exclude_tasks": args.exclude_tasks,
        "trajectories_without_outcome": missing_outcome,
        "place_correct_wrong_bin_source": pcwb_counts,
        "place_correct_wrong_bin_trajectories": pcwb_trajs,
        "overall": _aggregate(records),
        "macro_over_tasks": {metric: _stats(means) for metric, means in macro.items()},
        "per_task": per_task,
        "per_color": {color: _aggregate(recs) for color, recs in sorted(by_color.items())},
        "per_bin": {b: _aggregate(recs) for b, recs in sorted(by_bin.items())},
    }
    if caption_results is not None:
        final_results["cosmos_caption"] = caption_results

    print(f"{len(records)} trajectories over {len(by_task)} tasks in {args.path}")
    if missing_outcome:
        print(f"Skipped (no outcome .json, e.g. not saved): {missing_outcome}")
    header = ["object_reached", "object_picked", "object_placed", "place_correct_wrong_bin", "aborted",
              "trajectory_execution_sec"]
    print(f"{'':10s}" + "".join(f"{h:>26s}" for h in header))
    for name, metrics in [("overall", final_results["overall"])] + list(per_task.items()):
        row = "".join(
            f"{metrics[h]['mean']:>18.3f} ± {metrics[h]['std']:<5.3f}" if h in metrics else f"{'-':>26s}"
            for h in header)
        print(f"{name:10s}{row}")
    pcwb_list = ["{}/{} ({})".format(t["task_id"], t["traj"], t["source"]) for t in pcwb_trajs]
    print(f"place_correct_wrong_bin: {len(pcwb_trajs)} trajectories "
          f"(values recorded in {pcwb_counts['recorded']} outcome files, inferred for "
          f"{pcwb_counts['inferred']}): {pcwb_list}")
    if caption_results is not None:
        print(f"Cosmos color acc: {caption_results['color_accuracy']['mean']:.3f}  "
              f"bin acc: {caption_results['bin_accuracy']['mean']:.3f}  "
              f"both: {caption_results['color_and_bin_accuracy']['mean']:.3f}  "
              f"(n={caption_results['color_accuracy']['n']}, mismatches={len(mismatches)})")

    suffix = f"{args.task_name}{'_no_aborted' if args.exclude_aborted else ''}"
    output_file = os.path.join(args.path, f"final_results_{suffix}.json")
    with open(output_file, "w") as stream:
        json.dump(final_results, stream, indent=4)
    print(f"Saved {output_file}")

    if caption_results is not None:
        details_file = os.path.join(args.path, f"task_description_vs_gt_{suffix}.json")
        with open(details_file, "w") as stream:
            json.dump(mismatches, stream, indent=4)
        print(f"Saved {details_file}")


if __name__ == "__main__":
    main()
