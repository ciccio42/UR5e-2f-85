#!/usr/bin/env python3
"""Verifica scala, allineamento e convenzione delle delta action UR5e."""

from __future__ import annotations

import argparse
import os
import re
from collections import defaultdict
from pathlib import Path

os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")

import numpy as np
import tensorflow_datasets as tfds
from scipy.spatial.transform import Rotation


DEFAULT_TFRECORD = Path("ur5e_pick_place-train.tfrecord-00000")
SHARD_RE = re.compile(r".+-train\.tfrecord-(\d+)$")


def angular_error(predicted: np.ndarray, expected: np.ndarray) -> np.ndarray:
    """Restituisce l'angolo della rotazione residua, in radianti."""
    residual = np.swapaxes(predicted, -1, -2) @ expected
    return Rotation.from_matrix(residual).magnitude()


def position_error(predicted: np.ndarray, expected: np.ndarray) -> np.ndarray:
    return np.linalg.norm(predicted - expected, axis=-1)


def summarize(values: np.ndarray, scale: float) -> tuple[float, float, float, float]:
    values = values * scale
    return np.mean(values), np.median(values), np.percentile(values, 95), np.max(values)


def print_table(title: str, metrics: dict[str, np.ndarray], mask: np.ndarray, scale: float, unit: str) -> None:
    print(f"\n========== {title} ({unit}) ==========")
    print(f"{'ipotesi':32s} {'media':>12s} {'mediana':>12s} {'p95':>12s} {'massimo':>12s}")
    for name, values in metrics.items():
        mean, median, p95, maximum = summarize(values[mask], scale)
        print(f"{name:32s} {mean:12.6f} {median:12.6f} {p95:12.6f} {maximum:12.6f}")


def read_shard(tfrecord_path: Path):
    match = SHARD_RE.match(tfrecord_path.name)
    if match is None:
        raise ValueError(f"Nome TFRecord non valido: {tfrecord_path.name}")

    shard_index = int(match.group(1))
    builder = tfds.builder_from_directory(str(tfrecord_path.parent))
    shard_lengths = list(builder.info.splits["train"].shard_lengths)
    if shard_index >= len(shard_lengths):
        raise ValueError(f"Shard {shard_index} assente dai metadati TFDS")

    start = sum(shard_lengths[:shard_index])
    stop = start + shard_lengths[shard_index]
    dataset = builder.as_dataset(split=f"train[{start}:{stop}]", shuffle_files=False)
    return dataset, shard_index, start, stop


def analyze_episode(
    episode: dict,
    scale_factor: float,
    euler_order: str,
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray], np.ndarray, dict[str, np.ndarray]]:
    steps = list(episode["steps"])
    if len(steps) < 2:
        raise ValueError("La traiettoria deve contenere almeno due step")

    actions = np.stack([step["action"].numpy() for step in steps]).astype(np.float64)
    states = np.stack([step["observation"]["EEF_state"].numpy() for step in steps]).astype(np.float64)

    current = states[:-1]
    expected = states[1:]
    action_t = actions[:-1]
    action_t1 = actions[1:]

    current_rot = Rotation.from_euler(euler_order, current[:, 3:6]).as_matrix()
    expected_rot = Rotation.from_euler(euler_order, expected[:, 3:6]).as_matrix()

    delta_t = action_t[:, :6] * scale_factor
    delta_t1 = action_t1[:, :6] * scale_factor
    delta_rot = Rotation.from_euler(euler_order, delta_t[:, 3:6]).as_matrix()
    delta_rot_t1 = Rotation.from_euler(euler_order, delta_t1[:, 3:6]).as_matrix()
    raw_delta_rot = Rotation.from_euler(euler_order, action_t[:, 3:6]).as_matrix()

    pos_predictions = {
        "action[t] scalata, base_link": current[:, :3] + delta_t[:, :3],
        "action[t] scalata, frame locale": current[:, :3]
        + np.einsum("tij,tj->ti", current_rot, delta_t[:, :3]),
        "action[t] NON scalata, base_link": current[:, :3] + action_t[:, :3],
        "action[t+1] scalata, base_link": current[:, :3] + delta_t1[:, :3],
    }

    rot_predictions = {
        "RPY corrente + action[t] scalata": Rotation.from_euler(
            euler_order, current[:, 3:6] + delta_t[:, 3:6]
        ).as_matrix(),
        "R_delta[t] @ R_corrente (base)": delta_rot @ current_rot,
        "R_corrente @ R_delta[t] (locale)": current_rot @ delta_rot,
        "RPY corrente + action[t] NON scalata": Rotation.from_euler(
            euler_order, current[:, 3:6] + action_t[:, 3:6]
        ).as_matrix(),
        "RPY corrente + action[t+1] scalata": Rotation.from_euler(
            euler_order, current[:, 3:6] + delta_t1[:, 3:6]
        ).as_matrix(),
        "R_delta[t+1] @ R_corrente": delta_rot_t1 @ current_rot,
        "R_corrente @ R_delta[t+1]": current_rot @ delta_rot_t1,
        "R_delta[t] non scalata @ R_corrente": raw_delta_rot @ current_rot,
    }

    pos_errors = {name: position_error(value, expected[:, :3]) for name, value in pos_predictions.items()}
    rot_errors = {name: angular_error(value, expected_rot) for name, value in rot_predictions.items()}
    details = {
        "current": current,
        "expected": expected,
        "action_saved": action_t,
        "delta_real": delta_t,
    }
    transition_indices = np.arange(len(current))
    return pos_errors, rot_errors, transition_indices, details


def best_hypothesis(metrics: dict[str, np.ndarray], mask: np.ndarray) -> tuple[str, float, str, float]:
    ranked = sorted((float(np.median(values[mask])), name) for name, values in metrics.items())
    best_value, best_name = ranked[0]
    second_value, second_name = ranked[1]
    return best_name, best_value, second_name, second_value


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tfrecord-path", type=Path, default=DEFAULT_TFRECORD)
    parser.add_argument("--episode-index", type=int, default=None)
    parser.add_argument("--scale-factor", type=float, default=0.05)
    parser.add_argument("--euler-order", default="xyz")
    parser.add_argument("--examples", type=int, default=6)
    args = parser.parse_args()

    tfrecord_path = args.tfrecord_path.resolve()
    if not tfrecord_path.is_file():
        raise FileNotFoundError(tfrecord_path)

    dataset, shard_index, start, stop = read_shard(tfrecord_path)
    print(f"TFRecord:       {tfrecord_path}")
    print(f"Shard:          {shard_index}")
    print(f"Episodi:        {stop - start} (indici globali {start}:{stop})")
    print(f"Scale factor:   {args.scale_factor}")
    print(f"Ordine Euler:   {args.euler_order}")

    all_pos = defaultdict(list)
    all_rot = defaultdict(list)
    all_transition_indices = []
    examples = []
    analyzed_episodes = 0

    for episode_index, episode in enumerate(dataset):
        if args.episode_index is not None and episode_index != args.episode_index:
            continue

        pos_errors, rot_errors, transition_indices, details = analyze_episode(
            episode, args.scale_factor, args.euler_order
        )
        for name, values in pos_errors.items():
            all_pos[name].append(values)
        for name, values in rot_errors.items():
            all_rot[name].append(values)
        all_transition_indices.append(transition_indices)

        room = max(0, args.examples - len(examples))
        for t in range(min(room, len(transition_indices))):
            examples.append((episode_index, t, details, pos_errors, rot_errors))

        analyzed_episodes += 1
        print(f"  episodio {episode_index:02d} (globale {start + episode_index:03d}): {len(transition_indices) + 1} step")

    if analyzed_episodes == 0:
        raise ValueError(f"Episode index non trovato: {args.episode_index}")

    pos_metrics = {name: np.concatenate(values) for name, values in all_pos.items()}
    rot_metrics = {name: np.concatenate(values) for name, values in all_rot.items()}
    transition_indices = np.concatenate(all_transition_indices)
    all_mask = np.ones_like(transition_indices, dtype=bool)
    after_first_mask = transition_indices > 0

    print(f"\nEpisodi analizzati:    {analyzed_episodes}")
    print(f"Transizioni totali:    {len(transition_indices)}")
    print(f"Transizioni con t > 0: {np.count_nonzero(after_first_mask)}")

    print_table("ERRORE POSIZIONE - TUTTE LE TRANSIZIONI", pos_metrics, all_mask, 1000.0, "mm")
    print_table("ERRORE ROTAZIONE - TUTTE LE TRANSIZIONI", rot_metrics, all_mask, 180.0 / np.pi, "gradi")

    if np.any(after_first_mask):
        print_table("ERRORE POSIZIONE - ESCLUSO t=0", pos_metrics, after_first_mask, 1000.0, "mm")
        print_table("ERRORE ROTAZIONE - ESCLUSO t=0", rot_metrics, after_first_mask, 180.0 / np.pi, "gradi")
        decision_mask = after_first_mask
    else:
        decision_mask = all_mask

    best_pos, pos_value, second_pos, second_pos_value = best_hypothesis(pos_metrics, decision_mask)
    best_rot, rot_value, second_rot, second_rot_value = best_hypothesis(rot_metrics, decision_mask)

    print("\n========== RISULTATO ==========")
    print(f"Posizione migliore: {best_pos}")
    print(f"  mediana: {pos_value * 1000.0:.9f} mm")
    print(f"  seconda: {second_pos} ({second_pos_value * 1000.0:.9f} mm)")
    print(f"Rotazione migliore: {best_rot}")
    print(f"  mediana: {np.degrees(rot_value):.9f} gradi")
    print(f"  seconda: {second_rot} ({np.degrees(second_rot_value):.9f} gradi)")

    print("\nInterpretazione della rotazione migliore:")
    print("  RPY corrente + action[t] scalata  -> somma componente per componente degli angoli RPY")
    print("  R_delta @ R_corrente             -> delta applicata negli assi fissi di base_link")
    print("  R_corrente @ R_delta             -> delta applicata negli assi locali dell'end-effector")

    if examples:
        print("\n========== ESEMPI ==========")
    for episode_index, t, details, pos_errors, rot_errors in examples:
        print(f"\nepisodio={episode_index} transizione={t}->{t + 1}")
        print(f"  EEF_state[t] RPY:       {np.array2string(details['current'][t, 3:6], precision=7)}")
        print(f"  action[t] salvata RPY:  {np.array2string(details['action_saved'][t, 3:6], precision=7)}")
        print(f"  action[t] reale RPY:    {np.array2string(details['delta_real'][t, 3:6], precision=7)}")
        print(f"  EEF_state[t+1] RPY:     {np.array2string(details['expected'][t, 3:6], precision=7)}")
        print("  Errori posizione [mm]:")
        for name, values in pos_errors.items():
            print(f"    {name:32s} {values[t] * 1000.0:.9f}")
        print("  Errori rotazione [gradi]:")
        for name, values in rot_errors.items():
            print(f"    {name:38s} {np.degrees(values[t]):.9f}")


if __name__ == "__main__":
    main()
