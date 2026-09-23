#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import tensorflow as tf
import tensorflow_datasets as tfds
from PIL import Image, ImageDraw


ACTION_SCALE_FACTOR = 0.05

# Dataset:
# action[6]:
#   0  = open
#   20 = closed
ACTION_GRIPPER_THRESHOLD = 10.0

# observation.state[6]:
#   0 = open
#   1 = closed
STATE_GRIPPER_THRESHOLD = 0.5


def as_numpy(value):
    if isinstance(value, tf.Tensor):
        return value.numpy()
    return np.asarray(value)


def get_steps(episode):
    """
    TFDS/RLDS normalmente restituisce episode["steps"] come tf.data.Dataset.
    """
    steps = episode["steps"]

    if isinstance(steps, tf.data.Dataset):
        return list(steps.as_numpy_iterator())

    raise TypeError(
        f"Tipo inatteso per episode['steps']: {type(steps)}"
    )


def decode_record(
    tfrecord_path: Path,
    record_index: int,
):
    """
    Decodifica un singolo episodio direttamente dallo shard TFRecord.
    Un record TFDS corrisponde a un episodio.
    """
    dataset_dir = tfrecord_path.parent

    print(f"Dataset dir:   {dataset_dir}")
    print(f"TFRecord:      {tfrecord_path}")
    print(f"Record index:  {record_index}")

    builder = tfds.builder_from_directory(
        str(dataset_dir)
    )

    raw_dataset = tf.data.TFRecordDataset(
        [str(tfrecord_path)],
        num_parallel_reads=1,
    )

    serialized = None

    for i, record in enumerate(raw_dataset):
        if i == record_index:
            serialized = record
            break

    if serialized is None:
        raise IndexError(
            f"Record {record_index} non presente in {tfrecord_path}"
        )

    episode = builder.info.features.deserialize_example(
        serialized
    )

    return episode


def get_task_id(episode):
    """
    Prova a recuperare il task_id senza rendere lo script dipendente
    dalla sua presenza.
    """
    try:
        value = (
            episode["traj_metadata"]
            ["episode_metadata"]
            ["task_id"]
        )

        if isinstance(value, tf.Tensor):
            value = value.numpy()

        return int(np.asarray(value).item())

    except Exception:
        return None


def find_first_reopen(action_gripper):
    """
    Trova:
      1. la prima action che chiude il gripper;
      2. la prima action successiva che torna OPEN.

    Questo evita di interpretare come 'prima apertura' le action OPEN
    presenti all'inizio della traiettoria.
    """

    first_close = None

    for t, value in enumerate(action_gripper):
        if value >= ACTION_GRIPPER_THRESHOLD:
            first_close = t
            break

    if first_close is None:
        raise RuntimeError(
            "Non è stata trovata alcuna action di chiusura."
        )

    first_reopen = None

    for t in range(first_close + 1, len(action_gripper)):
        if action_gripper[t] < ACTION_GRIPPER_THRESHOLD:
            first_reopen = t
            break

    if first_reopen is None:
        raise RuntimeError(
            "Il gripper viene chiuso ma non viene più riaperto."
        )

    return first_close, first_reopen


def find_state_open_after_close(
    state_gripper,
    start_t,
):
    """
    Cerca il primo timestep in cui lo STATO osservato del gripper
    risulta aperto dopo l'action di riapertura.
    """
    for t in range(start_t, len(state_gripper)):
        if state_gripper[t] < STATE_GRIPPER_THRESHOLD:
            return t

    return None


def ensure_rgb(image):
    image = np.asarray(image)

    if image.ndim != 3:
        raise ValueError(
            f"Immagine con shape inattesa: {image.shape}"
        )

    # CHW -> HWC, nel caso fosse necessario.
    if image.shape[0] == 3 and image.shape[-1] != 3:
        image = np.transpose(image, (1, 2, 0))

    if image.shape[-1] != 3:
        raise ValueError(
            f"Immagine non RGB: {image.shape}"
        )

    if image.dtype != np.uint8:
        image = np.clip(image, 0, 255).astype(np.uint8)

    return image


def save_frame(
    image,
    output_path,
    lines,
):
    image = ensure_rgb(image)

    pil = Image.fromarray(image, mode="RGB")

    top = 70

    canvas = Image.new(
        "RGB",
        (pil.width, pil.height + top),
        (255, 255, 255),
    )

    canvas.paste(
        pil,
        (0, top),
    )

    draw = ImageDraw.Draw(canvas)

    y = 5

    for line in lines:
        draw.text(
            (5, y),
            line,
            fill=(0, 0, 0),
        )
        y += 15

    canvas.save(output_path)


def make_panel(
    frames,
    output_path,
):
    widths = [frame.width for frame in frames]
    heights = [frame.height for frame in frames]

    panel = Image.new(
        "RGB",
        (
            sum(widths),
            max(heights),
        ),
        (255, 255, 255),
    )

    x = 0

    for frame in frames:
        panel.paste(frame, (x, 0))
        x += frame.width

    panel.save(output_path)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--tfrecord",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--record-index",
        type=int,
        default=0,
        help="Episodio/record da analizzare nello shard.",
    )

    parser.add_argument(
        "--window",
        type=int,
        default=2,
        help="Numero di step prima/dopo l'apertura da mostrare.",
    )

    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("/tmp/interleave_gripper_alignment"),
    )

    args = parser.parse_args()

    args.output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    # ================================================================
    # LOAD EPISODE
    # ================================================================

    episode = decode_record(
        args.tfrecord,
        args.record_index,
    )

    task_id = get_task_id(episode)

    steps = get_steps(episode)

    print()
    print("=" * 80)
    print("EPISODE")
    print("=" * 80)
    print(f"Task ID:       {task_id}")
    print(f"Num steps:     {len(steps)}")

    if len(steps) == 0:
        raise RuntimeError("Episodio vuoto.")

    # ================================================================
    # CHECK STRUCTURE
    # ================================================================

    first_step = steps[0]

    print()
    print("Step keys:")
    print(first_step.keys())

    if "observation" not in first_step:
        raise KeyError(
            "Chiave 'observation' non trovata nello step."
        )

    print()
    print("Observation keys:")
    print(first_step["observation"].keys())

    if "action" not in first_step:
        raise KeyError("Chiave 'action' non trovata.")

    if "state" not in first_step["observation"]:
        raise KeyError(
            "Chiave observation['state'] non trovata."
        )

    if "image_0" not in first_step["observation"]:
        raise KeyError(
            "Chiave observation['image_0'] non trovata."
        )

    # ================================================================
    # MATERIALIZE ARRAYS
    # ================================================================

    actions = np.stack(
        [
            np.asarray(step["action"], dtype=np.float32)
            for step in steps
        ]
    )

    states = np.stack(
        [
            np.asarray(
                step["observation"]["state"],
                dtype=np.float32,
            )
            for step in steps
        ]
    )

    images = [
        step["observation"]["image_0"]
        for step in steps
    ]

    print()
    print(f"Action shape:   {actions.shape}")
    print(f"State shape:    {states.shape}")

    if actions.shape[1] != 7:
        raise RuntimeError(
            f"Action dim inattesa: {actions.shape}"
        )

    if states.shape[1] != 7:
        raise RuntimeError(
            f"State dim inattesa: {states.shape}"
        )

    action_gripper = actions[:, 6]
    state_gripper = states[:, 6]

    # ================================================================
    # FIND CLOSE -> OPEN
    # ================================================================

    first_close_t, first_open_t = find_first_reopen(
        action_gripper
    )

    state_open_t = find_state_open_after_close(
        state_gripper,
        first_open_t,
    )

    print()
    print("=" * 80)
    print("GRIPPER EVENTS")
    print("=" * 80)

    print(
        f"Prima action CLOSE:          t={first_close_t} "
        f"action_g={action_gripper[first_close_t]:.6f}"
    )

    print(
        f"Prima action OPEN dopo CLOSE: t={first_open_t} "
        f"action_g={action_gripper[first_open_t]:.6f}"
    )

    if state_open_t is not None:
        print(
            f"Primo STATE OPEN >= evento:   t={state_open_t} "
            f"state_g={state_gripper[state_open_t]:.6f}"
        )

        print(
            f"Offset state_open - action_open = "
            f"{state_open_t - first_open_t} step"
        )
    else:
        print(
            "Lo stato del gripper non risulta più OPEN "
            "dopo il comando."
        )

    # ================================================================
    # IMPORTANT ALIGNMENT CHECK
    # ================================================================

    print()
    print("=" * 80)
    print("ALIGNMENT CHECK PRINCIPALE")
    print("=" * 80)

    t = first_open_t

    print(f"Timestep da controllare: t={t}")
    print()

    print(
        f"observation.state gripper = "
        f"{state_gripper[t]:.6f}"
    )

    print(
        f"action gripper RAW        = "
        f"{action_gripper[t]:.6f}"
    )

    if state_gripper[t] >= STATE_GRIPPER_THRESHOLD:
        print(
            "OBSERVATION: gripper numericamente CLOSED."
        )
    else:
        print(
            "OBSERVATION: gripper numericamente OPEN."
        )

    if action_gripper[t] < ACTION_GRIPPER_THRESHOLD:
        print(
            "ACTION:      comando OPEN."
        )
    else:
        print(
            "ACTION:      comando CLOSE."
        )

    print()
    print(
        "CONDIZIONE ATTESA:\n"
        "  observation_t -> gripper CLOSED sopra il bin\n"
        "  action_t[6]   -> OPEN\n"
        "  image_t       -> deve mostrare visivamente il gripper "
        "ancora chiuso."
    )

    # ================================================================
    # WINDOW AROUND EVENT
    # ================================================================

    start = max(
        0,
        first_open_t - args.window,
    )

    end = min(
        len(steps),
        first_open_t + args.window + 1,
    )

    print()
    print("=" * 80)
    print(
        f"WINDOW t={start} ... {end - 1}"
    )
    print("=" * 80)

    panel_frames = []

    for step_idx in range(start, end):

        state = states[step_idx]
        action_raw = actions[step_idx]

        # Dataset:
        # action[:6] = physical_delta / 0.05
        action_physical = action_raw.copy()
        action_physical[:6] *= ACTION_SCALE_FACTOR

        delta_xyz_m = action_physical[:3]
        delta_rpy_rad = action_physical[3:6]
        delta_rpy_deg = np.degrees(
            delta_rpy_rad
        )

        marker = (
            " <-- FIRST OPEN ACTION"
            if step_idx == first_open_t
            else ""
        )

        print()
        print(
            f"---------------- t={step_idx}{marker} ----------------"
        )

        print(
            "state =",
            np.array2string(
                state,
                precision=7,
                separator=", ",
            ),
        )

        print(
            f"state_gripper = {state[6]:.7f} "
            f"({'CLOSED' if state[6] >= 0.5 else 'OPEN'})"
        )

        print(
            "action_RAW =",
            np.array2string(
                action_raw,
                precision=7,
                separator=", ",
            ),
        )

        print(
            "physical_delta_xyz_m =",
            np.array2string(
                delta_xyz_m,
                precision=7,
                separator=", ",
            ),
        )

        print(
            "physical_delta_rpy_rad =",
            np.array2string(
                delta_rpy_rad,
                precision=7,
                separator=", ",
            ),
        )

        print(
            "physical_delta_rpy_deg =",
            np.array2string(
                delta_rpy_deg,
                precision=7,
                separator=", ",
            ),
        )

        print(
            f"action_gripper_RAW = {action_raw[6]:.7f} "
            f"({'CLOSED' if action_raw[6] >= 10 else 'OPEN'})"
        )

        # ------------------------------------------------------------
        # Save image
        # ------------------------------------------------------------

        frame_path = (
            args.output_dir
            / f"record_{args.record_index:03d}_t_{step_idx:04d}.png"
        )

        save_frame(
            images[step_idx],
            frame_path,
            [
                f"record={args.record_index} task={task_id}",
                f"t={step_idx}",
                f"state gripper={state[6]:.3f} "
                f"({'CLOSED' if state[6] >= 0.5 else 'OPEN'})",
                f"action gripper={action_raw[6]:.3f} "
                f"({'CLOSED' if action_raw[6] >= 10 else 'OPEN'})",
            ],
        )

        panel_frames.append(
            Image.open(frame_path).copy()
        )

    # ================================================================
    # PANEL
    # ================================================================

    panel_path = (
        args.output_dir
        / f"record_{args.record_index:03d}"
          f"_first_open_t_{first_open_t:04d}_panel.png"
    )

    make_panel(
        panel_frames,
        panel_path,
    )

    print()
    print("=" * 80)
    print("OUTPUT")
    print("=" * 80)

    print(
        f"Immagini salvate in:\n  {args.output_dir}"
    )

    print(
        f"\nPannello principale:\n  {panel_path}"
    )

    print()
    print(
        "Nel pannello guarda soprattutto il frame centrale "
        f"t={first_open_t}."
    )

    print(
        "Se l'allineamento è quello atteso, quel frame deve "
        "mostrare il gripper ancora CHIUSO sopra il bin, "
        "mentre action_gripper_RAW deve essere OPEN (~0)."
    )


if __name__ == "__main__":
    main()