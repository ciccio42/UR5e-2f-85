#!/usr/bin/env python3
"""Convert one UR5e TFRecord shard to Mimic Video's video/metas format."""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import cv2
import imageio.v2 as imageio
import numpy as np
import safetensors.numpy as st
import tensorflow_datasets as tfds
from scipy.spatial.transform import Rotation


DEFAULT_TFRECORD = Path(
    "/home/mivia/Desktop/UR-Application/dataset/vla-dataset/"
    "real_ur5e_pick_place_delta_removed_0_5_10_15/"
    "ur5e_pick_place-train.tfrecord-00000"
)

SHARD_RE = re.compile(r".+-train\.tfrecord-(\d{5})$")
DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parents[1] / "processed_data" / "ur5e_pick_place_video"
DEFAULT_ACTION_OUTPUT_DIR = Path(__file__).resolve().parents[1] / "processed_data" / "ur5e_pick_place_action"
VIDEO_SIZE = (320, 240)
SCALE_FACTOR = 0.05
NS_PER_SEC = 1_000_000_000
EULER_ORDER = "xyz"
WORKSPACE_CAMERA_KEY = "camera_front_image"
CAMERAS = (
    ("camera_front_image", "0"),
    ("camera_gripper_image", "1"),
)


def shard_index_from_path(path: Path) -> int:
    # Il primo tfrecord ha traiettorie da 0 a 39, il secondo da 40 a 79.
    # Il numero tfrecord presente nel path viene usato come shift per selezionare gli shard nel dataset unificato.
    match = SHARD_RE.match(path.name)
    if match is None:
        raise ValueError(f"Invalid train TFRecord name: {path.name}")
    return int(match.group(1))


def read_text(value) -> str:
    # TFDS returns TensorFlow tensors; language_instruction is stored as bytes.
    value = value.numpy()
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value)


def read_frame(step: dict, camera_key: str) -> np.ndarray:
    # accedo ad ogni RGB di ogni step della traiettoria.
    frame = step["observation"][camera_key].numpy()
    if frame.ndim != 3 or frame.shape[-1] != 3 or frame.dtype != np.uint8:
        raise ValueError(f"Invalid frame for {camera_key}: shape={frame.shape}, dtype={frame.dtype}")
    return frame


def read_episode_frames(episode: dict) -> dict[str, list[np.ndarray]]:
    # crea un dizionario 
    #   frames = {
    #       "camera_front_image": [],
    #       "camera_gripper_image": [],
    #   }

    frames = {camera_key: [] for camera_key, _suffix in CAMERAS}
    for step in episode["steps"]: # per ogni frame della traiettoria
        for camera_key, _suffix in CAMERAS:
            frames[camera_key].append(read_frame(step, camera_key)) # riempi le due liste prima di passare allo step successivo
    return frames


def resize_with_padding(frame: np.ndarray, output_size: tuple[int, int] = VIDEO_SIZE) -> np.ndarray:
    # Resize uniformly, then pad to 4:3 so the Cosmos loader can resize without distortion.
    output_width, output_height = output_size
    height, width = frame.shape[:2]
    scale = min(output_width / width, output_height / height)

    resized_width = round(width * scale)
    resized_height = round(height * scale)
    resized = cv2.resize(frame, (resized_width, resized_height), interpolation=cv2.INTER_AREA)

    canvas = np.zeros((output_height, output_width, 3), dtype=np.uint8)
    top = (output_height - resized_height) // 2
    left = (output_width - resized_width) // 2
    canvas[top:top + resized_height, left:left + resized_width] = resized
    return canvas


def make_timestamps(num_steps: int, fps: int) -> np.ndarray:
    # I tfrecord non salvano timestamp reali; assumo step equidistanti con lo stesso fps degli mp4.
    # Il dataloader degli autori lavora in nanosecondi, quindi converto gli indici temporali in uint64.
    if fps <= 0:
        raise ValueError("fps must be > 0")
    return (np.arange(num_steps) * NS_PER_SEC / fps).astype(np.uint64)


def normalize_gripper(values: np.ndarray) -> np.ndarray:
    # Mantengo una convenzione binaria semplice: 0 aperto, 1 chiuso.
    # Nel tfrecord il gripper dell'action puo' essere scalato, quindi prima viene riportato in [0, 1].
    return (values > 0.5).astype(np.float32)


def read_episode_actions(episode: dict) -> tuple[np.ndarray, np.ndarray]:
    # Leggo per ogni step la delta action da predire e lo stato assoluto corrente dell'end effector.
    actions = []
    action_worlds = []
    for step in episode["steps"]:
        actions.append(step["action"].numpy().astype(np.float32))
        action_worlds.append(step["observation"]["action_world"].numpy().astype(np.float32))
    return np.stack(actions), np.stack(action_worlds)


def write_action_safetensor(
    path: Path,
    language: str,
    workspace_rgb: np.ndarray,
    episode: dict,
    fps: int,
    overwrite: bool,
) -> None:
    # Ogni safetensor contiene una traiettoria completa: osservazioni, azioni e relativi timestamp.
    if path.exists() and not overwrite:
        return

    actions, action_worlds = read_episode_actions(episode)
    if not len(workspace_rgb) == len(actions) == len(action_worlds):
        raise ValueError(
            f"Length mismatch for {path.name}: "
            f"workspace_rgb={len(workspace_rgb)}, actions={len(actions)}, action_world={len(action_worlds)}"
        )

    timestamps = make_timestamps(len(actions), fps)

    # Il campo action nel tfrecord e' stato diviso per SCALE_FACTOR; qui recupero il delta reale.
    delta_pos = actions[:, :3] * SCALE_FACTOR
    delta_euler = actions[:, 3:6] * SCALE_FACTOR
    gripper_action = normalize_gripper(actions[:, 6:7] * SCALE_FACTOR)

    # Il modello lavora poi con rotazioni 6D, ma gli autori salvano prima una matrice e la trasformano via yaml.
    delta_rot = Rotation.from_euler(EULER_ORDER, delta_euler).as_matrix().astype(np.float32)

    # action_world descrive lo stato assoluto osservato: posizione, orientamento RPY e stato gripper corrente.
    eef_pos = action_worlds[:, :3]
    eef_rot = Rotation.from_euler(EULER_ORDER, action_worlds[:, 3:6]).as_quat().astype(np.float32)
    gripper = normalize_gripper(action_worlds[:, 6:7])

    path.parent.mkdir(parents=True, exist_ok=True)
    st.save_file(
        {
            # istruzione linguistica
            "language_instruction": np.frombuffer(language.strip().encode("utf-8"), dtype=np.uint8)[None],
            "language_instruction_timestamps": np.array([0], dtype=np.uint64),

            # stato visivo
            "workspace_rgb": workspace_rgb,
            "workspace_rgb_timestamps": timestamps,

            ## action in delta da predire
            "eef_pos_ref_delta_lowdim": delta_pos.astype(np.float32), #posa
            "eef_pos_ref_delta_lowdim_timestamps": timestamps.copy(),
            "eef_rot_ref_delta_lowdim": delta_rot, #rotazione
            "eef_rot_ref_delta_lowdim_timestamps": timestamps.copy(),
            "gripper_action_lowdim": gripper_action, #gripper
            "gripper_action_lowdim_timestamps": timestamps.copy(),
            ##
            ## stato robotico corrente
            "eef_pos_lowdim": eef_pos.astype(np.float32), #posa
            "eef_pos_lowdim_timestamps": timestamps.copy(),
            "eef_rot_lowdim": eef_rot, #rotazione
            "eef_rot_lowdim_timestamps": timestamps.copy(),
            "gripper_lowdim": gripper, #gripper
            "gripper_lowdim_timestamps": timestamps.copy(),
            ##
        },
        path,
    )


def write_mp4(path: Path, frames: list[np.ndarray], fps: int, overwrite: bool) -> None:
    # Keep the raw temporal rate in the MP4; target_fps is chosen later by the loader.
    if path.exists() and not overwrite:
        return

    path.parent.mkdir(parents=True, exist_ok=True)
    with imageio.get_writer(path, codec="libx264", fps=fps, quality=8) as writer:
        for frame in frames:
            writer.append_data(resize_with_padding(frame))


def write_meta(path: Path, language: str, overwrite: bool) -> None:
    # One meta file is shared by both camera videos through the final digit suffix.
    if path.exists() and not overwrite:
        return

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(language.strip() + "\n", encoding="utf-8")


def export_episode(
    episode: dict,
    output_dir: Path,
    action_output_dir: Path,
    global_index: int,
    fps: int,
    overwrite: bool,
) -> None:
    # Example names: ep_0000000.mp4 per front, ep_0000001.mp4 per gripper, metas/ep_000000.txt unico txt associato.
    # global index fa continuare il conteggio per ogni traiettoria anche di task diversi.
    episode_name = f"ep_{global_index:06d}"
    # raccolgo istruzione testuale
    language = read_text(episode["language_instruction"])
    # raccolgo frame traiettoria dalle due camere
    frames_by_camera = read_episode_frames(episode)

    # salva comando testuale per questa traiettoria in /metas
    write_meta(output_dir / "metas" / f"{episode_name}.txt", language, overwrite)

    for camera_key, suffix in CAMERAS:
        video_path = output_dir / "video" / f"{episode_name}{suffix}.mp4"
        write_mp4(video_path, frames_by_camera[camera_key], fps, overwrite)
        print(f"  wrote {video_path.name}: {len(frames_by_camera[camera_key])} frames from {camera_key}")

    # Per l'action head gli autori usano una sola vista chiamata workspace_rgb: qui uso la camera frontale.
    workspace_rgb = np.stack([resize_with_padding(frame) for frame in frames_by_camera[WORKSPACE_CAMERA_KEY]])
    action_path = action_output_dir / f"{episode_name}.safetensors"
    write_action_safetensor(action_path, language, workspace_rgb, episode, fps, overwrite)
    print(f"  wrote {action_path.name}: {len(workspace_rgb)} action steps")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tfrecord-path", type=Path, default=DEFAULT_TFRECORD)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--action-output-dir", type=Path, default=DEFAULT_ACTION_OUTPUT_DIR)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()

    tfrecord_path = args.tfrecord_path.resolve()
    dataset_dir = tfrecord_path.parent
    shard_index = shard_index_from_path(tfrecord_path)

    builder = tfds.builder_from_directory(str(dataset_dir))
    # ottengo lista [40] x 12 (campo shardLengths in dataset_info.json)
    shard_lengths = list(builder.info.splits["train"].shard_lengths)

    if shard_index >= len(shard_lengths):
        raise ValueError(f"Shard index {shard_index} not found in dataset metadata.")

    #sommi 40 fin dove sei arrivato adesso
    start = sum(shard_lengths[:shard_index])
    # ti fermi tra 40 traiettorie
    stop = start + shard_lengths[shard_index]

    print(f"tfrecord={tfrecord_path}")
    print(f"shard={shard_index} episodes={shard_lengths[shard_index]} global_range={start}:{stop}")
    print(f"output={args.output_dir.resolve()}")
    print(f"action_output={args.action_output_dir.resolve()}")

    dataset = builder.as_dataset(split=f"train[{start}:{stop}]", shuffle_files=False)

    for episode_index, episode in enumerate(dataset):
        global_index = start + episode_index
        print(f"\nepisode={episode_index} global={global_index}")
        export_episode(episode, args.output_dir, args.action_output_dir, global_index, args.fps, args.overwrite)


if __name__ == "__main__":
    main()
