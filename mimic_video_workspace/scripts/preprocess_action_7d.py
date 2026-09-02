#!/usr/bin/env python3
"""
Build clean 7D UR5e Action Head safetensors directly from the original TFRecords.

Main properties:
- reads the ORIGINAL TFRecord trajectories;
- does NOT duplicate robot states or actions;
- keeps both absolute orientation and action rotation in RPY;
- repairs the first and last valid pose transitions:
      state[0]  -> state[1]
      state[-2] -> state[-1]
- keeps the original gripper action for repaired transitions;
- processes workspace RGB with aspect-ratio-preserving resize + padding;
- reuses the already-computed language embeddings;
- reuses the already-computed visual embeddings obtained from the extended videos;
- remaps visual embedding anchors from the extended video timeline
  back to the original robot trajectory timeline;
- saves the final dataset to:
      processed_data/ur5e_pick_place_action_7d
"""

from __future__ import annotations

import argparse
import re
import shutil
from pathlib import Path

import cv2
import numpy as np
import safetensors.numpy as st_np
import tensorflow_datasets as tfds
from scipy.spatial.transform import Rotation


# ======================================================================
# PATHS / CONSTANTS
# ======================================================================

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
PROCESSED_DIR = WORKSPACE_ROOT / "processed_data"

DEFAULT_TFRECORD = Path(
    "/home/mivia/Desktop/UR-Application/dataset/vla-dataset/"
    "real_ur5e_pick_place_delta_removed_0_5_10_15/"
    "ur5e_pick_place-train.tfrecord-00000"
)

DEFAULT_OUTPUT_DIR = (
    PROCESSED_DIR / "ur5e_pick_place_action_7d"
)

DEFAULT_LANGUAGE_EMBEDDINGS_DIR = (
    PROCESSED_DIR
    / "ur5e_pick_place_video"
    / "language_embeddings"
)

DEFAULT_VIDEO_EMBEDDINGS_DIR = (
    PROCESSED_DIR
    / "ur5e_pick_place_video"
    / "video_embeddings"
)

SHARD_RE = re.compile(r".+-train\.tfrecord-(\d{5})$")

WORKSPACE_CAMERA_KEY = "camera_front_image"

# The original preprocessing produced a 4:3 image before the Cosmos loader
# resized it to 640x480.
VIDEO_SIZE = (320, 240)

ACTION_SCALE_FACTOR = 0.05
EULER_ORDER = "xyz"

DEFAULT_FPS = 10
NS_PER_SEC = 1_000_000_000

# Videos shorter than this were extended ONLY for Cosmos/video embeddings.
TARGET_LENGTH = 61

# Front camera:
# ep_000000.safetensors -> ep_0000000.safetensors
DEFAULT_VIDEO_SUFFIX = "0"

VISUAL_EMBEDDING_KEY = "workspace_rgb_embedding"
VISUAL_EMBEDDING_TIMESTAMPS_KEY = (
    f"{VISUAL_EMBEDDING_KEY}_timestamps"
)


# ======================================================================
# TFRECORD UTILITIES
# ======================================================================

def shard_index_from_path(path: Path) -> int:
    """
    Extract shard index from:
        ur5e_pick_place-train.tfrecord-00000
    """
    match = SHARD_RE.match(path.name)

    if match is None:
        raise ValueError(
            f"Invalid train TFRecord name: {path.name}"
        )

    return int(match.group(1))


def read_text(value) -> str:
    """
    TFDS stores language_instruction as bytes.
    """
    value = value.numpy()

    if isinstance(value, bytes):
        return value.decode(
            "utf-8",
            errors="replace",
        )

    return str(value)


def read_frame(
    step: dict,
    camera_key: str,
) -> np.ndarray:
    """
    Read one RGB frame from a TFRecord step.
    """
    frame = step["observation"][camera_key].numpy()

    if (
        frame.ndim != 3
        or frame.shape[-1] != 3
        or frame.dtype != np.uint8
    ):
        raise ValueError(
            f"Invalid frame for {camera_key}: "
            f"shape={frame.shape}, dtype={frame.dtype}"
        )

    return frame


# ======================================================================
# RGB PREPROCESSING
# ======================================================================

def resize_with_padding(
    frame: np.ndarray,
    output_size: tuple[int, int] = VIDEO_SIZE,
) -> np.ndarray:
    """
    Resize while preserving aspect ratio, then pad to 4:3.

    Original TFRecord frames are 224x224.

    They become:

        224x224
            ↓
        240x240 content
            ↓
        padding left/right
            ↓
        320x240

    Therefore the later Cosmos resize:

        320x240 -> 640x480

    preserves the 4:3 geometry and does not stretch the scene.
    """
    output_width, output_height = output_size

    height, width = frame.shape[:2]

    scale = min(
        output_width / width,
        output_height / height,
    )

    resized_width = round(width * scale)
    resized_height = round(height * scale)

    resized = cv2.resize(
        frame,
        (resized_width, resized_height),
        interpolation=cv2.INTER_AREA,
    )

    canvas = np.zeros(
        (output_height, output_width, 3),
        dtype=np.uint8,
    )

    top = (
        output_height - resized_height
    ) // 2

    left = (
        output_width - resized_width
    ) // 2

    canvas[
        top : top + resized_height,
        left : left + resized_width,
    ] = resized

    return canvas


def read_workspace_rgb(
    episode: dict,
) -> np.ndarray:
    """
    Read ONLY the original trajectory frames.

    No temporal extension is performed here.
    """
    frames = []

    for step in episode["steps"]:
        frame = read_frame(
            step,
            WORKSPACE_CAMERA_KEY,
        )

        frames.append(
            resize_with_padding(frame)
        )

    return np.stack(frames)


# ======================================================================
# TIMESTAMPS
# ======================================================================

def make_timestamps(
    num_steps: int,
    fps: int,
) -> np.ndarray:
    """
    Build the synthetic uniformly-spaced timeline used by Mimic Video.
    """
    if fps <= 0:
        raise ValueError("fps must be > 0")

    return (
        np.arange(num_steps)
        * NS_PER_SEC
        / fps
    ).astype(np.uint64)


# ======================================================================
# GRIPPER
# ======================================================================

def normalize_gripper(
    values: np.ndarray,
) -> np.ndarray:
    """
    Binary convention:
        0 = open
        1 = closed
    """
    return (
        values > 0.5
    ).astype(np.float32)


# ======================================================================
# POSE ACTION REPAIR
# ======================================================================

def repair_pose_action_raw(
    current_step: dict,
    next_step: dict,
) -> np.ndarray:
    """
    Recompute the pose action connecting:

        current_step -> next_step

    directly from the two absolute EEF poses.

    IMPORTANT:
    The returned action remains in the RAW TFRecord representation.

    Therefore:

        delta_position_raw = delta_position / ACTION_SCALE_FACTOR
        delta_rpy_raw      = delta_rpy      / ACTION_SCALE_FACTOR

    The gripper action is intentionally left unchanged.

    Rotation convention:

        R_next = R_delta @ R_current

    therefore:

        R_delta = R_next @ R_current.T
    """

    state_0 = (
        current_step["observation"]["EEF_state"]
        .numpy()
        .astype(np.float64)
    )

    state_1 = (
        next_step["observation"]["EEF_state"]
        .numpy()
        .astype(np.float64)
    )

    if (
        state_0.shape != (6,)
        or state_1.shape != (6,)
    ):
        raise ValueError(
            "Expected EEF states with shape (6,), "
            f"got {state_0.shape} and {state_1.shape}."
        )

    repaired_action = (
        current_step["action"]
        .numpy()
        .astype(np.float32)
        .copy()
    )

    if repaired_action.shape != (7,):
        raise ValueError(
            "Expected raw action with shape (7,), "
            f"got {repaired_action.shape}."
        )

    # --------------------------------------------------
    # Translation
    # --------------------------------------------------

    delta_position = (
        state_1[:3]
        - state_0[:3]
    )

    # --------------------------------------------------
    # Rotation
    # --------------------------------------------------

    rotation_0 = Rotation.from_euler(
        EULER_ORDER,
        state_0[3:6],
    ).as_matrix()

    rotation_1 = Rotation.from_euler(
        EULER_ORDER,
        state_1[3:6],
    ).as_matrix()

    delta_rotation = (
        rotation_1
        @ rotation_0.T
    )

    delta_rpy = Rotation.from_matrix(
        delta_rotation
    ).as_euler(EULER_ORDER)

    # --------------------------------------------------
    # Restore RAW TFRecord representation
    # --------------------------------------------------

    repaired_action[:3] = (
        delta_position
        / ACTION_SCALE_FACTOR
    ).astype(np.float32)

    repaired_action[3:6] = (
        delta_rpy
        / ACTION_SCALE_FACTOR
    ).astype(np.float32)

    # repaired_action[6] remains unchanged.

    return repaired_action


def read_episode_actions(
    episode: dict,
) -> tuple[
    np.ndarray,
    np.ndarray,
    np.ndarray,
]:
    """
    Read action, absolute EEF state and gripper state
    from the ORIGINAL trajectory.

    The first and final VALID transitions are repaired:

        action[0]
            state[0] -> state[1]

        action[N-2]
            state[N-2] -> state[N-1]

    action[N-1] is NOT reconstructed because no state[N]
    exists in the episode.
    """

    steps = list(episode["steps"])

    num_steps = len(steps)

    if num_steps < 2:
        raise ValueError(
            "Episode contains fewer than two steps."
        )

    actions = []
    robotic_state = []
    gripper_state = []

    repair_indices = {
        0,
        num_steps - 2,
    }

    for index, step in enumerate(steps):

        # --------------------------------------------------
        # Action
        # --------------------------------------------------

        if index in repair_indices:
            action = repair_pose_action_raw(
                step,
                steps[index + 1],
            )
        else:
            action = (
                step["action"]
                .numpy()
                .astype(np.float32)
            )

        actions.append(action)

        # --------------------------------------------------
        # Absolute EEF state
        #
        # Already:
        # [x, y, z, roll, pitch, yaw]
        # --------------------------------------------------

        robotic_state.append(
            step["observation"]["EEF_state"]
            .numpy()
            .astype(np.float32)
        )

        # --------------------------------------------------
        # Gripper state
        # --------------------------------------------------

        gripper_state.append(
            step["observation"]["gripper_state"]
            .numpy()
            .astype(np.float32)[1:2]
        )

    return (
        np.stack(actions),
        np.stack(robotic_state),
        np.stack(gripper_state),
    )


# ======================================================================
# PRECOMPUTED EMBEDDING PATHS
# ======================================================================

def language_embedding_path(
    episode_name: str,
    language_embeddings_dir: Path,
) -> Path:
    """
    ep_000000 -> ep_000000.safetensors
    """
    return (
        language_embeddings_dir
        / f"{episode_name}.safetensors"
    )


def video_embedding_path(
    episode_name: str,
    video_embeddings_dir: Path,
    video_suffix: str,
) -> Path:
    """
    Front camera:

        ep_000000
            ->
        ep_0000000.safetensors
    """
    return (
        video_embeddings_dir
        / f"{episode_name}{video_suffix}.safetensors"
    )

def read_language_embedding(
    path: Path,
) -> np.ndarray:
    """
    Load the already-computed T5 embedding directly as NumPy.

    No PyTorch dependency is needed during preprocessing.
    """
    data = st_np.load_file(path)

    if "encoded_text" not in data:
        raise KeyError(
            f"{path.name}: missing 'encoded_text'"
        )

    encoded_text = data["encoded_text"].astype(
        np.float32,
        copy=False,
    )

    return np.ascontiguousarray(
        encoded_text[None]
    )

# ======================================================================
# EXTENSION MAPPING
#
# IMPORTANT:
# We NEVER extend the robot trajectory here.
#
# These functions only reconstruct the OLD mapping that had been used
# when producing the video embeddings.
# ======================================================================

def expected_extension_duplicate_indices(
    original_steps: int,
    duplicates: int,
) -> np.ndarray:
    """
    Reconstruct exactly which SECOND elements existed in the old
    61-frame extended video timeline.

    This uses the same extension rule as preprocessing_pipeline.py.
    """

    if duplicates == 0:
        return np.empty(
            0,
            dtype=np.int64,
        )

    duplicated_source_indices = set(
        np.round(
            np.linspace(
                0,
                original_steps - 1,
                duplicates,
            )
        )
        .astype(np.int64)
        .tolist()
    )

    if len(duplicated_source_indices) != duplicates:
        raise ValueError(
            "The old extension rule would select the same "
            "source step more than once. "
            "The extended video timeline cannot be reconstructed "
            "unambiguously."
        )

    expanded_index = 0
    duplicate_indices = []

    for source_index in range(original_steps):

        # Original step.
        expanded_index += 1

        # Old pipeline inserted one additional identical step here.
        if source_index in duplicated_source_indices:

            duplicate_indices.append(
                expanded_index
            )

            expanded_index += 1

    return np.asarray(
        duplicate_indices,
        dtype=np.int64,
    )


def make_compaction_map(
    old_steps: int,
    duplicate_indices: np.ndarray,
) -> tuple[
    np.ndarray,
    np.ndarray,
]:
    """
    Reused from action_preprocess.py.

    Maps the old extended video timeline onto the original
    non-duplicated robot timeline.
    """

    keep = np.ones(
        old_steps,
        dtype=bool,
    )

    keep[duplicate_indices] = False

    old_to_new = (
        np.cumsum(
            keep,
            dtype=np.int64,
        )
        - 1
    )

    if old_to_new[0] < 0:
        raise ValueError(
            "The first step cannot be removed."
        )

    return keep, old_to_new


def remap_visual_embeddings(
    video_data: dict[str, np.ndarray],
    original_steps: int,
    fps: int,
    target_length: int,
) -> tuple[
    np.ndarray,
    np.ndarray,
    int,
]:
    """
    Reuse the visual embeddings computed on the OLD video timeline
    while remapping their anchor indices onto the ORIGINAL robot
    timeline.

    For an episode originally shorter than 61:

        original timeline
              ↓
        old distributed extension
              ↓
        61-frame video
              ↓
        video embeddings

    Here we perform only the inverse anchor mapping:

        old embedding anchor
              ↓
        original robot step

    No state/action duplication is performed.
    """

    embeddings = video_data[
        "video_embeddings"
    ]

    anchor_indices = video_data[
        "video_embeddings_idxs"
    ].astype(np.int64)

    if np.any(anchor_indices < 0):
        raise ValueError(
            "Negative video embedding anchor index found."
        )

    video_len = int(
        np.asarray(
            video_data["video_len"]
        ).item()
    )

    video_fps = float(
        np.asarray(
            video_data["fps"]
        ).item()
    )

    if not np.isclose(
        video_fps,
        float(fps),
    ):
        raise ValueError(
            f"Video embedding FPS mismatch: "
            f"embedding={video_fps}, requested={fps}"
        )

    # --------------------------------------------------
    # Reconstruct the old video timeline
    # --------------------------------------------------

    if original_steps < target_length:

        duplicate_count = (
            target_length
            - original_steps
        )

        expected_video_len = target_length

        duplicate_indices = (
            expected_extension_duplicate_indices(
                original_steps,
                duplicate_count,
            )
        )

    else:

        expected_video_len = original_steps

        duplicate_indices = np.empty(
            0,
            dtype=np.int64,
        )

    if video_len != expected_video_len:
        raise ValueError(
            "Video embedding length does not match the timeline "
            "expected from the original preprocessing: "
            f"original_steps={original_steps}, "
            f"expected_video_len={expected_video_len}, "
            f"embedding_video_len={video_len}"
        )

    # --------------------------------------------------
    # Old extended timeline -> original timeline
    # --------------------------------------------------

    _keep, old_to_new = make_compaction_map(
        video_len,
        duplicate_indices,
    )

    compact_steps = (
        int(old_to_new[-1])
        + 1
    )

    if compact_steps != original_steps:
        raise ValueError(
            "Anchor compaction reconstructed the wrong "
            "trajectory length: "
            f"{compact_steps} != {original_steps}"
        )

    removed_steps = (
        video_len
        - original_steps
    )

    mapped_indices = np.empty_like(
        anchor_indices
    )

    # --------------------------------------------------
    # Anchors inside the actual old video
    # --------------------------------------------------

    in_video = (
        anchor_indices
        < video_len
    )

    mapped_indices[in_video] = (
        old_to_new[
            anchor_indices[in_video]
        ]
    )

    # --------------------------------------------------
    # Terminal padded anchors
    #
    # Keep their original offset relative to the end
    # of the episode, exactly as in action_preprocess.py.
    # --------------------------------------------------

    mapped_indices[~in_video] = (
        anchor_indices[~in_video]
        - removed_steps
    )

    # --------------------------------------------------
    # Multiple old anchors can collapse onto the same
    # original timestep because duplicated frames have
    # now been removed.
    #
    # Same logic as action_preprocess.py:
    # retain the later source anchor.
    # --------------------------------------------------

    best_row_by_new_index: dict[int, int] = {}

    for row, new_index in enumerate(
        mapped_indices.tolist()
    ):

        previous_row = (
            best_row_by_new_index.get(
                new_index
            )
        )

        if (
            previous_row is None
            or anchor_indices[row]
            > anchor_indices[previous_row]
        ):
            best_row_by_new_index[
                new_index
            ] = row

    ordered = sorted(
        best_row_by_new_index.items()
    )

    if not ordered:
        raise ValueError(
            "No visual embedding anchors remain "
            "after remapping."
        )

    kept_new_indices = np.asarray(
        [
            new_index
            for new_index, _row
            in ordered
        ],
        dtype=np.int64,
    )

    kept_rows = np.asarray(
        [
            row
            for _new_index, row
            in ordered
        ],
        dtype=np.int64,
    )

    remapped_embeddings = np.ascontiguousarray(
        embeddings[kept_rows]
    )

    dt_ns = (
        NS_PER_SEC
        // fps
    )

    remapped_timestamps = (
        kept_new_indices.astype(np.uint64)
        * np.uint64(dt_ns)
    )

    dropped_embeddings = (
        len(embeddings)
        - len(remapped_embeddings)
    )

    return (
        remapped_embeddings,
        remapped_timestamps,
        dropped_embeddings,
    )


# ======================================================================
# OUTPUT
# ======================================================================

def write_action_safetensor(
    path: Path,
    language: str,
    workspace_rgb: np.ndarray,
    episode: dict,
    episode_name: str,
    fps: int,
    language_embeddings_dir: Path,
    video_embeddings_dir: Path,
    video_suffix: str,
    overwrite: bool,
) -> tuple[int, int, int]:
    """
    Write one final Action Head safetensor.

    State:
        [x, y, z, roll, pitch, yaw, gripper]

    Action:
        [dx, dy, dz, droll, dpitch, dyaw, gripper]

    Position and angular action deltas are stored in real physical
    scale after applying ACTION_SCALE_FACTOR.
    """

    if path.exists() and not overwrite:
        return -1, -1, -1

    # --------------------------------------------------
    # ORIGINAL robot trajectory
    # --------------------------------------------------

    (
        actions_raw,
        robotic_state,
        gripper_state,
    ) = read_episode_actions(episode)

    num_steps = len(actions_raw)

    if not (
        len(workspace_rgb)
        == len(actions_raw)
        == len(robotic_state)
        == len(gripper_state)
    ):
        raise ValueError(
            f"Length mismatch for {path.name}: "
            f"workspace_rgb={len(workspace_rgb)}, "
            f"actions={len(actions_raw)}, "
            f"EEF_state={len(robotic_state)}, "
            f"gripper_state={len(gripper_state)}"
        )

    timestamps = make_timestamps(
        num_steps,
        fps,
    )

    # --------------------------------------------------
    # ACTION 7D
    #
    # TFRecord:
    # [dx, dy, dz, droll, dpitch, dyaw, gripper]
    #
    # First six components were divided by 0.05
    # when the TFRecord was created.
    #
    # NO rotation matrix conversion.
    # --------------------------------------------------

    delta_pos = (
        actions_raw[:, :3]
        * ACTION_SCALE_FACTOR
    ).astype(np.float32)

    delta_rpy = (
        actions_raw[:, 3:6]
        * ACTION_SCALE_FACTOR
    ).astype(np.float32)

    gripper_action = normalize_gripper(
        actions_raw[:, 6:7]
        * ACTION_SCALE_FACTOR
    )

    # --------------------------------------------------
    # STATE 7D
    #
    # EEF_state is ALREADY:
    # [x, y, z, roll, pitch, yaw]
    #
    # NO quaternion conversion.
    # --------------------------------------------------

    eef_pos = (
        robotic_state[:, :3]
        .astype(np.float32)
    )

    eef_rpy = (
        robotic_state[:, 3:6]
        .astype(np.float32)
    )

    gripper = normalize_gripper(
        gripper_state
    )

    # --------------------------------------------------
    # LANGUAGE EMBEDDING
    # --------------------------------------------------

    lang_path = language_embedding_path(
        episode_name,
        language_embeddings_dir,
    )

    if not lang_path.exists():
        raise FileNotFoundError(
            lang_path
        )

    language_embedding = (
        read_language_embedding(
            lang_path
        )
    )

    # --------------------------------------------------
    # VIDEO EMBEDDING
    # --------------------------------------------------

    vid_path = video_embedding_path(
        episode_name,
        video_embeddings_dir,
        video_suffix,
    )

    if not vid_path.exists():
        raise FileNotFoundError(
            vid_path
        )

    video_data = st_np.load_file(
        vid_path
    )

    (
        workspace_rgb_embedding,
        workspace_rgb_embedding_timestamps,
        dropped_embeddings,
    ) = remap_visual_embeddings(
        video_data=video_data,
        original_steps=num_steps,
        fps=fps,
        target_length=TARGET_LENGTH,
    )

    num_conditional_frames = int(
        np.asarray(
            video_data[
                "num_conditional_frames"
            ]
        ).item()
    )

    # --------------------------------------------------
    # SAVE
    # --------------------------------------------------

    path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    st_np.save_file(
        {
            # ==================================================
            # LANGUAGE
            # ==================================================

            "language_instruction": np.frombuffer(
                language.strip().encode("utf-8"),
                dtype=np.uint8,
            )[None],

            "language_instruction_timestamps": np.array(
                [0],
                dtype=np.uint64,
            ),

            "language_embedding": language_embedding,

            "language_embedding_timestamps": np.array(
                [0],
                dtype=np.uint64,
            ),

            # ==================================================
            # RGB OBSERVATION
            #
            # Original trajectory only.
            # No duplicated robot steps.
            # ==================================================

            "workspace_rgb": workspace_rgb,

            "workspace_rgb_timestamps": timestamps.copy(),

            # ==================================================
            # ACTION 7D
            #
            # [dx, dy, dz, droll, dpitch, dyaw, gripper]
            # ==================================================

            "eef_pos_ref_delta_lowdim": delta_pos,

            "eef_pos_ref_delta_lowdim_timestamps": (
                timestamps.copy()
            ),

            "eef_rot_ref_delta_lowdim": delta_rpy,

            "eef_rot_ref_delta_lowdim_timestamps": (
                timestamps.copy()
            ),

            "gripper_action_lowdim": gripper_action,

            "gripper_action_lowdim_timestamps": (
                timestamps.copy()
            ),

            # ==================================================
            # ROBOT STATE 7D
            #
            # [x, y, z, roll, pitch, yaw, gripper]
            # ==================================================

            "eef_pos_lowdim": eef_pos,

            "eef_pos_lowdim_timestamps": (
                timestamps.copy()
            ),

            "eef_rot_lowdim": eef_rpy,

            "eef_rot_lowdim_timestamps": (
                timestamps.copy()
            ),

            "gripper_lowdim": gripper,

            "gripper_lowdim_timestamps": (
                timestamps.copy()
            ),

            # ==================================================
            # PRECOMPUTED VISUAL EMBEDDINGS
            #
            # Values are reused exactly.
            # Only their anchors/timestamps are remapped.
            # ==================================================

            VISUAL_EMBEDDING_KEY: (
                workspace_rgb_embedding
            ),

            VISUAL_EMBEDDING_TIMESTAMPS_KEY: (
                workspace_rgb_embedding_timestamps
            ),

            "num_conditional_frames": np.array(
                [num_conditional_frames],
                dtype=np.int64,
            ),

            "num_conditional_frames_timestamps": np.array(
                [0],
                dtype=np.uint64,
            ),
        },
        path,
    )

    return (
        num_steps,
        len(video_data["video_embeddings"]),
        len(workspace_rgb_embedding),
    )


# ======================================================================
# EPISODE
# ======================================================================

def export_episode(
    episode: dict,
    output_dir: Path,
    global_index: int,
    fps: int,
    language_embeddings_dir: Path,
    video_embeddings_dir: Path,
    video_suffix: str,
    overwrite: bool,
) -> None:

    episode_name = (
        f"ep_{global_index:06d}"
    )

    language = read_text(
        episode["language_instruction"]
    )

    # IMPORTANT:
    #
    # No extension(episode).
    #
    # workspace_rgb follows the ORIGINAL robot trajectory.
    workspace_rgb = read_workspace_rgb(
        episode
    )

    action_path = (
        output_dir
        / f"{episode_name}.safetensors"
    )

    (
        num_steps,
        embeddings_before,
        embeddings_after,
    ) = write_action_safetensor(
        path=action_path,
        language=language,
        workspace_rgb=workspace_rgb,
        episode=episode,
        episode_name=episode_name,
        fps=fps,
        language_embeddings_dir=language_embeddings_dir,
        video_embeddings_dir=video_embeddings_dir,
        video_suffix=video_suffix,
        overwrite=overwrite,
    )

    if num_steps == -1:
        print(
            f"  skip {action_path.name}: output exists"
        )
        return

    print(
        f"  wrote {action_path.name}: "
        f"{num_steps} original robot steps, "
        f"visual embeddings "
        f"{embeddings_before}->{embeddings_after}"
    )


# ======================================================================
# CACHE
# ======================================================================

def invalidate_statistics_cache(
    output_dir: Path,
) -> None:
    """
    Remove normalization statistics from a previous version
    of this dataset, if present.
    """
    cache_dir = (
        output_dir
        / ".statistics_cache"
    )

    if cache_dir.exists():
        shutil.rmtree(cache_dir)

        print(
            f"Removed stale statistics cache: "
            f"{cache_dir}"
        )


# ======================================================================
# MAIN
# ======================================================================

def main() -> None:

    parser = argparse.ArgumentParser(
        description=__doc__
    )

    parser.add_argument(
        "--tfrecord-path",
        type=Path,
        default=DEFAULT_TFRECORD,
    )

    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
    )

    parser.add_argument(
        "--language-embeddings-dir",
        type=Path,
        default=DEFAULT_LANGUAGE_EMBEDDINGS_DIR,
    )

    parser.add_argument(
        "--video-embeddings-dir",
        type=Path,
        default=DEFAULT_VIDEO_EMBEDDINGS_DIR,
    )

    parser.add_argument(
        "--video-suffix",
        default=DEFAULT_VIDEO_SUFFIX,
    )

    parser.add_argument(
        "--fps",
        type=int,
        default=DEFAULT_FPS,
    )

    parser.add_argument(
        "--overwrite",
        action="store_true",
    )

    args = parser.parse_args()

    # --------------------------------------------------
    # Validate paths
    # --------------------------------------------------

    tfrecord_path = (
        args.tfrecord_path.resolve()
    )

    dataset_dir = (
        tfrecord_path.parent
    )

    output_dir = (
        args.output_dir.resolve()
    )

    language_embeddings_dir = (
        args.language_embeddings_dir.resolve()
    )

    video_embeddings_dir = (
        args.video_embeddings_dir.resolve()
    )

    if not tfrecord_path.exists():
        raise FileNotFoundError(
            tfrecord_path
        )

    if not language_embeddings_dir.is_dir():
        raise FileNotFoundError(
            language_embeddings_dir
        )

    if not video_embeddings_dir.is_dir():
        raise FileNotFoundError(
            video_embeddings_dir
        )

    if (
        args.fps <= 0
        or NS_PER_SEC % args.fps != 0
    ):
        raise ValueError(
            "--fps must be a positive integer divisor "
            "of 1,000,000,000."
        )

    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    invalidate_statistics_cache(
        output_dir
    )

    # --------------------------------------------------
    # Locate the requested TFRecord shard
    # --------------------------------------------------

    shard_index = shard_index_from_path(
        tfrecord_path
    )

    builder = tfds.builder_from_directory(
        str(dataset_dir)
    )

    shard_lengths = list(
        builder.info
        .splits["train"]
        .shard_lengths
    )

    if shard_index >= len(shard_lengths):
        raise ValueError(
            f"Shard index {shard_index} "
            "not found in dataset metadata."
        )

    start = sum(
        shard_lengths[:shard_index]
    )

    stop = (
        start
        + shard_lengths[shard_index]
    )

    print(
        f"tfrecord={tfrecord_path}"
    )

    print(
        f"shard={shard_index} "
        f"episodes={shard_lengths[shard_index]} "
        f"global_range={start}:{stop}"
    )

    print(
        f"output={output_dir}"
    )

    print(
        f"language_embeddings="
        f"{language_embeddings_dir}"
    )

    print(
        f"video_embeddings="
        f"{video_embeddings_dir}"
    )

    print(
        f"fps={args.fps}"
    )

    # --------------------------------------------------
    # Read only this shard's episodes
    # --------------------------------------------------

    dataset = builder.as_dataset(
        split=f"train[{start}:{stop}]",
        shuffle_files=False,
    )

    for episode_index, episode in enumerate(
        dataset
    ):

        global_index = (
            start
            + episode_index
        )

        print(
            f"\nepisode={episode_index} "
            f"global={global_index}"
        )

        export_episode(
            episode=episode,
            output_dir=output_dir,
            global_index=global_index,
            fps=args.fps,
            language_embeddings_dir=(
                language_embeddings_dir
            ),
            video_embeddings_dir=(
                video_embeddings_dir
            ),
            video_suffix=args.video_suffix,
            overwrite=args.overwrite,
        )


if __name__ == "__main__":
    main()