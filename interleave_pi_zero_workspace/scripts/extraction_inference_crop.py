import os
from pathlib import Path


import numpy as np
import tensorflow_datasets as tfds
from PIL import Image

# =============================================================================
# PATH
# =============================================================================

SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_DIR = SCRIPT_DIR.parent


def resolve_path_from_env(
    env_name: str,
    default_relative_path: str,
) -> Path:
    """
    Usa il path specificato nella variabile d'ambiente `env_name`.

    Se la variabile non è definita, usa un path relativo alla root
    della interleave_pi_zero_workspace.

    Anche un path fornito tramite environment può essere relativo:
    in quel caso viene interpretato rispetto a WORKSPACE_DIR.
    """
    value = os.environ.get(env_name)

    if value is None:
        path = WORKSPACE_DIR / default_relative_path
    else:
        path = Path(os.path.expandvars(os.path.expanduser(value)))

        if not path.is_absolute():
            path = WORKSPACE_DIR / path

    return path.resolve()


DATASET_DIR = resolve_path_from_env(
    "UR5E_INTERLEAVE_DATASET_DIR",
    "processed_data/ur5e_interleave/0.1.0",
)

OUTPUT_DIR = resolve_path_from_env(
    "INTERLEAVE_INSTRUCTION_IMAGE_DIR",
    "interleave_pi0_controller/instruction_images",
)

if not DATASET_DIR.is_dir():
    raise FileNotFoundError(
        f"Dataset directory not found: {DATASET_DIR}"
    )

OUTPUT_DIR.mkdir(
    parents=True,
    exist_ok=True,
)

print(f"Dataset: {DATASET_DIR}")
print(f"Output:  {OUTPUT_DIR}")

# Vecchi task_id del dataset:
# 0 = green -> bin 2
# 3 = yellow -> bin 1
# 6 = blue -> bin 1
# 9 = red -> bin 1
TARGET_TASKS = {
    0: "green_box.png",
    3: "yellow_box.png",
    6: "blue_box.png",
    9: "red_box.png",
}

builder = tfds.builder_from_directory(DATASET_DIR)

dataset = builder.as_dataset(
    split="train",
    shuffle_files=False,
)

saved = set()

for episode in tfds.as_numpy(dataset):
    task_id = int(
        episode["episode_metadata"]["task_id"].numpy()
    )

    if task_id not in TARGET_TASKS or task_id in saved:
        continue

    # `steps` è un tf.data.Dataset annidato.
    first_step = next(iter(episode["steps"]))

    # image_instruction ha shape (1, 224, 224, 3):
    # prendiamo l'unica instruction image dell'episodio.
    crop = first_step["interleaved_instruction"][
        "image_instruction"
    ][0].numpy()

    assert crop.shape == (224, 224, 3)
    assert crop.dtype == np.uint8

    output_path = OUTPUT_DIR / TARGET_TASKS[task_id]

    Image.fromarray(crop, mode="RGB").save(output_path)

    print(
        f"task_id={task_id} -> {output_path} "
        f"shape={crop.shape}"
    )

    saved.add(task_id)

    if len(saved) == len(TARGET_TASKS):
        break

if len(saved) != len(TARGET_TASKS):
    missing = set(TARGET_TASKS) - saved
    raise RuntimeError(f"Missing task ids: {sorted(missing)}")