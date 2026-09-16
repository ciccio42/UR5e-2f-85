#!/usr/bin/env bash

# =============================================================================
# VLA-JEPA + LeRobot + ROS 2 Jazzy runtime launcher
#
# Uso:
#
#   bash vla_jepa_controller.sh
#
# Opzioni:
#
#   --rebuild
#       forza la ricostruzione dell'immagine Docker.
#
#   --no-full-preflight
#       esegue tutti i test leggeri ma NON carica il modello da ~16 GB
#       e NON esegue la dummy inference completa.
#
#
# Il launcher può essere eseguito anche con:
#
#   - UR driver spento
#   - MoveIt spento
#   - ZED containers spenti
#
# I test locali del runtime rimangono validi.
# L'ultimo preflight sul ROS graph stampa soltanto WARNING per gli
# elementi non disponibili e non blocca il launcher.
#
#
# Variabili host opzionali:
#
#   UR5e_2f_85_PATH
#       root della repository che contiene questo controller.
#
#   VLA_JEPA_CHECKPOINT_HOST
#       directory host `pretrained_model` del checkpoint LeRobot.
#
#   VLA_JEPA_ROS_IMAGE
#       default: vla-jepa-ros:spark
#
#   CONTAINER_NAME
#       default: ur_robotiq_vla_jepa
#
#   CPUSET_CPUS
#       default: 0-19
#
#   SHM_SIZE
#       default: 4g
#
# =============================================================================

set -Eeuo pipefail


# =============================================================================
# ARGOMENTI
# =============================================================================

FORCE_REBUILD=0
FULL_PREFLIGHT=1

for arg in "$@"; do
    case "$arg" in

        --rebuild)
            FORCE_REBUILD=1
            ;;

        --no-full-preflight)
            FULL_PREFLIGHT=0
            ;;

        *)
            echo "Argomento sconosciuto: $arg" >&2
            echo
            echo "Uso:"
            echo "  $0 [--rebuild] [--no-full-preflight]"
            exit 2
            ;;

    esac
done


# =============================================================================
# PATH HOST
# =============================================================================

SCRIPT_DIR="$(
    cd "$(dirname "${BASH_SOURCE[0]}")"
    pwd
)"


# Struttura attesa:
#
# REPO_ROOT/
# └── ai_controller/
#     └── ai_controller/
#         └── models/
#             └── vla_jepa_controller/
#                 ├── vla_jepa.py
#                 ├── vla_jepa_utils.py
#                 ├── vla_jepa_controller.py
#                 ├── vla_jepa_config.yaml
#                 ├── vla_jepa_controller.sh
#                 ├── Docker/
#                 │   └── Dockerfile.vla_jepa_ros
#                 └── external/
#                     └── lerobot/
#
DEFAULT_REPO_ROOT="$(
    cd "$SCRIPT_DIR/../../../.."
    pwd
)"

REPO_ROOT="${UR5e_2f_85_PATH:-$DEFAULT_REPO_ROOT}"


CONTROLLER_DIR_HOST="$REPO_ROOT/ai_controller/ai_controller/models/vla_jepa_controller"

DOCKERFILE_HOST="$CONTROLLER_DIR_HOST/Docker/Dockerfile.vla_jepa_ros"

CONFIG_NAME="${VLA_JEPA_CONFIG_NAME:-vla_jepa_config.yaml}"

CONFIG_HOST="$CONTROLLER_DIR_HOST/$CONFIG_NAME"


# -----------------------------------------------------------------------------
# Checkpoint
# -----------------------------------------------------------------------------
#
# Il checkpoint si trova volutamente fuori dalla repository Alex.
# Non viene copiato né nella repo né nell'immagine Docker.
#
DEFAULT_CHECKPOINT_HOST="/home/asus-mivia/Desktop/UR-Control/UR5e-2f-85/ai_controller/checkpoint_folder/lerobot/vla-jepa/016000/pretrained_model"

CHECKPOINT_HOST="${VLA_JEPA_CHECKPOINT_HOST:-$DEFAULT_CHECKPOINT_HOST}"


# =============================================================================
# DOCKER
# =============================================================================

ROS_IMAGE="${VLA_JEPA_ROS_IMAGE:-vla-jepa-ros:spark}"

CONTAINER_NAME="${CONTAINER_NAME:-ur_robotiq_vla_jepa}"

CPUSET_CPUS="${CPUSET_CPUS:-0-19}"

SHM_SIZE="${SHM_SIZE:-4g}"


# =============================================================================
# PATH INTERNI AL CONTAINER
# =============================================================================

RUNTIME_SETUP="/opt/vla-jepa-ros/setup_runtime.sh"

RUNTIME_PYTHON="/opt/vla-jepa-ros/bin/python"


CONTROLLER_CONFIG_CONTAINER="/home/ros2_ws/src/ai_controller/ai_controller/models/vla_jepa_controller/$CONFIG_NAME"


# =============================================================================
# HUGGING FACE CACHE
# =============================================================================

DEFAULT_HF_CACHE_HOST="$REPO_ROOT/ai_controller/ai_controller/models/vla_jepa_controller/huggingface"

VLA_JEPA_HF_CACHE_HOST="${VLA_JEPA_HF_CACHE_HOST:-$DEFAULT_HF_CACHE_HOST}"

# Il mount deve corrispondere a checkpoint_path nello YAML:
#
#   checkpoint_path:
#       /workspace/checkpoints/vla-jepa/016000
#
CHECKPOINT_CONTAINER="/workspace/checkpoints/vla-jepa/016000"


# =============================================================================
# UTILITY
# =============================================================================

fail() {
    echo
    echo "ERROR: $*" >&2
    echo
    exit 1
}


# =============================================================================
# HEADER
# =============================================================================

echo
echo "============================================================"
echo " VLA-JEPA + LeRobot ROS launcher"
echo "============================================================"
echo "Repo root:             $REPO_ROOT"
echo "Controller directory:  $CONTROLLER_DIR_HOST"
echo "Runtime image:         $ROS_IMAGE"
echo "Container:             $CONTAINER_NAME"
echo "Config host:           $CONFIG_HOST"
echo "Checkpoint host:       $CHECKPOINT_HOST"
echo "Checkpoint container:  $CHECKPOINT_CONTAINER"
echo "Full preflight:        $FULL_PREFLIGHT"
echo "============================================================"
echo


# =============================================================================
# VERIFICA STRUTTURA HOST
# =============================================================================

for required_dir in \
    "$REPO_ROOT/ai_controller" \
    "$REPO_ROOT/moveit_controller" \
    "$REPO_ROOT/dataset_collector" \
    "$CONTROLLER_DIR_HOST" \
    "$CONTROLLER_DIR_HOST/external/lerobot"; do

    [[ -d "$required_dir" ]] || \
        fail "Directory host mancante: $required_dir"

done


for required_file in \
    "$CONTROLLER_DIR_HOST/vla_jepa.py" \
    "$CONTROLLER_DIR_HOST/vla_jepa_utils.py" \
    "$CONTROLLER_DIR_HOST/vla_jepa_controller.py" \
    "$CONFIG_HOST" \
    "$DOCKERFILE_HOST"; do

    [[ -f "$required_file" ]] || \
        fail "File host mancante: $required_file"

done


# =============================================================================
# VERIFICA CHECKPOINT HOST
# =============================================================================

[[ -d "$CHECKPOINT_HOST" ]] || \
    fail "Directory checkpoint non trovata: $CHECKPOINT_HOST"


for checkpoint_file in \
    config.json \
    model.safetensors \
    policy_preprocessor.json \
    policy_postprocessor.json \
    policy_postprocessor_ur5e.json \
    policy_preprocessor_step_3_normalizer_processor.safetensors \
    policy_postprocessor_step_2_unnormalizer_processor.safetensors; do

    [[ -f "$CHECKPOINT_HOST/$checkpoint_file" ]] || \
        fail "File checkpoint mancante: $CHECKPOINT_HOST/$checkpoint_file"

done


echo "Struttura host:          OK"
echo "Checkpoint structure:    OK"
echo


# =============================================================================
# BUILD IMMAGINE
# =============================================================================
#
# Dockerfile.vla_jepa_ros viene costruito usando come build context:
#
#   vla_jepa_controller/
#
# perché il Dockerfile contiene:
#
#   COPY external/lerobot ...
#
# =============================================================================

if [[ "$FORCE_REBUILD" == "1" ]] || \
   ! docker image inspect "$ROS_IMAGE" >/dev/null 2>&1; then

    echo "============================================================"
    echo " BUILD - $ROS_IMAGE"
    echo "============================================================"
    echo

    docker build \
        --progress=plain \
        -f "$DOCKERFILE_HOST" \
        -t "$ROS_IMAGE" \
        "$CONTROLLER_DIR_HOST"

    echo
    echo "Build completata."
    echo

else

    echo "Immagine $ROS_IMAGE già presente: build saltata."
    echo "Usare --rebuild per forzarne la ricostruzione."
    echo

fi


# =============================================================================
# RIMOZIONE EVENTUALE CONTAINER PRECEDENTE
# =============================================================================

if docker inspect "$CONTAINER_NAME" >/dev/null 2>&1; then

    echo "Rimozione del container precedente $CONTAINER_NAME..."

    docker rm -f "$CONTAINER_NAME" >/dev/null

fi


# =============================================================================
# COSTRUZIONE docker run
# =============================================================================

docker_args=(
    run
    -d

    --gpus all

    --network host
    --ipc host

    --cpuset-cpus="$CPUSET_CPUS"

    --ulimit memlock=-1:-1
    --shm-size="$SHM_SIZE"

    -e NVIDIA_VISIBLE_DEVICES=all
    -e NVIDIA_DRIVER_CAPABILITIES=all

    -e "VLA_JEPA_CHECKPOINT=$CHECKPOINT_CONTAINER"
    -e "VLA_JEPA_CONTROLLER_CONFIG=$CONTROLLER_CONFIG_CONTAINER"

    -v "$REPO_ROOT/ai_controller:/home/ros2_ws/src/ai_controller"

    -v "$REPO_ROOT/moveit_controller:/home/ros2_ws/src/moveit_controller"

    -v "$REPO_ROOT/dataset_collector:/home/ros2_ws/src/dataset_collector"

    -v "$CHECKPOINT_HOST:$CHECKPOINT_CONTAINER:ro"

    --name "$CONTAINER_NAME"
)


# Package UR5e opzionale.
if [[ -d "$REPO_ROOT/ur5e_2f_85" ]]; then

    docker_args+=(
        -v "$REPO_ROOT/ur5e_2f_85:/home/ros2_ws/src/ur5e_2f_85"
    )

fi


# Directory opzionali già utilizzate dal progetto.
if [[ -d "$REPO_ROOT/traj_tmp" ]]; then

    docker_args+=(
        -v "$REPO_ROOT/traj_tmp:/traj_tmp"
    )

fi


if [[ -d /home/asus-mivia/Desktop/saved_trajectories ]]; then

    docker_args+=(
        -v /home/asus-mivia/Desktop/saved_trajectories:/home/saved_trajectories
    )

fi


if [[ -d /home/asus-mivia/Desktop/dataset ]]; then

    docker_args+=(
        -v /home/asus-mivia/Desktop/dataset:/dataset
    )

fi


# Manteniamo lo stesso ROS_DOMAIN_ID degli altri container.
if [[ -n "${ROS_DOMAIN_ID:-}" ]]; then

    docker_args+=(
        -e "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    )

fi


# Manteniamo anche la stessa implementazione RMW, se esplicitamente scelta.
if [[ -n "${RMW_IMPLEMENTATION:-}" ]]; then

    docker_args+=(
        -e "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
    )

fi


# -----------------------------------------------------------------------------
# Cache Hugging Face opzionale
# -----------------------------------------------------------------------------
#
# Il caricamento VLA-JEPA può richiedere risorse associate a:
#
#   Qwen/Qwen3-VL-2B-Instruct
#   facebook/vjepa2-vitl-fpc64-256
#
# Se si possiede già una cache HF sulla macchina host è possibile passarla:
#
#   export VLA_JEPA_HF_CACHE_HOST=/path/to/huggingface/cache
#
# Non la creiamo automaticamente per evitare di generare file root-owned
# nella home dell'utente.
#
if [[ -n "${VLA_JEPA_HF_CACHE_HOST:-}" ]]; then

    [[ -d "$VLA_JEPA_HF_CACHE_HOST" ]] || \
        fail "VLA_JEPA_HF_CACHE_HOST non esiste: $VLA_JEPA_HF_CACHE_HOST"

    docker_args+=(
        -v "$VLA_JEPA_HF_CACHE_HOST:/workspace/checkpoints/huggingface"
    )

    echo "HuggingFace cache:       $VLA_JEPA_HF_CACHE_HOST"

fi


# =============================================================================
# AVVIO CONTAINER
# =============================================================================

echo
echo "Creazione del container $CONTAINER_NAME..."

docker "${docker_args[@]}" \
    "$ROS_IMAGE" \
    sleep infinity


# =============================================================================
# VERIFICA CONTAINER
# =============================================================================

for _ in $(seq 1 30); do

    if [[ "$(
        docker inspect \
            --format '{{.State.Running}}' \
            "$CONTAINER_NAME"
    )" == "true" ]]; then

        break

    fi

    sleep 1

done


if [[ "$(
    docker inspect \
        --format '{{.State.Running}}' \
        "$CONTAINER_NAME"
)" != "true" ]]; then

    docker logs "$CONTAINER_NAME" >&2 || true

    fail "Il container $CONTAINER_NAME non è rimasto in esecuzione."

fi


echo "Container avviato."
echo


# =============================================================================
# PREFLIGHT 1/6
# Python / ROS / CUDA / ABI NumPy-cv_bridge
# =============================================================================

echo "============================================================"
echo " PREFLIGHT 1/6 - Python / ROS / CUDA"
echo "============================================================"

docker exec "$CONTAINER_NAME" bash -lc "
    set -Eeo pipefail

    [[ -f '$RUNTIME_SETUP' ]] || {
        echo 'Runtime setup mancante: $RUNTIME_SETUP' >&2
        exit 1
    }

    source '$RUNTIME_SETUP'

    echo \"Python: \$(command -v python)\"
    python --version

    python - <<'PY'
import sys

if sys.version_info[:2] != (3, 12):
    raise RuntimeError(
        f'Expected Python 3.12, got {sys.version}'
    )

import numpy as np
import cv2

import rclpy
import cv_bridge
import message_filters
import tf2_ros

from sensor_msgs.msg import Image, JointState
from control_msgs.action import GripperCommand

import torch
import torchvision
import transformers
import diffusers
import safetensors
import einops
import PIL
import scipy
import omegaconf
import qwen_vl_utils


print(f'PyTorch:       {torch.__version__}')
print(f'Torchvision:   {torchvision.__version__}')
print(f'Torch CUDA:    {torch.version.cuda}')
print(f'Transformers:  {transformers.__version__}')
print(f'Diffusers:     {diffusers.__version__}')
print(f'NumPy:         {np.__version__}')
print(f'OpenCV:        {cv2.__version__}')
print(f'SciPy:         {scipy.__version__}')


if not torch.cuda.is_available():
    raise RuntimeError(
        'torch.cuda.is_available() == False'
    )


device = torch.device('cuda')

x = torch.ones(
    (1024, 1024),
    device=device,
)

y = x @ x

if not torch.isfinite(y).all():
    raise RuntimeError(
        'CUDA matrix multiplication produced non-finite values.'
    )


print(f'GPU:           {torch.cuda.get_device_name(0)}')
print(f'Capability:    {torch.cuda.get_device_capability(0)}')
print('CUDA tensor:   OK')


# ---------------------------------------------------------------------
# Test ABI NumPy <-> cv_bridge
#
# Questo test è importante nel runtime VLA-JEPA perché ROS Jazzy /
# cv_bridge utilizza moduli compilati compatibili con NumPy 1.x.
# ---------------------------------------------------------------------

bridge = cv_bridge.CvBridge()

image_np = np.zeros(
    (16, 16, 3),
    dtype=np.uint8,
)

msg = bridge.cv2_to_imgmsg(
    image_np,
    encoding='rgb8',
)

image_back = bridge.imgmsg_to_cv2(
    msg,
    desired_encoding='rgb8',
)

if not np.array_equal(
    image_np,
    image_back,
):
    raise RuntimeError(
        'cv_bridge NumPy round-trip failed.'
    )


print('cv_bridge ABI: OK')
print('ROS imports:   OK')
print('ML imports:    OK')
PY
"

echo


# =============================================================================
# PREFLIGHT 2/6
# LeRobot + checkpoint + processor serializzati
# =============================================================================
#
# Questo test NON carica model.safetensors nella GPU.
#
# Verifica:
#   - import LeRobot;
#   - config.json;
#   - contratto feature;
#   - action configuration;
#   - rename delle camere;
#   - pipeline postprocessor;
#   - state safetensors;
#   - statistiche action incorporate nel checkpoint.
# =============================================================================

echo "============================================================"
echo " PREFLIGHT 2/6 - LeRobot / checkpoint"
echo "============================================================"

docker exec "$CONTAINER_NAME" bash -lc "
    set -Eeo pipefail

    source '$RUNTIME_SETUP'

    python - <<'PY'
import json
import os
from pathlib import Path
from omegaconf import OmegaConf

import numpy as np
from safetensors import safe_open

import lerobot

from lerobot.configs.policies import (
    PreTrainedConfig,
)

from lerobot.policies.factory import (
    get_policy_class,
    make_pre_post_processors,
)

from lerobot.policies.vla_jepa.modeling_vla_jepa import (
    VLAJEPAPolicy,
)


checkpoint_path = Path(
    os.environ['VLA_JEPA_CHECKPOINT']
).resolve()

controller_config_path = Path(
    os.environ['VLA_JEPA_CONTROLLER_CONFIG']
).resolve()

controller_cfg = OmegaConf.load(
    controller_config_path
)

postprocessor_config_filename = str(
    controller_cfg.get(
        'postprocessor_config_filename',
        'policy_postprocessor.json',
    )
)

if not checkpoint_path.is_dir():
    raise FileNotFoundError(
        checkpoint_path
    )


required_files = (
    'config.json',
    'model.safetensors',
    'policy_preprocessor.json',
    'policy_postprocessor.json',
)

for filename in required_files:

    path = checkpoint_path / filename

    if not path.is_file():
        raise FileNotFoundError(path)


# ---------------------------------------------------------------------
# Policy config
# ---------------------------------------------------------------------

cfg = PreTrainedConfig.from_pretrained(
    checkpoint_path
)


if cfg.type != 'vla_jepa':
    raise RuntimeError(
        f'Expected policy type vla_jepa, got {cfg.type!r}'
    )


if int(cfg.action_dim) != 7:
    raise RuntimeError(
        f'Expected action_dim=7, got {cfg.action_dim}'
    )


if int(cfg.chunk_size) != 7:
    raise RuntimeError(
        f'Expected chunk_size=7, got {cfg.chunk_size}'
    )


if int(cfg.n_action_steps) != 7:
    raise RuntimeError(
        f'Expected n_action_steps=7, got {cfg.n_action_steps}'
    )


if int(cfg.num_inference_timesteps) != 4:
    raise RuntimeError(
        'Unexpected num_inference_timesteps: '
        f'{cfg.num_inference_timesteps}'
    )


if tuple(cfg.resize_images_to) != (224, 224):
    raise RuntimeError(
        'Expected resize_images_to=[224,224], got '
        f'{cfg.resize_images_to}'
    )


expected_input_features = {
    'observation.images.exterior_1_left',
    'observation.images.exterior_2_left',
}

actual_input_features = set(
    cfg.input_features.keys()
)

if actual_input_features != expected_input_features:
    raise RuntimeError(
        'Unexpected VLA-JEPA input features: '
        f'{sorted(actual_input_features)}'
    )


if 'observation.state' in actual_input_features:
    raise RuntimeError(
        'Current UR5e checkpoint unexpectedly declares '
        'observation.state as an input feature.'
    )


for key in (
    'action',
    'action.world',
):

    if key not in cfg.output_features:
        raise RuntimeError(
            f'Missing output feature {key!r}'
        )

    shape = tuple(
        cfg.output_features[key].shape
    )

    if shape != (7,):
        raise RuntimeError(
            f'{key} must have shape (7,), got {shape}'
        )


# Verifichiamo che la factory riesca a risolvere la classe
# senza ancora istanziare il modello.
policy_cls = get_policy_class(
    cfg.type
)

if policy_cls is not VLAJEPAPolicy:
    raise RuntimeError(
        f'Unexpected policy class: {policy_cls}'
    )


# ---------------------------------------------------------------------
# Preprocessor JSON
# ---------------------------------------------------------------------

preprocessor_path = (
    checkpoint_path
    / 'policy_preprocessor.json'
)

with preprocessor_path.open(
    'r',
    encoding='utf-8',
) as f:

    pre_cfg = json.load(f)


rename_steps = [
    step
    for step in pre_cfg['steps']
    if step.get('registry_name')
    == 'rename_observations_processor'
]

if len(rename_steps) != 1:
    raise RuntimeError(
        'Expected exactly one rename_observations_processor.'
    )


rename_map = rename_steps[0][
    'config'
][
    'rename_map'
]


expected_rename_map = {
    'observation.images.front':
        'observation.images.exterior_1_left',

    'observation.images.gripper':
        'observation.images.exterior_2_left',
}


if rename_map != expected_rename_map:
    raise RuntimeError(
        'Unexpected camera rename map: '
        f'{rename_map}'
    )


# ---------------------------------------------------------------------
# Postprocessor JSON
# ---------------------------------------------------------------------

postprocessor_path = (
    checkpoint_path
    / postprocessor_config_filename
)

if not postprocessor_path.is_file():
    raise FileNotFoundError(
        postprocessor_path
    )

with postprocessor_path.open(
    'r',
    encoding='utf-8',
) as f:

    post_cfg = json.load(f)


post_names = [
    step.get('registry_name')
    for step in post_cfg['steps']
]


expected_post_names = [
    'vla_jepa_clip_actions',
    'unnormalizer_processor',
    'device_processor',
]


if post_names != expected_post_names:
    raise RuntimeError(
        'Unexpected postprocessor pipeline: '
        f'{post_names}'
    )


# ---------------------------------------------------------------------
# Processor state files
# ---------------------------------------------------------------------

state_files = []

for processor_cfg in (
    pre_cfg,
    post_cfg,
):

    for step in processor_cfg['steps']:

        state_file = step.get(
            'state_file'
        )

        if state_file is not None:
            state_files.append(
                state_file
            )


for state_file in state_files:

    state_path = (
        checkpoint_path
        / state_file
    )

    if not state_path.is_file():
        raise FileNotFoundError(
            state_path
        )


# ---------------------------------------------------------------------
# Action statistics from postprocessor state
# ---------------------------------------------------------------------

unnormalizer_steps = [
    step
    for step in post_cfg['steps']
    if step.get('registry_name')
    == 'unnormalizer_processor'
]

if len(unnormalizer_steps) != 1:
    raise RuntimeError(
        'Expected exactly one unnormalizer_processor.'
    )


post_state_path = (
    checkpoint_path
    / unnormalizer_steps[0]['state_file']
)


with safe_open(
    post_state_path,
    framework='pt',
    device='cpu',
) as f:

    keys = set(
        f.keys()
    )

    for required_key in (
        'action.min',
        'action.max',
        'action.mean',
        'action.std',
    ):

        if required_key not in keys:
            raise RuntimeError(
                f'Missing statistic {required_key}'
            )

    action_min = (
        f.get_tensor('action.min')
        .float()
        .cpu()
        .numpy()
    )

    action_max = (
        f.get_tensor('action.max')
        .float()
        .cpu()
        .numpy()
    )


if action_min.shape != (7,):
    raise RuntimeError(
        f'action.min has shape {action_min.shape}'
    )

if action_max.shape != (7,):
    raise RuntimeError(
        f'action.max has shape {action_max.shape}'
    )


if not np.all(
    np.isfinite(action_min)
):
    raise RuntimeError(
        'action.min contains non-finite values.'
    )


if not np.all(
    np.isfinite(action_max)
):
    raise RuntimeError(
        'action.max contains non-finite values.'
    )


# Check specifico del dataset UR5e ricevuto.
if not np.isclose(
    action_min[6],
    0.0,
    atol=1e-5,
):
    raise RuntimeError(
        f'Expected gripper action.min=0, got {action_min[6]}'
    )


if not np.isclose(
    action_max[6],
    20.0,
    atol=1e-3,
):
    raise RuntimeError(
        f'Expected gripper action.max=20, got {action_max[6]}'
    )


print(f'LeRobot:       {Path(lerobot.__file__).resolve()}')
print(f'Checkpoint:    {checkpoint_path}')
print(f'Policy type:   {cfg.type}')
print(f'Action dim:    {cfg.action_dim}')
print(f'Chunk size:    {cfg.chunk_size}')
print(f'Action steps:  {cfg.n_action_steps}')
print(
    'Input images:  '
    + ', '.join(
        sorted(actual_input_features)
    )
)
print('Camera rename: OK')
print('Postprocessor: OK')
print('Processor state files: OK')
print(f'action.min:    {action_min}')
print(f'action.max:    {action_max}')
print('Checkpoint metadata: OK')
PY
"

echo


# =============================================================================
# PREFLIGHT 3/6
# Config controller + VLA-JEPA utils
# =============================================================================
#
# Anche questo test NON carica il modello.
#
# Verifica:
#   - YAML;
#   - 16 task;
#   - mapping camere;
#   - preprocessing front/gripper;
#   - formato observation LeRobot;
#   - scale factor 0.05;
#   - conversione delta -> target assoluto;
#   - convenzione gripper del controller.
# =============================================================================

echo "============================================================"
echo " PREFLIGHT 3/6 - Controller config / utils"
echo "============================================================"

docker exec "$CONTAINER_NAME" bash -lc "
    set -Eeo pipefail

    source '$RUNTIME_SETUP'

    cd /home/ros2_ws

    echo 'Build dipendenze ROS + ai_controller...'

    colcon build \
        --symlink-install \
        --packages-up-to ai_controller

    source /home/ros2_ws/install/setup.bash

    python - <<'PY'
import os
from pathlib import Path

import numpy as np
import torch

from omegaconf import OmegaConf

from ai_controller.models.vla_jepa_controller.vla_jepa_utils import (
    ACTION_DIM,
    DATASET_ACTION_SCALE,
    FRONT_CROP_MARGINS,
    IMAGE_SIZE,
    build_lerobot_observation,
    delta_action_to_absolute_target,
    gripper_binary_to_moveit,
    process_front_image,
    process_gripper_image,
)


config_path = Path(
    os.environ['VLA_JEPA_CONTROLLER_CONFIG']
).resolve()

checkpoint_path = Path(
    os.environ['VLA_JEPA_CHECKPOINT']
).resolve()


if not config_path.is_file():
    raise FileNotFoundError(
        config_path
    )


cfg = OmegaConf.load(
    config_path
)

OmegaConf.resolve(
    cfg
)


# ---------------------------------------------------------------------
# YAML runtime
# ---------------------------------------------------------------------

configured_checkpoint = Path(
    str(cfg.checkpoint_path)
)

if configured_checkpoint != checkpoint_path:
    raise RuntimeError(
        'Checkpoint path mismatch between launcher and YAML:\\n'
        f'  launcher: {checkpoint_path}\\n'
        f'  yaml:     {configured_checkpoint}'
    )


if str(cfg.device) != 'cuda':
    raise RuntimeError(
        f'Expected device=cuda, got {cfg.device}'
    )


if int(cfg.front_camera_index) != 0:
    raise RuntimeError(
        'Expected front_camera_index=0.'
    )


if int(cfg.gripper_camera_index) != 3:
    raise RuntimeError(
        'Expected gripper_camera_index=3.'
    )


if float(
    cfg.dataset_action_scale
) != 0.05:
    raise RuntimeError(
        'Expected dataset_action_scale=0.05.'
    )


if float(
    cfg.gripper_open_position
) != 0.0:
    raise RuntimeError(
        'Expected gripper_open_position=0.'
    )


if float(
    cfg.gripper_closed_position
) != 255.0:
    raise RuntimeError(
        'Expected gripper_closed_position=255.'
    )


if len(cfg.tasks) != 16:
    raise RuntimeError(
        f'Expected 16 tasks, got {len(cfg.tasks)}'
    )


expected_task_ids = {
    f'{i:02d}'
    for i in range(16)
}

actual_task_ids = {
    str(task_id)
    for task_id in cfg.tasks.keys()
}


if actual_task_ids != expected_task_ids:
    raise RuntimeError(
        'Unexpected task IDs: '
        f'{sorted(actual_task_ids)}'
    )


for task_id in sorted(
    expected_task_ids
):

    prompt = str(
        cfg.tasks[task_id].prompt
    ).strip()

    if not prompt:
        raise RuntimeError(
            f'Task {task_id} has an empty prompt.'
        )


# ---------------------------------------------------------------------
# Image preprocessing
# ---------------------------------------------------------------------

dummy_front = np.zeros(
    (376, 672, 3),
    dtype=np.uint8,
)

dummy_gripper = np.zeros(
    (376, 672, 3),
    dtype=np.uint8,
)


front = process_front_image(
    dummy_front,
    input_color_order='rgb',
)

gripper = process_gripper_image(
    dummy_gripper,
    input_color_order='rgb',
)


expected_shape = (
    3,
    IMAGE_SIZE,
    IMAGE_SIZE,
)


for name, image in (
    ('front', front),
    ('gripper', gripper),
):

    if tuple(
        image.shape
    ) != expected_shape:

        raise RuntimeError(
            f'{name} shape = {tuple(image.shape)}'
        )

    if image.dtype != torch.float32:
        raise RuntimeError(
            f'{name} dtype = {image.dtype}'
        )

    if (
        float(image.min()) < 0.0
        or float(image.max()) > 1.0
    ):
        raise RuntimeError(
            f'{name} is not in [0,1].'
        )


observation = build_lerobot_observation(
    front_image=front,
    gripper_image=gripper,
    task='Pick the green box and place it into the first bin',
)


expected_keys = {
    'observation.images.front',
    'observation.images.gripper',
    'task',
}


if set(
    observation.keys()
) != expected_keys:

    raise RuntimeError(
        f'Unexpected observation keys: {observation.keys()}'
    )


# ---------------------------------------------------------------------
# Geometric postprocessing
# ---------------------------------------------------------------------

reference_position = np.array(
    [
        0.10,
        0.20,
        0.30,
    ],
    dtype=np.float32,
)

reference_quaternion = np.array(
    [
        0.0,
        0.0,
        0.0,
        1.0,
    ],
    dtype=np.float32,
)


# action[0] = 1 nello spazio scalato del dataset
# deve diventare +0.05 m.
open_action = np.array(
    [
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ],
    dtype=np.float32,
)


(
    position,
    quaternion,
    gripper_state,
) = delta_action_to_absolute_target(
    postprocessed_action=open_action,
    reference_position=reference_position,
    reference_quaternion_xyzw=reference_quaternion,
    currently_closed=False,
)


expected_position = np.array(
    [
        0.15,
        0.20,
        0.30,
    ],
    dtype=np.float32,
)


if not np.allclose(
    position,
    expected_position,
    atol=1e-6,
):
    raise RuntimeError(
        'Dataset scale-factor test failed: '
        f'{position}'
    )


if not np.isclose(
    np.linalg.norm(quaternion),
    1.0,
    atol=1e-6,
):
    raise RuntimeError(
        'Quaternion output is not normalized.'
    )


if gripper_state != 0:
    raise RuntimeError(
        'Expected +1 VLA-JEPA gripper -> controller OPEN=0.'
    )


if gripper_binary_to_moveit(
    gripper_state
) != 0.0:
    raise RuntimeError(
        'Expected OPEN -> MoveIt 0.'
    )


close_action = open_action.copy()

# Gripper attualmente aperto:
# 19 > 18 -> deve chiudere.
close_action[6] = 19.0

(
    _,
    _,
    gripper_state,
) = delta_action_to_absolute_target(
    postprocessed_action=close_action,
    reference_position=reference_position,
    reference_quaternion_xyzw=reference_quaternion,
    currently_closed=False,
)



if gripper_state != 1:
    raise RuntimeError(
        'Expected -1 VLA-JEPA gripper -> controller CLOSED=1.'
    )


if gripper_binary_to_moveit(
    gripper_state
) != 255.0:
    raise RuntimeError(
        'Expected CLOSED -> MoveIt 255.'
    )


print(f'Config:             {config_path}')
print(f'Image size:         {IMAGE_SIZE}')
print(f'Front crop:         {FRONT_CROP_MARGINS}')
print(f'Action dimension:   {ACTION_DIM}')
print(f'Dataset scale:      {DATASET_ACTION_SCALE}')
print('Front preprocessing:   OK')
print('Gripper preprocessing: OK')
print('LeRobot observation:   OK')
print('Delta -> absolute:     OK')
print('Gripper 0/1 -> 0/255: OK')
print('Tasks:                 16/16 OK')
PY
"

echo


# =============================================================================
# PREFLIGHT 4/6
# ROS workspace / import controller
# =============================================================================

echo "============================================================"
echo " PREFLIGHT 4/6 - ROS workspace"
echo "============================================================"

docker exec "$CONTAINER_NAME" bash -lc "
    set -Eeo pipefail

    source '$RUNTIME_SETUP'

    cd /home/ros2_ws


    # ------------------------------------------------------------------
    # Verifica Python usato dall'entrypoint ROS.
    # ------------------------------------------------------------------

    ai_entrypoint=/home/ros2_ws/install/ai_controller/lib/ai_controller/ai_controller_node

    runtime_python='$RUNTIME_PYTHON'


    [[ -f \"\$ai_entrypoint\" ]] || {
        echo \"Entry point mancante: \$ai_entrypoint\" >&2
        exit 1
    }


    [[ -x \"\$runtime_python\" ]] || {
        echo \"Runtime Python mancante: \$runtime_python\" >&2
        exit 1
    }


    expected_shebang=\"#!\$runtime_python\"

    actual_shebang=\"\$(
        head -n 1 \"\$ai_entrypoint\"
    )\"


    if [[ \"\$actual_shebang\" != \"\$expected_shebang\" ]]; then

        echo 'Correzione shebang ai_controller_node:'
        echo \"  prima: \$actual_shebang\"
        echo \"  dopo:  \$expected_shebang\"

        sed -i \
            \"1c\\\\\$expected_shebang\" \
            \"\$ai_entrypoint\"

    fi


    actual_shebang=\"\$(
        head -n 1 \"\$ai_entrypoint\"
    )\"


    if [[ \"\$actual_shebang\" != \"\$expected_shebang\" ]]; then

        echo \"Shebang non corretta: \$actual_shebang\" >&2
        exit 1

    fi


    python - <<'PY'
import rclpy

from moveit_controller_srvs.srv import (
    GoHome,
    GoToPose,
)

from ai_controller.models.vla_jepa_controller.vla_jepa import (
    VLAJEPARuntime,
)

from ai_controller.models.vla_jepa_controller.vla_jepa_controller import (
    VLAJEPAController,
)

from ai_controller.models.vla_jepa_controller.vla_jepa_utils import (
    process_front_image,
    process_gripper_image,
    delta_action_to_absolute_target,
)


print('rclpy:                         OK')
print('moveit_controller_srvs:        OK')
print('VLAJEPARuntime:                OK')
print('VLAJEPAController:             OK')
print('VLA-JEPA utils:                OK')
PY


    echo
    echo \"ai_controller entrypoint:\"
    head -n 1 \"\$ai_entrypoint\"
"

echo


# =============================================================================
# PREFLIGHT 5/6
# FULL MODEL LOAD + DUMMY INFERENCE
# =============================================================================
#
# TEST FORTE E OPZIONALE.
#
# Questo test:
#
#   1. carica config.json;
#   2. costruisce VLA-JEPA;
#   3. carica model.safetensors;
#   4. carica processor serializzati;
#   5. carica Qwen/V-JEPA necessari;
#   6. preprocessa due immagini dummy;
#   7. esegue una vera select_action();
#   8. applica il postprocessor LeRobot;
#   9. converte la delta action in target UR5e assoluto.
#
# NON vengono creati:
#
#   - ROS publisher;
#   - ROS client;
#   - MoveIt command;
#   - UR command;
#
# Quindi questo test NON può muovere il robot.
#
# Può essere eseguito anche con tutti gli altri container spenti.
# =============================================================================

echo "============================================================"
echo " PREFLIGHT 5/6 - Full VLA-JEPA inference"
echo "============================================================"


if [[ "$FULL_PREFLIGHT" == "1" ]]; then

    echo "Caricamento completo del checkpoint..."
    echo "Questo test può richiedere parecchio tempo e memoria GPU."
    echo


    docker exec "$CONTAINER_NAME" bash -lc "
        set -Eeo pipefail

        source '$RUNTIME_SETUP'
        source /home/ros2_ws/install/setup.bash

        python - <<'PY'
import os

import numpy as np
import torch

from ai_controller.models.vla_jepa_controller.vla_jepa_controller import (
    VLAJEPAController,
)


config_path = os.environ[
    'VLA_JEPA_CONTROLLER_CONFIG'
]


# ---------------------------------------------------------------------
# Costruzione controller
#
# In questa fase viene realmente caricato il modello.
# ---------------------------------------------------------------------

controller = VLAJEPAController(
    model_config=config_path,
    task_name='pick_place',
)


controller.load_command(
    demo_path='',
    task_id='00',
)


# ---------------------------------------------------------------------
# Dummy cameras
#
# Il controller riceve la lista COMPLETA delle quattro camere:
#
#   0 -> front
#   1 -> left
#   2 -> right
#   3 -> gripper
#
# VLA-JEPA utilizza soltanto 0 e 3.
# ---------------------------------------------------------------------

dummy_front_rgb = np.zeros(
    (376, 672, 3),
    dtype=np.uint8,
)

dummy_left_rgb = np.zeros(
    (376, 672, 3),
    dtype=np.uint8,
)

dummy_right_rgb = np.zeros(
    (376, 672, 3),
    dtype=np.uint8,
)

dummy_gripper_rgb = np.zeros(
    (376, 672, 3),
    dtype=np.uint8,
)


dummy_images = [
    dummy_front_rgb,
    dummy_left_rgb,
    dummy_right_rgb,
    dummy_gripper_rgb,
]


# ---------------------------------------------------------------------
# Dummy robot state
#
# Non entra nel modello VLA-JEPA.
#
# Serve esclusivamente per convertire:
#
#   delta action -> target assoluto
#
# Formato:
#
#   [x, y, z, qx, qy, qz, qw, gripper_closed]
# ---------------------------------------------------------------------

dummy_robot_state = np.array(
    [
        -0.15552094619366708,
         0.34869994018501943,
         0.1532803451753288,
         0.9994452044624775,
         0.03161651380119412,
         0.0021438049655468088,
         0.010251021036213035,
         0.0,
    ],
    dtype=np.float64,
)


# ---------------------------------------------------------------------
# Vera inference
# ---------------------------------------------------------------------

out = controller.inference(
    input_data=[
        dummy_images,
        dummy_robot_state,
    ],
    t=0,
    save_path=None,
)


out = np.asarray(
    out,
    dtype=np.float32,
)


# ---------------------------------------------------------------------
# Validazione output
# ---------------------------------------------------------------------

if out.shape != (1, 8):

    raise RuntimeError(
        'Unexpected controller output shape: '
        f'{out.shape}'
    )


if not np.all(
    np.isfinite(out)
):

    raise RuntimeError(
        'Controller output contains non-finite values: '
        f'{out}'
    )


quat = out[
    0,
    3:7,
]


quat_norm = float(
    np.linalg.norm(
        quat
    )
)


if (
    not np.isfinite(quat_norm)
    or quat_norm < 0.99
    or quat_norm > 1.01
):

    raise RuntimeError(
        f'Invalid quaternion norm: {quat_norm}'
    )


gripper = float(
    out[0, 7]
)


if gripper not in (
    0.0,
    255.0,
):

    raise RuntimeError(
        'Unexpected final gripper command: '
        f'{gripper}'
    )


print()
print('============================================================')
print(' FULL VLA-JEPA PREFLIGHT PASSED')
print('============================================================')
print(f'Output shape:      {out.shape}')
print(f'Output action:     {out[0]}')
print(f'Quaternion norm:   {quat_norm:.9f}')
print(f'Gripper command:   {gripper}')
print('============================================================')
print()


del controller


if torch.cuda.is_available():

    torch.cuda.empty_cache()

PY
    "

else

    echo "Full model inference saltata (--no-full-preflight)."

fi

echo


# =============================================================================
# PREFLIGHT 6/6
# ROS GRAPH
# =============================================================================
#
# Questo è deliberatamente NON bloccante.
#
# Se gli altri container non sono ancora in esecuzione verranno stampati
# WARNING, ma il launcher continuerà normalmente.
# =============================================================================

echo "============================================================"
echo " PREFLIGHT 6/6 - ROS graph"
echo "============================================================"

docker exec "$CONTAINER_NAME" bash -lc "
    set -Eeo pipefail

    source '$RUNTIME_SETUP'
    source /home/ros2_ws/install/setup.bash

    sleep 2


    topics=\"\$(
        ros2 topic list \
            2>/dev/null \
            || true
    )\"


    services=\"\$(
        ros2 service list \
            2>/dev/null \
            || true
    )\"


    actions=\"\$(
        ros2 action list \
            2>/dev/null \
            || true
    )\"


    check_topic() {

        local name=\"\$1\"

        if grep -qx \"\$name\" <<< \"\$topics\"; then

            echo \"[OK]      topic   \$name\"

        else

            echo \"[WARNING] topic   \$name\"

        fi
    }


    check_service() {

        local name=\"\$1\"

        if grep -qx \"\$name\" <<< \"\$services\"; then

            echo \"[OK]      service \$name\"

        else

            echo \"[WARNING] service \$name\"

        fi
    }


    check_action() {

        local name=\"\$1\"

        if grep -qx \"\$name\" <<< \"\$actions\"; then

            echo \"[OK]      action  \$name\"

        else

            echo \"[WARNING] action  \$name\"

        fi
    }


    check_topic /joint_states

    check_topic /zed_front/zed_node/rgb/color/rect/image

    check_topic /zed_left/zed_node/rgb/color/rect/image

    check_topic /zed_right/zed_node/rgb/color/rect/image

    check_topic /zed_gripper/zed_node/rgb/color/rect/image


    check_service /set_robot_to_home

    check_service /set_robot_to_pose


    check_action /robotiq_gripper_controller/gripper_cmd
"

echo


# =============================================================================
# READY
# =============================================================================

echo "============================================================"
echo " VLA-JEPA RUNTIME PRONTO"
echo "============================================================"
echo
echo "Container:  $CONTAINER_NAME"
echo "Image:      $ROS_IMAGE"
echo "Config:     $CONTROLLER_CONFIG_CONTAINER"
echo "Checkpoint: $CHECKPOINT_CONTAINER"
echo
echo "I preflight locali sono completati."
echo
echo "NOTA:"
echo "Il comando ROS seguente sarà utilizzabile dopo l'aggiunta"
echo "di vla_jepa_controller in ai_controller_node.py."
echo
echo "ros2 run ai_controller ai_controller_node --ros-args \\"
echo "  -p ai_controller_target:=vla_jepa_controller \\"
echo "  -p model_config_path:=$CONTROLLER_CONFIG_CONTAINER"
echo
echo "Apertura shell interattiva..."
echo


# =============================================================================
# SHELL INTERATTIVA
# =============================================================================

exec docker exec -it "$CONTAINER_NAME" bash -lc "
    source '$RUNTIME_SETUP'
    source /home/ros2_ws/install/setup.bash

    echo
    echo '============================================================'
    echo ' VLA-JEPA + LeRobot + ROS 2 Jazzy'
    echo '============================================================'
    echo \"Python:     \$(command -v python)\"
    echo \"Checkpoint: \$VLA_JEPA_CHECKPOINT\"
    echo \"Config:     \$VLA_JEPA_CONTROLLER_CONFIG\"
    echo
    echo 'Comando AIControllerNode:'
    echo
    echo 'ros2 run ai_controller ai_controller_node --ros-args \\\\'
    echo '  -p ai_controller_target:=vla_jepa_controller \\\\'
    echo '  -p model_config_path=$CONTROLLER_CONFIG_CONTAINER'
    echo
    echo '============================================================'
    echo

    exec bash --noprofile --norc -i
"