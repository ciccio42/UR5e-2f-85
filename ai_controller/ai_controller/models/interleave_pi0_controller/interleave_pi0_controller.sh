#!/usr/bin/env bash

# =============================================================================
# Interleave-Pi0 + ROS 2 Jazzy runtime launcher
#
# Uso tipico:
#
#   ./run_interleave_pi0_container.sh
#
# Opzioni:
#
#   --rebuild
#       forza la ricostruzione di interleave-pizero-ros:spark
#
#   --no-full-preflight
#       salta la dummy inference completa.
#       Restano comunque attivi tutti gli smoke test di import/config/CUDA/ROS.
#
#
# Variabili host:
#
#   UR5e_2f_85_PATH
#       root della repository UR5e-2f-85
#
#   INTERLEAVE_PI0_CHECKPOINT_HOST
#       path host del checkpoint finale UR5e
#
#   INTERLEAVE_PI0_PALIGEMMA_HOST
#       path host della directory PaliGemma usata dal modello
#
# In alternativa, se INTERLEAVE_PI0_CHECKPOINT / INTERLEAVE_PI0_PALIGEMMA
# puntano già a path host oppure a path /workspace/..., vengono riutilizzati.
#
#   CONTAINER_NAME
#       default: ur_robotiq_interleave_pi0
#
#   INTERLEAVE_ROS_IMAGE
#       default: interleave-pizero-ros:spark
#
#   INTERLEAVE_BASE_IMAGE
#       default: interleave-pizero:spark
#
#   CPUSET_CPUS
#       default: 0-19
#
#   SHM_SIZE
#       default: 1g
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
            echo "Uso: $0 [--rebuild] [--no-full-preflight]" >&2
            exit 2
            ;;
    esac
done


# =============================================================================
# PATH HOST
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Se lo script è:
#
#   REPO_ROOT/
#     ai_controller/
#       ai_controller/
#         models/
#           interleave_pi0_controller/
#             run_interleave_pi0_container.sh
#
# ../../../.. porta a REPO_ROOT.
DEFAULT_REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
REPO_ROOT="${UR5e_2f_85_PATH:-$DEFAULT_REPO_ROOT}"

INTERLEAVE_WORKSPACE_HOST="$REPO_ROOT/interleave_pi_zero_workspace"

CHECKPOINT_HOST="$INTERLEAVE_WORKSPACE_HOST/checkpoints/posttraining/step66240.pt"

PALIGEMMA_HOST="$INTERLEAVE_WORKSPACE_HOST/checkpoints/paligemma/paligemma-3b-pt-224"

DOCKERFILE_HOST="$INTERLEAVE_WORKSPACE_HOST/Docker/Dockerfile.ros"


# =============================================================================
# DOCKER
# =============================================================================

BASE_IMAGE="${INTERLEAVE_BASE_IMAGE:-interleave-pizero:spark}"
ROS_IMAGE="${INTERLEAVE_ROS_IMAGE:-interleave-pizero-ros:spark}"

CONTAINER_NAME="${CONTAINER_NAME:-ur_robotiq_interleave_pi0}"

CPUSET_CPUS="${CPUSET_CPUS:-0-19}"
SHM_SIZE="${SHM_SIZE:-1g}"


# =============================================================================
# PATH INTERNI AL CONTAINER
# =============================================================================

RUNTIME_SETUP="/opt/interleave-pizero-ros/setup_runtime.sh"

CONTROLLER_CONFIG_CONTAINER="/home/ros2_ws/src/ai_controller/ai_controller/models/interleave_pi0_controller/interleave_pi0_config.yaml"

CHECKPOINT_CONTAINER="/models/interleave_pi0/checkpoint.pt"
PALIGEMMA_CONTAINER="/models/interleave_pi0/paligemma"


# =============================================================================
# UTILITY
# =============================================================================

fail() {
    echo
    echo "ERROR: $*" >&2
    echo
    exit 1
}


# Permette di specificare anche un path /workspace/... già usato nei container.
workspace_path_to_host() {
    local path="$1"

    if [[ "$path" == /workspace/* ]]; then
        printf '%s/%s\n' \
            "$INTERLEAVE_WORKSPACE_HOST" \
            "${path#/workspace/}"
    else
        printf '%s\n' "$path"
    fi
}


# =============================================================================
# VERIFICA STRUTTURA REPOSITORY
# =============================================================================

echo "============================================================"
echo " Interleave-Pi0 ROS launcher"
echo "============================================================"
echo "Repo root:             $REPO_ROOT"
echo "Interleave workspace:  $INTERLEAVE_WORKSPACE_HOST"
echo "Base image:            $BASE_IMAGE"
echo "Runtime image:         $ROS_IMAGE"
echo "Container:             $CONTAINER_NAME"
echo "============================================================"
echo


for required_dir in \
    "$REPO_ROOT/ai_controller" \
    "$REPO_ROOT/moveit_controller" \
    "$REPO_ROOT/dataset_collector" \
    "$INTERLEAVE_WORKSPACE_HOST" \
    "$INTERLEAVE_WORKSPACE_HOST/external/Interleave-VLA/open-pi-zero"; do

    [[ -d "$required_dir" ]] || \
        fail "Directory host mancante: $required_dir"
done


[[ -f "$DOCKERFILE_HOST" ]] || \
    fail "Dockerfile ROS mancante: $DOCKERFILE_HOST"


CONFIG_HOST="$REPO_ROOT/ai_controller/ai_controller/models/interleave_pi0_controller/interleave_pi0_config.yaml"

[[ -f "$CONFIG_HOST" ]] || \
    fail "Config runtime Interleave-Pi0 mancante: $CONFIG_HOST"


# =============================================================================
# CHECKPOINT E PALIGEMMA
# =============================================================================
#
# Preferiamo le variabili *_HOST.
#
# Per comodità accettiamo anche:
#
#   INTERLEAVE_PI0_CHECKPOINT
#   INTERLEAVE_PI0_PALIGEMMA
#
# se erano già state definite per il training.
# =============================================================================




[[ -f "$CHECKPOINT_HOST" ]] || \
    fail "Checkpoint non trovato: $CHECKPOINT_HOST"

[[ -d "$PALIGEMMA_HOST" ]] || \
    fail "Directory PaliGemma non trovata: $PALIGEMMA_HOST"


echo "Checkpoint host:        $CHECKPOINT_HOST"
echo "PaliGemma host:         $PALIGEMMA_HOST"
echo


# =============================================================================
# BUILD DELL'IMMAGINE ROS
# =============================================================================

if ! docker image inspect "$BASE_IMAGE" >/dev/null 2>&1; then
    fail "Immagine di training mancante: $BASE_IMAGE
Deve esistere prima di poter costruire l'immagine ROS derivata."
fi


if [[ "$FORCE_REBUILD" == "1" ]] || \
   ! docker image inspect "$ROS_IMAGE" >/dev/null 2>&1; then

    echo "============================================================"
    echo " Build immagine $ROS_IMAGE"
    echo "============================================================"

    docker build \
        -f "$DOCKERFILE_HOST" \
        --build-arg "BASE_IMAGE=$BASE_IMAGE" \
        -t "$ROS_IMAGE" \
        "$INTERLEAVE_WORKSPACE_HOST"

    echo
    echo "Build completata."
    echo

else
    echo "Immagine $ROS_IMAGE già presente: build saltata."
    echo "Usare --rebuild per forzare una nuova build."
    echo
fi


# =============================================================================
# CONTAINER
# =============================================================================
#
# Il container viene volutamente ricreato ad ogni launcher.
#
# Non contiene stato persistente importante:
#   - dipendenze -> immagine Docker
#   - codice      -> bind mount
#   - modelli     -> bind mount
#
# Evitiamo così container vecchi con mount/config obsolete.
# =============================================================================

if docker inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
    echo "Rimozione del container precedente $CONTAINER_NAME..."
    docker rm -f "$CONTAINER_NAME" >/dev/null
fi


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

    -e "INTERLEAVE_PI0_CHECKPOINT=$CHECKPOINT_CONTAINER"
    -e "INTERLEAVE_PI0_PALIGEMMA=$PALIGEMMA_CONTAINER"
    -e "INTERLEAVE_PI0_CONTROLLER_CONFIG=$CONTROLLER_CONFIG_CONTAINER"

    -e "INTERLEAVE_FULL_PREFLIGHT=$FULL_PREFLIGHT"

    -v "$INTERLEAVE_WORKSPACE_HOST:/workspace"

    -v "$REPO_ROOT/ai_controller:/home/ros2_ws/src/ai_controller"
    -v "$REPO_ROOT/moveit_controller:/home/ros2_ws/src/moveit_controller"
    -v "$REPO_ROOT/dataset_collector:/home/ros2_ws/src/dataset_collector"

    -v "$CHECKPOINT_HOST:$CHECKPOINT_CONTAINER:ro"
    -v "$PALIGEMMA_HOST:$PALIGEMMA_CONTAINER:ro"

    --name "$CONTAINER_NAME"
)


# Alcuni package/dependency graph possono riferirsi anche al package UR5e.
if [[ -d "$REPO_ROOT/ur5e_2f_85" ]]; then
    docker_args+=(
        -v "$REPO_ROOT/ur5e_2f_85:/home/ros2_ws/src/ur5e_2f_85"
    )
fi


# Mount opzionali già usati dal progetto.
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


# Mantieni lo stesso ROS_DOMAIN_ID degli altri container, se impostato.
if [[ -n "${ROS_DOMAIN_ID:-}" ]]; then
    docker_args+=(
        -e "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    )
fi

# Analogamente per la RMW implementation.
if [[ -n "${RMW_IMPLEMENTATION:-}" ]]; then
    docker_args+=(
        -e "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
    )
fi


echo "Creazione del container $CONTAINER_NAME..."

docker "${docker_args[@]}" \
    "$ROS_IMAGE" \
    sleep infinity


# =============================================================================
# VERIFICA CONTAINER
# =============================================================================

for _ in $(seq 1 30); do

    if [[ "$(docker inspect --format '{{.State.Running}}' "$CONTAINER_NAME")" == "true" ]]; then
        break
    fi

    sleep 1
done


if [[ "$(docker inspect --format '{{.State.Running}}' "$CONTAINER_NAME")" != "true" ]]; then
    docker logs "$CONTAINER_NAME" >&2 || true
    fail "Il container $CONTAINER_NAME non è rimasto in esecuzione."
fi


echo "Container avviato."
echo


# =============================================================================
# PREFLIGHT 1
# Runtime, CUDA e import Python
# =============================================================================

echo "============================================================"
echo " PREFLIGHT 1/6 - Python / ROS / CUDA"
echo "============================================================"

docker exec "$CONTAINER_NAME" bash -lc "
    set -Eeuo pipefail

    [[ -f '$RUNTIME_SETUP' ]] || {
        echo 'Runtime setup mancante: $RUNTIME_SETUP' >&2
        exit 1
    }

    source '$RUNTIME_SETUP'

    echo \"Python: \$(command -v python)\"
    python --version

    python - <<'PY'
import sys

assert sys.version_info[:2] == (3, 12), sys.version

import rclpy
import cv_bridge
import message_filters
import tf2_ros

from sensor_msgs.msg import Image, JointState
from control_msgs.action import GripperCommand

import torch
import transformers
import numpy
import scipy
import hydra
import omegaconf
import einops
import PIL
import cv2
import bitsandbytes

print(f'PyTorch:       {torch.__version__}')
print(f'Torch CUDA:    {torch.version.cuda}')
print(f'Transformers:  {transformers.__version__}')
print(f'NumPy:         {numpy.__version__}')
print(f'SciPy:         {scipy.__version__}')

if not torch.cuda.is_available():
    raise RuntimeError('torch.cuda.is_available() == False')

device = torch.device('cuda')
x = torch.ones(4, device=device)
assert float(x.sum().item()) == 4.0

print(f'GPU:           {torch.cuda.get_device_name(0)}')
print(f'Capability:    {torch.cuda.get_device_capability(0)}')
print('CUDA tensor:   OK')
print('ROS imports:   OK')
print('ML imports:    OK')
PY
"

echo


# =============================================================================
# PREFLIGHT 2
# Open-Pi-Zero imports
# =============================================================================

echo "============================================================"
echo " PREFLIGHT 2/6 - Open-Pi-Zero"
echo "============================================================"

docker exec "$CONTAINER_NAME" bash -lc "
    set -Eeuo pipefail
    source '$RUNTIME_SETUP'

    [[ -d \"\$OPEN_PI_ZERO\" ]] || {
        echo \"open-pi-zero mancante: \$OPEN_PI_ZERO\" >&2
        exit 1
    }

    python - <<'PY'
from src.model.vla.interleaved_pizero import (
    InterleavedPiZeroInference,
)
from src.model.vla.interleaved_processing import (
    InterleavedVLAProcessor,
)

print('InterleavedPiZeroInference: OK')
print('InterleavedVLAProcessor:    OK')
PY
"

echo


# =============================================================================
# PREFLIGHT 3
# Config, statistics, instruction images, checkpoint, PaliGemma
# =============================================================================

echo "============================================================"
echo " PREFLIGHT 3/6 - Asset runtime"
echo "============================================================"

docker exec "$CONTAINER_NAME" bash -lc "
    set -Eeuo pipefail
    source '$RUNTIME_SETUP'

    python - <<'PY'
import json
import os
from pathlib import Path

from omegaconf import OmegaConf


config_path = Path(
    os.environ['INTERLEAVE_PI0_CONTROLLER_CONFIG']
).resolve()

checkpoint_path = Path(
    os.environ['INTERLEAVE_PI0_CHECKPOINT']
).resolve()

paligemma_path = Path(
    os.environ['INTERLEAVE_PI0_PALIGEMMA']
).resolve()


if not config_path.is_file():
    raise FileNotFoundError(config_path)

if not checkpoint_path.is_file():
    raise FileNotFoundError(checkpoint_path)

if not paligemma_path.is_dir():
    raise FileNotFoundError(paligemma_path)


cfg = OmegaConf.load(config_path)
OmegaConf.resolve(cfg)

config_dir = config_path.parent


# -------------------------------------------------------------------------
# Statistics
# -------------------------------------------------------------------------

stats_path = Path(str(cfg.dataset_statistics_path))

if not stats_path.is_absolute():
    stats_path = config_dir / stats_path

stats_path = stats_path.resolve()

if not stats_path.is_file():
    raise FileNotFoundError(
        f'Dataset statistics missing: {stats_path}'
    )

with stats_path.open('r', encoding='utf-8') as f:
    stats = json.load(f)


if stats.get('num_trajectories') != 456:
    raise RuntimeError(
        'Unexpected num_trajectories: '
        f\"{stats.get('num_trajectories')}\"
    )

if stats.get('num_transitions') != 23997:
    raise RuntimeError(
        'Unexpected num_transitions: '
        f\"{stats.get('num_transitions')}\"
    )

for group in ('action', 'proprio'):
    for key in ('p01', 'p99'):
        values = stats[group][key]
        if len(values) != 7:
            raise RuntimeError(
                f'{group}.{key} must contain 7 values'
            )


# -------------------------------------------------------------------------
# Tasks + instruction images
# -------------------------------------------------------------------------

if len(cfg.tasks) != 16:
    raise RuntimeError(
        f'Expected 16 tasks, found {len(cfg.tasks)}'
    )

expected_ids = {
    f'{index:02d}'
    for index in range(16)
}

actual_ids = {
    str(task_id)
    for task_id in cfg.tasks.keys()
}

if actual_ids != expected_ids:
    raise RuntimeError(
        f'Unexpected task IDs: {sorted(actual_ids)}'
    )


for task_id in sorted(expected_ids):
    task = cfg.tasks[task_id]

    prompt = str(task.prompt)

    if prompt.count('<image>') != 1:
        raise RuntimeError(
            f'Task {task_id}: prompt must contain exactly one <image>'
        )

    image_path = Path(str(task.instruction_image))

    if not image_path.is_absolute():
        image_path = config_dir / image_path

    image_path = image_path.resolve()

    if not image_path.is_file():
        raise FileNotFoundError(
            f'Task {task_id}: missing instruction image {image_path}'
        )


print(f'Config:        {config_path}')
print(f'Checkpoint:    {checkpoint_path}')
print(f'PaliGemma:     {paligemma_path}')
print(f'Statistics:    {stats_path}')
print('Dataset stats: 456 trajectories / 23997 transitions')
print('Tasks:         16/16 OK')
print('Instruction images: OK')
PY
"

echo


# =============================================================================
# PREFLIGHT 4
# Colcon build
# =============================================================================

echo "============================================================"
echo " PREFLIGHT 4/6 - ROS workspace"
echo "============================================================"

docker exec "$CONTAINER_NAME" bash -lc "
    set -Eeuo pipefail
    source '$RUNTIME_SETUP'

    cd /home/ros2_ws

    echo 'Package ROS trovati:'
    colcon list

    if ! colcon list | awk '{print \$1}' | grep -qx 'moveit_controller_srvs'; then
        echo 'Package moveit_controller_srvs non trovato.' >&2
        exit 1
    fi

    if ! colcon list | awk '{print \$1}' | grep -qx 'ai_controller'; then
        echo 'Package ai_controller non trovato.' >&2
        exit 1
    fi

    colcon build \
        --packages-select \
        moveit_controller_srvs \
        ai_controller

    source /home/ros2_ws/install/setup.bash


    # ----------------------------------------------------------------------
    # ros2 run deve utilizzare il Python del nuovo runtime.
    #
    # Come nel launcher Mimic, controlliamo esplicitamente la shebang
    # dell'entrypoint generato da colcon.
    # ----------------------------------------------------------------------

    ai_entrypoint=/home/ros2_ws/install/ai_controller/lib/ai_controller/ai_controller_node
    runtime_python=/opt/interleave-pizero-ros/bin/python

    [[ -f \"\$ai_entrypoint\" ]] || {
        echo \"Entry point mancante: \$ai_entrypoint\" >&2
        exit 1
    }

    [[ -x \"\$runtime_python\" ]] || {
        echo \"Runtime Python mancante: \$runtime_python\" >&2
        exit 1
    }

    expected_shebang=\"#!\$runtime_python\"
    actual_shebang=\"\$(head -n 1 \"\$ai_entrypoint\")\"

    if [[ \"\$actual_shebang\" != \"\$expected_shebang\" ]]; then
        echo \"Correzione shebang:\"
        echo \"  prima: \$actual_shebang\"
        echo \"  dopo:  \$expected_shebang\"

        sed -i \"1c\\\\\$expected_shebang\" \"\$ai_entrypoint\"
    fi

    actual_shebang=\"\$(head -n 1 \"\$ai_entrypoint\")\"

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

from ai_controller.models.interleave_pi0_controller.interleave_pi0 import (
    InterleavePi0Policy,
    load_interleave_pi0_config,
)

from ai_controller.models.interleave_pi0_controller.interleave_pi0_controller import (
    InterleavePi0Controller,
)

print('rclpy:                   OK')
print('moveit_controller_srvs:  OK')
print('InterleavePi0Policy:     OK')
print('InterleavePi0Controller: OK')
PY

    echo \"ai_controller entrypoint: \$(head -n 1 \"\$ai_entrypoint\")\"
"

echo


# =============================================================================
# PREFLIGHT 5
# Caricamento effettivo del modello + dummy inference
# =============================================================================
#
# Questo è il test forte:
#
#   - legge YAML
#   - carica checkpoint strict
#   - carica tokenizer / PaliGemma
#   - costruisce Interleave-Pi0
#   - carica stats
#   - carica instruction crop task 00
#   - preprocessa una RGB dummy
#   - costruisce proprio
#   - esegue una vera predict()
#   - denormalizza/postprocessa
#
# Non esiste alcuna connessione a MoveIt in questo test:
# NON può muovere il robot.
# =============================================================================

echo "============================================================"
echo " PREFLIGHT 5/6 - Modello"
echo "============================================================"

if [[ "$FULL_PREFLIGHT" == "1" ]]; then

    echo "Esecuzione dummy inference completa."
    echo "La prima torch.compile può richiedere tempo..."
    echo

    docker exec "$CONTAINER_NAME" bash -lc "
        set -Eeuo pipefail
        source '$RUNTIME_SETUP'
        source /home/ros2_ws/install/setup.bash

        python - <<'PY'
import os

import numpy as np
import torch

from ai_controller.models.interleave_pi0_controller.interleave_pi0_controller import (
    InterleavePi0Controller,
)


config_path = os.environ[
    'INTERLEAVE_PI0_CONTROLLER_CONFIG'
]


controller = InterleavePi0Controller(
    model_config=config_path,
    task_name='pick_place',
)


# load_command() carica anche:
#
#   - dataset statistics
#   - prompt
#   - instruction crop
#
controller.load_command(
    demo_path='',
    task_id='00',
)


# Dimensioni compatibili con la front camera originale del dataset.
dummy_front_rgb = np.zeros(
    (376, 672, 3),
    dtype=np.uint8,
)


# Pose vicina alla posa iniziale reale utilizzata dal nodo:
#
# [x, y, z, qx, qy, qz, qw, gripper_closed]
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


out = controller.inference(
    input_data=[
        [dummy_front_rgb],
        dummy_robot_state,
    ],
    t=0,
    save_path=None,
)


out = np.asarray(
    out,
    dtype=np.float32,
)


if out.shape != (1, 8):
    raise RuntimeError(
        f'Unexpected controller output shape: {out.shape}'
    )

if not np.all(np.isfinite(out)):
    raise RuntimeError(
        f'Controller output contains non-finite values: {out}'
    )


quat = out[0, 3:7]
quat_norm = float(np.linalg.norm(quat))

if not np.isfinite(quat_norm) or quat_norm < 0.99 or quat_norm > 1.01:
    raise RuntimeError(
        f'Invalid output quaternion norm: {quat_norm}'
    )


print()
print('============================================================')
print(' FULL INTERLEAVE-Pi0 PREFLIGHT PASSED')
print('============================================================')
print(f'Output shape:      {out.shape}')
print(f'Output action:     {out[0]}')
print(f'Quaternion norm:   {quat_norm:.9f}')
print('============================================================')
print()


del controller

if torch.cuda.is_available():
    torch.cuda.empty_cache()
PY
    "

else

    echo "Dummy inference saltata (--no-full-preflight)."

fi

echo


# =============================================================================
# PREFLIGHT 6
# ROS graph
# =============================================================================
#
# Non blocchiamo il launcher se qualche nodo non è ancora stato avviato:
# stampiamo chiaramente ciò che è visibile.
#
# Questo test è molto utile per verificare:
#
#   --network host
#   ROS_DOMAIN_ID
#   DDS
#   container UR
#   container ZED
#   moveit_controller_node
# =============================================================================

echo "============================================================"
echo " PREFLIGHT 6/6 - ROS graph"
echo "============================================================"

docker exec "$CONTAINER_NAME" bash -lc "
    set -Eeuo pipefail
    source '$RUNTIME_SETUP'
    source /home/ros2_ws/install/setup.bash

    sleep 2

    topics=\"\$(ros2 topic list 2>/dev/null || true)\"
    services=\"\$(ros2 service list 2>/dev/null || true)\"
    actions=\"\$(ros2 action list 2>/dev/null || true)\"

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
echo " INTERLEAVE-Pi0 RUNTIME PRONTO"
echo "============================================================"
echo
echo "Container: $CONTAINER_NAME"
echo
echo "Per avviare AIControllerNode dalla shell:"
echo
echo "ros2 run ai_controller ai_controller_node --ros-args \\"
echo "  -p ai_controller_target:=interleave_pi0_controller \\"
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
    echo ' Interleave-Pi0 + ROS 2 Jazzy'
    echo '============================================================'
    echo \"Python:     \$(command -v python)\"
    echo \"Checkpoint: \$INTERLEAVE_PI0_CHECKPOINT\"
    echo \"PaliGemma:  \$INTERLEAVE_PI0_PALIGEMMA\"
    echo
    echo 'Comando di inferenza:'
    echo
    echo 'ros2 run ai_controller ai_controller_node --ros-args \\\\'
    echo '  -p ai_controller_target:=interleave_pi0_controller \\\\'
    echo '  -p model_config_path:=$CONTROLLER_CONFIG_CONTAINER'
    echo
    echo '============================================================'
    echo

    exec bash --noprofile --norc -i
"