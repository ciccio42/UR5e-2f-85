#!/usr/bin/env bash

# Avvia il container ROS per Mimic Video riutilizzando il virtual environment
# persistente. Se il container esiste gia, apre una nuova shell configurata.
#
# Variabili host configurabili:
#   UR5e_2f_85_PATH  root della repository (rilevata automaticamente di default)
#   ROBOT_IP         indirizzo del robot (default: 172.16.174.59)
#   CONTAINER_NAME   nome del container (default: ur_robotiq_mimic_video)
#   MIMIC_IMAGE      immagine Docker (default: ur_robotiq_teleoperation:latest)
#   CPUSET_CPUS      CPU assegnate al container (default: 0-19)
#   SHM_SIZE         shared memory Docker (default: 1g)

setup_runtime_environment() {
    local ros_setup="/opt/ros/jazzy/setup.bash"
    local workspace_setup="/home/ros2_ws/install/setup.bash"
    local venv_activate="/opt/mimic-video-runtime/bin/activate"

    for required_file in "$ros_setup" "$venv_activate"; do
        if [[ ! -f "$required_file" ]]; then
            echo "File runtime mancante: $required_file" >&2
            return 1
        fi
    done

    # ROS deve essere inizializzato prima di attivare il virtual environment.
    source "$ros_setup"
    if [[ -f "$workspace_setup" ]]; then
        source "$workspace_setup"
    fi
    source "$venv_activate"

    export MIMIC_VIDEO_WORKSPACE=/workspace/mimic_video_workspace
    export MIMIC_MODEL_DIR="$MIMIC_VIDEO_WORKSPACE/external/mimic-video/model"
    export MIMIC_VIDEO_DATASET_STATISTICS_PATH="$MIMIC_VIDEO_WORKSPACE/checkpoints/dataset_statistics/ur5e_action.json"
    export MIMIC_VIDEO_CONTROLLER_CONFIG=/home/ros2_ws/src/ai_controller/ai_controller/models/mimic_video_controller/mimic_video_config.yaml

    # Indica a Cosmos dove trovare il tokenizer video e gli altri asset runtime.
    export COSMOS_PREDICT2_ARGS="--checkpoints $MIMIC_VIDEO_WORKSPACE/checkpoints"

    export CUDA_HOME=/usr/local/cuda-12.8
    export CUDA_PATH="$CUDA_HOME"
    export CUDNN_PY_ROOT=/usr/local/lib/python3.12/dist-packages/nvidia/cudnn
    export CUDNN_PATH="$CUDNN_PY_ROOT"
    export CUDNN_HOME="$CUDNN_PY_ROOT"
    export NCCL_ROOT=/usr/local/lib/python3.12/dist-packages/nvidia/nccl
    export NCCL_PATH="$NCCL_ROOT"
    export NCCL_HOME="$NCCL_ROOT"
    export NVTE_WITH_NCCL_EP=0

    # Inserisce una directory in testa a una variabile PATH-like senza duplicarla.
    prepend_env_path() {
        local variable_name="$1"
        local directory="$2"
        local current_value="${!variable_name:-}"

        if [[ ":$current_value:" != *":$directory:"* ]]; then
            if [[ -n "$current_value" ]]; then
                printf -v "$variable_name" '%s:%s' "$directory" "$current_value"
            else
                printf -v "$variable_name" '%s' "$directory"
            fi
        fi
        export "$variable_name"
    }

    prepend_env_path PYTHONPATH "$MIMIC_MODEL_DIR"
    prepend_env_path PATH "$CUDA_HOME/bin"

    prepend_env_path CPATH "$CUDNN_PY_ROOT/include"
    prepend_env_path CPATH "$NCCL_ROOT/include"
    prepend_env_path CPLUS_INCLUDE_PATH "$CUDNN_PY_ROOT/include"
    prepend_env_path CPLUS_INCLUDE_PATH "$NCCL_ROOT/include"
    prepend_env_path CMAKE_INCLUDE_PATH "$CUDNN_PY_ROOT/include"
    prepend_env_path CMAKE_INCLUDE_PATH "$NCCL_ROOT/include"

    local private_cudnn_lib=/opt/mimic-video-runtime/cudnn/lib
    local private_nccl_lib=/opt/mimic-video-runtime/nccl/lib
    for library_path in \
        "$CUDA_HOME/lib64" \
        "$CUDNN_PY_ROOT/lib" \
        "$NCCL_ROOT/lib" \
        "$private_cudnn_lib" \
        "$private_nccl_lib"; do
        prepend_env_path LIBRARY_PATH "$library_path"
        prepend_env_path CMAKE_LIBRARY_PATH "$library_path"
        prepend_env_path LD_LIBRARY_PATH "$library_path"
    done

    if [[ ! -f "$MIMIC_MODEL_DIR/pyproject.toml" ]]; then
        echo "Sorgenti Mimic Video mancanti: $MIMIC_MODEL_DIR" >&2
        return 1
    fi
    if [[ ! -f "$MIMIC_VIDEO_DATASET_STATISTICS_PATH" ]]; then
        echo "Statistiche UR5e mancanti: $MIMIC_VIDEO_DATASET_STATISTICS_PATH" >&2
        return 1
    fi
    local video_tokenizer="$MIMIC_VIDEO_WORKSPACE/checkpoints/video_backbone/tokenizer/tokenizer.pth"
    if [[ ! -f "$video_tokenizer" ]]; then
        echo "Tokenizer video Cosmos mancante: $video_tokenizer" >&2
        return 1
    fi

    echo "Runtime Mimic Video attivo"
    echo "  Python:      $(command -v python)"
    echo "  Model dir:   $MIMIC_MODEL_DIR"
    echo "  Checkpoints: $MIMIC_VIDEO_WORKSPACE/checkpoints"
    echo "  Statistics:  $MIMIC_VIDEO_DATASET_STATISTICS_PATH"
}

if [[ "${1:-}" == "--env-only" ]]; then
    if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
        echo "Usare: source $0 --env-only" >&2
        exit 2
    fi
    setup_runtime_environment
    return $?
fi

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
REPO_ROOT="${UR5e_2f_85_PATH:-$DEFAULT_REPO_ROOT}"

ROBOT_IP="${ROBOT_IP:-172.16.174.59}"
CONTAINER_NAME="${CONTAINER_NAME:-ur_robotiq_mimic_video}"
MIMIC_IMAGE="${MIMIC_IMAGE:-ur_robotiq_teleoperation:latest}"
CPUSET_CPUS="${CPUSET_CPUS:-0-19}"
SHM_SIZE="${SHM_SIZE:-1g}"

MIMIC_WORKSPACE_HOST="$REPO_ROOT/mimic_video_workspace"
STATS_HOST="$MIMIC_WORKSPACE_HOST/checkpoints/dataset_statistics/ur5e_action.json"
RUNTIME_SCRIPT_CONTAINER=/home/ros2_ws/src/ai_controller/ai_controller/models/mimic_video_controller/run_mimic_video_container.sh

for required_directory in \
    "$REPO_ROOT/ur5e_2f_85" \
    "$REPO_ROOT/dataset_collector" \
    "$REPO_ROOT/ai_controller" \
    "$REPO_ROOT/moveit_controller" \
    "$MIMIC_WORKSPACE_HOST"; do
    if [[ ! -d "$required_directory" ]]; then
        echo "Directory host mancante: $required_directory" >&2
        exit 1
    fi
done

if [[ ! -f "$STATS_HOST" ]]; then
    echo "Statistiche UR5e mancanti: $STATS_HOST" >&2
    echo "Copia in questa posizione il JSON prodotto dal training dell'Action Head." >&2
    exit 1
fi

if ! docker image inspect "$MIMIC_IMAGE" >/dev/null 2>&1; then
    echo "Immagine Docker mancante: $MIMIC_IMAGE" >&2
    exit 1
fi

if ! docker volume inspect mimic_video_runtime >/dev/null 2>&1; then
    echo "Volume Docker mimic_video_runtime mancante." >&2
    echo "Il volume deve contenere il virtual environment gia configurato." >&2
    exit 1
fi

if ! docker inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
    docker_args=(
        run -d
        --gpus all
        --privileged
        --cap-add=SYS_NICE
        --cpuset-cpus="$CPUSET_CPUS"
        --network host
        --ipc=host
        --pid=host
        --ulimit memlock=-1:-1
        --ulimit rtprio=99
        --shm-size="$SHM_SIZE"
        --security-opt seccomp=unconfined
        -e "DISPLAY=${DISPLAY:-:0}"
        -e "ROBOT_IP=$ROBOT_IP"
        -e NVIDIA_VISIBLE_DEVICES=all
        -e NVIDIA_DRIVER_CAPABILITIES=all
        -e XDG_RUNTIME_DIR=/tmp/runtime-root
        -v mimic_video_runtime:/opt/mimic-video-runtime
        -v mimic_video_build_cache:/root/.cache
        -v "$REPO_ROOT/ur5e_2f_85:/home/ros2_ws/src/ur5e_2f_85"
        -v "$REPO_ROOT/dataset_collector:/home/ros2_ws/src/dataset_collector"
        -v "$REPO_ROOT/ai_controller:/home/ros2_ws/src/ai_controller"
        -v "$REPO_ROOT/moveit_controller:/home/ros2_ws/src/moveit_controller"
        -v "$MIMIC_WORKSPACE_HOST:/workspace/mimic_video_workspace"
        --name "$CONTAINER_NAME"
    )

    if [[ -d /tmp/.X11-unix ]]; then
        docker_args+=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)
    fi
    if [[ -d /dev/input ]]; then
        docker_args+=(-v /dev/input:/dev/input)
    fi
    if [[ -d "$REPO_ROOT/zed_camera/zed_camera_calibration" ]]; then
        docker_args+=(-v "$REPO_ROOT/zed_camera/zed_camera_calibration:/home/ros2_ws/src/zed_camera/zed_camera_calibration:ro")
    fi
    if [[ -d "$REPO_ROOT/traj_tmp" ]]; then
        docker_args+=(-v "$REPO_ROOT/traj_tmp:/traj_tmp")
    fi
    if [[ -d /home/asus-mivia/Desktop/saved_trajectories ]]; then
        docker_args+=(-v /home/asus-mivia/Desktop/saved_trajectories:/home/saved_trajectories)
    fi
    if [[ -d /home/asus-mivia/Desktop/dataset ]]; then
        docker_args+=(-v /home/asus-mivia/Desktop/dataset:/dataset)
    fi

    if command -v xhost >/dev/null 2>&1; then
        xhost +local:docker >/dev/null
    fi

    echo "Creazione del container $CONTAINER_NAME..."
    docker "${docker_args[@]}" "$MIMIC_IMAGE" sleep infinity
else
    if [[ "$(docker inspect --format '{{.State.Running}}' "$CONTAINER_NAME")" != "true" ]]; then
        echo "Avvio del container esistente $CONTAINER_NAME..."
        docker start "$CONTAINER_NAME" >/dev/null
    else
        echo "Container $CONTAINER_NAME gia in esecuzione."
    fi
fi

# L'entrypoint dell'immagine compila i package ROS prima di eseguire sleep.
for _ in $(seq 1 180); do
    if [[ "$(docker inspect --format '{{.State.Running}}' "$CONTAINER_NAME")" != "true" ]]; then
        echo "Il container si e arrestato durante l'avvio:" >&2
        docker logs --tail 100 "$CONTAINER_NAME" >&2
        exit 1
    fi

    init_process="$(docker exec "$CONTAINER_NAME" sh -c 'cat /proc/1/comm' 2>/dev/null || true)"
    if [[ "$init_process" == "sleep" ]]; then
        break
    fi
    sleep 2
done

if [[ "${init_process:-}" != "sleep" ]]; then
    echo "Timeout durante il build iniziale dei package ROS." >&2
    docker logs --tail 100 "$CONTAINER_NAME" >&2
    exit 1
fi

echo "Installazione delle configurazioni UR5e per l'inferenza..."
docker exec "$CONTAINER_NAME" bash -lc '
    set -e
    workspace=/workspace/mimic_video_workspace
    source_dir="$workspace/configs/action_head/dataloading"
    target_dir="$workspace/external/mimic-video/model/cosmos_predict2/configs/dataloading"

    install -D -m 0644 "$source_dir/ur5e_videmb.yaml" "$target_dir/ur5e_videmb.yaml"
    install -D -m 0644 "$source_dir/dataset/ur5e.yaml" "$target_dir/dataset/ur5e.yaml"
    install -D -m 0644 \
        "$source_dir/dataset/transform/ur5e_to_ur5e_videmb.yaml" \
        "$target_dir/dataset/transform/ur5e_to_ur5e_videmb.yaml"
    install -D -m 0644 \
        "$source_dir/policy_io/ur5e_videmb.yaml" \
        "$target_dir/policy_io/ur5e_videmb.yaml"
'

echo "Applicazione delle registrazioni UR5e e delle patch d'inferenza..."
docker exec "$CONTAINER_NAME" bash -lc '
    set -e
    workspace=/workspace/mimic_video_workspace
    model_repo=/workspace/mimic_video_workspace/external/mimic-video
    patches=(
        "$workspace/patches/action_head/004_register_ur5e_action_pipe.patch"
        "$workspace/patches/action_head/005_configure_ur5e_action_experiments.patch"
        "$workspace/patches/action_head/007_register_ur5e_cosmos_checkpoint.patch"
        "$workspace/patches/inference/001_lazy_apex_optimizer_import.patch"
    )

    for patch_file in "${patches[@]}"; do
        if [[ ! -f "$patch_file" ]]; then
            echo "Patch richiesta mancante: $patch_file" >&2
            exit 1
        fi

        git_cmd=(git -c safe.directory="$model_repo" -C "$model_repo")
        if "${git_cmd[@]}" apply --reverse --check "$patch_file" >/dev/null 2>&1; then
            echo "Gia applicata: $(basename "$patch_file")"
        elif "${git_cmd[@]}" apply --check "$patch_file"; then
            "${git_cmd[@]}" apply "$patch_file"
            echo "Applicata: $(basename "$patch_file")"
        else
            echo "Impossibile applicare la patch: $patch_file" >&2
            exit 1
        fi
    done
'

echo "Verifica del package ROS ai_controller..."
docker exec "$CONTAINER_NAME" bash -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    cd /home/ros2_ws
    colcon build --packages-select ai_controller
'

echo "Apertura della shell Mimic Video nel container $CONTAINER_NAME..."
exec docker exec -it "$CONTAINER_NAME" bash -lc \
    "source '$RUNTIME_SCRIPT_CONTAINER' --env-only && exec bash -i"
