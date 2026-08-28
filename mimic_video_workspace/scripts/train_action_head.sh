#!/usr/bin/env bash

# Configurazione predefinita
# - Senza argomenti esegue lo smoke test; usare "train" per il training completo.
# - Modello: action head World2Action inizializzata da zero e Cosmos UR5e congelato.
# - Checkpoint Cosmos: percorso relativo configurabile con COSMOS_CKPT_REL.
# - Dati: safetensors UR5e con video embedding, testo, stato robotico e action delta.
# - Action horizon: 15 step a 10 Hz; observation horizon: 1 step.
# - Batch size fisico: 1; gradient accumulation: 12; batch effettivo: 12.
#   BATCH_SIZE=12 usa invece un batch fisico bilanciato e accumulation 1.
# - Action head: 10 canali (posizione 3D, rotazione 6D, gripper), max horizon 16.
# - DataLoader: 2 worker, prefetch factor 1, pin memory e worker persistenti attivi.
# - Training completo: 10000 optimizer step predefiniti, configurabili con MAX_ITER.
# - RUN_TAG aggiunge un identificatore al job; FRESH_START=1 impedisce il resume
#   se esiste gia un latest_checkpoint.txt per lo stesso job.
#   Gli optimizer step per epoca corrispondono al numero di chunk bilanciati diviso 12.
# - Validation: una iniziale e una al termine di ogni epoca completa.
# - Checkpoint: al termine di ogni epoca e alla conclusione del training.
#   La retention conserva gli ultimi 2 checkpoint completi e il best validato.
# - W&B online con credenziali esterne; master weights FP32 ed EMA disattivati;
#   precisione bfloat16.
# - Smoke test: 1 optimizer step su 12 microbatch, con 2 batch di validation iniziale
#   e altri 2 dopo lo step.

set -Eeuo pipefail

SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]}")"
REPO_ROOT="$(realpath "$(dirname "$SCRIPT_PATH")/../..")"
WORKSPACE="$REPO_ROOT/mimic_video_workspace"

IMAGE="${IMAGE:-mimic-video-train:gb10-cu129}"
BATCH_SIZE="${BATCH_SIZE:-1}"
if [[ -z "${GRAD_ACCUM_ITER:-}" ]]; then
    if [[ "$BATCH_SIZE" == "1" ]]; then
        GRAD_ACCUM_ITER=12
    else
        GRAD_ACCUM_ITER=1
    fi
fi
MAX_ITER="${MAX_ITER:-10000}"
RUN_TAG="${RUN_TAG:-}"
FRESH_START="${FRESH_START:-0}"
COSMOS_CKPT_REL="${COSMOS_CKPT_REL:-checkpoints/posttraining/video2world/v2w_ur5e_pick_place_videmb_lora_rank64_lr1.778e-04_bsz1_alpha32_train/checkpoints/model/iter_000001520_fused.pt}"
WANDB_ENV_FILE="${WANDB_ENV_FILE:-$HOME/.config/mimic-video/wandb.env}"

apply_patches() {
    local submodule="$WORKSPACE/external/mimic-video"
    local patch
    local index
    local -a cosmos_patches
    local -a action_patches

    git config --global --add safe.directory "$submodule"

    mapfile -t cosmos_patches < <(find "$WORKSPACE/patches/cosmos" -maxdepth 1 -type f -name '*.patch' | sort)
    mapfile -t action_patches < <(find "$WORKSPACE/patches/action_head" -maxdepth 1 -type f -name '*.patch' | sort)

    # La prima patch di ogni gruppo modifica un file esclusivo e identifica
    # senza ambiguita quale configurazione e attualmente applicata.
    if [[ ${#action_patches[@]} -gt 0 ]] && \
        git -C "$submodule" apply --reverse --check "${action_patches[0]}" 2>/dev/null; then
        for ((index=${#action_patches[@]} - 1; index >= 0; index--)); do
            patch="${action_patches[$index]}"
            if git -C "$submodule" apply --reverse --check "$patch" 2>/dev/null; then
                git -C "$submodule" apply --reverse "$patch"
                echo "Rimossa patch Action Head: $(basename "$patch")"
            fi
        done
    fi

    # Prima di configurare l'Action Head rimuovo le patch del training Cosmos,
    # in ordine inverso per rispettare le dipendenze tra patch successive.
    if [[ ${#cosmos_patches[@]} -gt 0 ]] && \
        git -C "$submodule" apply --reverse --check "${cosmos_patches[0]}" 2>/dev/null; then
        for ((index=${#cosmos_patches[@]} - 1; index >= 0; index--)); do
            patch="${cosmos_patches[$index]}"
            if git -C "$submodule" apply --reverse --check "$patch" 2>/dev/null; then
                git -C "$submodule" apply --reverse "$patch"
                echo "Rimossa patch Cosmos: $(basename "$patch")"
            fi
        done
    fi

    for patch in "${action_patches[@]}"; do
        if ! git -C "$submodule" apply --check "$patch" 2>/dev/null; then
            echo "Impossibile applicare la patch: $patch" >&2
            exit 1
        fi

        git -C "$submodule" apply "$patch"
        echo "Applicata: $(basename "$patch")"
    done
}

install_configs() {
    local source="$WORKSPACE/configs/action_head/dataloading"
    local target="$WORKSPACE/external/mimic-video/model/cosmos_predict2/configs/dataloading"

    install -D -m 0644 "$source/ur5e_videmb.yaml" "$target/ur5e_videmb.yaml"
    install -D -m 0644 "$source/dataset/ur5e.yaml" "$target/dataset/ur5e.yaml"
    install -D -m 0644 \
        "$source/dataset/transform/ur5e_to_ur5e_videmb.yaml" \
        "$target/dataset/transform/ur5e_to_ur5e_videmb.yaml"
    install -D -m 0644 \
        "$source/policy_io/ur5e_videmb.yaml" \
        "$target/policy_io/ur5e_videmb.yaml"
}

load_wandb_credentials() {
    # Il file resta fuori dalla repository ed e montato in sola lettura nel container.
    if [[ -f "$WANDB_ENV_FILE" ]]; then
        set -a
        # shellcheck disable=SC1090
        source "$WANDB_ENV_FILE"
        set +a
    fi

    if [[ -z "${WANDB_API_KEY:-}" ]]; then
        echo "WANDB_API_KEY non disponibile." >&2
        echo "Configura le credenziali in $WANDB_ENV_FILE." >&2
        exit 1
    fi
}

run_training() {
    local mode="$1"
    local max_iter="$MAX_ITER"
    local suffix="train"
    local cosmos_checkpoint
    local run_tag_suffix=""
    local -a smoke_overrides=()

    if [[ "$mode" == "smoke" ]]; then
        max_iter=1
        suffix="smoke"
        smoke_overrides+=(
            "trainer.max_val_iter=2"
            "trainer.validation_iter=1"
            "checkpoint.save_iter=1"
        )
    fi

    load_wandb_credentials
    apply_patches
    install_configs

    if [[ "$COSMOS_CKPT_REL" = /* ]]; then
        cosmos_checkpoint="$COSMOS_CKPT_REL"
    else
        cosmos_checkpoint="$WORKSPACE/$COSMOS_CKPT_REL"
    fi
    if [[ ! -f "$cosmos_checkpoint" ]]; then
        echo "Checkpoint Cosmos non trovato: $cosmos_checkpoint" >&2
        echo "Imposta COSMOS_CKPT_REL con il percorso del checkpoint fused." >&2
        exit 1
    fi

    export UR5E_COSMOS_CKPT="$cosmos_checkpoint"
    export CKPT_DIR="$WORKSPACE/checkpoints"
    export COSMOS_PREDICT2_ARGS="--checkpoints $CKPT_DIR"
    export IMAGINAIRE_OUTPUT_ROOT="$WORKSPACE/checkpoints"
    export WANDB_DIR="$WORKSPACE/checkpoints/wandb"
    export TRITON_PTXAS_PATH=/usr/local/cuda/bin/ptxas

    local experiment="w2a_ur5e_videmb_v2w_ur5e_finetuned_lr1.000e-04_layer20_bsz${BATCH_SIZE}"
    if [[ -n "$RUN_TAG" ]]; then
        run_tag_suffix="_${RUN_TAG}"
    fi
    local job_name="${experiment}_accum${GRAD_ACCUM_ITER}${run_tag_suffix}_${suffix}"
    local latest_checkpoint="$WORKSPACE/checkpoints/posttraining/world2action/$job_name/checkpoints/latest_checkpoint.txt"
    local log_dir="$WORKSPACE/outputs/training_logs"

    if [[ "$FRESH_START" == "1" && -f "$latest_checkpoint" ]]; then
        echo "Fresh start richiesto, ma il job possiede gia un checkpoint:" >&2
        echo "$latest_checkpoint" >&2
        echo "Usa un RUN_TAG nuovo; i checkpoint esistenti non verranno rimossi." >&2
        exit 1
    fi

    echo "Dataset: /workspace/UR5e-2f-85/mimic_video_workspace/processed_data/ur5e_pick_place_action_no_extension"
    echo "Job: $job_name"
    echo "Fresh start: $FRESH_START"

    mkdir -p "$WANDB_DIR" "$log_dir"
    cd "$WORKSPACE/external/mimic-video/model"

    /opt/mimic-video-venv/bin/torchrun \
        --standalone \
        --nproc_per_node=1 \
        -m scripts.train \
        --config=cosmos_predict2/configs/config.py \
        -- \
        experiment="$experiment" \
        job.name="$job_name" \
        trainer.max_iter="$max_iter" \
        trainer.grad_accum_iter="$GRAD_ACCUM_ITER" \
        "${smoke_overrides[@]}" \
        2>&1 | tee "$log_dir/${job_name}_$(date +%Y%m%d_%H%M%S).log"
}

run_container() {
    local mode="$1"
    local container_wandb_env="/run/secrets/mimic-video-wandb.env"
    local host_wandb_env

    if [[ ! -f "$WANDB_ENV_FILE" ]]; then
        echo "File W&B non trovato: $WANDB_ENV_FILE" >&2
        exit 1
    fi
    host_wandb_env="$(realpath "$WANDB_ENV_FILE")"

    docker run --rm \
        --gpus all \
        --ipc=host \
        --name "$CONTAINER_NAME" \
        -e BATCH_SIZE="$BATCH_SIZE" \
        -e GRAD_ACCUM_ITER="$GRAD_ACCUM_ITER" \
        -e MAX_ITER="$MAX_ITER" \
        -e RUN_TAG="$RUN_TAG" \
        -e FRESH_START="$FRESH_START" \
        -e COSMOS_CKPT_REL="$COSMOS_CKPT_REL" \
        -e WANDB_ENV_FILE="$container_wandb_env" \
        -v "$REPO_ROOT":/workspace/UR5e-2f-85 \
        -v "$host_wandb_env":"$container_wandb_env":ro \
        -w /workspace/UR5e-2f-85 \
        "$IMAGE" \
        bash /workspace/UR5e-2f-85/mimic_video_workspace/scripts/train_action_head.sh --inside "$mode"
}

start_tmux() {
    local mode="$1"
    local command
    local run_tag_suffix=""

    if [[ -n "$RUN_TAG" ]]; then
        run_tag_suffix="-${RUN_TAG}"
    fi

    SESSION_NAME="${SESSION_NAME:-action-head${run_tag_suffix}-bsz${BATCH_SIZE}-${mode}}"
    CONTAINER_NAME="${CONTAINER_NAME:-mimic-video-action${run_tag_suffix}-bsz${BATCH_SIZE}-${mode}}"

    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        echo "Sessione tmux gia esistente: $SESSION_NAME" >&2
        exit 1
    fi

    printf -v command \
        'IMAGE=%q BATCH_SIZE=%q GRAD_ACCUM_ITER=%q MAX_ITER=%q RUN_TAG=%q FRESH_START=%q COSMOS_CKPT_REL=%q WANDB_ENV_FILE=%q CONTAINER_NAME=%q bash %q --container %q; status=$?; echo "Training terminato con stato $status"; exec bash' \
        "$IMAGE" "$BATCH_SIZE" "$GRAD_ACCUM_ITER" "$MAX_ITER" "$RUN_TAG" "$FRESH_START" "$COSMOS_CKPT_REL" \
        "$WANDB_ENV_FILE" "$CONTAINER_NAME" "$SCRIPT_PATH" "$mode"

    tmux new-session -d -s "$SESSION_NAME" -n train "$command"
    echo "Sessione avviata: $SESSION_NAME"
    echo "Distacco: Ctrl+B, poi D"
    if [[ -z "${TMUX:-}" ]]; then
        tmux attach -t "$SESSION_NAME"
    else
        echo "Collegamento: tmux attach -t $SESSION_NAME"
    fi
}

if [[ ! "$BATCH_SIZE" =~ ^(1|4|8|12|32|64|128|256)$ ]]; then
    echo "BATCH_SIZE deve essere 1, 4, 8, 12, 32, 64, 128 oppure 256." >&2
    exit 1
fi
if [[ ! "$GRAD_ACCUM_ITER" =~ ^[1-9][0-9]*$ ]]; then
    echo "GRAD_ACCUM_ITER deve essere un intero positivo." >&2
    exit 1
fi
if [[ ! "$MAX_ITER" =~ ^[1-9][0-9]*$ ]]; then
    echo "MAX_ITER deve essere un intero positivo." >&2
    exit 1
fi
if [[ -n "$RUN_TAG" && ! "$RUN_TAG" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
    echo "RUN_TAG deve iniziare con un carattere alfanumerico e contenere solo lettere, numeri, punto, trattino o underscore." >&2
    exit 1
fi
if [[ ! "$FRESH_START" =~ ^[01]$ ]]; then
    echo "FRESH_START deve valere 0 oppure 1." >&2
    exit 1
fi

case "${1:-smoke}" in
    smoke|train)
        start_tmux "${1:-smoke}"
        ;;
    --container)
        CONTAINER_NAME="${CONTAINER_NAME:?CONTAINER_NAME non impostato}"
        run_container "${2:?specificare smoke o train}"
        ;;
    --inside)
        run_training "${2:?specificare smoke o train}"
        ;;
    *)
        echo "Uso: bash $SCRIPT_PATH [smoke|train]" >&2
        exit 1
        ;;
esac
