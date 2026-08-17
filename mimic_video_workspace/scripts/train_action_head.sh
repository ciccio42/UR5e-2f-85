#!/usr/bin/env bash

# Configurazione predefinita
# - Senza argomenti esegue lo smoke test; usare "train" per il training completo.
# - Modello: action head World2Action inizializzata da zero e Cosmos UR5e congelato.
# - Checkpoint Cosmos: percorso relativo configurabile con COSMOS_CKPT_REL.
# - Dati: safetensors UR5e con video embedding, testo, stato robotico e action delta.
# - Action horizon: 15 step a 10 Hz; observation horizon: 1 step.
# - Batch size fisico: 1; gradient accumulation: 1. Entrambi sono configurabili.
# - Action head: 10 canali (posizione 3D, rotazione 6D, gripper), max horizon 16.
# - DataLoader: 2 worker, prefetch factor 1, pin memory e worker persistenti attivi.
# - Training completo: 500000 optimizer step predefiniti, configurabili con MAX_ITER.
#   Con batch size 1 corrispondono indicativamente a 176-180 epoche sul dataset attuale.
# - Validation: una iniziale e una al termine di ogni epoca completa.
# - Checkpoint: al termine di ogni epoca e alla conclusione del training.
#   La retention conserva soltanto gli ultimi 2 checkpoint completi.
# - W&B offline; master weights FP32 ed EMA disattivati; precisione bfloat16.
# - Smoke test: 1 optimizer step, 2 batch di validation iniziale e altri 2 dopo lo step.

set -Eeuo pipefail

SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]}")"
REPO_ROOT="$(realpath "$(dirname "$SCRIPT_PATH")/../..")"
WORKSPACE="$REPO_ROOT/mimic_video_workspace"

IMAGE="${IMAGE:-mimic-video-train:gb10-cu129}"
BATCH_SIZE="${BATCH_SIZE:-1}"
GRAD_ACCUM_ITER="${GRAD_ACCUM_ITER:-1}"
MAX_ITER="${MAX_ITER:-500000}"
COSMOS_CKPT_REL="${COSMOS_CKPT_REL:-checkpoints/video_backbone/v2w_ur5e_finetuned.pt}"

apply_patches() {
    local submodule="$WORKSPACE/external/mimic-video"
    local patch

    git config --global --add safe.directory "$submodule"
    for patch in "$WORKSPACE"/patches/action_head/*.patch; do
        if git -C "$submodule" apply --check "$patch" 2>/dev/null; then
            git -C "$submodule" apply "$patch"
            echo "Applicata: $(basename "$patch")"
        elif git -C "$submodule" apply --reverse --check "$patch" 2>/dev/null; then
            echo "Gia applicata: $(basename "$patch")"
        else
            echo "Impossibile applicare la patch: $patch" >&2
            exit 1
        fi
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

run_training() {
    local mode="$1"
    local max_iter="$MAX_ITER"
    local suffix="train"
    local cosmos_checkpoint
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
    local job_name="${experiment}_accum${GRAD_ACCUM_ITER}_${suffix}"
    local log_dir="$WORKSPACE/outputs/training_logs"

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

    docker run --rm \
        --gpus all \
        --ipc=host \
        --name "$CONTAINER_NAME" \
        -e BATCH_SIZE="$BATCH_SIZE" \
        -e GRAD_ACCUM_ITER="$GRAD_ACCUM_ITER" \
        -e MAX_ITER="$MAX_ITER" \
        -e COSMOS_CKPT_REL="$COSMOS_CKPT_REL" \
        -v "$REPO_ROOT":/workspace/UR5e-2f-85 \
        -w /workspace/UR5e-2f-85 \
        "$IMAGE" \
        bash /workspace/UR5e-2f-85/mimic_video_workspace/scripts/train_action_head.sh --inside "$mode"
}

start_tmux() {
    local mode="$1"
    local command

    SESSION_NAME="${SESSION_NAME:-action-head-bsz${BATCH_SIZE}-${mode}}"
    CONTAINER_NAME="${CONTAINER_NAME:-mimic-video-action-bsz${BATCH_SIZE}-${mode}}"

    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        echo "Sessione tmux gia esistente: $SESSION_NAME" >&2
        exit 1
    fi

    printf -v command \
        'IMAGE=%q BATCH_SIZE=%q GRAD_ACCUM_ITER=%q MAX_ITER=%q COSMOS_CKPT_REL=%q CONTAINER_NAME=%q bash %q --container %q; status=$?; echo "Training terminato con stato $status"; exec bash' \
        "$IMAGE" "$BATCH_SIZE" "$GRAD_ACCUM_ITER" "$MAX_ITER" "$COSMOS_CKPT_REL" \
        "$CONTAINER_NAME" "$SCRIPT_PATH" "$mode"

    tmux new-session -d -s "$SESSION_NAME" -n train "$command"
    echo "Sessione avviata: $SESSION_NAME"
    echo "Distacco: Ctrl+B, poi D"
    if [[ -z "${TMUX:-}" ]]; then
        tmux attach -t "$SESSION_NAME"
    else
        echo "Collegamento: tmux attach -t $SESSION_NAME"
    fi
}

if [[ ! "$BATCH_SIZE" =~ ^(1|4|8|32|64|128|256)$ ]]; then
    echo "BATCH_SIZE deve essere 1, 4, 8, 32, 64, 128 oppure 256." >&2
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
