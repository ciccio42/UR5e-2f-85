#!/usr/bin/env bash

# Configurazione predefinita
# - Senza argomenti esegue lo smoke test; usare "train" per il training completo.
# - Esperimento: Cosmos Predict2 Video2World 2B, 480p, 10 fps, LoRA sul dataset UR5e.
# - Dati: 456 traiettorie di train e 24 di validation, sola camera frontale,
#   video da 61 frame, 5 frame osservati e video embedding precomputati.
# - Batch size fisico: 1; gradient accumulation: 12; batch effettivo: 12.
# - LoRA rank: 64; LoRA alpha: 32. Il rank puo essere impostato con RANK=64|128|256.
# - DataLoader: 2 worker, prefetch factor 1 per worker, pin memory e worker persistenti attivi.
# - Training completo: 5000 optimizer step. Con 456 esempi e batch effettivo 12,
#   un'epoca vale 38 step e il training copre circa 131,58 epoche.
# - Validation completa: all'avvio e al termine di ogni epoca completata.
# - Checkpoint: al termine della prima epoca, poi a fine epoca se sono trascorsi
#   almeno 30 minuti dal precedente, e sempre alla conclusione del training.
#   La retention conserva gli ultimi 2 checkpoint completi, inclusi i LoRA fused.
# - Logging ogni 100 optimizer step; W&B offline; EMA, guardrail e CPU offload disattivati.
# - Ottimizzatore FusedAdamW, learning rate costante 1.778e-4, master weights FP32 disattivi,
#   precisione bfloat16 e selective activation checkpointing attivo su ogni blocco.
# - Smoke test: 1 optimizer step (12 micro-batch), con 2 batch di validation prima
#   del training e altri 2 dopo l'aggiornamento; usa output separati dal training completo.

set -Eeuo pipefail

SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]}")"
REPO_ROOT="$(realpath "$(dirname "$SCRIPT_PATH")/../..")"
WORKSPACE="$REPO_ROOT/mimic_video_workspace"

IMAGE="${IMAGE:-mimic-video-train:gb10-cu129}"
RANK="${RANK:-64}"
LORA_ALPHA="${LORA_ALPHA:-32}"

apply_patches() {
    local submodule="$WORKSPACE/external/mimic-video"
    local patch

    git config --global --add safe.directory "$submodule"
    for patch in "$WORKSPACE"/patches/cosmos/*.patch; do
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

run_training() {
    local mode="$1"
    local max_iter=5000
    local suffix="train"
    local -a smoke_overrides=()

    if [[ "$mode" == "smoke" ]]; then
        max_iter=1
        suffix="smoke"
        smoke_overrides+=("trainer.max_val_iter=2" "trainer.validation_iter=1")
    fi

    apply_patches

    export CKPT_DIR="$WORKSPACE/checkpoints"
    export COSMOS_PREDICT2_ARGS="--checkpoints $CKPT_DIR"
    export IMAGINAIRE_OUTPUT_ROOT="$WORKSPACE/checkpoints"
    export WANDB_DIR="$WORKSPACE/checkpoints/wandb"
    export TRITON_PTXAS_PATH=/usr/local/cuda/bin/ptxas

    local experiment="v2w_ur5e_pick_place_videmb_lora_rank${RANK}_lr1.778e-04_bsz1"
    local job_name="${experiment}_alpha${LORA_ALPHA}_${suffix}"
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
        model.config.lora_alpha="$LORA_ALPHA" \
        trainer.max_iter="$max_iter" \
        "${smoke_overrides[@]}" \
        2>&1 | tee "$log_dir/${job_name}_$(date +%Y%m%d_%H%M%S).log"
}

run_container() {
    local mode="$1"

    docker run --rm \
        --gpus all \
        --ipc=host \
        --name "$CONTAINER_NAME" \
        -e RANK="$RANK" \
        -e LORA_ALPHA="$LORA_ALPHA" \
        -v "$REPO_ROOT":/workspace/UR5e-2f-85 \
        -w /workspace/UR5e-2f-85 \
        "$IMAGE" \
        bash /workspace/UR5e-2f-85/mimic_video_workspace/scripts/train_cosmos_v2w.sh --inside "$mode"
}

start_tmux() {
    local mode="$1"
    local command

    SESSION_NAME="${SESSION_NAME:-cosmos-r${RANK}-${mode}}"
    CONTAINER_NAME="${CONTAINER_NAME:-mimic-video-cosmos-r${RANK}-${mode}}"

    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        echo "Sessione tmux gia esistente: $SESSION_NAME" >&2
        exit 1
    fi

    printf -v command \
        'IMAGE=%q RANK=%q LORA_ALPHA=%q CONTAINER_NAME=%q bash %q --container %q; status=$?; echo "Training terminato con stato $status"; exec bash' \
        "$IMAGE" "$RANK" "$LORA_ALPHA" "$CONTAINER_NAME" "$SCRIPT_PATH" "$mode"

    tmux new-session -d -s "$SESSION_NAME" -n train "$command"
    echo "Sessione avviata: $SESSION_NAME"
    echo "Distacco: Ctrl+B, poi D"
    if [[ -z "${TMUX:-}" ]]; then
        tmux attach -t "$SESSION_NAME"
    else
        echo "Collegamento: tmux attach -t $SESSION_NAME"
    fi
}

if [[ "$RANK" != "64" && "$RANK" != "128" && "$RANK" != "256" ]]; then
    echo "RANK deve essere 64, 128 oppure 256." >&2
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
