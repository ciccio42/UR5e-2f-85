#!/usr/bin/env bash

# Launcher per fine-tuning Interleave-VLA / Interleave-pi0 su UR5e.
#
# Modalita:
#
#   smoke
#       - 12 optimizer update
#       - torch.compile=False
#       - validation completa al termine
#       - checkpoint agli update 4, 8 e 12
#       - permette di verificare anche retention:
#           step8.pt
#           step12.pt
#           best.pt
#
#   compile-smoke
#       - 2 optimizer update
#       - torch.compile=True
#       - validation disabilitata
#       - serve solo a verificare compilazione e memoria
#
#   train
#       - training completo usando i valori dello YAML
#
# Lo script:
#   host -> tmux -> Docker -> uv run scripts/run.py


set -Eeuo pipefail


# ================================================================
# PATH DEL PROGETTO
# ================================================================

SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]}")"

REPO_ROOT="$(realpath "$(dirname "$SCRIPT_PATH")/../..")"

WORKSPACE="$REPO_ROOT/interleave_pi_zero_workspace"

OPEN_PI_ZERO="$WORKSPACE/external/Interleave-VLA/open-pi-zero"


# ================================================================
# CONFIGURAZIONE HOST
# ================================================================

# Nome dell'immagine Docker gia costruita.
IMAGE="${IMAGE:-interleave-pizero:spark}"

# Directory host che contiene:
#
#   paligemma-3b-pt-224/
#
# Esempio:
# TRANSFORMERS_CACHE_HOST=/path/to/transformers
TRANSFORMERS_CACHE="${TRANSFORMERS_CACHE:-/workspace/models}"


# Checkpoint da usare come inizializzazione.
# Deve essere relativo a interleave_pi_zero_workspace.
#
# Esempio:
# checkpoints/.../bridge_checkpoint.pt
INTERLEAVE_CHECKPOINT="${INTERLEAVE_CHECKPOINT:-/workspace/models/interleave-pi0-bridge/step34799.pt}"

# Credenziali W&B esterne alla repository.
WANDB_ENV_FILE="${WANDB_ENV_FILE:-$HOME/.config/interleave-vla/wandb.env}"

RUN_TAG="${RUN_TAG:-}"


# ================================================================
# SMOKE TEST
# ================================================================

SMOKE_UPDATES="${SMOKE_UPDATES:-12}"
SMOKE_SAVE_FREQ="${SMOKE_SAVE_FREQ:-4}"

COMPILE_SMOKE_UPDATES="${COMPILE_SMOKE_UPDATES:-2}"


# ================================================================
# PATH INTERNI AL CONTAINER
# ================================================================

CONTAINER_WORKSPACE="/workspace"

CONTAINER_OPEN_PI_ZERO="$CONTAINER_WORKSPACE/external/Interleave-VLA/open-pi-zero"

CONTAINER_VLA_DATA_DIR="$CONTAINER_WORKSPACE/processed_data/ur5e_interleave/0.1.0"

CONTAINER_VLA_LOG_DIR="$CONTAINER_WORKSPACE/outputs"

CONTAINER_WANDB_ENV="/run/secrets/interleave-vla-wandb.env"


# ================================================================
# UTILITY
# ================================================================

die() {
    echo "ERRORE: $*" >&2
    exit 1
}


check_host_configuration() {

    command -v docker >/dev/null 2>&1 \
        || die "docker non trovato."

    command -v tmux >/dev/null 2>&1 \
        || die "tmux non trovato."

    [[ -n "$IMAGE" ]] \
        || die "IMAGE non impostata."

    docker image inspect "$IMAGE" >/dev/null 2>&1 \
        || die "Immagine Docker non trovata: $IMAGE"

    [[ -d "$WORKSPACE" ]] \
        || die "Workspace non trovato: $WORKSPACE"

    [[ -d "$OPEN_PI_ZERO" ]] \
        || die "Repository open-pi-zero non trovata: $OPEN_PI_ZERO"

    [[ -d "$WORKSPACE/processed_data/ur5e_interleave/0.1.0" ]] \
        || die "Dataset processato non trovato."

    [[ -d "$WORKSPACE/models/paligemma-3b-pt-224" ]] \
        || die "PaliGemma non trovato: $WORKSPACE/models/paligemma-3b-pt-224"

    [[ -f "$WORKSPACE/models/interleave-pi0-bridge/step34799.pt" ]] \
        || die "Checkpoint Bridge non trovato: $WORKSPACE/models/interleave-pi0-bridge/step34799.pt"

    [[ -f "$WANDB_ENV_FILE" ]] \
        || die "File W&B non trovato: $WANDB_ENV_FILE"
}


load_wandb_credentials() {

    set -a

    # shellcheck disable=SC1090
    source "$WANDB_ENV_FILE"

    set +a

    [[ -n "${WANDB_API_KEY:-}" ]] \
        || die "WANDB_API_KEY non presente in $WANDB_ENV_FILE"

    [[ -n "${VLA_WANDB_ENTITY:-}" ]] \
        || die "VLA_WANDB_ENTITY non presente in $WANDB_ENV_FILE"
}


# ================================================================
# TRAINING DENTRO IL CONTAINER
# ================================================================

run_training() {

    local mode="$1"
    local -a overrides=()

    load_wandb_credentials

    export VLA_DATA_DIR="$CONTAINER_VLA_DATA_DIR"
    export VLA_LOG_DIR="$CONTAINER_VLA_LOG_DIR"
    export TRANSFORMERS_CACHE
    export INTERLEAVE_CHECKPOINT

    export WANDB_DIR="$CONTAINER_VLA_LOG_DIR/wandb"

    mkdir -p \
        "$VLA_LOG_DIR" \
        "$WANDB_DIR" \
        "$VLA_LOG_DIR/launcher_logs"

    case "$mode" in

        smoke)

            overrides+=(
                "n_updates=$SMOKE_UPDATES"
                "use_torch_compile=False"

                # Una sola validation completa alla fine dello smoke.
                "eval_freq=$SMOKE_UPDATES"

                # Con 12 update:
                # step4 -> step8 -> step12
                # La retention deve eliminare step4.
                "save_model_freq=$SMOKE_SAVE_FREQ"

                "save_model_start=0"
            )
            ;;

        compile-smoke)

            overrides+=(
                "n_updates=$COMPILE_SMOKE_UPDATES"
                "use_torch_compile=True"

                # Questo test serve esclusivamente a verificare
                # torch.compile e memoria GPU.
                "data.val=null"

                "save_model_freq=$COMPILE_SMOKE_UPDATES"
                "save_model_start=0"
            )
            ;;

        train)
            ;;

        *)
            die "Modalita interna non valida: $mode"
            ;;
    esac


    local run_name="interleave-ur5e-${mode}"

    if [[ -n "$RUN_TAG" ]]; then
        run_name="${run_name}-${RUN_TAG}"
    fi

    local logfile

    logfile="$VLA_LOG_DIR/launcher_logs/${run_name}_$(date +%Y%m%d_%H%M%S).log"


    echo
    echo "============================================================"
    echo " Interleave-VLA UR5e"
    echo "============================================================"
    echo
    echo "Mode:                    $mode"
    echo "VLA_DATA_DIR:            $VLA_DATA_DIR"
    echo "VLA_LOG_DIR:             $VLA_LOG_DIR"
    echo "TRANSFORMERS_CACHE:      $TRANSFORMERS_CACHE"
    echo "INTERLEAVE_CHECKPOINT:   $INTERLEAVE_CHECKPOINT"
    echo "Log launcher:            $logfile"
    echo


    cd "$CONTAINER_OPEN_PI_ZERO"


    uv run scripts/run.py \
        --config-name=interleaved_ur5e \
        "${overrides[@]}" \
        2>&1 | tee "$logfile"
}


# ================================================================
# DOCKER
# ================================================================

run_container() {

    local mode="$1"

    check_host_configuration

    local host_wandb_env

    host_wandb_env="$(realpath "$WANDB_ENV_FILE")"


    docker run --rm \
        --gpus all \
        --ipc=host \
        --name "$CONTAINER_NAME" \
        \
        -e SMOKE_UPDATES="$SMOKE_UPDATES" \
        -e SMOKE_SAVE_FREQ="$SMOKE_SAVE_FREQ" \
        -e COMPILE_SMOKE_UPDATES="$COMPILE_SMOKE_UPDATES" \
        -e RUN_TAG="$RUN_TAG" \
        \
        -e WANDB_ENV_FILE="$CONTAINER_WANDB_ENV" \
        -e TRANSFORMERS_CACHE="$TRANSFORMERS_CACHE" \
        -e INTERLEAVE_CHECKPOINT="$INTERLEAVE_CHECKPOINT" \
        \
        -v "$WORKSPACE":"$CONTAINER_WORKSPACE" \
        -v "$host_wandb_env":"$CONTAINER_WANDB_ENV":ro \
        \
        -w "$CONTAINER_OPEN_PI_ZERO" \
        \
        "$IMAGE" \
        bash "$CONTAINER_WORKSPACE/scripts/train_interleave.sh" \
            --inside "$mode"
}


# ================================================================
# TMUX
# ================================================================

start_tmux() {

    local mode="$1"
    local command
    local tag_suffix=""

    check_host_configuration

    if [[ -n "$RUN_TAG" ]]; then
        tag_suffix="-${RUN_TAG}"
    fi

    SESSION_NAME="${SESSION_NAME:-interleave-ur5e${tag_suffix}-${mode}}"

    CONTAINER_NAME="${CONTAINER_NAME:-interleave-ur5e${tag_suffix}-${mode}}"


    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        die "Sessione tmux gia esistente: $SESSION_NAME"
    fi


    printf -v command \
        'IMAGE=%q TRANSFORMERS_CACHE=%q INTERLEAVE_CHECKPOINT=%q WANDB_ENV_FILE=%q RUN_TAG=%q SMOKE_UPDATES=%q SMOKE_SAVE_FREQ=%q COMPILE_SMOKE_UPDATES=%q CONTAINER_NAME=%q bash %q --container %q; status=$?; echo "Processo terminato con stato $status"; exec bash' \
        "$IMAGE" \
        "$TRANSFORMERS_CACHE" \
        "$INTERLEAVE_CHECKPOINT" \
        "$WANDB_ENV_FILE" \
        "$RUN_TAG" \
        "$SMOKE_UPDATES" \
        "$SMOKE_SAVE_FREQ" \
        "$COMPILE_SMOKE_UPDATES" \
        "$CONTAINER_NAME" \
        "$SCRIPT_PATH" \
        "$mode"


    tmux new-session \
        -d \
        -s "$SESSION_NAME" \
        -n train \
        "$command"


    echo
    echo "Sessione tmux avviata: $SESSION_NAME"
    echo
    echo "Detach:"
    echo "  Ctrl+B, poi D"
    echo
    echo "Ricollegamento:"
    echo "  tmux attach -t $SESSION_NAME"
    echo


    if [[ -z "${TMUX:-}" ]]; then
        tmux attach -t "$SESSION_NAME"
    fi
}


# ================================================================
# VALIDAZIONE PARAMETRI
# ================================================================

[[ "$SMOKE_UPDATES" =~ ^[1-9][0-9]*$ ]] \
    || die "SMOKE_UPDATES deve essere un intero positivo."

[[ "$SMOKE_SAVE_FREQ" =~ ^[1-9][0-9]*$ ]] \
    || die "SMOKE_SAVE_FREQ deve essere un intero positivo."

[[ "$COMPILE_SMOKE_UPDATES" =~ ^[1-9][0-9]*$ ]] \
    || die "COMPILE_SMOKE_UPDATES deve essere un intero positivo."

if [[ -n "$RUN_TAG" && ! "$RUN_TAG" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
    die "RUN_TAG non valido."
fi


# ================================================================
# ENTRY POINT
# ================================================================

case "${1:-smoke}" in

    smoke|compile-smoke|train)
        start_tmux "${1:-smoke}"
        ;;

    --container)
        CONTAINER_NAME="${CONTAINER_NAME:?CONTAINER_NAME non impostato}"
        run_container "${2:?specificare smoke, compile-smoke o train}"
        ;;

    --inside)
        run_training "${2:?specificare smoke, compile-smoke o train}"
        ;;

    *)
        echo "Uso:"
        echo
        echo "  bash $SCRIPT_PATH smoke"
        echo "  bash $SCRIPT_PATH compile-smoke"
        echo "  bash $SCRIPT_PATH train"
        echo
        exit 1
        ;;

esac