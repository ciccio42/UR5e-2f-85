from __future__ import annotations

import os
from pathlib import Path
from typing import Sequence

import torch
from omegaconf import DictConfig, OmegaConf
from transformers import AutoTokenizer

from src.model.vla.interleaved_pizero import InterleavedPiZeroInference
from src.model.vla.interleaved_processing import InterleavedVLAProcessor


# =============================================================================
# CONFIGURAZIONE ATTESA DEL MODELLO UR5e
# =============================================================================
#
# Questi valori NON definiscono l'architettura del modello: l'architettura viene
# costruita dal file YAML ufficiale usato per il training.
#
# Servono soltanto come controlli di consistenza per evitare di caricare per
# errore un config incompatibile con il checkpoint UR5e.
#
# Training UR5e:
#   - una observation image
#   - una instruction image
#   - un proprioceptive token
#   - action chunk di 4 step
#   - action e proprio 7D
#   - immagini 224x224
#   - 256 token per immagine
#   - sequenza VLM massima di 540 token
#

BATCH_SIZE = 1

IMAGE_SIZE = 224
NUM_IMAGES = 2
NUM_IMAGE_TOKENS = 256
MAX_SEQ_LEN = 540

COND_STEPS = 1
PROPRIO_DIM = 7

ACTION_HORIZON = 4
ACTION_DIM = 7


def _resolve_path(path: str | Path) -> Path:
    """
    Espande variabili d'ambiente e '~' presenti in un path configurato.

    I path non risolti vengono rifiutati esplicitamente per evitare errori
    meno chiari durante il caricamento del modello.
    """
    expanded = os.path.expanduser(os.path.expandvars(str(path)))

    if "$" in expanded:
        raise EnvironmentError(
            f"Unresolved environment variable in path: {path}"
        )

    return Path(expanded)


# =============================================================================
# CONFIGURAZIONE
# =============================================================================


def load_interleave_pi0_config(
    config_path: str | Path,
) -> DictConfig:
    """
    Carica il config runtime di Interleave-Pi0.

    Il file contiene:
    - il path del checkpoint UR5e fine-tunato;
    - il path locale di PaliGemma, necessario per il tokenizer;
    - i parametri di inference;
    - la configurazione architetturale identica a quella usata nel training.

    Il modello non carica separatamente i pesi pretrained di PaliGemma:
    i pesi della rete vengono ripristinati direttamente dal checkpoint UR5e.
    """
    config_path = _resolve_path(config_path)

    if not config_path.is_file():
        raise FileNotFoundError(config_path)

    

    cfg = OmegaConf.load(config_path)

    # Il nuovo YAML di inference è autocontenuto: risolviamo subito
    # interpolazioni interne e variabili d'ambiente.
    OmegaConf.resolve(cfg)

    checkpoint_path = _resolve_path(cfg.checkpoint_path)
    pretrained_model_path = _resolve_path(cfg.pretrained_model_path)

    if not checkpoint_path.is_file():
        raise FileNotFoundError(checkpoint_path)

    if not pretrained_model_path.is_dir():
        raise FileNotFoundError(pretrained_model_path)

    # Manteniamo nel config i path già completamente risolti.
    cfg.checkpoint_path = str(checkpoint_path)
    cfg.pretrained_model_path = str(pretrained_model_path)

    _validate_ur5e_model_config(cfg)

    return cfg


def _validate_ur5e_model_config(cfg: DictConfig) -> None:
    """
    Controlla che il config passato corrisponda all'architettura fine-tunata
    sul dataset UR5e.

    Non modifica il config: serve esclusivamente a intercettare subito un
    eventuale config Bridge / Fractal / Pi0 incompatibile.
    """
    expected_values = {
        "cond_steps": COND_STEPS,
        "proprio_dim": PROPRIO_DIM,
        "horizon_steps": ACTION_HORIZON,
        "action_dim": ACTION_DIM,
        "max_seq_len": MAX_SEQ_LEN,
        "vision.config.image_size": IMAGE_SIZE,
        "vision.config.num_image_tokens": NUM_IMAGE_TOKENS,
    }

    actual_values = {
        "cond_steps": int(cfg.cond_steps),
        "proprio_dim": int(cfg.proprio_dim),
        "horizon_steps": int(cfg.horizon_steps),
        "action_dim": int(cfg.action_dim),
        "max_seq_len": int(cfg.max_seq_len),
        "vision.config.image_size": int(cfg.vision.config.image_size),
        "vision.config.num_image_tokens": int(
            cfg.vision.config.num_image_tokens
        ),
    }

    mismatches = []

    for key, expected in expected_values.items():
        actual = actual_values[key]

        if actual != expected:
            mismatches.append(
                f"{key}: expected {expected}, got {actual}"
            )

    if mismatches:
        raise ValueError(
            "The Interleave-Pi0 config does not match the UR5e model:\n"
            + "\n".join(mismatches)
        )


# =============================================================================
# CHECKPOINT
# =============================================================================


def _load_checkpoint_strict(
    model: InterleavedPiZeroInference,
    checkpoint_path: str | Path,
) -> None:
    """
    Carica esclusivamente i pesi del modello dal checkpoint UR5e.

    La logica replica quella utilizzata dal codice ufficiale di evaluation:
      1. checkpoint caricato inizialmente su CPU;
      2. estrazione di `checkpoint["model"]`;
      3. rimozione dell'eventuale prefisso `_orig_mod.` introdotto da
         torch.compile;
      4. load_state_dict(..., strict=True).

    A differenza dell'inizializzazione del training dal checkpoint Bridge,
    qui NON sono ammesse chiavi LoRA mancanti: il checkpoint UR5e è stato
    prodotto da un modello che contiene già i parametri LoRA.
    """
    checkpoint_path = _resolve_path(checkpoint_path)

    if not checkpoint_path.is_file():
        raise FileNotFoundError(checkpoint_path)

    checkpoint = torch.load(
        checkpoint_path,
        weights_only=True,
        map_location="cpu",
    )

    if "model" not in checkpoint:
        raise KeyError(
            f"Checkpoint {checkpoint_path} does not contain the 'model' key."
        )

    # Stessa gestione utilizzata dagli autori quando un checkpoint è stato
    # salvato a partire da un modello torch.compile.
    model_state = {
        key.replace("_orig_mod.", ""): value
        for key, value in checkpoint["model"].items()
    }

    # Il checkpoint UR5e deve corrispondere esattamente all'architettura
    # costruita dal config UR5e, LoRA compresa.
    model.load_state_dict(
        model_state,
        strict=True,
    )


# =============================================================================
# MODELLO
# =============================================================================


def load_interleave_pi0_model(
    cfg: DictConfig,
    device: torch.device,
    dtype: torch.dtype,
) -> InterleavedPiZeroInference:
    """
    Costruisce Interleave-Pi0 e carica il checkpoint UR5e.

    Viene utilizzata direttamente la classe ufficiale
    `InterleavedPiZeroInference`, che implementa già:
      - encoding SigLIP/PaliGemma;
      - KV cache di VLM e proprio;
      - inizializzazione del rumore action;
      - integrazione Euler del flow matching;
      - clipping finale delle action.

    Non viene quindi reimplementata alcuna parte della generazione delle
    azioni.
    """

    # L'architettura viene costruita dallo stesso config usato in training.
    # use_ddp=False perché il controller lavora su una singola GPU.
    model = InterleavedPiZeroInference(
        cfg,
        use_ddp=False,
    )

    # Carichiamo il checkpoint completo fine-tunato su UR5e.
    _load_checkpoint_strict(
        model=model,
        checkpoint_path=cfg.checkpoint_path,
    )

    # Nel training proprio e action expert condividono la stessa mixture.
    # Utilizziamo direttamente il metodo ufficiale del modello per ripristinare
    # la stessa relazione architetturale.
    model.tie_action_proprio_weights()

    # Per l'inferenza nessun parametro deve mantenere i gradienti.
    # Anche questa è un'implementazione già fornita dal modello ufficiale.
    model.freeze_all_weights()

    # Il training UR5e utilizza bfloat16. Lo spostamento avviene soltanto dopo
    # avere caricato il checkpoint su CPU.
    model.to(dtype)
    model.to(device)

    model.eval()

    return model


# =============================================================================
# INPUT PROCESSOR
# =============================================================================


def load_interleave_pi0_processor(
    cfg: DictConfig,
) -> InterleavedVLAProcessor:
    """
    Costruisce lo stesso processor multimodale utilizzato nel training.

    Non implementiamo manualmente:
      - tokenizzazione del testo;
      - token <image>, <img>, </img>;
      - sostituzione dei placeholder con i visual token;
      - normalizzazione delle immagini;
      - padding e truncation.

    Tutte queste operazioni sono già implementate in
    `InterleavedVLAProcessor`.
    """

    # Identica costruzione del tokenizer utilizzata da
    # InterleavedTrainAgent.
    tokenizer = AutoTokenizer.from_pretrained(
        cfg.pretrained_model_path,
        padding_side="right",
    )

    processor = InterleavedVLAProcessor(
        tokenizer,
        num_image_tokens=cfg.vision.config.num_image_tokens,
        max_seq_len=cfg.max_seq_len,
        tokenizer_padding=cfg.tokenizer_padding,
    )

    # Stesso controllo presente nel codice di training.
    if processor.image_token_id != cfg.image_token_index:
        raise RuntimeError(
            "Unexpected image token mismatch: "
            f"processor={processor.image_token_id}, "
            f"model={cfg.image_token_index}"
        )

    return processor


# =============================================================================
# POLICY
# =============================================================================


class InterleavePi0Policy:
    """
    Wrapper runtime di Interleave-Pi0 per il controller UR5e.

    Responsabilità di questa classe:
      - costruire esattamente l'architettura utilizzata nel training;
      - caricare il checkpoint UR5e;
      - esporre il processor multimodale ufficiale;
      - esporre i metodi ufficiali per causal mask e position ID;
      - eseguire l'inferenza Interleave-Pi0;
      - restituire un intero action chunk normalizzato.

    Non si occupa invece di:
      - resize/crop delle immagini provenienti da ROS;
      - scelta dell'instruction image;
      - normalizzazione del proprio UR5e;
      - denormalizzazione delle action;
      - conversione delta-action -> pose assolute MoveIt.

    Queste operazioni appartengono rispettivamente a pre_process() e
    post_process() del controller.
    """

    def __init__(
        self,
        config_path: str | Path,
    ) -> None:
        if not torch.cuda.is_available():
            raise RuntimeError(
                "Interleave-Pi0 inference requires a CUDA device."
            )

        self.cfg = load_interleave_pi0_config(config_path)
        
        self.device = torch.device(self.cfg.device)

        if self.device.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError(
                "Interleave-Pi0 inference requires a CUDA device."
            )

       

        # Stesso criterio del training:
        # use_bf16=True -> torch.bfloat16
        # altrimenti -> torch.float32.
        self.dtype = (
            torch.bfloat16
            if self.cfg.get("use_bf16", True)
            else torch.float32
        )

        # Modello non compilato.
        #
        # Lo manteniamo esplicitamente perché su questo oggetto sono esposti
        # anche i metodi model-specifici utilizzati dal preprocessing:
        #
        #   build_causal_mask_and_position_ids()
        #   split_full_mask_into_submasks()
        #
        self.model = load_interleave_pi0_model(
            cfg=self.cfg,
            device=self.device,
            dtype=self.dtype,
        )

        # Processor ufficiale, identico a quello utilizzato nel training.
        self.processor = load_interleave_pi0_processor(self.cfg)

        # Se non specificato dal controller, utilizziamo il valore presente
        # nel config UR5e.
        self.use_torch_compile = bool(
            self.cfg.get("use_torch_compile", True)
        )

        # Manteniamo `self.model` come modulo originale perché contiene anche
        # i metodi ausiliari usati dal preprocessing. La versione compilata
        # viene utilizzata esclusivamente per il forward di inferenza.
        if self.use_torch_compile:
            self._forward_model = torch.compile(
                self.model,
                mode="default",
            )
        else:
            self._forward_model = self.model

    # =========================================================================
    # PROCESSOR MULTIMODALE
    # =========================================================================

    def process_interleaved_inputs(
        self,
        texts: Sequence[str],
        images: torch.Tensor,
    ) -> dict[str, torch.Tensor]:
        """
        Applica il processor ufficiale Interleave-VLA.

        Per il nostro setup B=1 `images` deve contenere, nell'ordine:

            images[0] = observation image corrente
            images[1] = instruction image

        shape:
            (2, 3, 224, 224)

        dtype:
            torch.uint8

        `texts` deve contenere il prompt già espresso nel formato atteso dal
        processor, per esempio:

            "Pick the <img> and place it into the second bin"

        Non viene effettuato qui alcun resize e non viene ricostruita
        manualmente la sequenza di visual token: queste operazioni sono
        demandate al processor ufficiale.
        """
        if len(texts) != BATCH_SIZE:
            raise ValueError(
                f"Expected {BATCH_SIZE} text prompt, got {len(texts)}."
            )

        expected_images = (
            NUM_IMAGES,
            3,
            IMAGE_SIZE,
            IMAGE_SIZE,
        )

        if tuple(images.shape) != expected_images:
            raise ValueError(
                f"images must have shape {expected_images}, "
                f"got {tuple(images.shape)}"
            )

        if images.dtype != torch.uint8:
            raise TypeError(
                f"images must have dtype torch.uint8, got {images.dtype}"
            )

        # Chiamata diretta all'implementazione ufficiale.
        return self.processor(
            text=list(texts),
            images=images,
        )

    # =========================================================================
    # MASK E POSITION IDS
    # =========================================================================

    def build_inference_inputs(
        self,
        processor_output: dict[str, torch.Tensor],
        proprios: torch.Tensor,
    ) -> dict[str, torch.Tensor]:
        """
        Completa l'input del processor con proprio, causal mask e position ID.

        Questa funzione segue esattamente il percorso utilizzato nel codice
        ufficiale di training/evaluation.

        La logica delle maschere NON viene riscritta qui:
        vengono chiamati direttamente:

            model.build_causal_mask_and_position_ids(...)
            model.split_full_mask_into_submasks(...)

        `proprios` deve essere già stato convertito e normalizzato dal
        pre_process() del controller.

        Shape attesa:
            proprios = (1, 1, 7)
        """
        required_keys = {
            "input_ids",
            "pixel_values",
            "attention_mask",
        }

        missing_keys = required_keys - processor_output.keys()

        if missing_keys:
            raise KeyError(
                "Processor output is missing keys: "
                + ", ".join(sorted(missing_keys))
            )

        expected_proprio = (
            BATCH_SIZE,
            COND_STEPS,
            PROPRIO_DIM,
        )

        if tuple(proprios.shape) != expected_proprio:
            raise ValueError(
                f"proprios must have shape {expected_proprio}, "
                f"got {tuple(proprios.shape)}"
            )

        if not torch.is_floating_point(proprios):
            raise TypeError("proprios must be a floating-point tensor.")

        # ---------------------------------------------------------------------
        # Blocco causale e position IDs.
        #
        # Implementazione ufficiale PiZero: non ricostruiamo manualmente né
        # l'attention pattern né le posizioni.
        # ---------------------------------------------------------------------
        (
            causal_mask,
            vlm_position_ids,
            proprio_position_ids,
            action_position_ids,
        ) = self.model.build_causal_mask_and_position_ids(
            processor_output["attention_mask"],
            dtype=self.dtype,
        )

        (
            image_text_proprio_mask,
            action_mask,
        ) = self.model.split_full_mask_into_submasks(
            causal_mask
        )

        # Stessa struttura passata a InterleavedPiZeroInference.forward()
        # dal codice ufficiale di evaluation.
        inputs = {
            "input_ids": processor_output["input_ids"],
            "pixel_values": processor_output["pixel_values"].to(
                self.dtype
            ),
            "image_text_proprio_mask": image_text_proprio_mask,
            "action_mask": action_mask,
            "vlm_position_ids": vlm_position_ids,
            "proprio_position_ids": proprio_position_ids,
            "action_position_ids": action_position_ids,
            "proprios": proprios.to(self.dtype),
        }

        # Il processor lavora su CPU. Tutti i tensor vengono trasferiti sulla
        # GPU soltanto dopo avere completato la costruzione dell'input.
        inputs = {
            key: value.to(self.device)
            for key, value in inputs.items()
        }

        return inputs

    # =========================================================================
    # VALIDAZIONE INPUT INFERENCE
    # =========================================================================

    @staticmethod
    def _validate_inputs(
        input_ids: torch.Tensor,
        pixel_values: torch.Tensor,
        image_text_proprio_mask: torch.Tensor,
        action_mask: torch.Tensor,
        vlm_position_ids: torch.Tensor,
        proprio_position_ids: torch.Tensor,
        action_position_ids: torch.Tensor,
        proprios: torch.Tensor,
    ) -> None:
        """
        Controlla le dimensioni fondamentali dell'input prima del forward.

        Non modifica i tensor.
        """
        expected_input_ids = (
            BATCH_SIZE,
            MAX_SEQ_LEN,
        )

        expected_pixel_values = (
            NUM_IMAGES,
            3,
            IMAGE_SIZE,
            IMAGE_SIZE,
        )

        expected_proprios = (
            BATCH_SIZE,
            COND_STEPS,
            PROPRIO_DIM,
        )

        expected_vlm_positions = (
            BATCH_SIZE,
            MAX_SEQ_LEN,
        )

        expected_proprio_positions = (
            BATCH_SIZE,
            COND_STEPS,
        )

        expected_action_positions = (
            BATCH_SIZE,
            ACTION_HORIZON,
        )

        total_tokens = (
            MAX_SEQ_LEN
            + COND_STEPS
            + ACTION_HORIZON
        )

        expected_image_text_proprio_mask = (
            BATCH_SIZE,
            1,
            MAX_SEQ_LEN + COND_STEPS,
            MAX_SEQ_LEN + COND_STEPS,
        )

        expected_action_mask = (
            BATCH_SIZE,
            1,
            ACTION_HORIZON,
            total_tokens,
        )

        checks = (
            ("input_ids", input_ids, expected_input_ids),
            (
                "pixel_values",
                pixel_values,
                expected_pixel_values,
            ),
            (
                "image_text_proprio_mask",
                image_text_proprio_mask,
                expected_image_text_proprio_mask,
            ),
            (
                "action_mask",
                action_mask,
                expected_action_mask,
            ),
            (
                "vlm_position_ids",
                vlm_position_ids,
                expected_vlm_positions,
            ),
            (
                "proprio_position_ids",
                proprio_position_ids,
                expected_proprio_positions,
            ),
            (
                "action_position_ids",
                action_position_ids,
                expected_action_positions,
            ),
            (
                "proprios",
                proprios,
                expected_proprios,
            ),
        )

        for name, tensor, expected_shape in checks:
            if tuple(tensor.shape) != expected_shape:
                raise ValueError(
                    f"{name} must have shape {expected_shape}, "
                    f"got {tuple(tensor.shape)}"
                )

        if input_ids.dtype != torch.long:
            raise TypeError(
                f"input_ids must have dtype torch.long, "
                f"got {input_ids.dtype}"
            )

        if not torch.is_floating_point(pixel_values):
            raise TypeError(
                "pixel_values must be a floating-point tensor."
            )

        if not torch.is_floating_point(proprios):
            raise TypeError(
                "proprios must be a floating-point tensor."
            )

    # =========================================================================
    # INFERENCE
    # =========================================================================

    @torch.inference_mode()
    def predict(
        self,
        input_ids: torch.Tensor,
        pixel_values: torch.Tensor,
        image_text_proprio_mask: torch.Tensor,
        action_mask: torch.Tensor,
        vlm_position_ids: torch.Tensor,
        proprio_position_ids: torch.Tensor,
        action_position_ids: torch.Tensor,
        proprios: torch.Tensor,
    ) -> torch.Tensor:
        """
        Esegue una query Interleave-Pi0.

        Gli input devono essere già stati prodotti da:

            process_interleaved_inputs(...)
                +
            build_inference_inputs(...)

        Il metodo ufficiale `InterleavedPiZeroInference.forward()`:
          1. elabora VLM + proprio;
          2. costruisce e riutilizza le KV cache;
          3. campiona il rumore iniziale delle action;
          4. integra il flow field per `num_inference_steps`;
          5. applica il clipping finale configurato.

        Output:
            tensor normalizzato con shape (1, 4, 7)

        IMPORTANTE:
        le action sono ancora nello spazio normalizzato del training.
        La denormalizzazione appartiene a
        InterleavePi0Controller.post_process().
        """
        self._validate_inputs(
            input_ids=input_ids,
            pixel_values=pixel_values,
            image_text_proprio_mask=image_text_proprio_mask,
            action_mask=action_mask,
            vlm_position_ids=vlm_position_ids,
            proprio_position_ids=proprio_position_ids,
            action_position_ids=action_position_ids,
            proprios=proprios,
        )

        # Non implementiamo manualmente il flow matching:
        # InterleavedPiZeroInference.forward() chiama direttamente
        # InterleavedPiZero.infer_action().
        action_chunk = self._forward_model(
            input_ids=input_ids,
            pixel_values=pixel_values,
            image_text_proprio_mask=image_text_proprio_mask,
            action_mask=action_mask,
            vlm_position_ids=vlm_position_ids,
            proprio_position_ids=proprio_position_ids,
            action_position_ids=action_position_ids,
            proprios=proprios,
        )

        expected_output = (
            BATCH_SIZE,
            ACTION_HORIZON,
            ACTION_DIM,
        )

        if tuple(action_chunk.shape) != expected_output:
            raise RuntimeError(
                "Interleave-Pi0 returned shape "
                f"{tuple(action_chunk.shape)}, "
                f"expected {expected_output}"
            )

        return action_chunk