from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Optional

import numpy as np
import torch
from omegaconf import DictConfig
import json
from PIL import Image as PILImage

from ai_controller.utils.ai_controller import AIController

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from interleave_pi0 import ACTION_HORIZON, InterleavePi0Policy, _resolve_path
from interleave_pi0_utils import (
    build_proprio,
    delta_action_chunk_to_absolute_targets,
    denormalize_action_chunk,
    load_instruction_image,
    prepare_interleaved_prompt,
    prepare_proprio_tensor,
    process_front_image,
    stack_interleaved_images,
)


ROBOT_STATE_DIM = 8




class InterleavePi0Controller(AIController):
    """Adatta Mimic Video al ciclo di controllo del nodo ROS per UR5e."""

    def __init__(self, model_config: str, task_name: str = "pick_place") -> None:
        self.task_name = task_name

        # Salviamo il path assoluto del config.
        # La directory del config sarà anche la radice per risolvere
        # instruction_images/... presenti nello YAML.
        self.config_path = _resolve_path(model_config).resolve()
        self.config_dir = self.config_path.parent

        self.cfg: Optional[DictConfig] = None
        self._policy: Optional[InterleavePi0Policy] = None

        self.action_buffer: Optional[np.ndarray] = None
        self.action_idx = 0
        self.command: Optional[str] = None
        self.instruction_image: Optional[torch.Tensor] = None
        self.current_task_id: Optional[str] = None

        
        self.gripper_closed = False
        
        self._fixed_orientation_xyzw: Optional[np.ndarray] = None
        self.proprio_p01: Optional[np.ndarray] = None
        self.proprio_p99: Optional[np.ndarray] = None
        self.action_p01: Optional[np.ndarray] = None
        self.action_p99: Optional[np.ndarray] = None

        super().__init__(str(self.config_path))


    def load_model(self, model_config: str):
        """
        Costruisce Interleave-Pi0 a partire dal config runtime.

        InterleavePi0Policy si occupa di:
        - caricare e validare lo YAML;
        - costruire l'architettura;
        - caricare il checkpoint UR5e;
        - costruire il processor multimodale;
        - spostare il modello sul device configurato.
        """
        self._policy = InterleavePi0Policy(
            config_path=model_config,
        )

        # Riutilizziamo lo stesso DictConfig già caricato dalla policy:
        # il controller e il modello hanno così un'unica sorgente di verità.
        self.cfg = self._policy.cfg

        # -------------------------------------------------------------------------
        # Optional fixed-orientation ablation
        # -------------------------------------------------------------------------
        #
        # Se specificato nello YAML, il quaternion fisso sostituisce
        # l'orientamento predetto dal modello prima di inviare la singola
        # action al robot.
        #
        # Formato ROS:
        #   [qx, qy, qz, qw]
        #
        fixed_orientation = self.cfg.get(
            "fixed_orientation_xyzw",
            None,
        )

        if fixed_orientation is None:
            self._fixed_orientation_xyzw = None

        else:
            fixed_orientation = np.asarray(
                fixed_orientation,
                dtype=np.float64,
            )

            if fixed_orientation.shape != (4,):
                raise ValueError(
                    "fixed_orientation_xyzw must have shape (4,), "
                    f"got {fixed_orientation.shape}"
                )

            if not np.all(np.isfinite(fixed_orientation)):
                raise ValueError(
                    "fixed_orientation_xyzw must contain only finite values."
                )

            norm = np.linalg.norm(fixed_orientation)

            if norm <= 1e-8:
                raise ValueError(
                    "fixed_orientation_xyzw cannot be a zero quaternion."
                )

            # Normalizziamo per sicurezza.
            self._fixed_orientation_xyzw = (
                fixed_orientation / norm
            ).astype(np.float32)

            print(
                "[InterleavePi0Controller] Fixed orientation override enabled: "
                f"{self._fixed_orientation_xyzw}"
            )

        num_execute_actions = int(
            self.cfg.get(
                "num_execute_actions",
                ACTION_HORIZON,
            )
        )

        if not 1 <= num_execute_actions <= ACTION_HORIZON:
            raise ValueError(
                f"num_execute_actions must be between 1 and "
                f"{ACTION_HORIZON}, got {num_execute_actions}"
            )

        # Rendiamo esplicito il default anche nel DictConfig,
        # così in seguito possiamo usare direttamente
        # self.cfg.num_execute_actions.
        self.cfg.num_execute_actions = num_execute_actions

        return self._policy.model

    
    def move_model_to_device(self, device):
        """La policy carica gia entrambe le pipeline su CUDA."""
        if torch.device(device).type != "cuda":
            raise ValueError("Mimic Video supports only a CUDA device.")

    def load_command(
        self,
        demo_path: str,
        task_id: str | None = None,
        **kwargs,
    ):
        """
        Seleziona il task Interleave-Pi0 da eseguire.

        Questa funzione viene chiamata quando AIControllerNode riceve/seleziona
        un nuovo task, prima dell'inizio del rollout.

        A partire dal task_id:
        1. recupera prompt e instruction image dallo YAML;
        2. converte <image> nel placeholder atteso dal processor;
        3. carica una sola volta il crop statico 224x224 del box;
        4. salva comando e instruction image nello stato del controller.

        Il crop non viene ricaricato a ogni inference: rimane costante per
        tutta l'esecuzione del task.
        """
        del demo_path
        del kwargs

        if self.cfg is None:
            raise RuntimeError("Controller configuration has not been loaded.")


        # -------------------------------------------------------------------------
        # Dataset statistics
        # -------------------------------------------------------------------------
        #
        # Le statistiche sono indipendenti dal task e vengono quindi caricate
        # soltanto alla prima chiamata di load_command().
        #
        # Serviranno:
        #   - nel pre_process() per normalizzare il proprio;
        #   - nel post_process() per denormalizzare le action.
        #
        if self.proprio_p01 is None:

            stats_path = Path(
                str(self.cfg.dataset_statistics_path)
            )

            # Il path nello YAML è relativo alla directory del config.
            if not stats_path.is_absolute():
                stats_path = self.config_dir / stats_path

            stats_path = stats_path.resolve()

            if not stats_path.is_file():
                raise FileNotFoundError(
                    f"Dataset statistics not found: {stats_path}"
                )

            with open(stats_path, "r", encoding="utf-8") as stats_file:
                stats = json.load(stats_file)

            try:
                self.proprio_p01 = np.asarray(
                    stats["proprio"]["p01"],
                    dtype=np.float32,
                )
                self.proprio_p99 = np.asarray(
                    stats["proprio"]["p99"],
                    dtype=np.float32,
                )

                self.action_p01 = np.asarray(
                    stats["action"]["p01"],
                    dtype=np.float32,
                )
                self.action_p99 = np.asarray(
                    stats["action"]["p99"],
                    dtype=np.float32,
                )

            except KeyError as exc:
                raise KeyError(
                    f"Invalid dataset statistics file {stats_path}: "
                    f"missing key {exc}"
                ) from exc

            # Tutti i vettori devono corrispondere alle 7 dimensioni
            # utilizzate dal modello UR5e.
            for name, values in (
                ("proprio_p01", self.proprio_p01),
                ("proprio_p99", self.proprio_p99),
                ("action_p01", self.action_p01),
                ("action_p99", self.action_p99),
            ):
                if values.shape != (7,):
                    raise ValueError(
                        f"{name} must have shape (7,), got {values.shape}"
                    )

            print(
                f"[InterleavePi0Controller] Loaded dataset statistics "
                f"from {stats_path}"
            )


        if task_id is None:
            raise KeyError("task_id is required for Interleave-Pi0.")

        # AIControllerNode usa task ID nel formato:
        #   "00", "01", ..., "15"
        #
        # Manteniamo compatibilità anche con eventuali valori numerici.
        task_id = str(task_id).zfill(2)
        self.current_task_id = task_id

        task_cfg = self.cfg.tasks.get(task_id)

        if task_cfg is None:
            raise KeyError(
                f"Unknown Interleave-Pi0 task_id: {task_id}"
            )

        prompt = task_cfg.get("prompt")
        instruction_image_path = task_cfg.get("instruction_image")

        if not prompt:
            raise ValueError(
                f"Task {task_id} does not define a prompt."
            )

        if not instruction_image_path:
            raise ValueError(
                f"Task {task_id} does not define an instruction image."
            )

        # -------------------------------------------------------------------------
        # Prompt
        # -------------------------------------------------------------------------
        #
        # Nello YAML manteniamo lo stesso formato del dataset:
        #
        #   "Pick the <image> and place it into the second bin"
        #
        # Prima del processor bisogna invece usare:
        #
        #   <image_placeholder>
        #
        self.command = prepare_interleaved_prompt(prompt)

        # -------------------------------------------------------------------------
        # Instruction image
        # -------------------------------------------------------------------------
        #
        # Il path nello YAML è relativo alla directory del controller:
        #
        #   instruction_images/green_box.png
        #
        image_path = Path(instruction_image_path)

        if not image_path.is_absolute():
            image_path = self.config_dir / image_path

        image_path = image_path.resolve()

        # Il PNG è già:
        #   - RGB
        #   - crop bbox espansa del 20%
        #   - 224x224
        #
        # load_instruction_image() verifica queste proprietà e produce
        # un tensor uint8 (3, 224, 224).
        self.instruction_image = load_instruction_image(
            image_path
        )

        # Un nuovo comando invalida qualsiasi chunk eventualmente rimasto
        # dal task precedente.
        self.action_buffer = None
        self.action_idx = 0

        print(
            f"[InterleavePi0Controller] Loaded task {task_id}: "
            f"{prompt!r}, instruction_image={image_path.name}"
        )
        

    def reset(self):
        """
        Azzera lo stato runtime relativo al rollout corrente.

        Viene chiamata da AIControllerNode all'inizio di ogni nuova traiettoria,
        prima di riportare il robot alla posa iniziale e prima di load_command().

        Non vengono ricaricati né modificati:
        - il modello Interleave-Pi0;
        - il processor multimodale;
        - il config;
        - le statistiche di normalizzazione.

        Vengono eliminati solamente gli stati associati al rollout/task
        precedente.
        """

        # Eventuali action non ancora eseguite del chunk precedente
        # non devono essere riutilizzate nel nuovo rollout.
        self.action_buffer = None
        self.action_idx = 0

        # Il nuovo task verrà impostato immediatamente dopo tramite
        # load_command().
        self.command = None
        self.instruction_image = None
        self.current_task_id = None

        # Verrà aggiornato con lo stato reale ricevuto dal robot
        # durante il successivo pre_process().
        self.gripper_closed = False


    
    def _save_front_image(
        self,
        front_image: torch.Tensor,
        save_path: str | Path,
    ) -> None:
        """
        Salva la front image 224x224 realmente fornita a Interleave-Pi0.

        front_image:
            torch.uint8, shape (3, 224, 224), RGB

        Il nome del file è mantenuto compatibile con AIControllerNode:
            pre_processed_img_0.png
        """
        if not isinstance(front_image, torch.Tensor):
            raise TypeError(
                f"front_image must be a torch.Tensor, got {type(front_image)}"
            )

        if front_image.shape != (3, 224, 224):
            raise ValueError(
                "Expected front_image shape (3, 224, 224), "
                f"got {tuple(front_image.shape)}"
            )

        if front_image.dtype != torch.uint8:
            raise ValueError(
                f"Expected front_image dtype torch.uint8, got {front_image.dtype}"
            )

        image_hwc = (
            front_image
            .detach()
            .cpu()
            .permute(1, 2, 0)
            .numpy()
        )

        save_path = Path(save_path)
        save_path.mkdir(parents=True, exist_ok=True)

        PILImage.fromarray(image_hwc, mode="RGB").save(
            save_path / "pre_processed_img_0.png"
        )
    
    def pre_process(self, input_data):
        """
        Prepara gli input correnti per una nuova query Interleave-Pi0.

        Viene eseguita soltanto quando `inference()` deve generare un nuovo
        action chunk, cioè quando `self.action_buffer` è vuoto.

        Prepara:
        1. observation image frontale corrente;
        2. instruction image statica associata al task;
        3. prompt interleaved selezionato da load_command();
        4. stato propriocettivo corrente del robot;
        5. input_ids, pixel_values, mask e position IDs richiesti dal modello.

        Il preprocessing specifico PaliGemma/SigLIP non viene reimplementato:
        viene delegato a InterleavePi0Policy.
        """
        if not isinstance(input_data, (list, tuple)) or len(input_data) != 2:
            raise ValueError(
                "input_data must be [images, robot_state]."
            )

        images, robot_state = input_data

        # -------------------------------------------------------------------------
        # Verifica comando
        # -------------------------------------------------------------------------

        # load_command() deve essere stata chiamata prima dell'inizio del rollout.
        if self.command is None:
            raise RuntimeError(
                "No Interleave-Pi0 command loaded. "
                "load_command() must be called before inference()."
            )

        if self.instruction_image is None:
            raise RuntimeError(
                "No instruction image loaded. "
                "load_command() must be called before inference()."
            )

        if self._policy is None:
            raise RuntimeError(
                "Interleave-Pi0 policy has not been loaded."
            )

        # -------------------------------------------------------------------------
        # Camera frontale
        # -------------------------------------------------------------------------

        if not images:
            raise ValueError(
                "Interleave-Pi0 requires the front camera image."
            )

        # images[0] è il frame RGB corrente della camera frontale.
        #
        # process_front_image() replica il preprocessing del dataset:
        #
        #   RGB live
        #       -> crop globale (0, 10, 130, 100)
        #       -> resize 224x224 con cv2.INTER_LINEAR
        #       -> HWC -> CHW
        #       -> uint8
        #
        # NON viene applicata l'espansione del 20%:
        # quella serviva esclusivamente per creare l'instruction crop.
        front_image = process_front_image(
            images[0],
            input_color_order="rgb",
        )

        # Costruiamo le due immagini nello stesso ordine usato nel training:
        #
        #   images[0] = observation image corrente
        #   images[1] = instruction image del box
        #
        # Shape finale:
        #   (2, 3, 224, 224)
        interleaved_images = stack_interleaved_images(
            front_image=front_image,
            instruction_image=self.instruction_image,
        )

        # -------------------------------------------------------------------------
        # Stato robot
        # -------------------------------------------------------------------------

        robot_state = np.asarray(
            robot_state,
            dtype=np.float64,
        )

        if (
            robot_state.shape != (ROBOT_STATE_DIM,)
            or not np.all(np.isfinite(robot_state))
        ):
            raise ValueError(
                "robot_state must be finite and shaped (8,): "
                "[x, y, z, qx, qy, qz, qw, gripper_closed]."
            )

        gripper_state = float(robot_state[7])
        self.gripper_closed = gripper_state >= 0.5

        # build_proprio() converte:
        #
        #   [x, y, z, qx, qy, qz, qw, gripper]
        #
        # nel formato utilizzato durante il training:
        #
        #   [x, y, z, roll, pitch, yaw, gripper]
        #
        # usando Euler XYZ / RPY.
        proprio = build_proprio(
            position=robot_state[:3],
            quaternion_xyzw=robot_state[3:7],
            gripper_closed=self.gripper_closed,
        )

        # Applichiamo la stessa normalizzazione BOUNDS del training
        # utilizzando p01/p99 calcolati esclusivamente sul train set.
        #
        # Shape:
        #   (7,) -> (1, 1, 7)
        proprio_tensor = prepare_proprio_tensor(
            proprio=proprio,
            proprio_p01=self.proprio_p01,
            proprio_p99=self.proprio_p99,
        )

        # -------------------------------------------------------------------------
        # Processor multimodale Interleave
        # -------------------------------------------------------------------------

        # self.command è già stato preparato una volta da load_command():
        #
        #   <image> -> <image_placeholder>
        #
        # Il processor ufficiale produce:
        #
        #   input_ids
        #   pixel_values
        #   attention_mask
        #
        processor_output = self._policy.process_interleaved_inputs(
            texts=[self.command],
            images=interleaved_images,
        )

        # -------------------------------------------------------------------------
        # Mask, position IDs e trasferimento su CUDA
        # -------------------------------------------------------------------------

        # Il wrapper del modello usa le funzioni ufficiali Pi0 per costruire:
        #
        #   image_text_proprio_mask
        #   action_mask
        #   vlm_position_ids
        #   proprio_position_ids
        #   action_position_ids
        #
        # e porta tutti gli input sul device configurato.
        model_inputs = self._policy.build_inference_inputs(
            processor_output=processor_output,
            proprios=proprio_tensor,
        )

        # Manteniamo anche la pose NON normalizzata corrente:
        # servirà nel post_process() come riferimento per integrare le
        # delta-action previste dal modello.
        return {
            "model_inputs": model_inputs,

            "reference_position": robot_state[:3].copy(),
            "reference_quaternion": robot_state[3:7].copy(),

            "gripper_closed": self.gripper_closed,

            # Utili per debug/log.
            "front_image": front_image,
            "proprio": proprio,
            "proprio_tensor": proprio_tensor,
        }

    def post_process(self, output_data):
        """
        Converte l'intero action chunk prodotto da Interleave-Pi0
        in target assoluti utilizzabili dal controller robotico.

        Viene eseguita SOLO quando il modello genera un nuovo chunk.

        Pipeline:
            action normalizzate (1, 4, 7)
                -> denormalizzazione BOUNDS
                -> delta action fisiche (4, 7)
                -> integrazione sequenziale delle delta pose
                -> target assoluti (4, 8)

        Ogni target finale ha formato:

            [x, y, z, qx, qy, qz, qw, gripper]

        Il metodo NON invia direttamente le action al robot.
        L'intero chunk viene restituito a `inference()`, che lo salva
        in `self.action_buffer` e restituisce una sola action per volta
        al loop di AIControllerNode.
        """

        if self.action_p01 is None or self.action_p99 is None:
            raise RuntimeError(
                "Action statistics are not loaded. "
                "load_command() must be called before inference()."
            )

        if "action_chunk" not in output_data:
            raise KeyError(
                "output_data must contain 'action_chunk'."
            )

        if "reference_position" not in output_data:
            raise KeyError(
                "output_data must contain 'reference_position'."
            )

        if "reference_quaternion" not in output_data:
            raise KeyError(
                "output_data must contain 'reference_quaternion'."
            )

        # -------------------------------------------------------------------------
        # 1. Denormalizzazione delle action
        # -------------------------------------------------------------------------
        #
        # Interleave-Pi0 restituisce:
        #
        #   shape = (1, 4, 7)
        #
        # nello spazio normalizzato usato durante il training.
        #
        # denormalize_action_chunk():
        #   - denormalizza dx, dy, dz, droll, dpitch, dyaw con p01/p99;
        #   - lascia invariato il gripper, perché nel training la settima
        #     componente non veniva normalizzata.
        #
        # Output:
        #
        #   shape = (4, 7)
        #
        #   [dx, dy, dz, droll, dpitch, dyaw, gripper]
        #
        action_chunk = denormalize_action_chunk(
            action_chunk=output_data["action_chunk"],
            action_p01=self.action_p01,
            action_p99=self.action_p99,
        )

        if self.cfg.trace_action_conversions:
            self._trace_action_chunk(
                query_step=output_data["query_step"],
                normalized_chunk=output_data["action_chunk"],
                denormalized_chunk=action_chunk,
            )

        # -------------------------------------------------------------------------
        # 2. Delta action -> pose assolute
        # -------------------------------------------------------------------------
        #
        # Il primo delta è applicato alla pose reale osservata al momento
        # della query.
        #
        # I delta successivi vengono integrati sequenzialmente:
        #
        #   target_1 = current_pose + delta_0
        #   target_2 = target_1    + delta_1
        #   target_3 = target_2    + delta_2
        #   target_4 = target_3    + delta_3
        #
        # Per l'orientamento viene replicata la convenzione del dataset:
        #
        #   R_next = R_delta @ R_current
        #
        positions, quaternions_xyzw, gripper_commands = (
            delta_action_chunk_to_absolute_targets(
                action_chunk=action_chunk,
                reference_position=output_data["reference_position"],
                reference_quaternion_xyzw=output_data[
                    "reference_quaternion"
                ],
            )
        )

        # -------------------------------------------------------------------------
        # 3. Costruzione del formato richiesto da AIControllerNode
        # -------------------------------------------------------------------------
        #
        # Ogni action restituita dal controller deve essere:
        #
        #   [x, y, z, qx, qy, qz, qw, gripper]
        #
        # quindi concateniamo:
        #
        #   positions            -> (4, 3)
        #   quaternions_xyzw     -> (4, 4)
        #   gripper_commands     -> (4, 1)
        #
        absolute_chunk = np.concatenate(
            (
                positions,
                quaternions_xyzw,
                gripper_commands[:, None],
            ),
            axis=1,
        ).astype(
            np.float32,
            copy=False,
        )

        expected_shape = (
            ACTION_HORIZON,
            8,
        )

        if absolute_chunk.shape != expected_shape:
            raise RuntimeError(
                f"Unexpected absolute action chunk shape "
                f"{absolute_chunk.shape}, expected {expected_shape}."
            )

        if self.cfg.trace_action_conversions:
            self._trace_absolute_chunk(
                query_step=output_data["query_step"],
                absolute_chunk=absolute_chunk,
            )

        return absolute_chunk

    @staticmethod
    def _format_array(value) -> str:
        """Formatta array/tensor in modo compatto per i log di debug."""
        if isinstance(value, torch.Tensor):
            value = value.detach().float().cpu().numpy()

        return np.array2string(
            np.asarray(value),
            precision=7,
            suppress_small=False,
            separator=", ",
        )


    def _trace_model_input(
        self,
        query_step: int,
        processed: dict,
    ) -> None:
        """
        Stampa gli input principali utilizzati per una nuova query Interleave-Pi0.

        Viene chiamata subito dopo pre_process(), soltanto quando il buffer
        è vuoto e quindi sta per essere eseguita una nuova inferenza.
        """
        proprio = np.asarray(processed["proprio"])

        proprio_normalized = (
            processed["proprio_tensor"]
            .detach()
            .float()
            .cpu()
            .numpy()[0, 0]
        )

        front_image = processed["front_image"]

        print(
            f"[InterleavePi0Trace][query={query_step}][INPUT]\n"
            f"  task_id={self.current_task_id}\n"
            f"  command={self.command!r}\n"
            f"  reference_position="
            f"{self._format_array(processed['reference_position'])}\n"
            f"  reference_quaternion_xyzw="
            f"{self._format_array(processed['reference_quaternion'])}\n"
            f"  quaternion_norm="
            f"{np.linalg.norm(processed['reference_quaternion']):.9f}\n"
            f"  proprio_physical="
            f"{self._format_array(proprio)}\n"
            f"  proprio_rpy_deg="
            f"{self._format_array(np.degrees(proprio[3:6]))}\n"
            f"  proprio_normalized="
            f"{self._format_array(proprio_normalized)}\n"
            f"  gripper_closed={processed['gripper_closed']}\n"
            f"  front_image_shape={tuple(front_image.shape)} "
            f"dtype={front_image.dtype}\n"
            f"  instruction_image_shape="
            f"{tuple(self.instruction_image.shape)} "
            f"dtype={self.instruction_image.dtype}"
        )


    def _trace_action_chunk(
        self,
        query_step: int,
        normalized_chunk,
        denormalized_chunk,
    ) -> None:
        """
        Confronta direttamente l'output normalizzato del modello con le action
        fisiche ottenute dopo la denormalizzazione BOUNDS.

        Viene chiamata nel post_process().
        """
        if isinstance(normalized_chunk, torch.Tensor):
            normalized_chunk = (
                normalized_chunk
                .detach()
                .float()
                .cpu()
                .numpy()
            )

        normalized_chunk = np.asarray(normalized_chunk)

        if normalized_chunk.shape == (1, ACTION_HORIZON, 7):
            normalized_chunk = normalized_chunk[0]

        denormalized_chunk = np.asarray(denormalized_chunk)

        print(
            f"[InterleavePi0Trace][query={query_step}]"
            "[ACTION_CHUNK]"
        )

        for i, (normalized, physical) in enumerate(
            zip(normalized_chunk, denormalized_chunk)
        ):
            print(
                f"  action[{i}]\n"
                f"    normalized="
                f"{self._format_array(normalized)}\n"
                f"    physical_delta_xyz="
                f"{self._format_array(physical[:3])}\n"
                f"    physical_delta_rpy_rad="
                f"{self._format_array(physical[3:6])}\n"
                f"    physical_delta_rpy_deg="
                f"{self._format_array(np.degrees(physical[3:6]))}\n"
                f"    gripper={physical[6]:.7f}"
            )


    def _trace_absolute_chunk(
        self,
        query_step: int,
        absolute_chunk: np.ndarray,
    ) -> None:
        """
        Stampa i target assoluti ottenuti dopo l'integrazione sequenziale
        delle delta-action.

        Viene chiamata alla fine del post_process().
        """
        print(
            f"[InterleavePi0Trace][query={query_step}]"
            "[ABSOLUTE_TARGETS]"
        )

        for i, action in enumerate(absolute_chunk):
            position = action[:3]
            quaternion = action[3:7]
            gripper = action[7]

            print(
                f"  target[{i}]\n"
                f"    position="
                f"{self._format_array(position)}\n"
                f"    quaternion_xyzw="
                f"{self._format_array(quaternion)}\n"
                f"    quaternion_norm="
                f"{np.linalg.norm(quaternion):.9f}\n"
                f"    gripper={gripper:.1f}"
            )


    def _trace_buffer_action(
        self,
        step: int,
        buffer_index: int,
        action: np.ndarray,
    ) -> None:
        """
        Stampa la singola action che inference() sta restituendo al loop ROS.

        Questa funzione è utile per distinguere:
        - quando viene effettuata una nuova query;
        - quali action appartengono allo stesso chunk;
        - quale target viene realmente inviato al robot a ogni ciclo.
        """
        print(
            f"[InterleavePi0Trace][step={step}]"
            f"[buffer_action={buffer_index}]\n"
            f"  position={self._format_array(action[:3])}\n"
            f"  quaternion_xyzw={self._format_array(action[3:7])}\n"
            f"  quaternion_norm={np.linalg.norm(action[3:7]):.9f}\n"
            f"  gripper={action[7]:.1f}"
        )

    def inference(
        self,
        input_data,
        t: int = 0,
        save_path: str | Path | None = None,
    ):
        """
        Restituisce una singola action assoluta per ogni chiamata del loop ROS.

        Quando il buffer è vuoto:
            1. acquisisce e preprocessa l'osservazione corrente;
            2. esegue una nuova query Interleave-Pi0;
            3. denormalizza e converte l'intero action chunk in target assoluti;
            4. salva nel buffer le prime `num_execute_actions`.

        Quando il buffer contiene ancora action:
            - NON esegue preprocessing;
            - NON richiama il modello;
            - restituisce semplicemente la prossima action del chunk.

        Output:
            [[x, y, z, qx, qy, qz, qw, gripper]]
        """

        # save_path è mantenuto nella firma per compatibilità con AIController,
        # ma attualmente Interleave-Pi0 non salva immagini/history su disco.

        if self._policy is None or self.cfg is None:
            raise RuntimeError(
                "Interleave-Pi0 controller has not been initialized."
            )

        if self.command is None or self.instruction_image is None:
            raise RuntimeError(
                "load_command() must be called before inference()."
            )

        # =========================================================================
        # NUOVA QUERY DEL MODELLO
        # =========================================================================
        #
        # Preprocessing e inferenza vengono eseguiti SOLO quando non ci sono
        # action precedentemente predette ancora da eseguire.
        #
        if self.action_buffer is None:

            # ---------------------------------------------------------------------
            # 1. Preprocessing
            # ---------------------------------------------------------------------
            #
            # Usa l'osservazione robotica più recente:
            #   - front image corrente;
            #   - proprio corrente;
            #   - prompt e instruction image del task.
            #
            processed = self.pre_process(input_data)

            if save_path is not None:
                self._save_front_image(
                    front_image=processed["front_image"],
                    save_path=save_path,
                )

            if self.cfg.trace_action_conversions:
                self._trace_model_input(
                    query_step=t,
                    processed=processed,
                )

            # ---------------------------------------------------------------------
            # 2. Interleave-Pi0 inference
            # ---------------------------------------------------------------------
            #
            # model_inputs contiene già:
            #   input_ids
            #   pixel_values
            #   image_text_proprio_mask
            #   action_mask
            #   vlm_position_ids
            #   proprio_position_ids
            #   action_position_ids
            #   proprios
            #
            # e tutti i tensor sono già sul device corretto.
            #
            raw_chunk = self._policy.predict(
                **processed["model_inputs"]
            )

            # raw_chunk:
            #   shape = (1, 4, 7)
            #   spazio normalizzato del training

            # ---------------------------------------------------------------------
            # 3. Postprocessing dell'intero chunk
            # ---------------------------------------------------------------------
            #
            # Output:
            #   absolute_chunk shape = (4, 8)
            #
            # Ogni riga:
            #   [x, y, z, qx, qy, qz, qw, gripper]
            #
            absolute_chunk = self.post_process(
                {
                    "action_chunk": raw_chunk,
                    "reference_position": processed[
                        "reference_position"
                    ],
                    "reference_quaternion": processed[
                        "reference_quaternion"
                    ],
                    "query_step": t,
                }
            )

            # ---------------------------------------------------------------------
            # 4. Action buffer
            # ---------------------------------------------------------------------
            #
            # Il modello produce sempre ACTION_HORIZON=4 action.
            # Possiamo decidere di eseguirne soltanto K prima di ripianificare.
            #
            num_execute_actions = int(
                self.cfg.num_execute_actions
            )

            self.action_buffer = absolute_chunk[
                :num_execute_actions
            ].copy()

            self.action_idx = 0

            print(
                f"[InterleavePi0Controller] "
                f"Inference query t={t}: "
                f"generated {ACTION_HORIZON} actions, "
                f"buffered {len(self.action_buffer)}."
            )

        # =========================================================================
        # ESTRAZIONE DELLA PROSSIMA ACTION
        # =========================================================================

        buffer_action_idx = self.action_idx

        action = self.action_buffer[
            buffer_action_idx
        ].copy()

        # =========================================================================
        # OPTIONAL FIXED-ORIENTATION ABLATION
        # =========================================================================
        #
        # Se configurato nello YAML, sostituiamo esclusivamente il quaternion
        # previsto da Interleave-Pi0.
        #
        # Posizione e gripper rimangono quelli prodotti dal modello.
        #
        if self._fixed_orientation_xyzw is not None:

            predicted_orientation = action[3:7].copy()

            action[3:7] = self._fixed_orientation_xyzw

            if self.cfg.trace_action_conversions:
                quaternion_dot = np.clip(
                    abs(
                        float(
                            np.dot(
                                predicted_orientation,
                                self._fixed_orientation_xyzw,
                            )
                        )
                    ),
                    0.0,
                    1.0,
                )

                orientation_difference_deg = float(
                    np.degrees(
                        2.0 * np.arccos(quaternion_dot)
                    )
                )

                print(
                    f"[InterleavePi0Trace][step={t}]"
                    f"[buffer_action={buffer_action_idx}]"
                    "[ORIENTATION_OVERRIDE]\n"
                    f"  predicted_quaternion_xyzw="
                    f"{self._format_array(predicted_orientation)}\n"
                    f"  commanded_quaternion_xyzw="
                    f"{self._format_array(self._fixed_orientation_xyzw)}\n"
                    f"  difference_deg="
                    f"{orientation_difference_deg:.9f}"
                )

        # Log della action che verrà effettivamente restituita al nodo.
        if self.cfg.trace_action_conversions:
            self._trace_buffer_action(
                step=t,
                buffer_index=buffer_action_idx,
                action=action,
            )

        # =========================================================================
        # AVANZAMENTO DEL BUFFER
        # =========================================================================

        self.action_idx += 1

        # Quando tutte le action selezionate sono state consumate,
        # invalidiamo il buffer.
        #
        # Alla chiamata successiva di inference() verrà quindi acquisita una
        # nuova osservazione e verrà effettuata una nuova query del modello.
        if self.action_idx >= len(self.action_buffer):
            self.action_buffer = None
            self.action_idx = 0

        # AIControllerNode si aspetta una lista contenente una action.
        return [[float(value) for value in action]]