from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Optional

import numpy as np
import torch
from omegaconf import DictConfig, OmegaConf
from PIL import Image as PILImage

from ai_controller.utils.ai_controller import AIController
from ai_controller.utils.utils import seed_everything


# =============================================================================
# IMPORT LOCALI
# =============================================================================

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))

if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from vla_jepa import VLAJEPARuntime
from vla_jepa_utils import (
    ACTION_DIM,
    DATASET_ACTION_SCALE,
    IMAGE_SIZE,
    build_lerobot_observation,
    delta_action_to_absolute_target,
    gripper_binary_to_moveit,
    process_front_image,
    process_gripper_image,
    recover_physical_delta,
    validate_vla_jepa_action,
)


# =============================================================================
# COSTANTI
# =============================================================================

# Anche se VLA-JEPA non usa observation.state come input del modello,
# il controller necessita della pose corrente del robot per convertire
# la delta-action predetta in un target assoluto MoveIt.
#
# Formato:
#
#   [x, y, z, qx, qy, qz, qw, gripper_closed]
#
ROBOT_STATE_DIM = 8


# =============================================================================
# UTILITY PATH
# =============================================================================


def _resolve_path(
    path: str | Path,
    base_dir: str | Path | None = None,
) -> Path:
    """
    Risolve un path espandendo ~ e variabili d'ambiente.

    Se il path è relativo e viene fornito base_dir, viene interpretato
    rispetto a base_dir.
    """
    expanded = os.path.expandvars(
        os.path.expanduser(str(path))
    )

    resolved = Path(expanded)

    if not resolved.is_absolute():
        if base_dir is None:
            resolved = Path.cwd() / resolved
        else:
            resolved = Path(base_dir) / resolved

    return resolved.resolve()


# =============================================================================
# CONTROLLER
# =============================================================================


class VLAJEPAController(AIController):
    """
    Adatta una policy VLA-JEPA/LeRobot al ciclo di controllo ROS dell'UR5e.

    Responsabilità del controller:
        - caricare il runtime LeRobot;
        - selezionare il task linguistico;
        - preprocessare le due camere UR5e;
        - costruire l'osservazione raw LeRobot;
        - richiedere una action alla policy;
        - convertire la delta-action UR5e in un target assoluto;
        - convertire il gripper nella convenzione MoveIt.


    """

    def __init__(
        self,
        model_config: str,
        task_name: str = "pick_place",
    ) -> None:

        self.task_name = task_name

        # -----------------------------------------------------------------
        # Config runtime del controller
        # -----------------------------------------------------------------

        self.config_path = _resolve_path(
            model_config
        )

        if not self.config_path.is_file():
            raise FileNotFoundError(
                f"VLA-JEPA controller config not found: "
                f"{self.config_path}"
            )

        self.config_dir = self.config_path.parent

        self.cfg: DictConfig = OmegaConf.load(
            self.config_path
        )

        # -----------------------------------------------------------------
        # Stato runtime
        # -----------------------------------------------------------------

        self._runtime: Optional[VLAJEPARuntime] = None

        self.command: Optional[str] = None
        self.current_task_id: Optional[str] = None

        # -----------------------------------------------------------------
        # Parametri controller
        # -----------------------------------------------------------------

        self.front_camera_index = int(
            self.cfg.get(
                "front_camera_index",
                0,
            )
        )

        # AIControllerNode ordina attualmente le camere come:
        #
        #   0 front
        #   1 left
        #   2 right
        #   3 gripper
        #
        self.gripper_camera_index = int(
            self.cfg.get(
                "gripper_camera_index",
                3,
            )
        )

        self.input_color_order = str(
            self.cfg.get(
                "input_color_order",
                "rgb",
            )
        ).lower()

        self.dataset_action_scale = float(
            self.cfg.get(
                "dataset_action_scale",
                DATASET_ACTION_SCALE,
            )
        )

        self.gripper_open_position = float(
            self.cfg.get(
                "gripper_open_position",
                0.0,
            )
        )

        self.gripper_closed_position = float(
            self.cfg.get(
                "gripper_closed_position",
                255.0,
            )
        )

        self.trace_action_conversions = bool(
            self.cfg.get(
                "trace_action_conversions",
                False,
            )
        )

        # ---------------------------------------------------------------
        # Random seed
        #
        # VLA-JEPA genera l'action chunk partendo da rumore casuale.
        # Impostiamo quindi esplicitamente il seed prima del caricamento
        # del modello.
        # ---------------------------------------------------------------

        self.seed = int(
            self.cfg.get(
                "seed",
                1000,
            )
        )

        seed_everything(
            self.seed
        )

        # AIController.__init__() richiama load_model().
        super().__init__(
            str(self.config_path)
        )


    # =========================================================================
    # MODEL LOADING
    # =========================================================================

    def load_model(
        self,
        model_config: str,
    ):
        """
        Carica VLA-JEPA attraverso il wrapper VLAJEPARuntime.

        Il checkpoint deve essere una directory LeRobot `pretrained_model`
        contenente almeno:
            - config.json;
            - model.safetensors;
            - policy_preprocessor.json;
            - policy_postprocessor.json;
            - relativi state safetensors.

        VLAJEPARuntime si occupa di:
            - leggere config.json;
            - costruire VLAJEPAPolicy;
            - caricare model.safetensors;
            - caricare preprocessor e postprocessor;
            - spostare la policy sul device configurato.
        """

        del model_config

        checkpoint_value = self.cfg.get(
            "checkpoint_path",
            None,
        )

        if checkpoint_value is None:
            raise KeyError(
                "VLA-JEPA config must define checkpoint_path."
            )

        checkpoint_path = _resolve_path(
            checkpoint_value,
            base_dir=self.config_dir,
        )

        if not checkpoint_path.is_dir():
            raise FileNotFoundError(
                f"VLA-JEPA checkpoint directory not found: "
                f"{checkpoint_path}"
            )

        # Verifica minima della struttura LeRobot.
        required_files = (
            "config.json",
            "model.safetensors",
            "policy_preprocessor.json",
            "policy_postprocessor.json",
        )

        missing_files = [
            filename
            for filename in required_files
            if not (checkpoint_path / filename).is_file()
        ]

        if missing_files:
            raise FileNotFoundError(
                "Incomplete VLA-JEPA pretrained_model directory. "
                f"Missing files: {missing_files}"
            )

        device = str(
            self.cfg.get(
                "device",
                "cuda",
            )
        )

        postprocessor_config_filename = str(
            self.cfg.get(
                "postprocessor_config_filename",
                "policy_postprocessor.json",
            )
        )

        self._runtime = VLAJEPARuntime(
            checkpoint_path=checkpoint_path,
            device=device,
            postprocessor_config_filename=postprocessor_config_filename,
        )

        self._runtime.load()

        # -----------------------------------------------------------------
        # Validazione del contratto specifico del checkpoint UR5e
        # -----------------------------------------------------------------

        if self._runtime.action_dim != ACTION_DIM:
            raise RuntimeError(
                "Current UR5e controller expects a 7D VLA-JEPA action, "
                f"but checkpoint action_dim={self._runtime.action_dim}."
            )

        if self._runtime.uses_state:
            raise RuntimeError(
                "This VLA-JEPA controller implementation targets the "
                "current image-only UR5e checkpoint, but the loaded "
                "checkpoint declares observation.state as an input."
            )

        resize_images_to = getattr(
            self._runtime.config,
            "resize_images_to",
            None,
        )

        if (
            resize_images_to is not None
            and tuple(resize_images_to)
            != (IMAGE_SIZE, IMAGE_SIZE)
        ):
            raise RuntimeError(
                "Image preprocessing mismatch: controller produces "
                f"{IMAGE_SIZE}x{IMAGE_SIZE}, checkpoint expects "
                f"{resize_images_to}."
            )

        print(
            "[VLAJEPAController] Model loaded successfully.\n"
            f"  checkpoint={checkpoint_path}\n"
            f"  device={device}\n"
            f"  internal_image_features="
            f"{self._runtime.model_image_feature_keys}\n"
            f"  uses_state={self._runtime.uses_state}\n"
            f"  action_dim={self._runtime.action_dim}\n"
            f"  chunk_size={self._runtime.chunk_size}\n"
            f"  n_action_steps={self._runtime.n_action_steps}"
        )

        return self._runtime.policy


    def move_model_to_device(
        self,
        device,
    ):
        """
        VLAJEPARuntime carica già policy e processor sul device configurato.

        Non spostiamo successivamente soltanto la rete perché il device
        utilizzato dalla policy deve restare coerente con quello del
        preprocessor LeRobot.
        """

        if self._runtime is None:
            raise RuntimeError(
                "VLA-JEPA runtime has not been loaded."
            )

        requested_device = torch.device(
            device
        )

        if (
            requested_device.type
            != self._runtime.device.type
        ):
            raise ValueError(
                "Cannot move only the VLA-JEPA policy to another "
                "device after processor initialization. "
                f"Runtime device={self._runtime.device}, "
                f"requested={requested_device}."
            )


    # =========================================================================
    # TASK
    # =========================================================================

    def load_command(
        self,
        demo_path: str,
        task_id: str | None = None,
        **kwargs,
    ):
        """
        Seleziona il comando linguistico associato al task.

        A differenza di Interleave-Pi0 non viene caricata alcuna
        instruction image: VLA-JEPA riceve direttamente:
            - front image;
            - gripper image;
            - istruzione linguistica.
        """

        del demo_path
        del kwargs

        if self._runtime is None:
            raise RuntimeError(
                "VLA-JEPA runtime has not been loaded."
            )

        if task_id is None:
            raise KeyError(
                "task_id is required for VLA-JEPA."
            )

        task_id = str(
            task_id
        ).zfill(2)

        tasks_cfg = self.cfg.get(
            "tasks",
            None,
        )

        if tasks_cfg is None:
            raise KeyError(
                "VLA-JEPA config does not define tasks."
            )

        task_cfg = tasks_cfg.get(
            task_id
        )

        if task_cfg is None:
            raise KeyError(
                f"Unknown VLA-JEPA task_id: {task_id}"
            )

        prompt = task_cfg.get(
            "prompt"
        )

        if not prompt:
            raise ValueError(
                f"Task {task_id} does not define a prompt."
            )

        prompt = str(
            prompt
        ).strip()

        if not prompt:
            raise ValueError(
                f"Task {task_id} defines an empty prompt."
            )

        self.current_task_id = task_id
        self.command = prompt

        # Un nuovo task non deve mai utilizzare action rimaste nella
        # queue interna della policy precedente.
        self._runtime.reset()

        print(
            f"[VLAJEPAController] Loaded task "
            f"{task_id}: {self.command!r}"
        )


    # =========================================================================
    # RESET
    # =========================================================================

    def reset(self):
        """
        Azzera lo stato runtime della traiettoria corrente.

        Il modello e i processor restano caricati, mentre:
            - viene svuotata l'action queue interna di VLA-JEPA;
            - viene eliminato il comando precedente;
            - viene eliminato il task ID precedente.
        """

        if self._runtime is not None:
            self._runtime.reset()

        self.command = None
        self.current_task_id = None


    # =========================================================================
    # PREPROCESS
    # =========================================================================

    def pre_process(
        self,
        input_data,
    ):
        """
        Costruisce l'osservazione raw fornita a VLAJEPARuntime.

        input_data:
            [images, robot_state]

        images:
            lista RGB proveniente da AIControllerNode.

            Nel setup corrente:
                images[0] = front
                images[3] = gripper

        robot_state:
            [x, y, z, qx, qy, qz, qw, gripper_closed]

        IMPORTANTE:
        robot_state NON viene fornito al modello VLA-JEPA.
        La pose viene mantenuta soltanto come riferimento per convertire
        la delta-action prevista in un target assoluto.
        """

        if (
            not isinstance(
                input_data,
                (list, tuple),
            )
            or len(input_data) != 2
        ):
            raise ValueError(
                "input_data must be [images, robot_state]."
            )

        images, robot_state = input_data

        if self._runtime is None:
            raise RuntimeError(
                "VLA-JEPA runtime has not been loaded."
            )

        if self.command is None:
            raise RuntimeError(
                "No VLA-JEPA command loaded. "
                "load_command() must be called before inference()."
            )

        # -----------------------------------------------------------------
        # Camere
        # -----------------------------------------------------------------

        if images is None:
            raise ValueError(
                "VLA-JEPA requires camera images."
            )

        required_image_index = max(
            self.front_camera_index,
            self.gripper_camera_index,
        )

        if len(images) <= required_image_index:
            raise ValueError(
                "Not enough camera images for VLA-JEPA. "
                f"Need indexes "
                f"{self.front_camera_index} and "
                f"{self.gripper_camera_index}, "
                f"received {len(images)} images."
            )

        # Camera front:
        #
        #   RGB live
        #       -> crop (0, 10, 140, 90)
        #       -> resize 224x224
        #       -> CHW float32 [0,1]
        #
        front_image = process_front_image(
            images[
                self.front_camera_index
            ],
            input_color_order=
                self.input_color_order,
        )

        # Camera gripper:
        #
        #   RGB live
        #       -> resize 224x224
        #       -> CHW float32 [0,1]
        #
        gripper_image = process_gripper_image(
            images[
                self.gripper_camera_index
            ],
            input_color_order=
                self.input_color_order,
        )

        # -----------------------------------------------------------------
        # Osservazione raw LeRobot
        # -----------------------------------------------------------------

        observation = build_lerobot_observation(
            front_image=front_image,
            gripper_image=gripper_image,
            task=self.command,
        )

        # -----------------------------------------------------------------
        # Pose robot corrente
        # -----------------------------------------------------------------

        robot_state = np.asarray(
            robot_state,
            dtype=np.float64,
        )

        if (
            robot_state.shape
            != (ROBOT_STATE_DIM,)
            or not np.all(
                np.isfinite(robot_state)
            )
        ):
            raise ValueError(
                "robot_state must be finite and shaped (8,): "
                "[x, y, z, qx, qy, qz, qw, gripper_closed]."
            )

        reference_position = (
            robot_state[:3]
            .astype(
                np.float32,
                copy=True,
            )
        )

        reference_quaternion = (
            robot_state[3:7]
            .astype(
                np.float32,
                copy=True,
            )
        )

        quaternion_norm = np.linalg.norm(
            reference_quaternion
        )

        if quaternion_norm < 1e-8:
            raise ValueError(
                "Current robot quaternion has near-zero norm."
            )

        return {
            "observation": observation,

            # Usate esclusivamente dal nostro post_process().
            "reference_position":
                reference_position,

            "reference_quaternion":
                reference_quaternion,

            # Utili per salvataggio e debug.
            "front_image":
                front_image,

            "gripper_image":
                gripper_image,

            "current_gripper_closed":
                bool(robot_state[7] >= 0.5),
        }


    # =========================================================================
    # POSTPROCESS
    # =========================================================================

    def post_process(
        self,
        output_data,
    ) -> np.ndarray:
        """
        Converte una singola action VLA-JEPA nel target assoluto UR5e.

        Pipeline:

            VLA-JEPA
                ↓
            LeRobot postprocessor
                ↓
            action[7] nello spazio del dataset
                ↓
            recupero scala fisica × 0.05 sulle prime 6 componenti
                ↓
            delta position + delta RPY
                ↓
            p_new = p_current + delta_p
            R_new = R_delta @ R_current
                ↓
            conversione gripper
                ↓
            [x, y, z, qx, qy, qz, qw, gripper_position]

        La normalizzazione/denormalizzazione NON viene effettuata qui.
        """

        required_keys = (
            "action",
            "reference_position",
            "reference_quaternion",
            "current_gripper_closed",
        )

        for key in required_keys:
            if key not in output_data:
                raise KeyError(
                    f"output_data must contain {key!r}."
                )

        action = validate_vla_jepa_action(
            output_data["action"]
        )

        # -----------------------------------------------------------------
        # Delta action -> target assoluto
        # -----------------------------------------------------------------

        (
            target_position,
            target_quaternion_xyzw,
            gripper_state,
        ) = delta_action_to_absolute_target(
            postprocessed_action=action,
            reference_position=
                output_data[
                    "reference_position"
                ],
            reference_quaternion_xyzw=
                output_data[
                    "reference_quaternion"
                ],
            currently_closed=
                bool(
                    output_data[
                        "current_gripper_closed"
                    ]
                ),
            scale_factor=
                self.dataset_action_scale,
            gripper_min=0.0,
            gripper_max=20.0,
            open_threshold=0.7,
            close_threshold=0.9,
        )

        # -----------------------------------------------------------------
        # Gripper:
        #
        # LeRobot postprocessor
        #       ↓
        # {-1,+1}
        #       ↓
        # utils
        #       ↓
        # {0=open, 1=closed}
        #       ↓
        # MoveIt/Robotiq
        #       ↓
        # {0,255}
        # -----------------------------------------------------------------

        gripper_command = (
            gripper_binary_to_moveit(
                gripper_state=gripper_state,
                open_position=
                    self.gripper_open_position,
                closed_position=
                    self.gripper_closed_position,
            )
        )

        absolute_target = np.concatenate(
            (
                target_position,
                target_quaternion_xyzw,
                np.array(
                    [gripper_command],
                    dtype=np.float32,
                ),
            )
        ).astype(
            np.float32,
            copy=False,
        )

        if absolute_target.shape != (8,):
            raise RuntimeError(
                "Unexpected VLA-JEPA absolute target shape: "
                f"{absolute_target.shape}"
            )

        if not np.all(
            np.isfinite(
                absolute_target
            )
        ):
            raise RuntimeError(
                "VLA-JEPA absolute target contains "
                f"non-finite values: {absolute_target}"
            )

        return absolute_target


    # =========================================================================
    # DEBUG / SAVING
    # =========================================================================

    @staticmethod
    def _save_tensor_image(
        image: torch.Tensor,
        output_path: Path,
    ) -> None:
        """
        Salva un'immagine CHW float32 [0,1] come PNG RGB.
        """

        if (
            not isinstance(
                image,
                torch.Tensor,
            )
            or image.shape
            != (3, IMAGE_SIZE, IMAGE_SIZE)
        ):
            raise ValueError(
                "Expected image tensor with shape "
                f"(3, {IMAGE_SIZE}, {IMAGE_SIZE})."
            )

        image_hwc = (
            image
            .detach()
            .cpu()
            .clamp(0.0, 1.0)
            .permute(1, 2, 0)
            .numpy()
        )

        image_uint8 = np.rint(
            image_hwc * 255.0
        ).astype(
            np.uint8
        )

        PILImage.fromarray(
            image_uint8,
            mode="RGB",
        ).save(
            output_path
        )


    def _save_preprocessed_images(
        self,
        front_image: torch.Tensor,
        gripper_image: torch.Tensor,
        save_path: str | Path,
    ) -> None:

        save_path = Path(
            save_path
        )

        save_path.mkdir(
            parents=True,
            exist_ok=True,
        )

        # Manteniamo pre_processed_img_0.png per compatibilità con
        # AIControllerNode, che cerca esplicitamente questo nome.
        self._save_tensor_image(
            front_image,
            save_path
            / "pre_processed_img_0.png",
        )

        self._save_tensor_image(
            gripper_image,
            save_path
            / "pre_processed_img_1.png",
        )


    @staticmethod
    def _format_array(
        value,
    ) -> str:

        if isinstance(
            value,
            torch.Tensor,
        ):
            value = (
                value
                .detach()
                .float()
                .cpu()
                .numpy()
            )

        return np.array2string(
            np.asarray(value),
            precision=7,
            suppress_small=False,
            separator=", ",
        )


    def _trace_inference(
        self,
        step: int,
        processed: dict,
        postprocessed_action,
        absolute_target,
    ) -> None:
        """
        Logga le quantità principali lungo la conversione
        VLA-JEPA -> UR5e.
        """

        action = validate_vla_jepa_action(
            postprocessed_action
        )

        physical_delta = recover_physical_delta(
            action,
            scale_factor=
                self.dataset_action_scale,
        )

        print(
            f"[VLAJEPATrace][step={step}]\n"
            f"  task_id={self.current_task_id}\n"
            f"  command={self.command!r}\n"
            f"  reference_position="
            f"{self._format_array(processed['reference_position'])}\n"
            f"  reference_quaternion_xyzw="
            f"{self._format_array(processed['reference_quaternion'])}\n"
            f"  postprocessed_action="
            f"{self._format_array(action)}\n"
            f"  physical_delta_xyz="
            f"{self._format_array(physical_delta[:3])}\n"
            f"  physical_delta_rpy_rad="
            f"{self._format_array(physical_delta[3:6])}\n"
            f"  physical_delta_rpy_deg="
            f"{self._format_array(np.degrees(physical_delta[3:6]))}\n"
            f"  model_gripper={action[6]:.7f}\n"
            f"  absolute_target="
            f"{self._format_array(absolute_target)}\n"
            f"  target_quaternion_norm="
            f"{np.linalg.norm(absolute_target[3:7]):.9f}"
        )


    # =========================================================================
    # INFERENCE
    # =========================================================================

    def inference(
        self,
        input_data,
        t: int = 0,
        save_path: str | Path | None = None,
    ):
        """
        Esegue un singolo step del controller VLA-JEPA.

        A ogni chiamata:

            1. acquisisce front + gripper correnti;
            2. prepara l'osservazione raw LeRobot;
            3. chiama VLAJEPARuntime.select_action();
            4. LeRobot restituisce una singola action;
            5. converte la delta-action nel target assoluto UR5e;
            6. restituisce una action 8D ad AIControllerNode.

        IMPORTANTE
        ----------
        Non viene implementato un action buffer nel controller.

        VLAJEPAPolicy.select_action() gestisce già internamente il chunk:
        quando la queue interna è vuota produce un nuovo chunk di 7 action,
        altrimenti restituisce la action successiva già memorizzata.
        """

        if self._runtime is None:
            raise RuntimeError(
                "VLA-JEPA controller has not been initialized."
            )

        if self.command is None:
            raise RuntimeError(
                "load_command() must be called before inference()."
            )

        # -----------------------------------------------------------------
        # 1. UR5e -> raw LeRobot observation
        # -----------------------------------------------------------------

        processed = self.pre_process(
            input_data
        )

        if save_path is not None:
            self._save_preprocessed_images(
                front_image=
                    processed[
                        "front_image"
                    ],
                gripper_image=
                    processed[
                        "gripper_image"
                    ],
                save_path=save_path,
            )

        # -----------------------------------------------------------------
        # 2. LeRobot inference
        #
        # Il runtime esegue:
        #
        #   serialized preprocessor
        #       -> VLAJEPAPolicy.select_action()
        #       -> serialized postprocessor
        #
        # Restituisce già una singola action [7] su CPU.
        # -----------------------------------------------------------------

        postprocessed_action = (
            self._runtime.select_action(
                processed[
                    "observation"
                ]
            )
        )

        # -----------------------------------------------------------------
        # 3. Delta action -> absolute UR5e target
        # -----------------------------------------------------------------

        absolute_target = self.post_process(
            {
                "action":
                    postprocessed_action,

                "reference_position":
                    processed[
                        "reference_position"
                    ],

                "reference_quaternion":
                    processed[
                        "reference_quaternion"
                    ],

                "current_gripper_closed":
                    processed["current_gripper_closed"],
            }
        )

        if self.trace_action_conversions:
            self._trace_inference(
                step=t,
                processed=processed,
                postprocessed_action=
                    postprocessed_action,
                absolute_target=
                    absolute_target,
            )

        # AIControllerNode si aspetta una lista di action.
        return [
            [
                float(value)
                for value
                in absolute_target
            ]
        ]