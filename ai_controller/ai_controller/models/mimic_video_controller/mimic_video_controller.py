from __future__ import annotations

import os
import sys
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np
import torch
import yaml
from PIL import Image as PILImage
from safetensors.torch import load_file

from ai_controller.utils.ai_controller import AIController

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from mimic_video import ACTION_HORIZON, MimicVideoPolicy, _resolve_path
from mimic_video_utils import (
    action_chunk_to_absolute_poses,
    build_lowdim_state,
    process_rgb_image,
)


NUM_INPUT_FRAMES = 5
ROBOT_STATE_DIM = 8
PROMPT_SHAPE = (1, 512, 1024)

@dataclass
class MimicVideoConfig:
    experiment_name: str
    video_model_path: str
    action_model_path: str
    dataset_statistics_path: str
    language_conditioning: str = "precomputed"
    precomputed_embeddings_dir: str = ""
    precomputed_embeddings: dict[str, str] = field(default_factory=dict)
    text_encoder_path: str = ""
    offload_text_encoder: bool = True
    downcast_text_encoder: bool = True
    seed: int = 0
    use_cuda_graphs: bool = False
    num_execute_actions: int = 10
    trace_action_conversions: bool = False


class MimicVideoController(AIController):
    """Adatta Mimic Video al ciclo di controllo del nodo ROS per UR5e."""

    def __init__(self, model_config: str, task_name: str = "pick_place") -> None:
        self.task_name = task_name
        self.cfg: Optional[MimicVideoConfig] = None
        self._policy: Optional[MimicVideoPolicy] = None

        self.image_history: deque[np.ndarray] = deque(maxlen=NUM_INPUT_FRAMES)
        self.action_buffer: Optional[np.ndarray] = None
        self.action_idx = 0
        self.command: Optional[str] = None
        self.prompt_embedding: Optional[torch.Tensor] = None
        self.gripper_closed = False
        self._embedding_cache: dict[str, torch.Tensor] = {}

        super().__init__(model_config)

    def load_model(self, model_config: str):
        """Legge lo YAML e costruisce le pipeline ufficiali Video2World e World2Action."""
        with open(model_config, "r", encoding="utf-8") as config_file:
            config_dict = yaml.safe_load(config_file)

        if not isinstance(config_dict, dict):
            raise ValueError(f"Invalid Mimic Video configuration: {model_config}")

        valid_fields = MimicVideoConfig.__dataclass_fields__
        unknown_fields = set(config_dict) - set(valid_fields)
        if unknown_fields:
            raise ValueError(f"Unknown Mimic Video configuration fields: {sorted(unknown_fields)}")

        self.cfg = MimicVideoConfig(**config_dict)
        if self.cfg.language_conditioning not in {"precomputed", "runtime_t5"}:
            raise ValueError("language_conditioning must be 'precomputed' or 'runtime_t5'.")
        if not 1 <= self.cfg.num_execute_actions <= ACTION_HORIZON:
            raise ValueError(
                f"num_execute_actions must be between 1 and {ACTION_HORIZON}, "
                f"got {self.cfg.num_execute_actions}"
            )

        self._policy = MimicVideoPolicy(
            experiment_name=self.cfg.experiment_name,
            video_model_path=self.cfg.video_model_path,
            action_model_path=self.cfg.action_model_path,
            dataset_statistics_path=self.cfg.dataset_statistics_path,
            language_conditioning=self.cfg.language_conditioning,
            text_encoder_path=self.cfg.text_encoder_path or None,
            offload_text_encoder=self.cfg.offload_text_encoder,
            downcast_text_encoder=self.cfg.downcast_text_encoder,
        )
        return self._policy.model

    def move_model_to_device(self, device):
        """La policy carica gia entrambe le pipeline su CUDA."""
        if torch.device(device).type != "cuda":
            raise ValueError("Mimic Video supports only a CUDA device.")

    def load_command(self, demo_path: str, task_id: str | None = None, **kwargs):
        """Seleziona il comando e prepara una sola volta il relativo embedding T5."""
        del demo_path
        
        if self.cfg.language_conditioning == "runtime_t5":
            command = kwargs.get("command")
            if command is None:
                raise KeyError(f"Command not found")
            command = str(command).strip()
            if not command:
                raise ValueError("The Mimic Video command must not be empty.")
            self.command = command
            
            cache_key = f"runtime_t5:{command}"
            if cache_key not in self._embedding_cache:
                self._embedding_cache[cache_key] = self._policy.encode_command(command).float().cpu().contiguous()
            self.prompt_embedding = self._embedding_cache[cache_key]
            return


        if task_id is None:
            raise KeyError(f"Task ID not found")
        
        task_id = str(task_id).zfill(2)
        embedding_name = self.cfg.precomputed_embeddings.get(task_id)
        if embedding_name is None:
            raise KeyError(f"No precomputed embedding configured for task ID {task_id}")

        embedding_path = _resolve_path(self.cfg.precomputed_embeddings_dir) / embedding_name
        cache_key = f"precomputed:{embedding_path}"
        if cache_key not in self._embedding_cache:
            if not embedding_path.is_file():
                raise FileNotFoundError(embedding_path)
            tensors = load_file(str(embedding_path), device="cpu")
            if "encoded_text" not in tensors:
                raise KeyError(f"Missing 'encoded_text' in {embedding_path}")
            embedding = tensors["encoded_text"].float()
            if embedding.ndim == 2:
                embedding = embedding.unsqueeze(0)
            if tuple(embedding.shape) != PROMPT_SHAPE:
                raise ValueError(
                    f"Unexpected embedding shape in {embedding_path}: "
                    f"{tuple(embedding.shape)}, expected {PROMPT_SHAPE}"
                )
            self._embedding_cache[cache_key] = embedding.contiguous()
        self.prompt_embedding = self._embedding_cache[cache_key]

    def reset(self):
        """Azzera lo stato temporale della traiettoria senza ricaricare i modelli."""
        self.image_history.clear()
        self.action_buffer = None
        self.action_idx = 0
        self.command = None
        self.prompt_embedding = None
        self.gripper_closed = False

    def pre_process(self, input_data):
        """Aggiorna la storia RGB e costruisce lo stato propriocettivo 10D."""
        if not isinstance(input_data, (list, tuple)) or len(input_data) != 2:
            raise ValueError("input_data must be [images, robot_state].")

        images, robot_state = input_data
        if not images:
            raise ValueError("Mimic Video requires the front camera image.")

        robot_state = np.asarray(robot_state, dtype=np.float64)
        if robot_state.shape != (ROBOT_STATE_DIM,) or not np.all(np.isfinite(robot_state)):
            raise ValueError(
                "robot_state must be finite and shaped (8,): "
                "[x, y, z, qx, qy, qz, qw, gripper_closed]."
            )

        processed_frame = process_rgb_image(images[0], input_color_order="rgb")
        self.image_history.append(processed_frame)

        gripper_state = float(robot_state[7])
        self.gripper_closed = gripper_state >= 0.5
        lowdim_state = build_lowdim_state(
            eef_position=robot_state[:3],
            eef_quaternion=robot_state[3:7],
            gripper_state=gripper_state,
        )

        video = None
        if len(self.image_history) == NUM_INPUT_FRAMES:
            video = torch.from_numpy(np.concatenate(tuple(self.image_history), axis=1)[None])

        return {
            "video": video,
            "state": torch.from_numpy(lowdim_state[None, None]),
            "reference_position": robot_state[:3].copy(),
            "reference_quaternion": robot_state[3:7].copy(),
            "gripper_closed": self.gripper_closed,
            "processed_frame": processed_frame,
        }

    def post_process(self, output_data):
        """Converte l'intero chunk 10D in target assoluti 8D nel frame base_link."""
        action_chunk = output_data["action_chunk"]
        if isinstance(action_chunk, torch.Tensor):
            action_chunk = action_chunk.detach().float().cpu().numpy()

        absolute_chunk, _ = action_chunk_to_absolute_poses(
            action_chunk=action_chunk,
            current_position=output_data["reference_position"],
            current_quaternion=output_data["reference_quaternion"],
            gripper_closed=output_data["gripper_closed"],
            diagnostics_callback=(
                lambda diagnostics: self._trace_action_conversion(
                    output_data["query_step"], diagnostics
                )
            )
            if self.cfg.trace_action_conversions
            else None,
        )
        return absolute_chunk

    @staticmethod
    def _format_array(value) -> str:
        return np.array2string(
            np.asarray(value),
            precision=7,
            suppress_small=False,
            separator=", ",
        )

    def _trace_model_input(self, query_step: int, processed: dict) -> None:
        lowdim_state = processed["state"].detach().float().cpu().numpy()[0, 0]
        print(
            f"[MimicVideoTrace][query={query_step}][INPUT]\n"
            f"  reference_position={self._format_array(processed['reference_position'])}\n"
            f"  reference_quaternion_xyzw={self._format_array(processed['reference_quaternion'])} "
            f"norm={np.linalg.norm(processed['reference_quaternion']):.9f}\n"
            f"  lowdim_state_10d={self._format_array(lowdim_state)}\n"
            f"  gripper_closed={processed['gripper_closed']}"
        )

    def _trace_action_conversion(self, query_step: int, diagnostics: dict[str, object]) -> None:
        index = diagnostics["index"]
        raw_action = np.asarray(diagnostics["raw_action"])
        print(
            f"[MimicVideoTrace][query={query_step}][action={index}][MODEL_OUTPUT_DENORMALIZED]\n"
            f"  action_10d={self._format_array(raw_action)}\n"
            f"  delta_xyz={self._format_array(raw_action[:3])}\n"
            f"  rotation_6d={self._format_array(raw_action[3:9])}\n"
            f"  gripper_model={raw_action[9]:.9f}"
        )
        print(
            f"[MimicVideoTrace][query={query_step}][action={index}][ROT6D_TO_MATRIX]\n"
            f"  raw_row_norms={self._format_array(diagnostics['raw_rotation_row_norms'])} "
            f"raw_row_dot={diagnostics['raw_rotation_row_dot']:.9f}\n"
            f"  delta_rotation=\n{self._format_array(diagnostics['delta_rotation'])}\n"
            f"  determinant={diagnostics['delta_rotation_determinant']:.9f} "
            f"orthogonality_error={diagnostics['delta_rotation_orthogonality_error']:.3e}\n"
            f"  delta_euler_xyz_deg={self._format_array(diagnostics['delta_euler_xyz_deg'])} "
            f"delta_angle_deg={diagnostics['delta_angle_deg']:.7f}"
        )
        print(
            f"[MimicVideoTrace][query={query_step}][action={index}][COMPOSE]\n"
            f"  previous_position={self._format_array(diagnostics['previous_position'])}\n"
            f"  target_position={self._format_array(diagnostics['target_position'])}\n"
            f"  previous_quaternion_xyzw={self._format_array(diagnostics['previous_quaternion'])}\n"
            f"  previous_rotation=\n{self._format_array(diagnostics['previous_rotation'])}\n"
            f"  target_rotation=delta_rotation@previous_rotation=\n"
            f"{self._format_array(diagnostics['target_rotation'])}\n"
            f"  target_euler_xyz_deg={self._format_array(diagnostics['target_euler_xyz_deg'])}\n"
            f"  target_quaternion_xyzw={self._format_array(diagnostics['target_quaternion'])} "
            f"norm={np.linalg.norm(diagnostics['target_quaternion']):.9f}"
        )
        print(
            f"[MimicVideoTrace][query={query_step}][action={index}][GRIPPER]\n"
            f"  was_closed={diagnostics['gripper_was_closed']} "
            f"is_closed={diagnostics['gripper_is_closed']} "
            f"command={diagnostics['gripper_command']}"
        )

    @staticmethod
    def _processed_frame_to_uint8(processed_frame: np.ndarray) -> np.ndarray:
        """Converte un frame normalizzato Cx1xHxW in un'immagine RGB visualizzabile."""
        processed_frame = np.asarray(processed_frame)
        if processed_frame.ndim != 4 or processed_frame.shape[:2] != (3, 1):
            raise ValueError(
                "Expected a processed frame with shape (3, 1, H, W), "
                f"got {processed_frame.shape}"
            )

        image = np.moveaxis(processed_frame[:, 0], 0, -1)
        return np.clip((image + 1.0) * 127.5, 0, 255).astype(np.uint8)

    @classmethod
    def _save_processed_frame(cls, processed_frame: np.ndarray, save_path: str | Path) -> None:
        """Salva l'ultimo frame realmente fornito al modello per il debug del nodo."""
        save_path = Path(save_path)
        save_path.mkdir(parents=True, exist_ok=True)
        PILImage.fromarray(cls._processed_frame_to_uint8(processed_frame)).save(
            save_path / "pre_processed_img_0.png"
        )

    def _save_input_history(self, query_step: int, save_path: str | Path) -> Path:
        """Salva, dal piu vecchio al piu recente, i cinque frame usati dalla query."""
        if len(self.image_history) != NUM_INPUT_FRAMES:
            raise RuntimeError(
                f"Cannot save a {NUM_INPUT_FRAMES}-frame history: "
                f"only {len(self.image_history)} frames are available."
            )

        history_root = Path(save_path).parent / "mimic_video_input_history"
        query_path = history_root / f"query_step_{query_step:06d}"
        query_path.mkdir(parents=True, exist_ok=True)

        for frame_index, frame in enumerate(self.image_history):
            suffix = ""
            if frame_index == 0:
                suffix = "_oldest"
            elif frame_index == NUM_INPUT_FRAMES - 1:
                suffix = "_newest"
            frame_name = f"frame_{frame_index:02d}{suffix}.png"
            PILImage.fromarray(self._processed_frame_to_uint8(frame)).save(
                query_path / frame_name
            )

        return query_path

    def inference(self, input_data, t: int = 0, save_path: str | Path | None = None):
        """Restituisce una sola azione del buffer e rigenera il chunk ogni K azioni."""
        if self.prompt_embedding is None:
            raise RuntimeError("load_command() must be called before inference().")

        if self.cfg.language_conditioning == "runtime_t5" and self.command is None:
            raise RuntimeError("A text command is required with runtime_t5.")

        processed = self.pre_process(input_data)
        if save_path is not None:
            self._save_processed_frame(processed["processed_frame"], save_path)

        if processed["video"] is None:
            raise RuntimeError(
                f"Mimic Video has received {len(self.image_history)}/{NUM_INPUT_FRAMES} real frames. "
                "Complete the ROS warm-up before inference."
            )

        if self.action_buffer is None:
            if save_path is not None:
                history_path = self._save_input_history(t, save_path)
                print(
                    f"[MimicVideoController] Saved {NUM_INPUT_FRAMES} model input frames "
                    f"for query t={t} to {history_path}"
                )
            if self.cfg.trace_action_conversions:
                self._trace_model_input(t, processed)
            raw_chunk = self._policy.predict(
                input_vid=processed["video"],
                state=processed["state"],
                command=self.command or "",
                prompt_embedding=self.prompt_embedding,
                seed=self.cfg.seed,
                use_cuda_graphs=self.cfg.use_cuda_graphs,
            )
            absolute_chunk = self.post_process(
                {
                    "action_chunk": raw_chunk,
                    "reference_position": processed["reference_position"],
                    "reference_quaternion": processed["reference_quaternion"],
                    "gripper_closed": processed["gripper_closed"],
                    "query_step": t,
                }
            )
            self.action_buffer = absolute_chunk[: self.cfg.num_execute_actions].copy()
            self.action_idx = 0
            print(
                f"[MimicVideoController] Inference t={t}: generated {ACTION_HORIZON} actions, "
                f"buffered {len(self.action_buffer)}"
            )

        action = self.action_buffer[self.action_idx].copy()
        self.action_idx += 1

        if self.action_idx == len(self.action_buffer):
            self.action_buffer = None
            self.action_idx = 0

        return [[float(value) for value in action]]
