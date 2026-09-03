from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Literal

import torch
import transformer_engine.pytorch.attention as te_attention

# Transformer Engine 2.3+ moved RoPE helpers into the ``attention.rope``
# module, while the official Mimic Video sources import the legacy namespace.
if not hasattr(te_attention, "apply_rotary_pos_emb"):
    from transformer_engine.pytorch.attention.rope import apply_rotary_pos_emb

    setattr(te_attention, "apply_rotary_pos_emb", apply_rotary_pos_emb)

from cosmos_predict2.configs.config import make_config
from cosmos_predict2.data.action.utils import extract_normalization_types
from cosmos_predict2.pipelines.video2world import Video2WorldPipeline
from cosmos_predict2.pipelines.video2world2action import Video2World2ActionPipeline
from cosmos_predict2.pipelines.world2action import World2ActionPipeline
from imaginaire.lazy_config import instantiate
from imaginaire.utils.config_helper import override


BATCH_SIZE = 1
NUM_INPUT_FRAMES = 5
VIDEO_HEIGHT = 480
VIDEO_WIDTH = 640

STATE_HORIZON = 1
#STATE_DIM = 10
STATE_DIM = 7
ACTION_HORIZON = 15
#ACTION_DIM = 10
ACTION_DIM = 7

PROMPT_TOKENS = 512
PROMPT_EMBED_DIM = 1024
NUM_SAMPLING_STEPS = 35
STOP_AFTER_STEP = 0

LanguageConditioning = Literal["precomputed", "runtime_t5"]
LANGUAGE_CONDITIONING_MODES = {"precomputed", "runtime_t5"}


def _resolve_path(path: str | Path) -> Path:
    """Expand environment variables and user directories in a configured path."""
    expanded = os.path.expanduser(os.path.expandvars(str(path)))
    if "$" in expanded:
        raise EnvironmentError(f"Unresolved environment variable in path: {path}")
    return Path(expanded)


def load_mimic_video_pipeline(
    experiment_name: str,
    video_model_path: str | Path,
    action_model_path: str | Path,
    dataset_statistics_path: str | Path,
    language_conditioning: LanguageConditioning = "precomputed",
    text_encoder_path: str | Path | None = None,
    offload_text_encoder: bool = True,
    downcast_text_encoder: bool = True,
    dtype: torch.dtype = torch.bfloat16,
) -> Video2World2ActionPipeline:
    """Load the official Mimic Video pipeline with the UR5e normalizer."""
    if language_conditioning not in LANGUAGE_CONDITIONING_MODES:
        raise ValueError(
            f"language_conditioning must be one of {sorted(LANGUAGE_CONDITIONING_MODES)}, "
            f"got {language_conditioning!r}"
        )

    video_model_path = _resolve_path(video_model_path)
    action_model_path = _resolve_path(action_model_path)
    dataset_statistics_path = _resolve_path(dataset_statistics_path)

    for path in (video_model_path, action_model_path, dataset_statistics_path):
        if not path.is_file():
            raise FileNotFoundError(path)

    config = override(
        make_config(),
        ["--", f"experiment={experiment_name}"],
    )
    video_pipe_config = config.model.config.video_pipe_config
    video_pipe_config.guardrail_config.enabled = False

    use_text_encoder = language_conditioning == "runtime_t5"
    if use_text_encoder:
        if text_encoder_path is None:
            raise ValueError("text_encoder_path is required when language_conditioning='runtime_t5'.")
        text_encoder_path = _resolve_path(text_encoder_path)
        if not text_encoder_path.is_dir():
            raise FileNotFoundError(text_encoder_path)
        video_pipe_config.text_encoder.t5.ckpt_path = str(text_encoder_path)

    video_pipe = Video2WorldPipeline.from_config(
        config=video_pipe_config,
        dit_path=str(video_model_path),
        use_text_encoder=use_text_encoder,
        offload_text_encoder=offload_text_encoder,
        downcast_text_encoder=downcast_text_encoder,
        device="cuda",
        torch_dtype=dtype,
        load_ema_to_reg=False,
    )
    action_pipe = World2ActionPipeline.from_config(
        config.model.config.pipe_config,
        dit_path=str(action_model_path),
        device="cuda",
        dtype=dtype,
    )

    data_config = instantiate(config.data_config)
    with dataset_statistics_path.open("rb") as stats_file:
        stats = json.load(stats_file)

    action_pipe.normalizer.build_from_stats(
        stats,
        normalization_types=extract_normalization_types(data_config.policy_io.policy_io),
        concat_groups=data_config.policy_io.concat_groups,
        device="cuda",
        dtype=dtype,
    )

    return Video2World2ActionPipeline(video_pipe, action_pipe).cuda()


class MimicVideoPolicy:
    """Load Mimic Video and return one complete UR5e action chunk per query."""

    def __init__(
        self,
        experiment_name: str,
        video_model_path: str | Path,
        action_model_path: str | Path,
        dataset_statistics_path: str | Path,
        language_conditioning: LanguageConditioning = "precomputed",
        text_encoder_path: str | Path | None = None,
        offload_text_encoder: bool = True,
        downcast_text_encoder: bool = True,
        dtype: torch.dtype = torch.bfloat16,
    ) -> None:
        if not torch.cuda.is_available():
            raise RuntimeError("Mimic Video inference requires a CUDA device.")

        self.dtype = dtype
        self.language_conditioning = language_conditioning
        self.model = load_mimic_video_pipeline(
            experiment_name=experiment_name,
            video_model_path=video_model_path,
            action_model_path=action_model_path,
            dataset_statistics_path=dataset_statistics_path,
            language_conditioning=language_conditioning,
            text_encoder_path=text_encoder_path,
            offload_text_encoder=offload_text_encoder,
            downcast_text_encoder=downcast_text_encoder,
            dtype=dtype,
        )
        self.model.eval()
        self.model.requires_grad_(False)

    @torch.inference_mode()
    def encode_command(self, command: str) -> torch.Tensor:
        """Encode one command with the same T5 path used by the official precompute script."""
        if self.language_conditioning != "runtime_t5":
            raise RuntimeError("encode_command is available only with language_conditioning='runtime_t5'.")
        command = command.strip()
        if not command:
            raise ValueError("command must not be empty.")

        embedding = self.model.video2world_pipeline.encode_prompt(
            command,
            max_length=PROMPT_TOKENS,
            return_mask=False,
        )
        expected_prompt = (BATCH_SIZE, PROMPT_TOKENS, PROMPT_EMBED_DIM)
        if tuple(embedding.shape) != expected_prompt:
            raise RuntimeError(
                f"T5 returned shape {tuple(embedding.shape)}, expected {expected_prompt}"
            )
        return embedding.detach()

    @staticmethod
    def _validate_inputs(
        input_vid: torch.Tensor,
        state: torch.Tensor,
        prompt_embedding: torch.Tensor,
    ) -> None:
        expected_video = (BATCH_SIZE, 3, NUM_INPUT_FRAMES, VIDEO_HEIGHT, VIDEO_WIDTH)
        expected_state = (BATCH_SIZE, STATE_HORIZON, STATE_DIM)
        expected_prompt = (BATCH_SIZE, PROMPT_TOKENS, PROMPT_EMBED_DIM)

        if tuple(input_vid.shape) != expected_video:
            raise ValueError(f"input_vid must have shape {expected_video}, got {tuple(input_vid.shape)}")
        if tuple(state.shape) != expected_state:
            raise ValueError(f"state must have shape {expected_state}, got {tuple(state.shape)}")
        if tuple(prompt_embedding.shape) != expected_prompt:
            raise ValueError(
                f"prompt_embedding must have shape {expected_prompt}, got {tuple(prompt_embedding.shape)}"
            )
        if not torch.is_floating_point(input_vid):
            raise TypeError("input_vid must be a floating-point tensor normalized to [-1, 1].")
        if not torch.is_floating_point(state):
            raise TypeError("state must be a floating-point tensor.")
        if not torch.is_floating_point(prompt_embedding):
            raise TypeError("prompt_embedding must be a floating-point tensor.")

    @torch.inference_mode()
    def predict(
        self,
        input_vid: torch.Tensor,
        state: torch.Tensor,
        command: str,
        prompt_embedding: torch.Tensor,
        seed: int = 0,
        use_cuda_graphs: bool = False,
    ) -> torch.Tensor:
        """Return denormalized actions shaped ``(1, 15, 10)``."""
        self._validate_inputs(input_vid, state, prompt_embedding)

        input_vid = input_vid.to(device="cuda", dtype=self.dtype)
        state = state.to(device="cuda", dtype=self.dtype)
        prompt_embedding = prompt_embedding.to(device="cuda", dtype=self.dtype)

        action_chunk = self.model(
            input_vid=input_vid,
            state_B_HO_O=state,
            prompt=command,
            prompt_embedding=prompt_embedding,
            num_sampling_step=NUM_SAMPLING_STEPS,
            stop_after_step=STOP_AFTER_STEP,
            seed=seed,
            use_cuda_graphs=use_cuda_graphs,
        )

        expected_output = (BATCH_SIZE, ACTION_HORIZON, ACTION_DIM)
        if tuple(action_chunk.shape) != expected_output:
            raise RuntimeError(
                f"Mimic Video returned shape {tuple(action_chunk.shape)}, expected {expected_output}"
            )
        return action_chunk
