from __future__ import annotations

import json
from pathlib import Path

import torch

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
STATE_DIM = 10
ACTION_HORIZON = 15
ACTION_DIM = 10

PROMPT_TOKENS = 512
PROMPT_EMBED_DIM = 1024
NUM_SAMPLING_STEPS = 35
STOP_AFTER_STEP = 0


def load_mimic_video_pipeline(
    experiment_name: str,
    video_model_path: str | Path,
    action_model_path: str | Path,
    dataset_statistics_path: str | Path,
    dtype: torch.dtype = torch.bfloat16,
) -> Video2World2ActionPipeline:
    """Load the official Mimic Video pipeline with the UR5e normalizer."""
    video_model_path = Path(video_model_path)
    action_model_path = Path(action_model_path)
    dataset_statistics_path = Path(dataset_statistics_path)

    for path in (video_model_path, action_model_path, dataset_statistics_path):
        if not path.is_file():
            raise FileNotFoundError(path)

    config = override(
        make_config(),
        ["--", f"experiment={experiment_name}"],
    )
    config.model.config.video_pipe_config.guardrail_config.enabled = False

    video_pipe = Video2WorldPipeline.from_config(
        config=config.model.config.video_pipe_config,
        dit_path=str(video_model_path),
        use_text_encoder=False,
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
        dtype: torch.dtype = torch.bfloat16,
    ) -> None:
        if not torch.cuda.is_available():
            raise RuntimeError("Mimic Video inference requires a CUDA device.")

        self.dtype = dtype
        self.model = load_mimic_video_pipeline(
            experiment_name=experiment_name,
            video_model_path=video_model_path,
            action_model_path=action_model_path,
            dataset_statistics_path=dataset_statistics_path,
            dtype=dtype,
        )
        self.model.eval()
        self.model.requires_grad_(False)

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
        stop_after_step: int = STOP_AFTER_STEP,
        seed: int = 0,
        use_cuda_graphs: bool = True,
    ) -> torch.Tensor:
        """Return denormalized actions shaped ``(1, 15, 10)``."""
        self._validate_inputs(input_vid, state, prompt_embedding)
        if not 0 <= stop_after_step <= NUM_SAMPLING_STEPS:
            raise ValueError(
                f"stop_after_step must be between 0 and {NUM_SAMPLING_STEPS}, got {stop_after_step}"
            )

        input_vid = input_vid.to(device="cuda", dtype=self.dtype)
        state = state.to(device="cuda", dtype=self.dtype)
        prompt_embedding = prompt_embedding.to(device="cuda", dtype=self.dtype)

        action_chunk = self.model(
            input_vid=input_vid,
            state_B_HO_O=state,
            prompt=command,
            prompt_embedding=prompt_embedding,
            num_sampling_step=NUM_SAMPLING_STEPS,
            stop_after_step=stop_after_step,
            seed=seed,
            use_cuda_graphs=use_cuda_graphs,
        )

        expected_output = (BATCH_SIZE, ACTION_HORIZON, ACTION_DIM)
        if tuple(action_chunk.shape) != expected_output:
            raise RuntimeError(
                f"Mimic Video returned shape {tuple(action_chunk.shape)}, expected {expected_output}"
            )
        return action_chunk
