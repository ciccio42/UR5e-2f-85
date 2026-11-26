"""World-Model-conditioned policy inference for VLA-JEPA on UR5e.

No LeRobot source changes. This module reuses VLAJEPAPolicy.select_action's
ORIGINAL queue logic by overriding ONLY predict_action_chunk(). The baseline
VLAJEPARuntime is unchanged apart from two protected factory hooks.

Aligned with the 2026-11 action_head_wm_conditioned *training package*:
- Qwen current front+gripper and language, same forward for both readout types;
- observed front/gripper image pairs (t-1,t), t=0 duplicates current;
- frozen V-JEPA2 and Predictor: S0 + first 8 Qwen action tokens -> S1_hat;
- concat([32 z_embodiment, 256 S1_hat]) -> [B,288,2048];
- original action_model.predict_action() performs unchanged flow matching;
- BF16 trained embedding row is already in model.safetensors from export.
"""
from __future__ import annotations

from pathlib import Path

import torch
from safetensors import safe_open
from torch import Tensor

from lerobot.policies.utils import populate_queues
from lerobot.utils.constants import ACTION
from lerobot.policies.vla_jepa.modeling_vla_jepa import VLAJEPAPolicy

from vla_jepa import VLAJEPARuntime
from vla_jepa_wm_conditioning import build_observed_pair, build_wm_conditioning


class WMConditionedVLAJEPAPolicy(VLAJEPAPolicy):
    """Only chunk generation is replaced; inherited select_action manages queue."""

    def reset(self) -> None:
        super().reset()
        self._wm_observed_video: Tensor | None = None
        self._wm_chunks_generated = 0
        self._wm_last_video_shape: tuple[int, ...] | None = None
        self._wm_last_conditioning_shape: tuple[int, ...] | None = None

    def set_observed_video(self, video: Tensor) -> None:
        if self._wm_observed_video is not None:
            raise RuntimeError("An observation was not cleared after previous inference")
        self._wm_observed_video = video

    def clear_observed_video(self) -> None:
        self._wm_observed_video = None

    @torch.no_grad()
    def predict_action_chunk(self, batch: dict[str, Tensor], noise: Tensor | None = None) -> Tensor:
        """Drop-in override of the native 7-action chunk generator.

        Everything else, including the double `populate_queues` behavior in
        native VLAJEPAPolicy, remains exactly as in the original source.
        """
        self.eval()
        self._queues = populate_queues(self._queues, batch, exclude_keys=[ACTION])
        if self._wm_observed_video is None:
            raise RuntimeError("Missing 2-frame observed video before WM chunk generation")
        inputs = self._prepare_model_inputs(batch, training=False)
        device = self.model.video_encoder.device
        observed_video = self._wm_observed_video.to(device=device, non_blocking=True)
        conditioning = build_wm_conditioning(
            self.model, inputs["images"], inputs["instructions"], observed_video,
        )
        self._wm_last_video_shape = tuple(observed_video.shape)
        self._wm_last_conditioning_shape = tuple(conditioning.shape)
        self._wm_chunks_generated += 1
        state = inputs.get("state")
        actions = self.model.action_model.predict_action(
            conditioning.float(), state.float() if state is not None else None,
        )
        return actions.to(device=self.config.device, dtype=torch.float32)


class VLAJEPAWMRuntime(VLAJEPARuntime):
    """Track image history at each controller step; predict WM only at chunk boundaries."""

    RAW_IMAGE_KEYS = ("observation.images.front", "observation.images.gripper")

    def __init__(self, checkpoint_path: str | Path, device: str = "cuda",
                 postprocessor_config_filename: str = "policy_postprocessor.json",
                 log_chunk_inference_time: bool = False) -> None:
        super().__init__(checkpoint_path, device=device,
                         postprocessor_config_filename=postprocessor_config_filename,
                         log_chunk_inference_time=log_chunk_inference_time)
        self._previous_views: tuple[Tensor, Tensor] | None = None

    def _configure_model_config(self, config) -> None:
        # VLAJEPAConfig.__post_init__ switches OFF the WM when freeze_qwen=True.
        # The already-loaded checkpoint has trained weights for all components;
        # freeze all parameters AFTER constructing the full model.
        config.freeze_qwen = False
        config.enable_world_model = True
        config.causal_world_model_context = True
        config.reinit_modules = None
        # Mirrors the exact stage-2 WM-conditioned training loader.
        config.resize_images_to = None

        expected_image_keys = {
            "observation.images.exterior_1_left",
            "observation.images.exterior_2_left",
        }
        if set(config.image_features) != expected_image_keys:
            raise ValueError(f"Wrong image features: {list(config.image_features)}")
        if (config.action_dim, config.chunk_size, config.n_action_steps) != (7, 7, 7):
            raise ValueError("This deployment requires 7D actions and a seven-action queue")
        if config.num_video_frames != 8 or config.num_world_model_views != 2:
            raise ValueError("WM checkpoint must use 8 training frames and 2 video views")
        if config.num_action_tokens_per_timestep != 8:
            raise ValueError("WM checkpoint must use 8 Qwen action tokens per timestep")
        if config.num_embodied_action_tokens_per_instruction != 32:
            raise ValueError("WM checkpoint must have 32 embodiment-token readouts")
        if config.jepa_tubelet_size != 2:
            raise ValueError("This training requires V-JEPA2 tubelet_size=2")
        if getattr(config, "use_peft", False):
            raise ValueError("WM-conditioned stage 2 requires fully exported Qwen weights")
        for field in ("use_relative_actions", "pre_snap_gripper_action",
                      "binarize_gripper_action", "clip_normalized_actions"):
            if bool(getattr(config, field, False)):
                raise ValueError(f"Expected {field}=False, as in training")

    def _load_policy(self, config):
        # For the new experiment, no silently missing video_predictor parameters.
        path = Path(self.checkpoint_path)
        with safe_open(path / "model.safetensors", framework="pt", device="cpu") as weights:
            keys = weights.keys()
            for prefix in (
                "model.qwen.", "model.action_model.",
                "model.video_encoder.", "model.video_predictor.",
            ):
                if not any(key.startswith(prefix) for key in keys):
                    raise RuntimeError(f"WM-conditioned checkpoint missing {prefix}")

        policy = WMConditionedVLAJEPAPolicy.from_pretrained(
            self.checkpoint_path, config=config, strict=True,
        )
        if not isinstance(policy, WMConditionedVLAJEPAPolicy):
            raise RuntimeError("LeRobot did not instantiate the WM policy subclass")
        policy.requires_grad_(False)
        if policy.model.video_encoder is None or policy.model.video_predictor is None:
            raise RuntimeError("Failed to instantiate V-JEPA2 or World Predictor")
        if policy.model.video_processor is None:
            raise RuntimeError("V-JEPA2 video processor unavailable")
        if policy.model.video_encoder.config.tubelet_size != 2:
            raise RuntimeError("Loaded V-JEPA2 has wrong tubelet_size")
        return policy

    def reset(self) -> None:
        super().reset()
        self._previous_views = None

    def select_action(self, observation: dict) -> Tensor:
        # Raw CHW tensors are already 224x224 RGB [0,1]. Reusing these exact
        # processed images for the video stream reproduces the training input.
        for key in self.RAW_IMAGE_KEYS:
            if key not in observation:
                raise KeyError(f"WM mode requires raw key {key}")
        current = (observation[self.RAW_IMAGE_KEYS[0]], observation[self.RAW_IMAGE_KEYS[1]])
        video = build_observed_pair(self._previous_views, current)
        policy: WMConditionedVLAJEPAPolicy = self.policy
        policy.set_observed_video(video)
        try:
            action = super().select_action(observation)
        finally:
            policy.clear_observed_video()
        # Commit history only after a successful inference. This executes
        # EVEN when LeRobot's action queue returns a cached action.
        self._previous_views = tuple(img.detach().cpu().clone() for img in current)
        return action
