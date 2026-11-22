"""Shared, dependency-light inference math for the frozen WM-conditioned DiT.

Exactly reproduces the conditioning branch of
`src/action_head_wm_conditioned/policy.py::WMPredictiveFlowMatchingLoss`.

Input video convention: [B, V=2, T=2, C=3, H=224, W=224], RGB float [0, 1].
View order: front, gripper. Time order: previous, current. No future frame.

This module deliberately does NOT implement a second action sampler, change
LeRobot's normalization, or alter the original DiT's Flow Matching process.
"""
from __future__ import annotations

import torch
from torch import Tensor

NUM_VIEWS = 2
NUM_FRAMES = 2
IMAGE_SIZE = 224
NUM_EMBODIMENT_TOKENS = 32
NUM_PREDICTED_TOKENS = 256
CONDITIONING_HIDDEN_SIZE = 2048


def _check_image(image: Tensor, *, name: str) -> None:
    if not isinstance(image, Tensor):
        raise TypeError(f"{name} must be a torch.Tensor")
    if tuple(image.shape) != (3, IMAGE_SIZE, IMAGE_SIZE):
        raise ValueError(f"{name}: expected [3,224,224], got {tuple(image.shape)}")
    if not image.is_floating_point():
        raise TypeError(f"{name}: expected floating-point RGB in [0,1]")
    if not bool(torch.isfinite(image).all().item()):
        raise ValueError(f"{name} contains nonfinite pixel values")
    if float(image.min().item()) < 0 or float(image.max().item()) > 1:
        raise ValueError(f"{name} must be RGB float [0,1]")


def build_observed_pair(
    previous: tuple[Tensor, Tensor] | None,
    current: tuple[Tensor, Tensor],
) -> Tensor:
    """Build [1,2,2,3,224,224] from consecutive *controller* observations.

    A new episode begins with (first,first), just like dataset t=0.
    Frames are not sourced from previous action chunks.
    """
    if len(current) != NUM_VIEWS:
        raise ValueError("Expected current=(front,gripper)")
    for i, frame in enumerate(current):
        _check_image(frame, name=f"current_view_{i}")
    if previous is None:
        previous = current
    if len(previous) != NUM_VIEWS:
        raise ValueError("Expected previous=(front,gripper)")
    for i, frame in enumerate(previous):
        _check_image(frame, name=f"previous_view_{i}")
    return torch.stack(
        [torch.stack((previous[v], current[v]), dim=0) for v in range(NUM_VIEWS)],
        dim=0,
    ).unsqueeze(0)


@torch.no_grad()
def predict_s1_from_observed_pair(
    model,
    observed_video: Tensor,
    qwen_action_tokens: Tensor,
) -> Tensor:
    """Frozen V-JEPA2 + Predictor, exactly as in WM-conditioned training.

    Input action tokens come from the SAME Qwen forward as z_embodiment.
    Only the first 8 Qwen action readouts condition the immediate prediction.
    """
    if observed_video.ndim != 6 or tuple(observed_video.shape[1:]) != (
        NUM_VIEWS, NUM_FRAMES, 3, IMAGE_SIZE, IMAGE_SIZE,
    ):
        raise ValueError(f"Unexpected WM video shape: {tuple(observed_video.shape)}")
    b, v, t, c, h, w = observed_video.shape
    n_action = int(model.config.num_action_tokens_per_timestep)
    if n_action != 8:
        raise ValueError(f"Expected 8 action readouts per position, got {n_action}")
    if qwen_action_tokens is None or qwen_action_tokens.ndim != 3:
        raise ValueError("Missing Qwen latent action tokens")
    if qwen_action_tokens.shape[0] != b or qwen_action_tokens.shape[1] < n_action:
        raise ValueError(f"Qwen action tokens incompatible: {tuple(qwen_action_tokens.shape)}")
    if model.video_encoder is None or model.video_processor is None or model.video_predictor is None:
        raise RuntimeError("V-JEPA2 and World Predictor were not loaded")

    flat = observed_video.reshape(b * v, t, c, h, w)
    # Match WMPredictiveFlowMatchingLoss.predict_s1 EXACTLY: video processor
    # consumes RGB float [0,1], and do_rescale=False avoids multiplying by 1/255.
    pixels = model.video_processor(
        videos=list(flat),
        return_tensors="pt",
        device=model.video_encoder.device,
        do_rescale=False,
    )["pixel_values_videos"]
    features = model.video_encoder.get_vision_features(pixel_values_videos=pixels)
    s0 = model._merge_views(features, b, v)
    if tuple(s0.shape) != (b, NUM_PREDICTED_TOKENS, CONDITIONING_HIDDEN_SIZE):
        raise RuntimeError(f"Unexpected S0 shape: {tuple(s0.shape)}")

    s1_hat = model.video_predictor(
        s0.float(),
        qwen_action_tokens[:, :n_action].float(),
    )
    if tuple(s1_hat.shape) != (b, NUM_PREDICTED_TOKENS, CONDITIONING_HIDDEN_SIZE):
        raise RuntimeError(f"Unexpected S1_hat shape: {tuple(s1_hat.shape)}")
    if not bool(torch.isfinite(s1_hat).all().item()):
        raise RuntimeError("World Predictor produced nonfinite latent values")
    return s1_hat


@torch.no_grad()
def build_wm_conditioning(model, images, instructions, observed_video: Tensor) -> Tensor:
    """One Qwen forward + one frozen WM prediction, without an embedding hook.

    During training the hook provided 32 copies of one trainable BF16 row.
    Export copied that row back into the original Qwen table. Therefore the
    normal Qwen inference forward already sees the learned embedding.
    """
    z_embodiment, z_action = model._encode_qwen(
        images, instructions, need_action_tokens=True,
    )
    b = observed_video.shape[0]
    if tuple(z_embodiment.shape) != (b, NUM_EMBODIMENT_TOKENS, CONDITIONING_HIDDEN_SIZE):
        raise RuntimeError(f"Unexpected Qwen embodied token shape: {tuple(z_embodiment.shape)}")
    if z_action is None or z_action.shape[0] != b or z_action.shape[2] != CONDITIONING_HIDDEN_SIZE:
        raise RuntimeError("Unexpected Qwen latent action tokens")
    s1_hat = predict_s1_from_observed_pair(model, observed_video, z_action)
    conditioning = torch.cat((z_embodiment, s1_hat.to(z_embodiment.dtype)), dim=1)
    if tuple(conditioning.shape) != (b, 288, CONDITIONING_HIDDEN_SIZE):
        raise RuntimeError(f"Unexpected WM conditioning: {tuple(conditioning.shape)}")
    return conditioning
