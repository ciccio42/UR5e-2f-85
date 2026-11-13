"""
Runtime wrapper for GraspMolmo.

This module is intentionally independent from ROS and from the HTTP server.

Responsibilities
----------------
- Load the official GraspMolmo model.
- Predict the task-oriented grasp point on an RGB image.
- Select a 6-DoF grasp among candidates produced by an external grasp
  proposer such as M2T2.

Non-responsibilities
--------------------
This module does NOT:
- acquire camera images;
- build point clouds from depth;
- run M2T2;
- perform camera-to-base transformations;
- perform robot motion planning;
- expose HTTP endpoints.

Those responsibilities belong to the other modules of the grasping package.
"""

from __future__ import annotations

import random
import numpy as np
from PIL import Image

import torch

import time

from transformers import (
    AutoModelForCausalLM,
    AutoProcessor,
    GenerationConfig,
)

from graspmolmo.inference.grasp_predictor import parse_point
from graspmolmo.inference.utils import get_grasp_points

import os
from types import MethodType
import torch.nn.functional as F

import copy


class _Fp8DecodeOnlyMLP(torch.nn.Module):
    """
    Use the original BF16 MLP for prefill and
    compiled FP8 weight-only MLP for decoding.
    """

    def __init__(self, original_mlp):
        super().__init__()

        from torchao.quantization import (
            Float8WeightOnlyConfig,
            quantize_,
        )

        self.bf16_mlp = original_mlp
        fp8_mlp = copy.deepcopy(original_mlp).eval()

        quantize_(
            fp8_mlp,
            Float8WeightOnlyConfig(),
        )

        self.fp8_compiled = torch.compile(
            fp8_mlp,
            mode="default",
            fullgraph=True,
        )

        self.decode_calls = 0
        self.prefill_calls = 0

    def forward(self, x):

        if x.ndim == 3 and x.shape[1] == 1:
            self.decode_calls += 1
            return self.fp8_compiled(x)

        self.prefill_calls += 1
        return self.bf16_mlp(x)

class GraspMolmoRuntime:
    """Thin wrapper around the official GraspMolmo inference API."""

    def __init__(self) -> None:
        self.model = None
        self.processor = None
        self.gen_cfg = None

        self.model_name = "allenai/GraspMolmo"

        self.prompt_pfx = (
            "Point to the grasp that would accomplish the following task: "
        )

        self._loaded = False

    def load(self) -> None:
        """Load GraspMolmo once and keep it resident on the GPU."""

        if self._loaded:
            return

        print("[GraspMolmoRuntime] Loading GraspMolmo in BF16 on CUDA...")

        self.processor = AutoProcessor.from_pretrained(
            self.model_name,
            trust_remote_code=True,
        )

        self.model = AutoModelForCausalLM.from_pretrained(
            self.model_name,
            torch_dtype=torch.bfloat16,
            device_map={"": 0},
            trust_remote_code=True,
        )

        self.model.eval()

        print(
            "[GraspMolmo][PROFILE] torch_version:",
            torch.__version__,
        )

        print(
            "[GraspMolmo][PROFILE] cuda_version:",
            torch.version.cuda,
        )

        print(
            "[GraspMolmo][PROFILE] gpu:",
            torch.cuda.get_device_name(0),
        )

        print(
            "[GraspMolmo][PROFILE] attention_type:",
            getattr(self.model.config, "attention_type", None),
        )

        print(
            "[GraspMolmo][PROFILE] float32_attention:",
            getattr(self.model.config, "float32_attention", None),
        )

        print(
            "[GraspMolmo][PROFILE] use_cache:",
            getattr(self.model.config, "use_cache", None),
        )

        # Experimental native GQA for autoregressive decoding.
        # Disabled by default.
        self._gqa_enabled = (
            os.environ.get("SEEDO_GRASPMOLMO_GQA", "0") == "1"
        )

        self._gqa_decode_calls = 0
        self._gqa_mask_verified = None

        if self._gqa_enabled:
            self._install_native_gqa_patch()

        print(
            "[GraspMolmo][GQA] enabled:",
            self._gqa_enabled,
        )
        
        # ---------------------------------------------------------
        # GRASPMOLMO - Experimental FP8 decode-only MLPs
        # ---------------------------------------------------------

        self._fp8_test_mlps = {}

        fp8_layer_spec = os.environ.get(
            "SEEDO_GRASPMOLMO_FP8_TEST_LAYER", ""
        ).strip()

        if fp8_layer_spec:

            blocks = self.model.model.transformer.blocks

            if fp8_layer_spec.lower() == "all":
                fp8_layer_indices = list(range(len(blocks)))
            else:
                fp8_layer_indices = sorted({
                    int(item.strip())
                    for item in fp8_layer_spec.split(",")
                    if item.strip()
                })

            if not fp8_layer_indices or any(
                idx < 0 or idx >= len(blocks)
                for idx in fp8_layer_indices
            ):
                raise ValueError(
                    f"Invalid FP8 layer selection: {fp8_layer_spec}"
                )

            generator = torch.Generator(
                device=self.model.device
            ).manual_seed(1234)

            test_input = torch.randn(
                1,
                1,
                self.model.config.hidden_size,
                device=self.model.device,
                dtype=torch.bfloat16,
                generator=generator,
            )

            for idx in fp8_layer_indices:

                print(
                    f"[GraspMolmo][FP8] Preparing layer {idx}...",
                    flush=True,
                )

                wrapper = _Fp8DecodeOnlyMLP(
                    blocks[idx].mlp
                )

                # Compile/warm up the FP8 MLP and compare
                # against its original pretrained BF16 weights.
                with torch.inference_mode():

                    reference = wrapper.bf16_mlp(
                        test_input
                    ).float()

                    candidate = wrapper.fp8_compiled(
                        test_input
                    ).float()

                    difference = reference - candidate

                    if not torch.isfinite(candidate).all().item():
                        raise RuntimeError(
                            f"FP8 layer {idx} produced non-finite values"
                        )

                    rmse = difference.square().mean().sqrt()
                    ref_rms = reference.square().mean().sqrt()

                    relative_rmse = (
                        rmse / ref_rms.clamp_min(1e-12)
                    ).item()

                blocks[idx].mlp = wrapper

                self._fp8_test_mlps[idx] = wrapper

                print(
                    f"[GraspMolmo][FP8] layer={idx} "
                    f"warmup_relative_rmse={relative_rmse:.6f}",
                    flush=True,
                )

            torch.cuda.synchronize()

            print(
                "[GraspMolmo][FP8] decode_only_layers:",
                fp8_layer_indices,
            )

            print(
                "[GraspMolmo][FP8] cuda_allocated_gib:",
                round(torch.cuda.memory_allocated() / (1024 ** 3), 3),
            )

        self.gen_cfg = GenerationConfig(
            max_new_tokens=256,
            stop_strings="<|endoftext|>",
            do_sample=False,
        )

        self._loaded = True

        print(
            "[GraspMolmoRuntime] GraspMolmo ready "
            "(BF16, full CUDA)."
        )

    def _install_native_gqa_patch(self) -> None:
        """
        Experimental decoding-only native GQA optimization.

        Prefill uses the original Molmo attention implementation.

        During single-token decoding, the official Molmo implementation
        constructs a causal mask whose final row is fully permissive.
        Under the guarded conditions below, native GQA can therefore
        operate without an explicit attention mask.
        """

        if self.model.config.attention_type != "sdpa":
            raise RuntimeError(
                "Native GQA patch requires attention_type='sdpa'."
            )

        blocks = self.model.model.transformer.blocks

        if not blocks:
            raise RuntimeError(
                "No Molmo decoder blocks found."
            )

        for block in blocks:

            attention = block.attn

            if type(attention).__name__ != "MolmoAttention":
                raise RuntimeError(
                    "Unexpected Molmo attention implementation."
                )

            original_method = (
                attention._scaled_dot_product_attention
            )

            def optimized_sdpa(
                module,
                q,
                k,
                v,
                attention_mask=None,
                dropout_p=0.0,
                is_causal=False,
                *,
                _original=original_method,
            ):

                native_gqa_eligible = (
                    q.device.type == "cuda"
                    and q.ndim == 4
                    and k.ndim == 4
                    and v.ndim == 4
                    and q.shape[-2] == 1
                    and k.shape[-2] > 1
                    and k.shape == v.shape
                    and q.shape[1] == 28
                    and k.shape[1] == 4
                    and attention_mask is not None
                    and attention_mask.dtype == torch.bool
                    and attention_mask.shape == (
                        1, 1, 1, k.shape[-2]
                    )
                    and dropout_p == 0.0
                    and not is_causal
                )

                if native_gqa_eligible:

                    # Validate the first decoding mask of each
                    # prediction. The official Molmo model rebuilds
                    # the same fully permissive last causal row
                    # during subsequent decoding steps.
                    if self._gqa_mask_verified is None:
                        self._gqa_mask_verified = bool(
                            attention_mask.all().item()
                        )

                    if self._gqa_mask_verified:

                        self._gqa_decode_calls += 1

                        return F.scaled_dot_product_attention(
                            q,
                            k,
                            v,
                            attn_mask=None,
                            dropout_p=0.0,
                            is_causal=False,
                            enable_gqa=True,
                        )

                # Original implementation for prefill and
                # all unsupported configurations.
                return _original(
                    q,
                    k,
                    v,
                    attention_mask=attention_mask,
                    dropout_p=dropout_p,
                    is_causal=is_causal,
                )

            attention._scaled_dot_product_attention = (
                MethodType(
                    optimized_sdpa,
                    attention,
                )
            )

        print(
            "[GraspMolmo][GQA] Patched decoder layers:",
            len(blocks),
        )

    def predict_point(
        self,
        rgb: np.ndarray,
        task: str,
        verbosity: int = 0,
        seed: int | None = None,
    ) -> np.ndarray | None:
        """
        Predict the task-oriented grasp point in image coordinates.

        This reproduces the official GraspMolmo inference path while adding
        the batch dimension required by Molmo's generate_from_batch().
        """
        self._require_loaded()

        if seed is not None:

            seed = int(
                seed
            )

            random.seed(
                seed
            )

            np.random.seed(
                seed
            )

            torch.manual_seed(
                seed
            )

            if torch.cuda.is_available():
                torch.cuda.manual_seed_all(
                    seed
                )

            if verbosity >= 1:
                print(
                    "[GraspMolmoRuntime] Random seed:",
                    seed,
                )

        t_total_start = time.perf_counter()

        image = self._to_pil_rgb(rgb)

        t_image = time.perf_counter()

        t0 = time.perf_counter()

        inputs = self.processor.process(
            text=f"{self.prompt_pfx}{task}",
            images=[image],
            return_tensors="pt",
        )

        t1 = time.perf_counter()

        # Molmo's processor returns an unbatched sample.
        # generate_from_batch() expects (B, ...), therefore B=1 is added here.
        inputs = {
            key: value.unsqueeze(0).to(self.model.device)
            for key, value in inputs.items()
        }

        # Image tensors must match the model dtype.
        if "images" in inputs:
            inputs["images"] = inputs["images"].to(torch.bfloat16)

        t2 = time.perf_counter()

        # ---------------------------------------------------------
        # GRASPMOLMO PROFILING - Prefill / Decode GPU timing
        # ---------------------------------------------------------

        forward_events = []

        def _forward_start(module, args):
            start = torch.cuda.Event(enable_timing=True)
            start.record()
            forward_events.append([start, None])

        def _forward_end(module, args, output):
            end = torch.cuda.Event(enable_timing=True)
            end.record()
            forward_events[-1][1] = end

        pre_handle = self.model.register_forward_pre_hook(
            _forward_start
        )

        post_handle = self.model.register_forward_hook(
            _forward_end
        )

        # ---------------------------------------------------------
        # GRASPMOLMO PROFILING - Decoder components
        # ---------------------------------------------------------

        decoder_profile_enabled = (
            os.environ.get("SEEDO_GRASPMOLMO_PROFILE_DECODER", "0") == "1"
        )

        decoder_handles = []
        decoder_events = {}
        decoder_layers = []

        if decoder_profile_enabled:

            blocks = self.model.model.transformer.blocks

            # First, middle and last decoder layers.
            decoder_layers = sorted({
                0,
                len(blocks) // 2,
                len(blocks) - 1,
            })

            def _profile_component(label, component):

                events = []
                decoder_events[label] = events

                def on_start(module, args):

                    x = args[0]

                    # Only autoregressive decoding, not prefill.
                    if x.ndim != 3 or x.shape[1] != 1:
                        return

                    start = torch.cuda.Event(enable_timing=True)
                    start.record()

                    events.append([start, None])

                def on_end(module, args, output):

                    if events and events[-1][1] is None:

                        end = torch.cuda.Event(enable_timing=True)
                        end.record()

                        events[-1][1] = end

                decoder_handles.append(
                    component.register_forward_pre_hook(on_start)
                )

                decoder_handles.append(
                    component.register_forward_hook(on_end)
                )

            for idx in decoder_layers:

                block = blocks[idx]

                components = {
                    "attn": block.attn,
                    "mlp": block.mlp,
                    "att_proj": block.attn.att_proj,
                    "attn_out": block.attn.attn_out,
                }

                # Individual linear hooks remain available for BF16 MLPs.
                # For compiled FP8 MLPs, profile the complete wrapper instead.
                if not isinstance(block.mlp, _Fp8DecodeOnlyMLP):
                    components["ff_proj"] = block.mlp.ff_proj
                    components["ff_out"] = block.mlp.ff_out

                for name, component in components.items():

                    _profile_component(
                        f"{idx}.{name}",
                        component,
                    )

        try:
            if self._gqa_enabled:
                self._gqa_mask_verified = None
                gqa_calls_before = self._gqa_decode_calls
            if self._fp8_test_mlps:
                fp8_calls_before = {
                    idx: (wrapper.decode_calls, wrapper.prefill_calls)
                    for idx, wrapper in self._fp8_test_mlps.items()
                }
            with torch.inference_mode():
                output = self.model.generate_from_batch(
                    inputs,
                    self.gen_cfg,
                    tokenizer=self.processor.tokenizer,
                )
        finally:
            pre_handle.remove()
            post_handle.remove()

            for handle in decoder_handles:
                handle.remove()

        torch.cuda.synchronize()

        t3 = time.perf_counter()

        if forward_events:
            forward_times = [
                start.elapsed_time(end) / 1000.0
                for start, end in forward_events
            ]

            prefill_time = forward_times[0]
            decode_time = sum(forward_times[1:])

            print(
                "[GraspMolmo][PROFILE] forward_calls:",
                len(forward_times),
            )

            print(
                "[GraspMolmo][PROFILE] prefill_gpu:",
                f"{prefill_time:.3f} s",
            )

            print(
                "[GraspMolmo][PROFILE] decode_gpu:",
                f"{decode_time:.3f} s",
            )

        print(
            "[GraspMolmo][PROFILE] forward_gpu_total:",
            f"{sum(forward_times):.3f} s",
        )

        # ---------------------------------------------------------
        # GRASPMOLMO GQA - Optimization diagnostics
        # ---------------------------------------------------------

        if self._gqa_enabled:
            print(
                "[GraspMolmo][GQA] optimized_attention_calls:",
                self._gqa_decode_calls - gqa_calls_before,
            )

            print(
                "[GraspMolmo][GQA] mask_verified:",
                self._gqa_mask_verified,
            )

        # ---------------------------------------------------------

        # ---------------------------------------------------------
        # GRASPMOLMO PROFILING - Decoder breakdown
        # ---------------------------------------------------------

        if decoder_profile_enabled:

            print(
                "[GraspMolmo][DECODER] sample_layers:",
                decoder_layers,
            )

            for idx in decoder_layers:

                times = {}
                counts = {}

                for name in (
                    "attn", "mlp", "att_proj", "attn_out",
                    "ff_proj", "ff_out",
                ):

                    pairs = decoder_events.get(f"{idx}.{name}", [])

                    counts[name] = len(pairs)

                    times[name] = (
                        sum(
                            start.elapsed_time(end)
                            for start, end in pairs
                            if end is not None
                        ) / 1000.0
                        if pairs else None
                    )

                attn_rest = (
                    times["attn"]
                    - times["att_proj"]
                    - times["attn_out"]
                )

                mlp_rest = None

                if (
                    times["mlp"] is not None
                    and times["ff_proj"] is not None
                    and times["ff_out"] is not None
                ):
                    mlp_rest = (
                        times["mlp"]
                        - times["ff_proj"]
                        - times["ff_out"]
                    )

                def fmt(value):
                    return "n/a" if value is None else f"{value:.3f}s"

                print(
                    f"[GraspMolmo][DECODER] layer={idx} "
                    f"calls={counts['attn']} "
                    f"attn={fmt(times['attn'])} "
                    f"qkv={fmt(times['att_proj'])} "
                    f"attn_out={fmt(times['attn_out'])} "
                    f"attn_rest={fmt(attn_rest)} "
                    f"mlp={fmt(times['mlp'])} "
                    f"ff_proj={fmt(times['ff_proj'])} "
                    f"ff_out={fmt(times['ff_out'])} "
                    f"mlp_rest={fmt(mlp_rest)}"
                )

        if self._fp8_test_mlps:

            decode_deltas = []
            prefill_deltas = []

            for idx, wrapper in self._fp8_test_mlps.items():

                old_decode, old_prefill = fp8_calls_before[idx]

                decode_deltas.append(
                    wrapper.decode_calls - old_decode
                )

                prefill_deltas.append(
                    wrapper.prefill_calls - old_prefill
                )

            print(
                "[GraspMolmo][FP8] active_layers:",
                len(self._fp8_test_mlps),
            )

            print(
                "[GraspMolmo][FP8] total_decode_calls:",
                sum(decode_deltas),
            )

            print(
                "[GraspMolmo][FP8] total_bf16_prefill_calls:",
                sum(prefill_deltas),
            )

            print(
                "[GraspMolmo][FP8] decode_calls_per_layer:",
                sorted(set(decode_deltas)),
            )

        generated_tokens = output[
            0,
            inputs["input_ids"].size(1):,
        ]

        generated_text = self.processor.tokenizer.decode(
            generated_tokens,
            skip_special_tokens=True,
        )

        t4 = time.perf_counter()

        # ---------------------------------------------------------
        # GRASPMOLMO PROFILING - Token generation diagnostics
        # ---------------------------------------------------------

        print(
            "[GraspMolmo][PROFILE] input_tokens:",
            inputs["input_ids"].shape[1],
        )

        print(
            "[GraspMolmo][PROFILE] generated_tokens:",
            generated_tokens.numel(),
        )

        if "images" in inputs:
            print(
                "[GraspMolmo][PROFILE] image_tensor_shape:",
                tuple(inputs["images"].shape),
            )

        print(
            "[GraspMolmo][PROFILE] point_tag_complete:",
            "</point>" in generated_text
            or "</points>" in generated_text,
        )

        # ---------------------------------------------------------

        if verbosity >= 1:
            print("Output:", generated_text)

        point = parse_point(
            generated_text,
            image.size,
        )

        t5 = time.perf_counter()

        print(
            f"[GraspMolmo] image prep: "
            f"{t_image - t_total_start:.3f} s"
        )

        print(
            f"[GraspMolmo] processor: "
            f"{t1 - t0:.3f} s"
        )

        print(
            f"[GraspMolmo] tensor prep: "
            f"{t2 - t1:.3f} s"
        )

        print(
            f"[GraspMolmo] generation: "
            f"{t3 - t2:.3f} s"
        )

        print(
            f"[GraspMolmo] decode: "
            f"{t4 - t3:.3f} s"
        )

        print(
            f"[GraspMolmo] parse point: "
            f"{t5 - t4:.3f} s"
        )

        print(
            f"[GraspMolmo] total inference: "
            f"{t5 - t_total_start:.3f} s"
        )

        if verbosity >= 1:
            print("Predicted point:", point)

        if point is None:
            return None

        return np.asarray(point, dtype=np.float32)

    def select_grasp(
        self,
        rgb: np.ndarray,
        point_cloud: np.ndarray,
        task: str,
        grasps: np.ndarray,
        camera_intrinsics: np.ndarray,
        verbosity: int = 0,
        seed: int | None = None,
    ) -> int | None:
        """
        Select the candidate grasp that best matches the task semantics.
        """
        self._require_loaded()

        point_cloud = np.asarray(point_cloud, dtype=np.float32)
        grasps = np.asarray(grasps, dtype=np.float32)
        camera_intrinsics = np.asarray(
            camera_intrinsics,
            dtype=np.float32,
        )

        self._validate_inputs(
            point_cloud=point_cloud,
            grasps=grasps,
            camera_intrinsics=camera_intrinsics,
        )

        point = self.predict_point(
            rgb=rgb,
            task=task,
            verbosity=verbosity,
            seed=seed,
        )

        if point is None:
            return None

        grasp_points = get_grasp_points(
            point_cloud,
            grasps,
        )

        grasp_points_2d = grasp_points @ camera_intrinsics.T

        grasp_points_2d = (
            grasp_points_2d[:, :2]
            / grasp_points_2d[:, 2:3]
        )

        distances = np.linalg.norm(
            grasp_points_2d - point[None],
            axis=1,
        )

        return int(np.argmin(distances))

    def _require_loaded(self) -> None:
        if not self._loaded or self.model is None:
            raise RuntimeError(
                "GraspMolmoRuntime is not loaded. "
                "Call load() before inference."
            )

    @staticmethod
    def _to_pil_rgb(rgb: np.ndarray) -> Image.Image:
        rgb = np.asarray(rgb)

        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError(
                "rgb must have shape (H, W, 3), "
                f"got {rgb.shape}"
            )

        if rgb.dtype != np.uint8:
            raise ValueError(
                "rgb must have dtype uint8, "
                f"got {rgb.dtype}"
            )

        return Image.fromarray(rgb, mode="RGB")

    @staticmethod
    def _validate_inputs(
        point_cloud: np.ndarray,
        grasps: np.ndarray,
        camera_intrinsics: np.ndarray,
    ) -> None:

        if (
            point_cloud.ndim != 2
            or point_cloud.shape[1] != 3
        ):
            raise ValueError(
                "point_cloud must have shape (N, 3), "
                f"got {point_cloud.shape}"
            )

        if (
            grasps.ndim != 3
            or grasps.shape[1:] != (4, 4)
        ):
            raise ValueError(
                "grasps must have shape (K, 4, 4), "
                f"got {grasps.shape}"
            )

        if len(grasps) == 0:
            raise ValueError(
                "grasps must contain at least one candidate."
            )

        if camera_intrinsics.shape != (3, 3):
            raise ValueError(
                "camera_intrinsics must have shape (3, 3), "
                f"got {camera_intrinsics.shape}"
            )