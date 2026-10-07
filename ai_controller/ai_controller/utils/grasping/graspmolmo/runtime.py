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

        image = self._to_pil_rgb(rgb)

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

        with torch.inference_mode():
            output = self.model.generate_from_batch(
                inputs,
                self.gen_cfg,
                tokenizer=self.processor.tokenizer,
            )

        t3 = time.perf_counter()

        print(f"[GraspMolmo] processor: {t1 - t0:.3f} s")
        print(f"[GraspMolmo] tensor prep: {t2 - t1:.3f} s")
        print(f"[GraspMolmo] generation: {t3 - t2:.3f} s")

        generated_tokens = output[
            0,
            inputs["input_ids"].size(1):,
        ]

        generated_text = self.processor.tokenizer.decode(
            generated_tokens,
            skip_special_tokens=True,
        )

        if verbosity >= 1:
            print("Output:", generated_text)

        point = parse_point(
            generated_text,
            image.size,
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