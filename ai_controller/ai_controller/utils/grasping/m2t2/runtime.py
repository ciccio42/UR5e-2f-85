#!/usr/bin/env python3
"""
Runtime wrapper for the M2T2 grasp proposal backend.

This module runs inside the dedicated M2T2 virtual environment.
It owns the heavy PyTorch / CUDA model and keeps it resident on the GPU
between inference requests.

The public predict_grasps() API accepts an Nx3 point cloud and returns
generic 6-DoF grasp candidates in the same coordinate frame.
"""

from __future__ import annotations

import random
import sys
from pathlib import Path

import numpy as np
import torch
from omegaconf import OmegaConf


# =============================================================================
# Paths
# =============================================================================

DEFAULT_M2T2_SOURCE = Path("/opt/M2T2")

DEFAULT_CONFIG_PATH = (
    DEFAULT_M2T2_SOURCE
    / "config.yaml"
)

DEFAULT_CHECKPOINT_PATH = Path(
    "/home/ros2_ws/src/UR5e-2f-85/"
    ".runtime/m2t2_checkpoints/m2t2.pth"
)


# =============================================================================
# Upstream M2T2 import
# =============================================================================

_source_path = str(DEFAULT_M2T2_SOURCE)

if _source_path not in sys.path:
    sys.path.insert(
        0,
        _source_path,
    )


from m2t2.dataset import collate  # noqa: E402
from m2t2.dataset_utils import sample_points  # noqa: E402
from m2t2.m2t2 import M2T2  # noqa: E402


# =============================================================================
# Runtime
# =============================================================================

class M2T2Runtime:
    """
    Persistent runtime for the generic M2T2 model.
    """

    def __init__(
        self,
        checkpoint_path: str | Path = DEFAULT_CHECKPOINT_PATH,
        config_path: str | Path = DEFAULT_CONFIG_PATH,
        device: str = "cuda",
    ) -> None:

        self.checkpoint_path = Path(
            checkpoint_path
        )

        self.config_path = Path(
            config_path
        )

        self.device = torch.device(
            device
        )

        self.cfg = None
        self.model = None

        self._loaded = False

    @property
    def loaded(self) -> bool:
        """
        Return True if the M2T2 model is loaded and ready.
        """

        return self._loaded

    def load(self) -> None:
        """
        Build the M2T2 architecture, load the checkpoint and move the model
        to the configured device.
        """

        if self._loaded:
            return

        if not DEFAULT_M2T2_SOURCE.is_dir():
            raise FileNotFoundError(
                "M2T2 source directory not found: "
                f"{DEFAULT_M2T2_SOURCE}"
            )

        if not self.config_path.is_file():
            raise FileNotFoundError(
                "M2T2 config not found: "
                f"{self.config_path}"
            )

        if not self.checkpoint_path.is_file():
            raise FileNotFoundError(
                "M2T2 checkpoint not found: "
                f"{self.checkpoint_path}"
            )

        if (
            self.device.type == "cuda"
            and not torch.cuda.is_available()
        ):
            raise RuntimeError(
                "CUDA requested for M2T2, "
                "but torch.cuda.is_available() is False."
            )

        print(
            "[M2T2Runtime] Loading config: "
            f"{self.config_path}"
        )

        self.cfg = OmegaConf.load(
            self.config_path
        )

        print(
            "[M2T2Runtime] Building M2T2 model..."
        )

        model = M2T2.from_config(
            self.cfg.m2t2
        )

        print(
            "[M2T2Runtime] Loading checkpoint: "
            f"{self.checkpoint_path}"
        )

        checkpoint = torch.load(
            self.checkpoint_path,
            map_location="cpu",
            weights_only=False,
        )

        if not isinstance(
            checkpoint,
            dict,
        ):
            raise TypeError(
                "Invalid M2T2 checkpoint: "
                "expected dict, got "
                f"{type(checkpoint).__name__}"
            )

        if "model" not in checkpoint:
            raise KeyError(
                "Invalid M2T2 checkpoint: "
                "missing 'model' key."
            )

        print(
            "[M2T2Runtime] Loading model state dict..."
        )

        model.load_state_dict(
            checkpoint["model"],
            strict=True,
        )

        print(
            "[M2T2Runtime] Moving model to "
            f"{self.device}..."
        )

        model = model.to(
            self.device
        )

        model.eval()

        self.model = model
        self._loaded = True

        num_params = sum(
            parameter.numel()
            for parameter in model.parameters()
        )

        print(
            "[M2T2Runtime] M2T2 ready."
        )

        print(
            "[M2T2Runtime] Device:",
            self.device,
        )

        print(
            "[M2T2Runtime] Parameters:",
            f"{num_params:,}",
        )

        if self.device.type == "cuda":
            print(
                "[M2T2Runtime] GPU:",
                torch.cuda.get_device_name(
                    self.device
                ),
            )

            print(
                "[M2T2Runtime] Compute capability:",
                torch.cuda.get_device_capability(
                    self.device
                ),
            )

            print(
                "[M2T2Runtime] CUDA memory allocated:",
                (
                    f"{torch.cuda.memory_allocated(self.device) / 1024**3:.2f} "
                    "GiB"
                ),
            )

    def predict_grasps(
        self,
        point_cloud: np.ndarray,
        num_runs: int | None = None,
        seed: int | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate generic M2T2 grasp proposals.

        Parameters
        ----------
        point_cloud:
            Nx3 point cloud. The returned grasp poses and contact points use
            the same coordinate frame as this point cloud.

        num_runs:
            Number of M2T2 forward passes. Each pass samples a new subset of
            the input point cloud. If None, cfg.eval.num_runs is used.
        
        seed:
            Random seed used for reproducible M2T2 sampling.
            If None, RNG state is left unchanged.

        Returns
        -------
        grasps:
            Array with shape (G, 4, 4).

        contacts:
            Array with shape (G, 3).

        confidence:
            Array with shape (G,).
        """

        if not self._loaded:
            self.load()

        point_cloud = np.asarray(
            point_cloud,
            dtype=np.float32,
        )

        if (
            point_cloud.ndim != 2
            or point_cloud.shape[1] < 3
        ):
            raise ValueError(
                "point_cloud must have shape (N, 3) "
                f"or (N, >=3), got {point_cloud.shape}"
            )

        # M2T2 only needs XYZ with the current generic checkpoint.
        point_cloud = point_cloud[:, :3]

        # Remove invalid samples.
        valid = np.isfinite(
            point_cloud
        ).all(axis=1)

        point_cloud = point_cloud[
            valid
        ]

        if point_cloud.shape[0] == 0:
            raise ValueError(
                "point_cloud contains no valid points."
            )

        xyz = torch.from_numpy(
            point_cloud
        ).float()

        if num_runs is None:
            num_runs = int(
                self.cfg.eval.num_runs
            )

        if num_runs < 1:
            raise ValueError(
                "num_runs must be >= 1."
            )

        # -----------------------------------------------------------------
        # Reproducible inference sampling
        # -----------------------------------------------------------------

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

            print(
                "[M2T2Runtime] Random seed:",
                seed,
            )

        all_grasps = []
        all_contacts = []
        all_confidence = []

        num_points = int(
            self.cfg.data.num_points
        )

        num_object_points = int(
            self.cfg.data.num_object_points
        )

        for _ in range(num_runs):

            # -----------------------------------------------------------------
            # Sample scene points exactly as in the official M2T2 demo.
            # -----------------------------------------------------------------

            point_indices = sample_points(
                xyz,
                num_points,
            )

            sampled_xyz = xyz[
                point_indices
            ]

            # M2T2 centers XYZ for the PointNet++ input, while preserving the
            # original coordinates separately in "points". The latter are used
            # by the action decoder to construct the final 6-DoF grasp poses.
            model_inputs = (
                sampled_xyz
                - sampled_xyz.mean(
                    dim=0,
                    keepdim=True,
                )
            )

            # Generic pick inference still executes the object encoder because
            # the checkpoint also contains the place branch. For a pick task,
            # M2T2 later masks these object features out. This dummy tensor
            # mirrors the behavior of the official dataset loader.
            object_inputs = torch.rand(
                num_object_points,
                6,
                dtype=torch.float32,
            )

            data = {
                "inputs": model_inputs,
                "points": sampled_xyz,
                "object_inputs": object_inputs,
                "cam_pose": torch.eye(
                    4,
                    dtype=torch.float32,
                ),
                "ee_pose": torch.eye(
                    4,
                    dtype=torch.float32,
                ),
                "bottom_center": torch.zeros(
                    3,
                    dtype=torch.float32,
                ),
                "task": "pick",
            }

            batch = collate(
                [data]
            )

            # Move all tensors in our batch to the runtime device.
            for key, value in batch.items():
                if isinstance(
                    value,
                    torch.Tensor,
                ):
                    batch[key] = value.to(
                        self.device
                    )

            with torch.inference_mode():
                outputs = self.model.infer(
                    batch,
                    self.cfg.eval,
                )

            # -------------------------------------------------------------
            # M2T2 groups grasps by detected object query.
            # GraspMolmo needs one flat pool of candidate grasps, therefore
            # flatten all detected groups.
            # -------------------------------------------------------------

            grasp_groups = outputs[
                "grasps"
            ][0]

            contact_groups = outputs[
                "grasp_contacts"
            ][0]

            confidence_groups = outputs[
                "grasp_confidence"
            ][0]

            for (
                grasps,
                contacts,
                confidence,
            ) in zip(
                grasp_groups,
                contact_groups,
                confidence_groups,
            ):

                if grasps.shape[0] == 0:
                    continue

                all_grasps.append(
                    grasps.detach().cpu()
                )

                all_contacts.append(
                    contacts.detach().cpu()
                )

                all_confidence.append(
                    confidence.detach().cpu()
                )

        if not all_grasps:
            return (
                np.empty(
                    (0, 4, 4),
                    dtype=np.float32,
                ),
                np.empty(
                    (0, 3),
                    dtype=np.float32,
                ),
                np.empty(
                    (0,),
                    dtype=np.float32,
                ),
            )

        grasps = torch.cat(
            all_grasps,
            dim=0,
        ).numpy().astype(
            np.float32,
            copy=False,
        )

        contacts = torch.cat(
            all_contacts,
            dim=0,
        ).numpy().astype(
            np.float32,
            copy=False,
        )

        confidence = torch.cat(
            all_confidence,
            dim=0,
        ).numpy().astype(
            np.float32,
            copy=False,
        )

        if not (
            grasps.shape[0]
            == contacts.shape[0]
            == confidence.shape[0]
        ):
            raise RuntimeError(
                "M2T2 returned inconsistent output sizes: "
                f"grasps={grasps.shape}, "
                f"contacts={contacts.shape}, "
                f"confidence={confidence.shape}"
            )

        return (
            grasps,
            contacts,
            confidence,
        )