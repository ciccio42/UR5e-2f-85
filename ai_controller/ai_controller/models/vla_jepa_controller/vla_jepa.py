"""
LeRobot runtime wrapper for VLA-JEPA inference.

This module is intentionally independent from ROS and from the UR5e controller.

Responsibilities
----------------
- Load a pretrained VLA-JEPA checkpoint through the official LeRobot API.
- Load the preprocessor and postprocessor saved with the checkpoint.
- Move the policy to the requested device and switch it to evaluation mode.
- Execute the official LeRobot inference pipeline:

      raw LeRobot observation
          -> preprocessor
          -> VLAJEPAPolicy.select_action()
          -> postprocessor
          -> physical action

- Reset the internal state of the policy and processor pipelines.
- Expose checkpoint metadata useful to the higher-level controller.

Non-responsibilities
--------------------
This module does NOT:
- read ROS messages;
- select or preprocess UR5e cameras;
- construct the UR5e proprioceptive state;
- interpret action semantics;
- convert delta actions to absolute robot targets;
- apply robot safety limits.

Those responsibilities belong to vla_jepa_controller.py and
vla_jepa_utils.py.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

import torch

from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import (
    get_policy_class,
    make_pre_post_processors,
)


LOGGER = logging.getLogger(__name__)


class VLAJEPARuntime:
    """
    Thin runtime wrapper around the LeRobot VLA-JEPA policy.

    Parameters
    ----------
    checkpoint_path:
        Local LeRobot checkpoint directory or Hugging Face repository ID.

        The checkpoint is expected to contain the metadata required by
        LeRobot, including the policy config and the serialized
        pre/postprocessor pipelines.

    device:
        Device used for policy inference, normally ``"cuda"`` on the
        DGX Spark.

    postprocessor_overrides:
        Optional LeRobot postprocessor overrides.

        This is intentionally left generic. No dataset-specific gripper
        workaround is hard-coded here because the semantics of the future
        checkpoint are not known yet.
    """

    POLICY_TYPE = "vla_jepa"

    def __init__(
        self,
        checkpoint_path: str | Path,
        device: str = "cuda",
        postprocessor_overrides: dict[str, Any] | None = None,
    ) -> None:
        self.checkpoint_path = str(checkpoint_path)
        self.device = torch.device(device)

        self.postprocessor_overrides = postprocessor_overrides or {}

        self.config: PreTrainedConfig | None = None
        self.policy: torch.nn.Module | None = None
        self.preprocessor: Any | None = None
        self.postprocessor: Any | None = None

        self._loaded = False

    # ------------------------------------------------------------------
    # Loading
    # ------------------------------------------------------------------

    def load(self) -> None:
        """
        Load config, VLA-JEPA policy and serialized processor pipelines.

        The implementation intentionally follows the LeRobot inference
        path instead of recreating the policy or its normalization logic
        manually.
        """

        if self._loaded:
            LOGGER.info("VLA-JEPA runtime is already loaded.")
            return

        self._validate_device()

        LOGGER.info(
            "Loading VLA-JEPA checkpoint from '%s' on device '%s'.",
            self.checkpoint_path,
            self.device,
        )

        # --------------------------------------------------------------
        # 1. Load the policy configuration saved with the checkpoint.
        # --------------------------------------------------------------
        config = PreTrainedConfig.from_pretrained(self.checkpoint_path)

        if config.type != self.POLICY_TYPE:
            raise ValueError(
                "Checkpoint policy type mismatch: "
                f"expected '{self.POLICY_TYPE}', got '{config.type}'."
            )

        # Inference device is a runtime choice.
        config.device = str(self.device)

        # --------------------------------------------------------------
        # 2. Resolve and load the actual VLA-JEPA policy class.
        #
        # Do not use make_policy() here: without a LeRobot dataset or
        # environment, make_policy() would try to derive feature shapes
        # externally. A pretrained checkpoint already contains its
        # resolved input/output feature specification.
        # --------------------------------------------------------------
        policy_cls = get_policy_class(config.type)

        policy = policy_cls.from_pretrained(
            self.checkpoint_path,
            config=config,
        )

        policy.to(self.device)
        policy.eval()

        # --------------------------------------------------------------
        # 3. Load the processor pipelines SAVED WITH THE CHECKPOINT.
        #
        # This is important because normalization statistics and
        # processor configuration are part of the trained policy
        # contract. We must not reconstruct them from our UR5e data.
        # --------------------------------------------------------------
        preprocessor, postprocessor = make_pre_post_processors(
            policy_cfg=config,
            pretrained_path=self.checkpoint_path,
            postprocessor_overrides=self.postprocessor_overrides,
        )

        self.config = config
        self.policy = policy
        self.preprocessor = preprocessor
        self.postprocessor = postprocessor
        self._loaded = True

        # Start from a clean action queue / processor state.
        self.reset()

        LOGGER.info(
            "VLA-JEPA runtime loaded successfully. "
            "image_features=%s, state_dim=%s, action_dim=%s, "
            "chunk_size=%s, n_action_steps=%s",
            self.image_feature_keys,
            self.state_dim,
            self.action_dim,
            self.chunk_size,
            self.n_action_steps,
        )

    # ------------------------------------------------------------------
    # Inference
    # ------------------------------------------------------------------

    def select_action(
        self,
        observation: dict[str, Any],
    ) -> torch.Tensor:
        """
        Run one LeRobot policy step.

        Parameters
        ----------
        observation:
            Raw observation in LeRobot format.

            Conceptually::

                {
                    "observation.images.<camera>": torch.Tensor[C, H, W],
                    "observation.state": torch.Tensor[state_dim],
                    "task": "task instruction",
                }

            Exact feature keys and dimensions are determined by the
            checkpoint configuration.

            The caller should provide physical/non-normalized values.
            Batching, device transfer and normalization are delegated to
            the checkpoint preprocessor.

        Returns
        -------
        torch.Tensor
            One postprocessed action.

            The action is returned using the convention represented by
            the checkpoint. This class deliberately does not interpret
            its individual dimensions.
        """

        self._require_loaded()

        if not isinstance(observation, dict):
            raise TypeError(
                "VLA-JEPA observation must be a dictionary, "
                f"got {type(observation).__name__}."
            )

        self._validate_observation_keys(observation)

        # The preprocessor may modify the supplied mapping, depending on
        # the serialized processor implementation. Keep the caller's
        # top-level dictionary untouched.
        policy_input = dict(observation)

        with torch.inference_mode():
            policy_input = self.preprocessor(policy_input)

            action = self.policy.select_action(policy_input)

            action = self.postprocessor(action)

        if not isinstance(action, torch.Tensor):
            raise TypeError(
                "LeRobot VLA-JEPA postprocessor returned an unexpected "
                f"type: {type(action).__name__}."
            )

        return action

    # ------------------------------------------------------------------
    # Runtime state
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """
        Reset inference state.

        VLA-JEPA's LeRobot policy internally caches an action chunk.
        Resetting the policy clears that action queue.

        The processor pipelines are reset as well, following LeRobot's
        normal runtime behavior.
        """

        if not self._loaded:
            return

        self.policy.reset()

        if hasattr(self.preprocessor, "reset"):
            self.preprocessor.reset()

        if hasattr(self.postprocessor, "reset"):
            self.postprocessor.reset()

    # ------------------------------------------------------------------
    # Checkpoint metadata
    # ------------------------------------------------------------------

    @property
    def image_feature_keys(self) -> tuple[str, ...]:
        """Image feature names expected by the loaded checkpoint."""

        if self.config is None:
            return ()

        image_features = getattr(self.config, "image_features", {})
        return tuple(image_features.keys())

    @property
    def input_features(self) -> dict[str, Any]:
        """Full input feature specification stored in the checkpoint."""

        if self.config is None:
            return {}

        return dict(getattr(self.config, "input_features", {}))

    @property
    def output_features(self) -> dict[str, Any]:
        """Full output feature specification stored in the checkpoint."""

        if self.config is None:
            return {}

        return dict(getattr(self.config, "output_features", {}))

    @property
    def state_dim(self) -> int | None:
        if self.config is None:
            return None

        value = getattr(self.config, "state_dim", None)
        return int(value) if value is not None else None

    @property
    def action_dim(self) -> int | None:
        if self.config is None:
            return None

        value = getattr(self.config, "action_dim", None)
        return int(value) if value is not None else None

    @property
    def chunk_size(self) -> int | None:
        if self.config is None:
            return None

        value = getattr(self.config, "chunk_size", None)
        return int(value) if value is not None else None

    @property
    def n_action_steps(self) -> int | None:
        if self.config is None:
            return None

        value = getattr(self.config, "n_action_steps", None)
        return int(value) if value is not None else None

    def describe_checkpoint(self) -> dict[str, Any]:
        """
        Return the main checkpoint properties relevant to integration.

        This will be useful as soon as the pretrained checkpoint becomes
        available, before implementing embodiment-specific conversions.
        """

        self._require_loaded()

        return {
            "policy_type": self.config.type,
            "device": str(self.device),
            "image_feature_keys": list(self.image_feature_keys),
            "input_features": {
                key: str(value)
                for key, value in self.input_features.items()
            },
            "output_features": {
                key: str(value)
                for key, value in self.output_features.items()
            },
            "state_dim": self.state_dim,
            "action_dim": self.action_dim,
            "chunk_size": self.chunk_size,
            "n_action_steps": self.n_action_steps,
            "enable_world_model": getattr(
                self.config,
                "enable_world_model",
                None,
            ),
            "resize_images_to": getattr(
                self.config,
                "resize_images_to",
                None,
            ),
            "gripper_dim": getattr(
                self.config,
                "gripper_dim",
                None,
            ),
        }

    # ------------------------------------------------------------------
    # Internal validation
    # ------------------------------------------------------------------

    def _validate_device(self) -> None:
        if self.device.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError(
                "VLA-JEPA was configured to run on CUDA, but "
                "torch.cuda.is_available() returned False."
            )

    def _require_loaded(self) -> None:
        if not self._loaded:
            raise RuntimeError(
                "VLA-JEPA runtime has not been loaded. "
                "Call load() before inference."
            )

    def _validate_observation_keys(
        self,
        observation: dict[str, Any],
    ) -> None:
        """
        Perform only structural validation.

        We deliberately avoid validating state/action semantics here:
        those depend on the embodiment represented by the checkpoint.
        """

        missing_images = [
            key
            for key in self.image_feature_keys
            if key not in observation
        ]

        if missing_images:
            raise KeyError(
                "Missing image features required by VLA-JEPA checkpoint: "
                f"{missing_images}. "
                f"Expected image features: {list(self.image_feature_keys)}."
            )

        if "observation.state" in self.input_features:
            if "observation.state" not in observation:
                raise KeyError(
                    "Checkpoint requires 'observation.state', "
                    "but it is missing from the observation."
                )

        if "task" not in observation:
            LOGGER.warning(
                "No 'task' field supplied to VLA-JEPA. "
                "The policy adapter may fall back to its generic "
                "instruction."
            )