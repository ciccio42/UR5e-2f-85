"""
Runtime wrapper for VLA-JEPA through LeRobot.

This module is intentionally independent from ROS and from the UR5e-specific
control logic.

Responsibilities
----------------
- Load the VLA-JEPA checkpoint through the official LeRobot API.
- Load the serialized checkpoint preprocessor and postprocessor.
- Execute the official inference pipeline:

      raw LeRobot observation
          -> checkpoint preprocessor
          -> VLAJEPAPolicy.select_action()
          -> checkpoint postprocessor
          -> one physical action

- Reset the internal LeRobot action queue and processor state.
- Expose checkpoint metadata useful to the higher-level controller.

Non-responsibilities
--------------------
This module does NOT:
- read ROS messages;
- crop or resize UR5e camera images;
- construct robot proprioceptive state;
- normalize or denormalize actions manually;
- interpret the seven action dimensions;
- rescale dataset actions by the UR5e dataset SCALE_FACTOR;
- convert delta actions to absolute robot targets;
- convert the gripper output to MoveIt commands.

Those responsibilities belong to vla_jepa_controller.py and
vla_jepa_utils.py.
"""

from __future__ import annotations

from contextlib import nullcontext
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
    Thin wrapper around a pretrained LeRobot VLA-JEPA policy.

    Parameters
    ----------
    checkpoint_path:
        Directory containing the LeRobot ``pretrained_model`` checkpoint.

        Expected files include at least:
            - config.json
            - model.safetensors
            - policy_preprocessor.json
            - policy_preprocessor_step_*.safetensors
            - policy_postprocessor.json
            - policy_postprocessor_step_*.safetensors

    device:
        Device used for policy inference. On the DGX Spark this is normally
        ``"cuda"``.

    Notes
    -----
    The checkpoint currently used for the UR5e expects two visual features
    internally:

        observation.images.exterior_1_left
        observation.images.exterior_2_left

    However, the serialized preprocessor performs the mapping:

        observation.images.front
            -> observation.images.exterior_1_left

        observation.images.gripper
            -> observation.images.exterior_2_left

    Therefore this runtime does NOT require callers to use the internal
    ``exterior_*`` names.

    The exact raw input contract is defined by the serialized preprocessor.
    """

    POLICY_TYPE = "vla_jepa"

    def __init__(
        self,
        checkpoint_path: str | Path,
        device: str = "cuda",
    ) -> None:
        self.checkpoint_path = str(checkpoint_path)  # path in cui ci sono i safetensor del checkpoint
        self.device = torch.device(device)

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
        Load config, policy and checkpoint processor pipelines.

        The processor pipelines are loaded from the checkpoint itself.
        Normalization statistics, observation renaming and gripper processing
        are therefore not reconstructed manually.
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
        # 1. Configuration saved with the checkpoint
        # --------------------------------------------------------------
        config = PreTrainedConfig.from_pretrained(
            self.checkpoint_path
        )

        if config.type != self.POLICY_TYPE:
            raise ValueError(
                "Checkpoint policy type mismatch: "
                f"expected '{self.POLICY_TYPE}', "
                f"got '{config.type}'."
            )

        # Runtime device may differ from the device saved during training.
        config.device = str(self.device)

        # --------------------------------------------------------------
        # 2. Policy
        # --------------------------------------------------------------
        policy_cls = get_policy_class(config.type)

        policy = policy_cls.from_pretrained(
            self.checkpoint_path,
            config=config,
        )

        policy.to(self.device)
        policy.eval()

        # --------------------------------------------------------------
        # 3. Serialized checkpoint processors
        #
        # Important:
        # - do NOT rebuild normalization statistics;
        # - do NOT recreate the front/gripper rename map;
        # - do NOT recreate the gripper postprocessing.
        #
        # The only runtime override is the device of the preprocessor.
        # This preserves every other checkpoint setting exactly as saved.
        # --------------------------------------------------------------
        preprocessor, postprocessor = make_pre_post_processors(
            policy_cfg=config,
            pretrained_path=self.checkpoint_path,
            preprocessor_overrides={
                "device_processor": {
                    "device": str(self.device),
                }
            },
        )

        self.config = config
        self.policy = policy
        self.preprocessor = preprocessor
        self.postprocessor = postprocessor

        self._loaded = True

        # Start from an empty action queue and clean processor state.
        self.reset()

        LOGGER.info(
            "VLA-JEPA runtime loaded successfully. "
            "model_image_features=%s, "
            "uses_state=%s, "
            "action_dim=%s, "
            "chunk_size=%s, "
            "n_action_steps=%s",
            self.model_image_feature_keys,
            self.uses_state,
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
        Execute one LeRobot inference step.

        Parameters
        ----------
        observation:
            Raw observation BEFORE the serialized LeRobot preprocessor.

            For the current UR5e checkpoint the intended structure is:

            {
                "observation.images.front": Tensor[C, 224, 224],
                "observation.images.gripper": Tensor[C, 224, 224],
                "task": str,
            }

            No ``observation.state`` is required by the current checkpoint.

            Images must already contain the deterministic UR5e preprocessing
            used during training:
                - front: crop + resize 224x224
                - gripper: resize 224x224

            Batching, feature renaming, device movement and checkpoint
            normalization are handled by the serialized LeRobot preprocessor.

        Returns
        -------
        torch.Tensor
            One postprocessed action with shape ``[action_dim]``.

            For the current checkpoint:
                shape == [7]

            The tensor is returned on CPU.

            IMPORTANT:
            the physical interpretation and UR5e-specific rescaling of the
            seven values is deliberately NOT performed here.
        """

        self._require_loaded()

        if not isinstance(observation, dict):
            raise TypeError(
                "VLA-JEPA observation must be a dictionary, "
                f"got {type(observation).__name__}."
            )

        if "task" not in observation:
            raise KeyError(
                "VLA-JEPA observation is missing the required "
                "'task' instruction."
            )

        # Keep caller-owned top-level mapping untouched.
        policy_input = dict(observation)

        # --------------------------------------------------------------
        # Preprocessing
        #
        # Current checkpoint:
        #
        # front   -> exterior_1_left
        # gripper -> exterior_2_left
        #
        # then:
        #   batching
        #   device -> CUDA
        #   normalizer
        # --------------------------------------------------------------
        policy_input = self.preprocessor(policy_input)

        # Validate AFTER preprocessing.
        #
        # The checkpoint config describes the feature names seen by the
        # policy, not necessarily the raw feature names supplied by the
        # robot controller.
        self._validate_processed_observation(policy_input)

        # --------------------------------------------------------------
        # Policy inference
        #
        # VLAJEPAPolicy.select_action() owns the internal action queue.
        # Do NOT create a second chunk buffer here.
        # --------------------------------------------------------------
        use_amp = bool(
            getattr(self.config, "use_amp", False)
        )

        autocast_context = (
            torch.autocast(device_type=self.device.type)
            if use_amp and self.device.type == "cuda"
            else nullcontext()
        )

        with torch.inference_mode(), autocast_context:
            action = self.policy.select_action(policy_input)

        # --------------------------------------------------------------
        # Checkpoint postprocessing
        #
        # Current checkpoint pipeline:
        #
        # normalized action
        #       -> clip [-1, 1]
        #       -> pre-snap gripper
        #       -> MIN_MAX unnormalization
        #       -> binarize gripper
        #       -> CPU
        # --------------------------------------------------------------
        action = self.postprocessor(action)

        if not isinstance(action, torch.Tensor):
            raise TypeError(
                "VLA-JEPA postprocessor returned an unexpected type: "
                f"{type(action).__name__}."
            )

        # select_action normally returns [B, action_dim] with B=1.
        if action.ndim == 2:
            if action.shape[0] != 1:
                raise ValueError(
                    "VLA-JEPA runtime currently supports a single "
                    "observation at inference time, but received "
                    f"an action batch with shape {tuple(action.shape)}."
                )

            action = action.squeeze(0)

        if action.ndim != 1:
            raise ValueError(
                "Expected one VLA-JEPA action with shape "
                f"[action_dim], got {tuple(action.shape)}."
            )

        if (
            self.action_dim is not None
            and action.shape[0] != self.action_dim
        ):
            raise ValueError(
                "VLA-JEPA action dimensionality mismatch: "
                f"checkpoint expects {self.action_dim}, "
                f"postprocessor returned {action.shape[0]}."
            )

        return action.detach().cpu()

    # ------------------------------------------------------------------
    # Runtime state
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """
        Reset all stateful inference components.

        In particular, VLAJEPAPolicy internally caches the generated action
        chunk. Resetting the policy clears that queue.
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
    def input_features(self) -> dict[str, Any]:
        """
        Model input feature specification from config.json.

        These are the feature names AFTER preprocessing.
        """

        if self.config is None:
            return {}

        return dict(
            getattr(self.config, "input_features", {})
        )

    @property
    def output_features(self) -> dict[str, Any]:
        """Model output feature specification from config.json."""

        if self.config is None:
            return {}

        return dict(
            getattr(self.config, "output_features", {})
        )

    @property
    def model_image_feature_keys(self) -> tuple[str, ...]:
        """
        Image keys expected internally by VLA-JEPA AFTER preprocessing.

        Current checkpoint:
            observation.images.exterior_1_left
            observation.images.exterior_2_left
        """

        if self.config is None:
            return ()

        image_features = getattr(
            self.config,
            "image_features",
            {},
        )

        return tuple(image_features.keys())

    @property
    def uses_state(self) -> bool:
        """
        Whether observation.state is an actual input feature.

        The current UR5e checkpoint has state_dim=13 configured internally,
        but observation.state is NOT present in input_features. Therefore
        the current checkpoint does not receive proprioceptive state through
        the normal LeRobot inference input contract.
        """

        return "observation.state" in self.input_features

    @property
    def configured_state_dim(self) -> int | None:
        """
        State dimension configured in VLA-JEPA.

        Note that a configured state_dim does not imply that state is an
        active input feature. Check ``uses_state`` as well.
        """

        if self.config is None:
            return None

        value = getattr(
            self.config,
            "state_dim",
            None,
        )

        return (
            int(value)
            if value is not None
            else None
        )

    @property
    def action_dim(self) -> int | None:
        if self.config is None:
            return None

        value = getattr(
            self.config,
            "action_dim",
            None,
        )

        return (
            int(value)
            if value is not None
            else None
        )

    @property
    def chunk_size(self) -> int | None:
        if self.config is None:
            return None

        value = getattr(
            self.config,
            "chunk_size",
            None,
        )

        return (
            int(value)
            if value is not None
            else None
        )

    @property
    def n_action_steps(self) -> int | None:
        if self.config is None:
            return None

        value = getattr(
            self.config,
            "n_action_steps",
            None,
        )

        return (
            int(value)
            if value is not None
            else None
        )

    def describe_checkpoint(self) -> dict[str, Any]:
        """
        Return checkpoint properties relevant to the ROS integration.

        Useful for startup diagnostics before executing the real robot.
        """

        self._require_loaded()

        return {
            "policy_type": self.config.type,
            "device": str(self.device),

            "model_image_features": list(
                self.model_image_feature_keys
            ),

            "input_features": {
                key: str(value)
                for key, value in self.input_features.items()
            },

            "output_features": {
                key: str(value)
                for key, value in self.output_features.items()
            },

            "uses_state": self.uses_state,

            "configured_state_dim":
                self.configured_state_dim,

            "action_dim":
                self.action_dim,

            "chunk_size":
                self.chunk_size,

            "n_action_steps":
                self.n_action_steps,

            "num_inference_timesteps":
                getattr(
                    self.config,
                    "num_inference_timesteps",
                    None,
                ),

            "enable_world_model":
                getattr(
                    self.config,
                    "enable_world_model",
                    None,
                ),

            "resize_images_to":
                getattr(
                    self.config,
                    "resize_images_to",
                    None,
                ),

            "gripper_dim":
                getattr(
                    self.config,
                    "gripper_dim",
                    None,
                ),

            "gripper_threshold":
                getattr(
                    self.config,
                    "gripper_threshold",
                    None,
                ),

            "binarize_gripper_action":
                getattr(
                    self.config,
                    "binarize_gripper_action",
                    None,
                ),

            "pre_snap_gripper_action":
                getattr(
                    self.config,
                    "pre_snap_gripper_action",
                    None,
                ),

            "clip_normalized_actions":
                getattr(
                    self.config,
                    "clip_normalized_actions",
                    None,
                ),

            "torch_dtype":
                getattr(
                    self.config,
                    "torch_dtype",
                    None,
                ),
        }

    # ------------------------------------------------------------------
    # Internal validation
    # ------------------------------------------------------------------

    def _validate_device(self) -> None:
        if (
            self.device.type == "cuda"
            and not torch.cuda.is_available()
        ):
            raise RuntimeError(
                "VLA-JEPA was configured to run on CUDA, "
                "but torch.cuda.is_available() returned False."
            )

    def _require_loaded(self) -> None:
        if not self._loaded:
            raise RuntimeError(
                "VLA-JEPA runtime has not been loaded. "
                "Call load() before inference."
            )

    def _validate_processed_observation(
        self,
        observation: dict[str, Any],
    ) -> None:
        """
        Validate the observation AFTER the checkpoint preprocessor.

        This is deliberately done after preprocessing because the serialized
        processor renames the raw UR5e camera keys to the internal feature
        names stored in config.json.
        """

        missing_images = [
            key
            for key in self.model_image_feature_keys
            if key not in observation
        ]

        if missing_images:
            raise KeyError(
                "VLA-JEPA preprocessor did not produce all image "
                "features required by the checkpoint. "
                f"Missing: {missing_images}. "
                f"Expected: {list(self.model_image_feature_keys)}. "
                f"Available: {list(observation.keys())}."
            )

        if self.uses_state:
            if "observation.state" not in observation:
                raise KeyError(
                    "Checkpoint declares observation.state as an "
                    "input feature, but the processed observation "
                    "does not contain it."
                )

        if "task" not in observation:
            raise KeyError(
                "Task instruction disappeared during preprocessing."
            )