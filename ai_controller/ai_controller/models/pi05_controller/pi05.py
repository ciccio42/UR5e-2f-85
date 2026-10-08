"""
Runtime wrapper for PI0.5 through LeRobot.

This module is intentionally independent from ROS and from the UR5e-specific
control logic.

Responsibilities
----------------
- Load the PI0.5 checkpoint through the LeRobot API.
- Load the serialized checkpoint preprocessor and postprocessor.
- Execute the inference pipeline:

      raw LeRobot observation
          -> checkpoint preprocessor
          -> PI05Policy.select_action()
          -> checkpoint postprocessor
          -> one dataset-space action

- Reset the internal LeRobot action queue and processor state.
- Expose checkpoint metadata useful to the higher-level controller.

Non-responsibilities
--------------------
This module does NOT:
- read ROS messages;
- select camera indexes;
- crop or resize UR5e images;
- convert RGB/BGR images;
- construct the 13D UR5e proprioceptive state;
- normalize or denormalize state/actions manually;
- pad the state to PI0.5 max_state_dim;
- rescale dataset actions by 0.05;
- convert delta actions to absolute robot targets;
- convert the gripper output to MoveIt commands.

Those responsibilities belong to pi05_controller.py and pi05_utils.py.
"""

from __future__ import annotations

from contextlib import nullcontext
import logging
from pathlib import Path
from typing import Any
import os


import torch

from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import (
    get_policy_class,
    make_pre_post_processors,
)


LOGGER = logging.getLogger(__name__)


# =============================================================================
# CURRENT UR5e / PI0.5 CHECKPOINT CONTRACT
# =============================================================================

POLICY_TYPE = "pi05"

IMAGE_SIZE = 224
ACTION_DIM = 7

# Raw state actually used to compute the checkpoint normalization statistics:
#
#   joint_position       6
#   gripper              1
#   EEF state            6
#   ----------------------
#   total               13
#
# Order:
#
#   [
#       q1, q2, q3, q4, q5, q6,
#       gripper,
#       x, y, z, roll, pitch, yaw,
#   ]
#
# PI0.5 later handles its own internal preparation / padding.
RAW_STATE_DIM = 13


# Keys supplied by our future pi05_utils.py BEFORE the serialized
# LeRobot preprocessor.
RAW_FRONT_KEY = "observation.images.front"
RAW_GRIPPER_KEY = "observation.images.gripper"
RAW_STATE_KEY = "observation.state"
TASK_KEY = "task"


# The serialized checkpoint preprocessor renames:
#
#   observation.images.front
#       -> observation.images.base_0_rgb
#
#   observation.images.gripper
#       -> observation.images.left_wrist_0_rgb
#
# right_wrist_0_rgb is declared by the generic PI0.5 config but is not
# supplied by the UR5e dataset/runtime currently reconstructed.
INTERNAL_FRONT_KEY = "observation.images.base_0_rgb"
INTERNAL_GRIPPER_KEY = "observation.images.left_wrist_0_rgb"

OPTIONAL_RIGHT_WRIST_KEY = "observation.images.right_wrist_0_rgb"


# =============================================================================
# RUNTIME
# =============================================================================


class PI05Runtime:
    """
    Thin runtime wrapper around a pretrained LeRobot PI0.5 policy.

    Parameters
    ----------
    checkpoint_path:
        Directory containing the LeRobot ``pretrained_model`` checkpoint.

        Expected files include:
            - config.json
            - model.safetensors
            - policy_preprocessor.json
            - policy_preprocessor_step_*.safetensors
            - policy_postprocessor.json
            - policy_postprocessor_step_*.safetensors

    device:
        Device used for inference. On the DGX Spark this is normally ``cuda``.

    postprocessor_config_filename:
        Serialized LeRobot postprocessor JSON to load.

    Raw observation contract
    ------------------------
    The current UR5e PI0.5 deployment supplies:

        {
            "observation.images.front":
                Tensor[3, 224, 224],

            "observation.images.gripper":
                Tensor[3, 224, 224],

            "observation.state":
                Tensor[13],

            "task":
                str,
        }

    Image tensors are expected to be float32 RGB tensors in [0, 1].

    observation.state has the order:

        [
            q1, q2, q3, q4, q5, q6,
            gripper,
            x, y, z, roll, pitch, yaw,
        ]

    This wrapper does NOT construct those values.
    """

    POLICY_TYPE = POLICY_TYPE

    def __init__(
        self,
        checkpoint_path: str | Path,
        device: str = "cuda",
        postprocessor_config_filename: str = "policy_postprocessor.json",
        n_action_steps: int | None = None,
    ) -> None:

        self.checkpoint_path = str(
            Path(checkpoint_path).expanduser().resolve()
        )

        self.device = torch.device(device)

        self.postprocessor_config_filename = str(
            postprocessor_config_filename
        )

        self.config: PreTrainedConfig | None = None
        self.policy: torch.nn.Module | None = None

        self.preprocessor: Any | None = None
        self.postprocessor: Any | None = None

        self._loaded = False

        self.n_action_steps_override = (
            None
            if n_action_steps is None
            else int(n_action_steps)
        )


    # =========================================================================
    # LOADING
    # =========================================================================

    def load(self) -> None:
        """
        Load config, PI0.5 policy and checkpoint processor pipelines.

        Normalization statistics, observation renaming, PI0.5 state
        preparation, tokenization and action unnormalization are loaded
        directly from the serialized checkpoint.
        """

        if self._loaded:
            LOGGER.info(
                "PI0.5 runtime is already loaded."
            )
            return

        self._validate_device()
        self._validate_checkpoint_files()

        LOGGER.info(
            "Loading PI0.5 checkpoint from '%s' on device '%s'.",
            self.checkpoint_path,
            self.device,
        )

        # ---------------------------------------------------------------------
        # 1. Checkpoint configuration
        # ---------------------------------------------------------------------

        config = PreTrainedConfig.from_pretrained(
            self.checkpoint_path
        )

        paligemma_path = Path(
            os.environ.get(
                "PI05_PALIGEMMA_PATH",
                "/opt/pi05/paligemma-3b-pt-224",
            )
        ).expanduser().resolve()

        if not paligemma_path.is_dir():
            raise FileNotFoundError(
                "Local PaliGemma tokenizer directory not found: "
                f"{paligemma_path}"
            )

        # Keep the runtime config consistent with the local tokenizer.
        config.text_tokenizer_name = str(paligemma_path)

        checkpoint_n_action_steps = int(
            config.n_action_steps
        )

        chunk_size = int(
            config.chunk_size
        )

        if self.n_action_steps_override is not None:

            requested_n_action_steps = int(
                self.n_action_steps_override
            )

            if requested_n_action_steps < 1:
                raise ValueError(
                    "n_action_steps must be >= 1, "
                    f"got {requested_n_action_steps}."
                )

            if requested_n_action_steps > chunk_size:
                raise ValueError(
                    "n_action_steps cannot exceed chunk_size: "
                    f"n_action_steps={requested_n_action_steps}, "
                    f"chunk_size={chunk_size}."
                )

            config.n_action_steps = (
                requested_n_action_steps
            )

            LOGGER.info(
                "Overriding PI0.5 n_action_steps: "
                "%d -> %d",
                checkpoint_n_action_steps,
                requested_n_action_steps,
            )

        if config.type != self.POLICY_TYPE:
            raise ValueError(
                "Checkpoint policy type mismatch: "
                f"expected '{self.POLICY_TYPE}', "
                f"got '{config.type}'."
            )

        # Runtime device may differ from the value serialized during training.
        config.device = str(self.device)

        # ---------------------------------------------------------------------
        # 2. Validate the checkpoint contract before loading the heavy model
        # ---------------------------------------------------------------------

        self._validate_config_contract(
            config
        )

        # ---------------------------------------------------------------------
        # 3. PI0.5 policy
        # ---------------------------------------------------------------------

        policy_cls = get_policy_class(
            config.type
        )

        policy = policy_cls.from_pretrained(
            self.checkpoint_path,
            config=config,
        )

        policy.to(
            self.device
        )

        policy.eval()

        # ---------------------------------------------------------------------
        # 4. Serialized checkpoint processors
        #
        # IMPORTANT:
        #
        # Do NOT manually recreate:
        #
        #   - observation rename map;
        #   - state QUANTILES normalization;
        #   - action QUANTILES normalization;
        #   - PI0.5 state preparation;
        #   - PaliGemma tokenization;
        #   - action unnormalization.
        #
        # Those operations are serialized with the checkpoint.
        #
        # The only runtime override is the target device.
        # ---------------------------------------------------------------------

        preprocessor, postprocessor = make_pre_post_processors(
            policy_cfg=config,
            pretrained_path=self.checkpoint_path,
            postprocessor_config_filename=
                self.postprocessor_config_filename,
            preprocessor_overrides={
                "device_processor": {
                    "device": str(self.device),
                },
                 "tokenizer_processor": {
                    "tokenizer_name": str(paligemma_path),
                },
            },
        )

        self.config = config
        self.policy = policy
        self.preprocessor = preprocessor
        self.postprocessor = postprocessor

        self._loaded = True

        # Empty the internal action queue and reset processor state.
        self.reset()

        LOGGER.info(
            "PI0.5 runtime loaded successfully. "
            "image_features=%s, "
            "raw_state_dim=%s, "
            "model_state_dim=%s, "
            "action_dim=%s, "
            "chunk_size=%s, "
            "n_action_steps=%s, "
            "num_inference_steps=%s",
            self.model_image_feature_keys,
            RAW_STATE_DIM,
            self.model_state_feature_dim,
            self.action_dim,
            self.chunk_size,
            self.n_action_steps,
            self.num_inference_steps,
        )


    # =========================================================================
    # INFERENCE
    # =========================================================================

    def select_action(
        self,
        observation: dict[str, Any],
    ) -> torch.Tensor:
        """
        Execute one complete LeRobot PI0.5 inference call.

        Parameters
        ----------
        observation:
            Raw observation BEFORE the serialized LeRobot preprocessor.

            Expected current UR5e format:

                {
                    "observation.images.front":
                        Tensor[3, 224, 224],

                    "observation.images.gripper":
                        Tensor[3, 224, 224],

                    "observation.state":
                        Tensor[13],

                    "task":
                        str,
                }

        Returns
        -------
        torch.Tensor
            One postprocessed action with shape:

                [7]

            returned on CPU.

        Important
        ---------
        PI05Policy.select_action() manages its own action queue.

        The current checkpoint declares:

            chunk_size = 50
            n_action_steps = 50

        Therefore this wrapper deliberately does NOT implement another
        action-chunk buffer.
        """

        self._require_loaded()

        # Validate the exact robot-side contract BEFORE giving the observation
        # to LeRobot.
        self._validate_raw_observation(
            observation
        )

        # Keep caller-owned top-level dictionary untouched.
        policy_input = dict(
            observation
        )

        # ---------------------------------------------------------------------
        # 1. Serialized LeRobot preprocessing
        #
        # Current checkpoint pipeline includes:
        #
        #   front -> base_0_rgb
        #   gripper -> left_wrist_0_rgb
        #       ↓
        #   batching
        #       ↓
        #   relative-actions processor (disabled)
        #       ↓
        #   QUANTILES normalization
        #       ↓
        #   PI0.5 state preparation
        #       ↓
        #   PaliGemma tokenizer
        #       ↓
        #   device -> CUDA
        #
        # The raw 13D state is therefore NOT padded by us.
        # ---------------------------------------------------------------------

        policy_input = self.preprocessor(
            policy_input
        )

        self._validate_processed_observation(
            policy_input
        )

        # ---------------------------------------------------------------------
        # 2. PI0.5 inference
        # ---------------------------------------------------------------------

        use_amp = bool(
            getattr(
                self.config,
                "use_amp",
                False,
            )
        )

        autocast_context = (
            torch.autocast(
                device_type=self.device.type
            )
            if (
                use_amp
                and self.device.type == "cuda"
            )
            else nullcontext()
        )

        with torch.inference_mode(), autocast_context:
            action = self.policy.select_action(
                policy_input
            )

        # ---------------------------------------------------------------------
        # 3. Serialized LeRobot postprocessing
        #
        # Current checkpoint:
        #
        #   predicted normalized action
        #       ↓
        #   QUANTILES unnormalization
        #       ↓
        #   absolute-actions processor (disabled)
        #       ↓
        #   CPU
        #
        # The result is therefore in the dataset action representation.
        #
        # UR5e-specific:
        #
        #   * 0.05 scale recovery
        #   * delta -> absolute pose
        #   * gripper -> MoveIt
        #
        # happen OUTSIDE this runtime.
        # ---------------------------------------------------------------------

        action = self.postprocessor(
            action
        )

        if not isinstance(
            action,
            torch.Tensor,
        ):
            raise TypeError(
                "PI0.5 postprocessor returned an unexpected type: "
                f"{type(action).__name__}."
            )

        # LeRobot normally returns [B, action_dim] with B == 1.
        if action.ndim == 2:

            if action.shape[0] != 1:
                raise ValueError(
                    "PI0.5 runtime currently supports one observation "
                    "at a time, but received action batch shape "
                    f"{tuple(action.shape)}."
                )

            action = action.squeeze(0)

        if action.ndim != 1:
            raise ValueError(
                "Expected one PI0.5 action with shape "
                f"[{ACTION_DIM}], got {tuple(action.shape)}."
            )

        if action.shape[0] != ACTION_DIM:
            raise ValueError(
                "PI0.5 action dimensionality mismatch: "
                f"expected {ACTION_DIM}, "
                f"got {action.shape[0]}."
            )

        if not torch.isfinite(
            action
        ).all():
            raise ValueError(
                "PI0.5 postprocessed action contains non-finite values: "
                f"{action}"
            )

        return (
            action
            .detach()
            .cpu()
        )


    # =========================================================================
    # RESET
    # =========================================================================

    def reset(self) -> None:
        """
        Reset all stateful inference components.

        In particular, PI05Policy internally owns an action queue.
        Resetting the policy prevents actions from a previous episode from
        leaking into the next one.
        """

        if not self._loaded:
            return

        self.policy.reset()

        if hasattr(
            self.preprocessor,
            "reset",
        ):
            self.preprocessor.reset()

        if hasattr(
            self.postprocessor,
            "reset",
        ):
            self.postprocessor.reset()


    # =========================================================================
    # CHECKPOINT METADATA
    # =========================================================================

    @property
    def input_features(
        self,
    ) -> dict[str, Any]:

        if self.config is None:
            return {}

        return dict(
            getattr(
                self.config,
                "input_features",
                {},
            )
            or {}
        )


    @property
    def output_features(
        self,
    ) -> dict[str, Any]:

        if self.config is None:
            return {}

        return dict(
            getattr(
                self.config,
                "output_features",
                {},
            )
            or {}
        )


    @property
    def model_image_feature_keys(
        self,
    ) -> tuple[str, ...]:
        """
        Image features declared by config.json.

        Note:
            the current checkpoint may declare right_wrist_0_rgb even though
            the UR5e deployment only provides front + gripper.

        Therefore this property is informational only.
        """

        return tuple(
            key
            for key in self.input_features
            if key.startswith(
                "observation.images."
            )
        )


    @property
    def required_internal_image_keys(
        self,
    ) -> tuple[str, ...]:
        """
        Image features actually required from the current UR5e raw input
        after the serialized rename processor.
        """

        return (
            INTERNAL_FRONT_KEY,
            INTERNAL_GRIPPER_KEY,
        )


    @property
    def raw_state_dim(
        self,
    ) -> int:
        """
        Real state dimensionality entering the checkpoint normalizer.
        """

        return RAW_STATE_DIM


    @property
    def model_state_feature_dim(
        self,
    ) -> int | None:
        """
        State dimension declared by config.json.

        For the current PI0.5 checkpoint this is expected to be 32.

        IMPORTANT:
            this is NOT the dimension of the raw UR5e state.

        The checkpoint normalizer statistics prove that the raw state supplied
        during training is 13D. PI0.5 performs its own later preparation.
        """

        feature = self.input_features.get(
            RAW_STATE_KEY
        )

        if feature is None:
            return None

        shape = getattr(
            feature,
            "shape",
            None,
        )

        if not shape:
            return None

        return int(
            shape[0]
        )


    @property
    def action_dim(
        self,
    ) -> int | None:

        feature = self.output_features.get(
            "action"
        )

        if feature is None:
            return None

        shape = getattr(
            feature,
            "shape",
            None,
        )

        if not shape:
            return None

        return int(
            shape[0]
        )


    @property
    def chunk_size(
        self,
    ) -> int | None:

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
    def n_action_steps(
        self,
    ) -> int | None:

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


    @property
    def num_inference_steps(
        self,
    ) -> int | None:

        if self.config is None:
            return None

        value = getattr(
            self.config,
            "num_inference_steps",
            None,
        )

        return (
            int(value)
            if value is not None
            else None
        )


    def describe_checkpoint(
        self,
    ) -> dict[str, Any]:
        """
        Return checkpoint properties relevant to the UR5e integration.
        """

        self._require_loaded()

        return {
            "policy_type":
                self.config.type,

            "device":
                str(self.device),

            "input_features": {
                key: str(value)
                for key, value
                in self.input_features.items()
            },

            "output_features": {
                key: str(value)
                for key, value
                in self.output_features.items()
            },

            "declared_image_features":
                list(
                    self.model_image_feature_keys
                ),

            "required_ur5e_images":
                [
                    RAW_FRONT_KEY,
                    RAW_GRIPPER_KEY,
                ],

            "raw_state_dim":
                RAW_STATE_DIM,

            "raw_state_order": [
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6",
                "gripper",
                "eef_x",
                "eef_y",
                "eef_z",
                "eef_roll",
                "eef_pitch",
                "eef_yaw",
            ],

            "model_state_feature_dim":
                self.model_state_feature_dim,

            "max_state_dim":
                getattr(
                    self.config,
                    "max_state_dim",
                    None,
                ),

            "action_dim":
                self.action_dim,

            "max_action_dim":
                getattr(
                    self.config,
                    "max_action_dim",
                    None,
                ),

            "chunk_size":
                self.chunk_size,

            "n_action_steps":
                self.n_action_steps,

            "num_inference_steps":
                self.num_inference_steps,

            "dtype":
                getattr(
                    self.config,
                    "dtype",
                    None,
                ),

            "use_amp":
                getattr(
                    self.config,
                    "use_amp",
                    None,
                ),

            "use_relative_actions":
                getattr(
                    self.config,
                    "use_relative_actions",
                    None,
                ),

            "image_resolution":
                getattr(
                    self.config,
                    "image_resolution",
                    None,
                ),

            "normalization_mapping":
                getattr(
                    self.config,
                    "normalization_mapping",
                    None,
                ),
        }


    # =========================================================================
    # INTERNAL VALIDATION
    # =========================================================================

    def _validate_device(
        self,
    ) -> None:

        if (
            self.device.type == "cuda"
            and not torch.cuda.is_available()
        ):
            raise RuntimeError(
                "PI0.5 was configured to run on CUDA, "
                "but torch.cuda.is_available() returned False."
            )


    def _validate_checkpoint_files(
        self,
    ) -> None:

        checkpoint = Path(
            self.checkpoint_path
        )

        if not checkpoint.is_dir():
            raise FileNotFoundError(
                "PI0.5 checkpoint directory does not exist: "
                f"{checkpoint}"
            )

        required_files = (
            "config.json",
            "model.safetensors",
            "policy_preprocessor.json",
            "policy_postprocessor.json",
        )

        missing = [
            filename
            for filename in required_files
            if not (
                checkpoint / filename
            ).is_file()
        ]

        if missing:
            raise FileNotFoundError(
                "PI0.5 checkpoint is incomplete. "
                f"Missing files: {missing}"
            )


    @staticmethod
    def _validate_config_contract(
        config: PreTrainedConfig,
    ) -> None:
        """
        Validate the known current UR5e PI0.5 checkpoint contract.

        The right-wrist image is deliberately NOT required.
        """

        input_features = dict(
            getattr(
                config,
                "input_features",
                {},
            )
            or {}
        )

        output_features = dict(
            getattr(
                config,
                "output_features",
                {},
            )
            or {}
        )

        for required_key in (
            INTERNAL_FRONT_KEY,
            INTERNAL_GRIPPER_KEY,
            RAW_STATE_KEY,
        ):
            if required_key not in input_features:
                raise RuntimeError(
                    "Current UR5e PI0.5 checkpoint is missing "
                    f"required input feature {required_key!r}."
                )

        action_feature = output_features.get(
            "action"
        )

        if action_feature is None:
            raise RuntimeError(
                "PI0.5 checkpoint does not define output feature 'action'."
            )

        action_shape = tuple(
            action_feature.shape
        )

        if action_shape != (
            ACTION_DIM,
        ):
            raise RuntimeError(
                "Current UR5e controller expects action shape "
                f"({ACTION_DIM},), got {action_shape}."
            )

        image_resolution = getattr(
            config,
            "image_resolution",
            None,
        )

        if (
            image_resolution is not None
            and tuple(image_resolution)
            != (IMAGE_SIZE, IMAGE_SIZE)
        ):
            raise RuntimeError(
                "PI0.5 checkpoint image resolution mismatch: "
                f"expected {(IMAGE_SIZE, IMAGE_SIZE)}, "
                f"got {image_resolution}."
            )


    def _validate_raw_observation(
        self,
        observation: dict[str, Any],
    ) -> None:
        """
        Validate the robot-side input BEFORE LeRobot preprocessing.
        """

        if not isinstance(
            observation,
            dict,
        ):
            raise TypeError(
                "PI0.5 observation must be a dictionary, "
                f"got {type(observation).__name__}."
            )

        required_keys = (
            RAW_FRONT_KEY,
            RAW_GRIPPER_KEY,
            RAW_STATE_KEY,
            TASK_KEY,
        )

        missing = [
            key
            for key in required_keys
            if key not in observation
        ]

        if missing:
            raise KeyError(
                "PI0.5 raw observation is missing required keys: "
                f"{missing}"
            )

        # ---------------------------------------------------------------------
        # Images
        # ---------------------------------------------------------------------

        for key in (
            RAW_FRONT_KEY,
            RAW_GRIPPER_KEY,
        ):

            image = observation[key]

            if not isinstance(
                image,
                torch.Tensor,
            ):
                raise TypeError(
                    f"{key} must be a torch.Tensor, "
                    f"got {type(image).__name__}."
                )

            expected_shape = (
                3,
                IMAGE_SIZE,
                IMAGE_SIZE,
            )

            if tuple(
                image.shape
            ) != expected_shape:
                raise ValueError(
                    f"{key} must have shape {expected_shape}, "
                    f"got {tuple(image.shape)}."
                )

            if image.dtype != torch.float32:
                raise TypeError(
                    f"{key} must have dtype torch.float32, "
                    f"got {image.dtype}."
                )

            if not torch.isfinite(
                image
            ).all():
                raise ValueError(
                    f"{key} contains non-finite values."
                )

            image_min = float(
                image.min().item()
            )

            image_max = float(
                image.max().item()
            )

            if (
                image_min < 0.0
                or image_max > 1.0
            ):
                raise ValueError(
                    f"{key} must be in [0, 1], "
                    f"got range [{image_min}, {image_max}]."
                )

        # ---------------------------------------------------------------------
        # State
        # ---------------------------------------------------------------------

        state = observation[
            RAW_STATE_KEY
        ]

        if not isinstance(
            state,
            torch.Tensor,
        ):
            raise TypeError(
                "observation.state must be a torch.Tensor, "
                f"got {type(state).__name__}."
            )

        if tuple(
            state.shape
        ) != (
            RAW_STATE_DIM,
        ):
            raise ValueError(
                "Raw PI0.5 observation.state must have shape "
                f"({RAW_STATE_DIM},), got {tuple(state.shape)}."
            )

        if state.dtype != torch.float32:
            raise TypeError(
                "observation.state must have dtype torch.float32, "
                f"got {state.dtype}."
            )

        if not torch.isfinite(
            state
        ).all():
            raise ValueError(
                "observation.state contains non-finite values."
            )

        # ---------------------------------------------------------------------
        # Task
        # ---------------------------------------------------------------------

        task = observation[
            TASK_KEY
        ]

        if not isinstance(
            task,
            str,
        ):
            raise TypeError(
                "PI0.5 task must be a string, "
                f"got {type(task).__name__}."
            )

        if not task.strip():
            raise ValueError(
                "PI0.5 task instruction must not be empty."
            )


    @staticmethod
    def _validate_processed_observation(
        observation: dict[str, Any],
    ) -> None:
        """
        Minimal validation AFTER the serialized LeRobot preprocessor.

        Only the two visual features actually produced by the UR5e camera
        mapping are required.

        right_wrist_0_rgb is deliberately NOT required.
        """

        if not isinstance(
            observation,
            dict,
        ):
            raise TypeError(
                "PI0.5 preprocessor returned an unexpected type: "
                f"{type(observation).__name__}."
            )

        missing_images = [
            key
            for key in (
                INTERNAL_FRONT_KEY,
                INTERNAL_GRIPPER_KEY,
            )
            if key not in observation
        ]

        if missing_images:
            raise KeyError(
                "PI0.5 preprocessor did not produce the expected "
                "UR5e image features. "
                f"Missing: {missing_images}. "
                f"Available: {list(observation.keys())}."
            )


    def _require_loaded(
        self,
    ) -> None:

        if not self._loaded:
            raise RuntimeError(
                "PI0.5 runtime has not been loaded. "
                "Call load() before inference."
            )