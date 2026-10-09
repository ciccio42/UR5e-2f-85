from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Optional

import numpy as np
import torch
from omegaconf import DictConfig, OmegaConf
from PIL import Image as PILImage

from ai_controller.utils.ai_controller import AIController
from ai_controller.utils.utils import seed_everything


# =============================================================================
# LOCAL IMPORTS
# =============================================================================

_THIS_DIR = os.path.dirname(
    os.path.abspath(__file__)
)

if _THIS_DIR not in sys.path:
    sys.path.insert(
        0,
        _THIS_DIR,
    )

from pi05 import PI05Runtime

from pi05_utils import (
    ACTION_DIM,
    DATASET_ACTION_SCALE,
    GRIPPER_CLOSE_THRESHOLD,
    GRIPPER_OPEN_THRESHOLD,
    IMAGE_SIZE,
    RAW_STATE_DIM,
    build_lerobot_observation,
    build_pi05_state,
    delta_action_to_absolute_target,
    gripper_binary_to_moveit,
    process_front_image,
    process_gripper_image,
    recover_physical_delta,
    validate_pi05_action,
)


# =============================================================================
# ROBOT-SIDE STATE CONTRACT
# =============================================================================

# Pose needed for delta -> absolute conversion:
#
#   [x, y, z, qx, qy, qz, qw, gripper_closed]
#
ROBOT_STATE_DIM = 8

JOINT_POSITION_DIM = 6


# =============================================================================
# PATH UTILITY
# =============================================================================


def _resolve_path(
    path: str | Path,
    base_dir: str | Path | None = None,
) -> Path:
    """
    Resolve ~, environment variables and relative paths.
    """

    expanded = os.path.expandvars(
        os.path.expanduser(
            str(path)
        )
    )

    resolved = Path(
        expanded
    )

    if not resolved.is_absolute():

        if base_dir is None:
            resolved = (
                Path.cwd()
                / resolved
            )

        else:
            resolved = (
                Path(base_dir)
                / resolved
            )

    return resolved.resolve()


# =============================================================================
# CONTROLLER
# =============================================================================


class PI05Controller(AIController):
    """
    Adapt a LeRobot PI0.5 policy to the UR5e ai_controller interface.

    Responsibilities
    ----------------
    - load PI0.5 through PI05Runtime;
    - select the language instruction for the current task;
    - select and preprocess front + gripper cameras;
    - build the 13D PI0.5 proprioceptive state;
    - build the raw LeRobot observation;
    - request one action from PI0.5;
    - convert dataset-space delta action to an absolute UR5e target;
    - convert the gripper action to the MoveIt / Robotiq convention.

    No experimental grasp correction or geometric offset is applied here.
    """

    def __init__(
        self,
        model_config: str,
        task_name: str = "pick_place",
    ) -> None:

        self.task_name = task_name

        # ---------------------------------------------------------------------
        # Runtime YAML
        # ---------------------------------------------------------------------

        self.config_path = _resolve_path(
            model_config
        )

        if not self.config_path.is_file():
            raise FileNotFoundError(
                "PI0.5 controller config not found: "
                f"{self.config_path}"
            )

        self.config_dir = (
            self.config_path.parent
        )

        self.cfg: DictConfig = OmegaConf.load(
            self.config_path
        )

        # ---------------------------------------------------------------------
        # Runtime state
        # ---------------------------------------------------------------------

        self._runtime: Optional[
            PI05Runtime
        ] = None

        self.command: Optional[
            str
        ] = None

        self.current_task_id: Optional[
            str
        ] = None

        # ---------------------------------------------------------------------
        # Camera configuration
        #
        # AIControllerNode convention:
        #
        #   images[0] = front
        #   images[1] = left
        #   images[2] = right
        #   images[3] = gripper
        #
        # PI0.5 deployment:
        #
        #   front + gripper only
        # ---------------------------------------------------------------------

        self.front_camera_index = int(
            self.cfg.get(
                "front_camera_index",
                0,
            )
        )

        self.gripper_camera_index = int(
            self.cfg.get(
                "gripper_camera_index",
                3,
            )
        )

        self.input_color_order = str(
            self.cfg.get(
                "input_color_order",
                "rgb",
            )
        ).lower()

        # ---------------------------------------------------------------------
        # Dataset action representation
        # ---------------------------------------------------------------------

        self.dataset_action_scale = float(
            self.cfg.get(
                "dataset_action_scale",
                DATASET_ACTION_SCALE,
            )
        )

        # ---------------------------------------------------------------------
        # Gripper
        # ---------------------------------------------------------------------

        self.gripper_open_position = float(
            self.cfg.get(
                "gripper_open_position",
                0.0,
            )
        )

        self.gripper_closed_position = float(
            self.cfg.get(
                "gripper_closed_position",
                255.0,
            )
        )

        self.gripper_open_threshold = float(
            self.cfg.get(
                "gripper_open_threshold",
                GRIPPER_OPEN_THRESHOLD,
            )
        )

        self.gripper_close_threshold = float(
            self.cfg.get(
                "gripper_close_threshold",
                GRIPPER_CLOSE_THRESHOLD,
            )
        )

        # ---------------------------------------------------------------------
        # Debug
        # ---------------------------------------------------------------------

        self.trace_action_conversions = bool(
            self.cfg.get(
                "trace_action_conversions",
                False,
            )
        )

        # ---------------------------------------------------------------------
        # Random seed
        #
        # PI0.5 uses iterative flow-matching inference initialized from noise.
        # ---------------------------------------------------------------------

        self.seed = int(
            self.cfg.get(
                "seed",
                1000,
            )
        )

        seed_everything(
            self.seed
        )

        # AIController.__init__() calls load_model().
        super().__init__(
            str(
                self.config_path
            )
        )


    # =========================================================================
    # MODEL LOADING
    # =========================================================================

    def load_model(
        self,
        model_config: str,
    ):
        """
        Load PI0.5 through PI05Runtime.
        """

        del model_config

        checkpoint_value = self.cfg.get(
            "checkpoint_path",
            None,
        )

        if checkpoint_value is None:
            raise KeyError(
                "PI0.5 config must define checkpoint_path."
            )

        checkpoint_path = _resolve_path(
            checkpoint_value,
            base_dir=self.config_dir,
        )

        if not checkpoint_path.is_dir():
            raise FileNotFoundError(
                "PI0.5 checkpoint directory not found: "
                f"{checkpoint_path}"
            )

        required_files = (
            "config.json",
            "model.safetensors",
            "policy_preprocessor.json",
            "policy_postprocessor.json",
        )

        missing_files = [
            filename
            for filename in required_files
            if not (
                checkpoint_path
                / filename
            ).is_file()
        ]

        if missing_files:
            raise FileNotFoundError(
                "Incomplete PI0.5 pretrained_model directory. "
                f"Missing files: {missing_files}"
            )

        device = str(
            self.cfg.get(
                "device",
                "cuda",
            )
        )

        postprocessor_config_filename = str(
            self.cfg.get(
                "postprocessor_config_filename",
                "policy_postprocessor.json",
            )
        )

        n_action_steps_cfg = self.cfg.get(
            "n_action_steps",
            None,
        )

        n_action_steps = (
            None
            if n_action_steps_cfg is None
            else int(n_action_steps_cfg)
        )

        self._runtime = PI05Runtime(
            checkpoint_path=checkpoint_path,
            device=device,
            postprocessor_config_filename=
                postprocessor_config_filename,
            n_action_steps=n_action_steps,
        )

        self._runtime.load()

        # ---------------------------------------------------------------------
        # Checkpoint contract validation
        # ---------------------------------------------------------------------

        if (
            self._runtime.action_dim
            != ACTION_DIM
        ):
            raise RuntimeError(
                "Current UR5e PI0.5 controller expects "
                f"a {ACTION_DIM}D action, but checkpoint "
                f"action_dim={self._runtime.action_dim}."
            )

        if (
            self._runtime.raw_state_dim
            != RAW_STATE_DIM
        ):
            raise RuntimeError(
                "Unexpected PI0.5 raw state dimension: "
                f"{self._runtime.raw_state_dim}."
            )

        image_resolution = getattr(
            self._runtime.config,
            "image_resolution",
            None,
        )

        if (
            image_resolution is not None
            and tuple(
                image_resolution
            )
            != (
                IMAGE_SIZE,
                IMAGE_SIZE,
            )
        ):
            raise RuntimeError(
                "Image preprocessing mismatch: "
                f"controller produces {IMAGE_SIZE}x{IMAGE_SIZE}, "
                f"checkpoint expects {image_resolution}."
            )

        print(
            "[PI05Controller] Model loaded successfully.\n"
            f"  checkpoint={checkpoint_path}\n"
            f"  device={device}\n"
            f"  declared_image_features="
            f"{self._runtime.model_image_feature_keys}\n"
            f"  raw_state_dim="
            f"{self._runtime.raw_state_dim}\n"
            f"  model_state_feature_dim="
            f"{self._runtime.model_state_feature_dim}\n"
            f"  action_dim="
            f"{self._runtime.action_dim}\n"
            f"  chunk_size="
            f"{self._runtime.chunk_size}\n"
            f"  n_action_steps="
            f"{self._runtime.n_action_steps}\n"
            f"  inference_steps="
            f"{self._runtime.num_inference_steps}"
        )

        return self._runtime.policy


    def move_model_to_device(
        self,
        device,
    ):
        """
        PI05Runtime already initializes policy and processors coherently on
        the configured device.
        """

        if self._runtime is None:
            raise RuntimeError(
                "PI0.5 runtime has not been loaded."
            )

        requested_device = torch.device(
            device
        )

        if (
            requested_device.type
            != self._runtime.device.type
        ):
            raise ValueError(
                "Cannot move only the PI0.5 policy after "
                "processor initialization. "
                f"Runtime device={self._runtime.device}, "
                f"requested={requested_device}."
            )


    # =========================================================================
    # TASK
    # =========================================================================

    def load_command(
        self,
        demo_path: str,
        task_id: str | None = None,
        **kwargs,
    ):
        """
        Select the textual instruction associated with the current task.

        PI0.5 receives the instruction directly through the LeRobot
        ``task`` field.
        """

        del demo_path
        del kwargs

        if self._runtime is None:
            raise RuntimeError(
                "PI0.5 runtime has not been loaded."
            )

        if task_id is None:
            raise KeyError(
                "task_id is required for PI0.5."
            )

        task_id = str(
            task_id
        ).zfill(
            2
        )

        tasks_cfg = self.cfg.get(
            "tasks",
            None,
        )

        if tasks_cfg is None:
            raise KeyError(
                "PI0.5 config does not define tasks."
            )

        task_cfg = tasks_cfg.get(
            task_id
        )

        if task_cfg is None:
            raise KeyError(
                f"Unknown PI0.5 task_id: {task_id}"
            )

        prompt = task_cfg.get(
            "prompt"
        )

        if not prompt:
            raise ValueError(
                f"Task {task_id} does not define a prompt."
            )

        prompt = str(
            prompt
        ).strip()

        if not prompt:
            raise ValueError(
                f"Task {task_id} defines an empty prompt."
            )

        self.current_task_id = (
            task_id
        )

        self.command = (
            prompt
        )

        # A new episode/task must never inherit actions remaining from the
        # previous PI0.5 action queue.
        self._runtime.reset()

        seed_everything(
            self.seed
        )

        print(
            f"[PI05Controller] Loaded task "
            f"{task_id}: {self.command!r}"
        )

        return self.command


    # =========================================================================
    # RESET
    # =========================================================================

    def reset(
        self,
    ):
        """
        Reset episode-specific state.

        Model weights and processors remain loaded, while the internal PI0.5
        action queue is cleared.
        """

        if self._runtime is not None:
            self._runtime.reset()

        self.command = None
        self.current_task_id = None


    # =========================================================================
    # PREPROCESS
    # =========================================================================

    def pre_process(
        self,
        input_data,
    ):
        """
        Construct the raw observation consumed by PI05Runtime.

        Expected input_data
        -------------------
        [
            images,
            robot_state,
        ]

        images
        ------
        List of live RGB camera images using the AIControllerNode convention:

            images[0] = front
            images[1] = left
            images[2] = right
            images[3] = gripper

        PI0.5 currently uses only front + gripper.

        robot_state
        -----------
        Dictionary produced by AIControllerNode._build_pi05_state():

            {
                "joint_positions": (6,),
                "gripper_qpos": (1,),
                "eef_position": (3,),
                "eef_quaternion": (4,),   # XYZW
                "gripper_closed": bool,
            }

        The controller constructs the raw PI0.5 state:

            [
                q1, q2, q3, q4, q5, q6,
                gripper,
                x, y, z, roll, pitch, yaw,
            ]

        shape = (13,)

        The quaternion -> RPY conversion is performed by build_pi05_state().
        """

        # -------------------------------------------------------------------------
        # Input contract
        # -------------------------------------------------------------------------

        if (
            not isinstance(
                input_data,
                (list, tuple),
            )
            or len(input_data) != 2
        ):
            raise ValueError(
                "PI0.5 input_data must be:\n"
                "  [images, robot_state]"
            )

        (
            images,
            robot_state,
        ) = input_data

        if self._runtime is None:
            raise RuntimeError(
                "PI0.5 runtime has not been loaded."
            )

        if self.command is None:
            raise RuntimeError(
                "No PI0.5 command loaded. "
                "load_command() must be called before inference()."
            )

        # -------------------------------------------------------------------------
        # Cameras
        # -------------------------------------------------------------------------

        if images is None:
            raise ValueError(
                "PI0.5 requires camera images."
            )

        required_image_index = max(
            self.front_camera_index,
            self.gripper_camera_index,
        )

        if len(images) <= required_image_index:
            raise ValueError(
                "Not enough camera images for PI0.5. "
                f"Need indexes {self.front_camera_index} "
                f"and {self.gripper_camera_index}, "
                f"received {len(images)} images."
            )

        # Front:
        #
        # RGB live
        #   -> deterministic crop
        #   -> resize 224x224
        #   -> CHW float32 [0,1]
        front_image = process_front_image(
            images[
                self.front_camera_index
            ],
            input_color_order=
                self.input_color_order,
        )

        # Gripper:
        #
        # RGB live
        #   -> resize 224x224
        #   -> CHW float32 [0,1]
        gripper_image = process_gripper_image(
            images[
                self.gripper_camera_index
            ],
            input_color_order=
                self.input_color_order,
        )

        # -------------------------------------------------------------------------
        # Structured robot state from AIControllerNode
        # -------------------------------------------------------------------------

        if not isinstance(
            robot_state,
            dict,
        ):
            raise TypeError(
                "PI0.5 robot_state must be a dictionary "
                "produced by AIControllerNode._build_pi05_state()."
            )

        required_state_keys = (
            "joint_positions",
            "eef_position",
            "eef_quaternion",
            "gripper_closed",
        )

        missing_keys = [
            key
            for key in required_state_keys
            if key not in robot_state
        ]

        if missing_keys:
            raise KeyError(
                "PI0.5 robot_state is missing required fields: "
                f"{missing_keys}"
            )

        # -------------------------------------------------------------------------
        # Joint positions
        # -------------------------------------------------------------------------

        joint_positions = np.asarray(
            robot_state[
                "joint_positions"
            ],
            dtype=np.float32,
        )

        if (
            joint_positions.shape
            != (
                JOINT_POSITION_DIM,
            )
            or not np.all(
                np.isfinite(
                    joint_positions
                )
            )
        ):
            raise ValueError(
                "robot_state['joint_positions'] must be finite "
                "and shaped (6,), "
                f"got {joint_positions.shape}."
            )

        # -------------------------------------------------------------------------
        # Current EEF pose
        # -------------------------------------------------------------------------

        reference_position = np.asarray(
            robot_state[
                "eef_position"
            ],
            dtype=np.float32,
        )

        if (
            reference_position.shape
            != (
                3,
            )
            or not np.all(
                np.isfinite(
                    reference_position
                )
            )
        ):
            raise ValueError(
                "robot_state['eef_position'] must be finite "
                "and shaped (3,), "
                f"got {reference_position.shape}."
            )

        reference_quaternion = np.asarray(
            robot_state[
                "eef_quaternion"
            ],
            dtype=np.float32,
        )

        if (
            reference_quaternion.shape
            != (
                4,
            )
            or not np.all(
                np.isfinite(
                    reference_quaternion
                )
            )
        ):
            raise ValueError(
                "robot_state['eef_quaternion'] must be finite "
                "and shaped (4,), "
                f"got {reference_quaternion.shape}."
            )

        quaternion_norm = float(
            np.linalg.norm(
                reference_quaternion
            )
        )

        if quaternion_norm < 1e-8:
            raise ValueError(
                "Current robot quaternion has near-zero norm."
            )

        # -------------------------------------------------------------------------
        # Gripper state
        # -------------------------------------------------------------------------
        #
        # PI0.5 training proprioception uses the binary [0,1] convention:
        #
        #   0 = open
        #   1 = closed
        #
        # This is NOT the raw Robotiq qpos value.
        # AIControllerNode already tracks the logical gripper state through
        # `gripper_closed`.
        # -------------------------------------------------------------------------

        current_gripper_closed = bool(
            robot_state[
                "gripper_closed"
            ]
        )

        model_gripper_state = (
            1.0
            if current_gripper_closed
            else 0.0
        )

        # -------------------------------------------------------------------------
        # PI0.5 raw 13D state
        # -------------------------------------------------------------------------
        #
        # build_pi05_state() performs:
        #
        #   quaternion XYZW
        #       -> Euler XYZ / RPY
        #
        # and constructs:
        #
        #   [
        #       q1, q2, q3, q4, q5, q6,
        #       gripper,
        #       x, y, z, roll, pitch, yaw,
        #   ]
        #
        # No 32D padding is performed here.
        # -------------------------------------------------------------------------

        state = build_pi05_state(
            joint_positions=
                joint_positions,

            gripper_state=
                model_gripper_state,

            eef_position=
                reference_position,

            eef_quaternion_xyzw=
                reference_quaternion,
        )

        if tuple(
            state.shape
        ) != (
            RAW_STATE_DIM,
        ):
            raise RuntimeError(
                "Unexpected PI0.5 state shape: "
                f"{tuple(state.shape)}."
            )

        # -------------------------------------------------------------------------
        # Raw LeRobot observation
        # -------------------------------------------------------------------------

        observation = build_lerobot_observation(
            front_image=
                front_image,

            gripper_image=
                gripper_image,

            state=
                state,

            task=
                self.command,
        )

        return {
            "observation":
                observation,

            # Used by physical postprocessing:
            # predicted delta -> absolute UR5e target.
            "reference_position":
                reference_position.copy(),

            "reference_quaternion":
                reference_quaternion.copy(),

            "current_gripper_closed":
                current_gripper_closed,

            # Debug / saving.
            "front_image":
                front_image,

            "gripper_image":
                gripper_image,

            "state":
                state,

            "joint_positions":
                joint_positions.copy(),

            "model_gripper_state":
                model_gripper_state,
        }


    # =========================================================================
    # POSTPROCESS
    # =========================================================================

    def post_process(
        self,
        output_data,
    ) -> np.ndarray:
        """
        Convert one postprocessed PI0.5 dataset-space action to an absolute
        UR5e / MoveIt target.

        Pipeline
        --------

        PI0.5
            ↓
        LeRobot QUANTILES postprocessor
            ↓
        action[7] in dataset representation
            ↓
        recover physical scale × 0.05 on first six values
            ↓
        delta XYZ + delta RPY
            ↓
        p_target = p_current + delta_p
        R_target = R_delta @ R_current
            ↓
        gripper 0..20 -> hysteresis -> binary
            ↓
        Robotiq / MoveIt 0..255
            ↓
        [x,y,z,qx,qy,qz,qw,gripper_position]

        No experimental corrections are applied.
        """

        required_keys = (
            "action",
            "reference_position",
            "reference_quaternion",
            "current_gripper_closed",
        )

        for key in required_keys:

            if key not in output_data:
                raise KeyError(
                    "output_data must contain "
                    f"{key!r}."
                )

        action = validate_pi05_action(
            output_data[
                "action"
            ]
        )

        (
            target_position,
            target_quaternion_xyzw,
            gripper_state,
        ) = delta_action_to_absolute_target(
            postprocessed_action=
                action,

            reference_position=
                output_data[
                    "reference_position"
                ],

            reference_quaternion_xyzw=
                output_data[
                    "reference_quaternion"
                ],

            currently_closed=
                bool(
                    output_data[
                        "current_gripper_closed"
                    ]
                ),

            scale_factor=
                self.dataset_action_scale,

            open_threshold=
                self.gripper_open_threshold,

            close_threshold=
                self.gripper_close_threshold,
        )

        gripper_command = (
            gripper_binary_to_moveit(
                gripper_state=
                    gripper_state,

                open_position=
                    self.gripper_open_position,

                closed_position=
                    self.gripper_closed_position,
            )
        )

        absolute_target = np.concatenate(
            (
                target_position,

                target_quaternion_xyzw,

                np.array(
                    [
                        gripper_command
                    ],
                    dtype=np.float32,
                ),
            )
        ).astype(
            np.float32,
            copy=False,
        )

        if absolute_target.shape != (
            8,
        ):
            raise RuntimeError(
                "Unexpected PI0.5 absolute target shape: "
                f"{absolute_target.shape}"
            )

        if not np.all(
            np.isfinite(
                absolute_target
            )
        ):
            raise RuntimeError(
                "PI0.5 absolute target contains "
                f"non-finite values: {absolute_target}"
            )

        return absolute_target


    # =========================================================================
    # DEBUG / SAVING
    # =========================================================================

    @staticmethod
    def _save_tensor_image(
        image: torch.Tensor,
        output_path: Path,
    ) -> None:
        """
        Save CHW float32 [0,1] image as RGB PNG.
        """

        if (
            not isinstance(
                image,
                torch.Tensor,
            )
            or image.shape
            != (
                3,
                IMAGE_SIZE,
                IMAGE_SIZE,
            )
        ):
            raise ValueError(
                "Expected image tensor with shape "
                f"(3, {IMAGE_SIZE}, {IMAGE_SIZE})."
            )

        image_hwc = (
            image
            .detach()
            .cpu()
            .clamp(
                0.0,
                1.0,
            )
            .permute(
                1,
                2,
                0,
            )
            .numpy()
        )

        image_uint8 = np.rint(
            image_hwc
            * 255.0
        ).astype(
            np.uint8
        )

        PILImage.fromarray(
            image_uint8,
            mode="RGB",
        ).save(
            output_path
        )


    def _save_preprocessed_images(
        self,
        front_image: torch.Tensor,
        gripper_image: torch.Tensor,
        save_path: str | Path,
    ) -> None:

        save_path = Path(
            save_path
        )

        save_path.mkdir(
            parents=True,
            exist_ok=True,
        )

        # Keep the conventional filename expected by existing debug tooling.
        self._save_tensor_image(
            front_image,
            save_path
            / "pre_processed_img_0.png",
        )

        self._save_tensor_image(
            gripper_image,
            save_path
            / "pre_processed_img_1.png",
        )


    @staticmethod
    def _format_array(
        value,
    ) -> str:

        if isinstance(
            value,
            torch.Tensor,
        ):
            value = (
                value
                .detach()
                .float()
                .cpu()
                .numpy()
            )

        return np.array2string(
            np.asarray(
                value
            ),
            precision=7,
            suppress_small=False,
            separator=", ",
        )


    def _trace_inference(
        self,
        step: int,
        processed: dict,
        postprocessed_action,
        absolute_target,
    ) -> None:
        """
        Log the complete PI0.5 -> UR5e conversion.
        """

        action = validate_pi05_action(
            postprocessed_action
        )

        physical_delta = recover_physical_delta(
            action,
            scale_factor=
                self.dataset_action_scale,
        )

        print(
            f"[PI05Trace][step={step}]\n"
            f"  task_id={self.current_task_id}\n"
            f"  command={self.command!r}\n"
            f"  joint_positions="
            f"{self._format_array(processed['joint_positions'])}\n"
            f"  model_gripper_state="
            f"{processed['model_gripper_state']:.7f}\n"
            f"  pi05_state_13d="
            f"{self._format_array(processed['state'])}\n"
            f"  reference_position="
            f"{self._format_array(processed['reference_position'])}\n"
            f"  reference_quaternion_xyzw="
            f"{self._format_array(processed['reference_quaternion'])}\n"
            f"  current_gripper_closed="
            f"{processed['current_gripper_closed']}\n"
            f"  postprocessed_action="
            f"{self._format_array(action)}\n"
            f"  physical_delta_xyz="
            f"{self._format_array(physical_delta[:3])}\n"
            f"  physical_delta_rpy_rad="
            f"{self._format_array(physical_delta[3:6])}\n"
            f"  physical_delta_rpy_deg="
            f"{self._format_array(np.degrees(physical_delta[3:6]))}\n"
            f"  predicted_gripper_0_20="
            f"{action[6]:.7f}\n"
            f"  absolute_target="
            f"{self._format_array(absolute_target)}\n"
            f"  target_quaternion_norm="
            f"{np.linalg.norm(absolute_target[3:7]):.9f}"
        )


    # =========================================================================
    # INFERENCE
    # =========================================================================

    def inference(
        self,
        input_data,
        t: int = 0,
        save_path: str | Path | None = None,
    ):
        """
        Execute one PI0.5 controller step.

        Each call:

            1. receives cameras + current robot state;
            2. builds the 13D PI0.5 proprioceptive state;
            3. builds the raw LeRobot observation;
            4. calls PI05Runtime.select_action();
            5. receives one postprocessed dataset-space action;
            6. converts it to an absolute UR5e target;
            7. returns one 8D target to AIControllerNode.

        No additional action buffer is implemented here.

        PI05Policy.select_action() owns the LeRobot action queue.
        """

        if self._runtime is None:
            raise RuntimeError(
                "PI0.5 controller has not been initialized."
            )

        if self.command is None:
            raise RuntimeError(
                "load_command() must be called before inference()."
            )

        # ---------------------------------------------------------------------
        # 1. UR5e -> raw LeRobot observation
        # ---------------------------------------------------------------------

        processed = self.pre_process(
            input_data
        )

        if save_path is not None:

            self._save_preprocessed_images(
                front_image=
                    processed[
                        "front_image"
                    ],

                gripper_image=
                    processed[
                        "gripper_image"
                    ],

                save_path=
                    save_path,
            )

        # ---------------------------------------------------------------------
        # 2. LeRobot PI0.5 inference
        #
        # serialized preprocessor
        #       ↓
        # PI05Policy.select_action()
        #       ↓
        # serialized postprocessor
        #
        # Returns one action [7] on CPU.
        # ---------------------------------------------------------------------

        postprocessed_action = (
            self._runtime.select_action(
                processed[
                    "observation"
                ]
            )
        )

        # ---------------------------------------------------------------------
        # 3. Dataset action -> absolute robot target
        # ---------------------------------------------------------------------

        absolute_target = self.post_process(
            {
                "action":
                    postprocessed_action,

                "reference_position":
                    processed[
                        "reference_position"
                    ],

                "reference_quaternion":
                    processed[
                        "reference_quaternion"
                    ],

                "current_gripper_closed":
                    processed[
                        "current_gripper_closed"
                    ],
            }
        )

        if self.trace_action_conversions:

            self._trace_inference(
                step=t,
                processed=processed,
                postprocessed_action=
                    postprocessed_action,
                absolute_target=
                    absolute_target,
            )

        # AIControllerNode expects a list of actions.
        return [
            [
                float(
                    value
                )
                for value in absolute_target
            ]
        ]