import os
import sys
from dataclasses import dataclass
from typing import Optional

import numpy as np
import torch
import yaml
from PIL import Image as PILImage

from ai_controller.utils.ai_controller import AIController

# openvla.py lives alongside this file; import it the same way it imports
# openvla_utils.py, so this works whether or not the installed ai_controller
# package copy is in sync with this source tree.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from openvla import OpenVLAPolicy


@dataclass
class OpenVLAConfig:
    """Configuration dataclass for OpenVLAController (loaded from YAML)."""

    # HuggingFace model ID or local path to the base checkpoint
    model_path: str = "openvla/openvla-7b"

    # ── Action head ──────────────────────────────────────────────────────── #
    use_l1_regression: bool = False
    use_diffusion: bool = False
    num_diffusion_steps: int = 50
    use_film: bool = False

    # ── Input images ─────────────────────────────────────────────────────── #
    # 1 = front camera only; 2 = front + second camera (e.g. wrist view)
    num_images_in_input: int = 1
    center_crop: bool = True
    resize_size: int = 224

    # ── Proprioception ───────────────────────────────────────────────────── #
    use_proprio: bool = False
    proprio_dim: int = 8

    # ── Inference ────────────────────────────────────────────────────────── #
    # Steps to execute open-loop before re-querying the model
    num_open_loop_steps: int = 1
    # Number of action steps the model outputs per query
    chunk_size: int = 1
    # Action unnormalization key - must match the training dataset key stored
    # inside the model's norm_stats dict. Leave empty to auto-resolve it from
    # task_suite_name at load time (see openvla_utils.check_unnorm_key).
    unnorm_key: str = ""
    # Dataset key used to look up action/proprio normalization stats in the
    # checkpoint's dataset_statistics.json (must match a top-level key there).
    task_suite_name: str = ""

    # ── Quantization ─────────────────────────────────────────────────────── #
    load_in_8bit: bool = False
    load_in_4bit: bool = False

    # ── Language prompt ──────────────────────────────────────────────────── #
    task_description: str = "pick up the object and place it in the target location"


class OpenVLAController(AIController):
    """
    Controller wrapping OpenVLA (7B VLA model) for UR5e end-effector control.

    Mirrors the CODController interface so it can be swapped in via the
    'ai_controller_target' ROS parameter.

    Model loading and inference replicate VLA-Bench's robosuite_test pipeline
    (robosuite_test/models/openvla.py, entrypoint run_robosuite_eval.py):
    model / processor / action-head / proprio-projector construction go
    through openvla_utils.get_model / get_processor / get_action_head /
    get_proprio_projector / get_noisy_action_projector / check_unnorm_key
    (see openvla.py), and inference goes through openvla_utils.get_vla_action.

    Model loading
    -------------
    model_config : path to a YAML file whose keys map to OpenVLAConfig fields.

    Inference pipeline
    ------------------
    1. pre_process  - pack raw camera images (+ optional robot state) into obs dict.
    2. inference    - build language prompt -> OpenVLAPolicy.get_action (get_vla_action).
    3. post_process - return action list unchanged for the caller.

    Action format
    -------------
    Each action is a 7-D numpy array: [dx, dy, dz, droll, dpitch, dyaw, gripper].
    Units and scale depend on the training dataset and unnorm_key.
    """

    def __init__(self, model_config: str, task_name: str = "pick_place"):
        self.task_name = task_name
        self.cfg: Optional[OpenVLAConfig] = None
        self._policy: Optional[OpenVLAPolicy] = None

        # AIController.__init__ calls self.load_model() which populates self.cfg
        super().__init__(model_config)

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.move_model_to_device(self.device)
        self.model.eval()

    # ------------------------------------------------------------------ #
    #  AIController abstract method implementations                        #
    # ------------------------------------------------------------------ #

    def load_model(self, model_config: str):
        """Load the OpenVLA model (+ processor/action head/proprio projector) from YAML config."""
        with open(model_config, "r") as f:
            config_dict = yaml.safe_load(f)

        valid_fields = OpenVLAConfig.__dataclass_fields__.keys()
        self.cfg = OpenVLAConfig(**{k: v for k, v in config_dict.items() if k in valid_fields})

        self._policy = OpenVLAPolicy(self.cfg)
        self._processor = self._policy.processor

        return self._policy.model

    def move_model_to_device(self, device):
        if not (self.cfg.load_in_8bit or self.cfg.load_in_4bit):
            self.model = self.model.to(device)

    def load_demo_dataset(self, demo_path: str, task_id: str):
        """OpenVLA is prompt-driven; demonstration loading is not required."""
        print(f"[OpenVLAController] Prompt-based model - skipping demo loading (task {task_id}).")

    def reset(self):
        pass

    def pre_process(self, input_data):
        """
        Pack raw camera images (and optionally robot state) for the model.

        Resizing/cropping is handled downstream by openvla_utils.get_vla_action
        (prepare_images_for_vla), matching the VLA-Bench reference pipeline.

        Parameters
        ----------
        input_data : [images, states]
            images : list of numpy uint8 RGB arrays (H, W, 3), one per camera,
                     in the same order as the camera_topic parameter:
                     [front, left, right].
            states : numpy array of shape (8,) -
                     [eef_x, eef_y, eef_z, roll, pitch, yaw, gripper_open, gripper_closed]
                     or None.

        Returns
        -------
        [obs_dict, states]
            obs_dict keys:
                "full_image"           - np.ndarray (H, W, 3) uint8
                "camera_gripper_image" - same shape, present only if num_images_in_input > 1
                "state"                - np.ndarray (proprio_dim,), present only if use_proprio
        """
        images, states = input_data[0], input_data[1]

        obs = {"full_image": images[0]}

        if self.cfg.num_images_in_input > 1 and len(images) > 1:
            obs["camera_gripper_image"] = images[1]

        if self.cfg.use_proprio and states is not None:
            obs["state"] = np.array(states, dtype=np.float64)

        return [obs, states]

    def post_process(self, output_data):
        """Pass the action list through unchanged."""
        return output_data

    def inference(self, input_data, t: int, save_path: Optional[str] = None):
        """
        Run one OpenVLA forward pass and return actions for open-loop execution.

        Parameters
        ----------
        input_data : [images, states]  - see pre_process for format.
        t          : current time step (used for debug image filenames).
        save_path  : if set, saves the model input image to this directory.

        Returns
        -------
        list of numpy arrays, length <= num_open_loop_steps.
        Each element is a 7-D action: [dx, dy, dz, droll, dpitch, dyaw, gripper].
        """
        obs, states = self.pre_process(input_data)

        if save_path is not None:
            os.makedirs(save_path, exist_ok=True)
            PILImage.fromarray(obs["full_image"]).save(
                os.path.join(save_path, f"openvla_input_t{t:03d}.png")
            )
        print(f"[OpenVLAController] Inference t={t}: running model on device {self.device}...")
        actions = self._policy.get_action(obs, task_label=self.cfg.task_description)
        return self.post_process(actions)
