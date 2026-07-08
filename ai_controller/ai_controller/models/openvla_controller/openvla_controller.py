import copy
import os
import sys
from dataclasses import dataclass
from typing import Optional

import numpy as np
import torch
import yaml
from PIL import Image as PILImage

from ai_controller.utils.ai_controller import AIController
from ai_controller.utils.utils import seed_everything

# openvla.py lives alongside this file; import it the same way it imports
# openvla_utils.py, so this works whether or not the installed ai_controller
# package copy is in sync with this source tree.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from openvla import OpenVLAPolicy
from openvla_utils import (
    COMMAND_TEMPLATE,
    SCALE_FACTOR,
    axisangle_to_euler,
    crop_front_image,
    euler_to_axis_angle,
    normalize_angle,
)


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
    task_suite_name: str = "real_ur5e_pick_place_rm_0_5_10_15"

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
    3. post_process - convert the predicted delta-action chunk into absolute
                       world-frame actions, mirroring VLA-Bench's
                       robosuite_test/models/openvla.py open_vla_policy.action_post_processing.

    Action format
    -------------
    Actions predicted by the model (pre post_process) are 7-D deltas:
    [dx, dy, dz, droll, dpitch, dyaw, gripper]. Units and scale depend on the
    training dataset and unnorm_key (see openvla_utils.SCALE_FACTOR).
    post_process() integrates these deltas against the current end-effector
    pose to produce absolute world-frame actions: [x, y, z, ax, ay, az, gripper],
    where [ax, ay, az] is an axis-angle orientation.
    """

    def __init__(self, model_config: str, task_name: str = "pick_place"):
        self.task_name = task_name
        self.cfg: Optional[OpenVLAConfig] = None
        self._policy: Optional[OpenVLAPolicy] = None

        # AIController.__init__ calls self.load_model() which populates self.cfg
        super().__init__(model_config)

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.move_model_to_device(self.device)
        seed_everything(0) 
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

    def load_command(self, demo_path: str, task_id: str, **kwargs):
        """OpenVLA is prompt-driven; demonstration loading is not required."""
        command = input(f"Write the task description for task {task_id} \n\t- Template {COMMAND_TEMPLATE.get(task_id, 'No template available')}\n\t(or press Enter to use default): ")
        if command.strip():
            self.cfg.task_description = command.strip()
            print(f"[OpenVLAController] Updated task description: {self.cfg.task_description}")
        else:
            # assign the template if no input is provided
            self.cfg.task_description = COMMAND_TEMPLATE.get(task_id, self.cfg.task_description)

    def reset(self):
        pass

    def pre_process(self, input_data):
        """
        Pack raw camera images (and optionally robot state) for the model.

        The front image is first cropped with crop_front_image() to match the
        agent field-of-view baked into this checkpoint's training data (see
        openvla_utils.TASK_CROP / real_ur5e_pick_place_delta_removed_0_5_10_15/
        ur5e_pick_place.py's crop_image_adj_bb()). camera_gripper_image is left
        uncropped, matching that same training pipeline (only camera_front_image
        is task-cropped there; eye_in_hand_image is resized as-is). Resizing to
        224x224 and the eval-time center crop are then handled downstream by
        openvla_utils.get_vla_action (prepare_images_for_vla).

        NOTE: camera_gripper_image must be a genuine wrist/eye-in-hand camera to
        match training. ai_controller_node's default `camera_topic` param
        (front/left/right static views) does NOT provide one - if no wrist
        camera is wired up, set num_images_in_input: 1 in the model config
        instead of passing a mismatched second image here.

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
                "full_image"           - np.ndarray (H, W, 3) uint8, cropped to TASK_CROP[task_name]
                "camera_gripper_image" - raw (uncropped), present only if num_images_in_input > 1
                "state"                - np.ndarray (proprio_dim,), present only if use_proprio
        """
        images, states = input_data[0], input_data[1]

        obs = {"full_image": crop_front_image(images[0], self.task_name)}
    
        if self.cfg.num_images_in_input > 1 and len(images) > 1:
            # resize 224x224
            obs["camera_gripper_image"] = np.array(PILImage.fromarray(images[-1]).resize((224, 224)), dtype=np.uint8)
            
        if self.cfg.use_proprio and states is not None:
            obs["state"] = np.array(states, dtype=np.float64)

        return [obs, states]

    def post_process(self, obs, states, action_chunk, n_steps=-1):
        """
        Convert a chunk of predicted delta-actions into absolute world-frame actions.

        Ports action_post_processing from VLA-Bench's robosuite_test/models/openvla.py
        (open_vla_policy.action_post_processing) verbatim, adapted to this
        controller's proprio state layout: `states` is the 8-dim vector
        [eef_x, eef_y, eef_z, roll, pitch, yaw, gripper_open, gripper_closed]
        built by ai_controller_node._build_openvla_state, so the current
        end-effector position and gripper-orientation Euler angles are read
        directly from `states[0:3]` / `states[3:6]` instead of being
        recomputed from a raw eef_quat via quat2mat/mat2euler.

        Parameters
        ----------
        obs          : observation dict returned by pre_process (unused by the
                       math below, kept for parity with the reference signature
                       and for future post-processing that depends on it).
        states       : 8-dim proprio vector, or None - see pre_process.
        action_chunk : list of 7-D delta actions predicted by the model:
                       [dx, dy, dz, droll, dpitch, dyaw, gripper].
        n_steps      : current time step, used only for logging.

        Returns
        -------
        list of 7-D numpy arrays: [x, y, z, ax, ay, az, gripper], where
        [ax, ay, az] is an axis-angle orientation.
        """
        post_processed_actions = []
        for indx, action in enumerate(action_chunk[: self.cfg.chunk_size]):
            action = action * SCALE_FACTOR
            print(f"[OpenVLAController] Action delta at time {n_steps} - Indx {indx}: {action}")

            action_world = np.zeros(7)
            if "abs_pose" in self.cfg.task_suite_name:
                action_world[0:3] = action[0:3]
            else:
                if indx == 0:
                    if states is None:
                        raise ValueError(
                            "post_process requires the robot's proprio state "
                            "(eef position/orientation) to integrate delta actions."
                        )
                    action_world[0:3] = states[0:3] + action[0:3]
                    current_gripper_orientation = [normalize_angle(a) for a in states[3:6]]
                else:
                    action_world[0:3] = post_processed_actions[-1][0:3] + action[0:3]
                    current_gripper_orientation = axisangle_to_euler(post_processed_actions[-1][3:6])
                    current_gripper_orientation = [normalize_angle(a) for a in current_gripper_orientation]

                gripper_orientation_action = current_gripper_orientation + action[3:6]
                gripper_orientation_action = [normalize_angle(a) for a in gripper_orientation_action]
                action_world[3:6] = euler_to_axis_angle(gripper_orientation_action)
                action_world[-1] = action[-1]
            post_processed_actions.append(copy.deepcopy(action_world))

            # print(f"[OpenVLAController] post_process: action {indx} = {action_world}")

        return post_processed_actions

    def inference(self, input_data, t: int, save_path: Optional[str] = None):
        """
        Run one OpenVLA forward pass and return actions for open-loop execution.

        Parameters
        ----------
        input_data : [images, states]  - see pre_process for format.
        t          : current time step (used for debug image filenames).
        save_path  : if set, saves the model input images to this directory.

        Returns
        -------
        list of numpy arrays, length <= num_open_loop_steps.
        Each element is a 7-D absolute action: [x, y, z, ax, ay, az, gripper].
        """
        obs, states = self.pre_process(input_data)

        if save_path is not None:
            os.makedirs(save_path, exist_ok=True)
            # Save every image actually fed to the model, matching
            # CODController.inference()'s pre_processed_imgs saving loop.
            image_keys = [k for k in ("full_image", "camera_gripper_image") if k in obs]
            for key in image_keys:
                PILImage.fromarray(obs[key]).save(
                    os.path.join(save_path, f"openvla_input_{key}_t{t:03d}.png")
                )
        print(f"[OpenVLAController] Inference t={t}: task description = {self.cfg.task_description}")
        print(f"[OpenVLAController] Inference t={t}: obs keys = {list(obs.keys())}, states = {states}")
        actions = self._policy.get_action(obs, task_label=self.cfg.task_description)
        return self.post_process(obs=obs, states=states, action_chunk=actions, n_steps=t)
