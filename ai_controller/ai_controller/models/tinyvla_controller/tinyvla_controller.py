import os
import sys
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import yaml
from PIL import Image as PILImage

from ai_controller.utils.ai_controller import AIController
from ai_controller.utils.utils import seed_everything, _quat2mat, _mat2euler_sxyz

# tinyvla.py/tinyvla_utils.py live alongside this file; import them the same
# way openvla_controller.py imports openvla.py/openvla_utils.py.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from tinyvla import TinyVLAPolicy
from tinyvla_utils import COMMAND_TEMPLATE, TINYVLA_IMAGE_SIZE, crop_front_image, normalize_angle, euler_to_axis_angle


@dataclass
class TinyVLAConfig:
    """Configuration dataclass for TinyVLAController (loaded from YAML)."""

    model_path: str = ""
    model_base: str = ""
    enable_lora: bool = True
    conv_mode: str = "pythia"
    action_head: str = "droid_diffusion"
    task_suite_name: str = "real_ur5e_pick_place_delta_removed_0_5_10_15"
    task_description: str = "pick up the object and place it in the target location"


class TinyVLAController(AIController):
    """
    Controller wrapping TinyVLA (Llava-Pythia) for UR5e end-effector control.

    Mirrors the OpenVLAController/CODController interface so it can be swapped
    in via the 'ai_controller_target' ROS parameter.

    Unlike OpenVLAController (which executes an open-loop chunk of actions per
    inference() call), TinyVLA's reference eval loop (VLA-Bench/robosuite_test/
    test/pick_place.py) re-queries the model every control-loop step and relies
    on ACT-style temporal aggregation (see TinyVLAPolicy.compute_action) instead
    of open-loop execution - so inference() here always returns a single action.

    Action format
    -------------
    inference() returns a 1-element list with a 7-D absolute world-frame action
    [x, y, z, ax, ay, az, gripper], where [ax, ay, az] is an axis-angle
    orientation and gripper is a Robotiq position in {0.0, 255.0} - the same
    shape ai_controller_node.py already handles for 'openvla_controller'.
    """

    def __init__(self, model_config: str, task_name: str = "pick_place"):
        self.task_name = task_name
        self.cfg: Optional[TinyVLAConfig] = None
        self._policy: Optional[TinyVLAPolicy] = None
        self.gripper_closed = False
        self.n_steps = 0
        self.starting_orientation = None

        # AIController.__init__ calls self.load_model() which populates self.cfg
        super().__init__(model_config)

        seed_everything(0)

    # ------------------------------------------------------------------ #
    #  AIController abstract method implementations                        #
    # ------------------------------------------------------------------ #

    def load_model(self, model_config: str):
        """Load the TinyVLA (Llava-Pythia) model (+ tokenizer/image processor/
        dataset stats) from YAML config."""
        with open(model_config, "r") as f:
            config_dict = yaml.safe_load(f)

        valid_fields = TinyVLAConfig.__dataclass_fields__.keys()
        self.cfg = TinyVLAConfig(**{k: v for k, v in config_dict.items() if k in valid_fields})

        self._policy = TinyVLAPolicy(self.cfg)
        return self._policy.policy

    def move_model_to_device(self, device):
        # load_pretrained_model() (called from TinyVLAPolicy.load_policy) already
        # places the model on CUDA internally; nothing further to do here.
        pass

    def load_command(self, demo_path: str, task_id: str, **kwargs):
        """TinyVLA is prompt-driven; demonstration loading is not required."""
        command = input(f"Write the task description for task {task_id} \n\t- Template {COMMAND_TEMPLATE.get(task_id, 'No template available')}\n\t(or press Enter to use default): ")
        if command.strip():
            self.cfg.task_description = command.strip()
            print(f"[TinyVLAController] Updated task description: {self.cfg.task_description}")
        else:
            self.cfg.task_description = COMMAND_TEMPLATE.get(task_id, self.cfg.task_description)

    def reset(self):
        self.gripper_closed = False
        self.n_steps = 0
        self.starting_orientation = None

    PREVIEW_WINDOW_NAME = "TinyVLAController - front | gripper"

    def _show_camera_preview(self, front_rgb, gripper_rgb):
        """Show the front + gripper camera images side by side and block until
        Enter is pressed."""
        height = min(front_rgb.shape[0], gripper_rgb.shape[0])

        def resize_to_height(img, height):
            if img.shape[0] == height:
                return img
            scale = height / img.shape[0]
            return cv2.resize(img, (int(img.shape[1] * scale), height))

        front_bgr = cv2.cvtColor(resize_to_height(front_rgb, height), cv2.COLOR_RGB2BGR)
        gripper_bgr = cv2.cvtColor(resize_to_height(gripper_rgb, height), cv2.COLOR_RGB2BGR)
        cv2.putText(front_bgr, "front", (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(gripper_bgr, "gripper", (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 0), 2, cv2.LINE_AA)

        combined = np.hstack([front_bgr, gripper_bgr])
        cv2.imshow(self.PREVIEW_WINDOW_NAME, combined)
        cv2.waitKey(1000)  # Wait 1 sec
        # print("[TinyVLAController] Showing front/gripper camera preview - press Enter to continue...")
        
        # while True:
        #     key = cv2.waitKey(0) & 0xFF
        #     if key in (13, 10):  # Enter
        #         break

    def _axis_angle_from_quat(self, eef_quat):
        eef_euler = np.array([normalize_angle(a) for a in _mat2euler_sxyz(_quat2mat(eef_quat))])
        return euler_to_axis_angle(eef_euler)

    def set_starting_orientation(self, eef_quat):
        """Seed the orientation post_process() pins the model's output to (see
        post_process) from an externally-measured eef_quat - e.g.
        ai_controller_node's initial_gripper_pose, captured via TF right after the
        robot settles at its initial pose, before the first inference step. Takes
        priority over pre_process's auto-capture since it runs first."""
        self.starting_orientation = self._axis_angle_from_quat(eef_quat)

    def pre_process(self, input_data):
        """
        Pack raw camera images and robot pose into the obs dict TinyVLAPolicy expects,
        applying the same front-camera crop (tinyvla_utils.TASK_CROP) and eye-in-hand
        resize that TinyVLAPolicy was trained on, so the preview window and the
        save_path debug images (see inference()) show exactly what the model sees
        rather than the raw camera frames.

        Parameters
        ----------
        input_data : [images, states]
            images : list of numpy uint8 RGB arrays (H, W, 3), one per camera, in
                     the same order as the camera_topic parameter: [front, ..., gripper].
                     The last image is treated as the eye-in-hand/gripper view.
            states  : numpy array of shape (7,) - [eef_x, eef_y, eef_z, qx, qy, qz, qw]
                      (see ai_controller_node._build_tinyvla_state), or None.

        Returns
        -------
        obs dict with keys 'camera_front_image' (cropped to TASK_CROP[task_name] and
        resized to 224x224), 'eye_in_hand_image' (resized to 224x224, no crop - see
        TASK_CROP comment in tinyvla_utils.py), and, if states is not None, 'eef_pos'
        (3,) / 'eef_quat' (4,).
        """
        images, states = input_data[0], input_data[1]

        eye_in_hand_image = images[-1] if len(images) > 1 else images[0]
        obs = {
            "camera_front_image": crop_front_image(images[0], self.task_name),
            "eye_in_hand_image": cv2.resize(eye_in_hand_image, (TINYVLA_IMAGE_SIZE, TINYVLA_IMAGE_SIZE)),
        }

        self._show_camera_preview(obs["camera_front_image"], obs["eye_in_hand_image"])

        if states is not None:
            states = np.asarray(states, dtype=np.float64)
            obs["eef_pos"] = states[0:3]
            obs["eef_quat"] = states[3:7]

            if self.starting_orientation is None:
                self.starting_orientation = self._axis_angle_from_quat(obs["eef_quat"])

        return obs

    def post_process(self, action_world):
        """
        Binarize the raw predicted gripper value to the {0.0, 255.0} Robotiq
        convention used elsewhere in this codebase (cod_controller, and
        ai_controller_node's close->open episode-done detection), applying the
        same open/close hysteresis as VLA-Bench/robosuite_test/test/pick_place.py:
        opening requires the value to drop below 0.7 once closed, while closing
        from open requires it to exceed 0.9.

        Orientation is pinned to the eef_quat given at the first step of the
        episode (captured in pre_process): the model's predicted orientation is
        discarded and replaced with that starting orientation on every step, so
        the gripper never rotates.

        Parameters
        ----------
        action_world : 7-D numpy array [x, y, z, ax, ay, az, gripper] from
                       TinyVLAPolicy.action_post_processing (gripper is still the
                       raw unnormalized model output at this point).

        Returns
        -------
        The same array with action_world[3:6] held at the episode's starting
        orientation and action_world[-1] replaced by 0.0 or 255.0.
        """
        if self.starting_orientation is not None:
            action_world[3:6] = self.starting_orientation

        gripper_value = action_world[-1]
        print(f"[TinyVLAController] Raw gripper value: {gripper_value:.3f} (closed={self.gripper_closed})")
        if self.gripper_closed:
            self.gripper_closed = gripper_value >= 0.3
        else:
            self.gripper_closed = gripper_value >= 0.3
        action_world[-1] = 255.0 if self.gripper_closed else 0.0
        
        # if action_world[2] < 0.05:
        #     print(f"[TinyVLAController] Adding x offset")
        #     action_world[0] -= 0.04
        # if action_world[2] < -0.05:
        #     # force the gripper to close
        #     self.gripper_closed = True
        #     action_world[-1] = 255.0
        
        return action_world

    def inference(self, input_data, t: int, save_path: Optional[str] = None):
        """
        Run one TinyVLA forward pass and return the action to execute this step.

        Parameters
        ----------
        input_data : [images, states] - see pre_process for format.
        t          : current time step (used for debug image filenames).
        save_path  : if set, saves the model input images to this directory.

        Returns
        -------
        1-element list containing a 7-D absolute action: [x, y, z, ax, ay, az, gripper].
        """
        obs = self.pre_process(input_data)

        if save_path is not None:
            os.makedirs(save_path, exist_ok=True)
            PILImage.fromarray(obs["camera_front_image"]).save(
                os.path.join(save_path, f"tinyvla_input_front_t{t:03d}.png"))
            PILImage.fromarray(obs["eye_in_hand_image"]).save(
                os.path.join(save_path, f"tinyvla_input_gripper_t{t:03d}.png"))

        print(f"[TinyVLAController] Inference t={t}: task description = {self.cfg.task_description}")
        actions, inf_time = self._policy.compute_action(
            obs=obs,
            gripper_closed=1 if self.gripper_closed else 0,
            task_description=self.cfg.task_description,
            n_steps=self.n_steps,
        )
        print(f"[TinyVLAController] Inference t={t}: latency = {inf_time:.3f}s: raw action = {actions}")
        self.n_steps += 1

        return [self.post_process(action) for action in actions]
