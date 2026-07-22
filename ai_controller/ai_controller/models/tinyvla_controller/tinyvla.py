"""
TinyVLA (Llava-Pythia) policy wrapper.

Ported from VLA-Bench's robosuite_test/models/tinyvla.py (`llava_pythia_act_policy`),
which is the validated test-time inference path for TinyVLA checkpoints
(~/Desktop/Multi-Task-LFD/repo/VLA-Bench/robosuite_test/models/tinyvla.py).

Differences from the reference implementation:
  - prepare_observation() uses ai_controller.utils.utils's _quat2mat/_mat2euler_sxyz
    (already ported to match this project's training-data convention) instead of
    robosuite.utils.transform_utils, so this controller doesn't need robosuite
    installed in the real-robot container.
  - obs dict keys match ai_controller's schema built by TinyVLAController.pre_process()
    ('camera_front_image', 'eye_in_hand_image', 'eef_pos', 'eef_quat') rather than a
    robosuite env observation dict.
"""

import os
import pickle
import time

import cv2
import numpy as np
import torch
from torchvision import transforms

from llava_pythia.conversation import conv_templates
from llava_pythia.mm_utils import tokenizer_image_token, get_model_name_from_path
from llava_pythia.model.builder import load_pretrained_model
from llava_pythia.model.language_model.pythia.llava_pythia import LlavaPythiaConfig
from llava_pythia.constants import IMAGE_TOKEN_INDEX, DEFAULT_IMAGE_TOKEN, DEFAULT_IM_START_TOKEN, DEFAULT_IM_END_TOKEN

from ai_controller.utils.utils import _quat2mat, _mat2euler_sxyz
from tinyvla_utils import SCALE_FACTOR, R_EE_TO_GRIPPER, crop_front_image, normalize_angle, euler_to_axis_angle, rot_6d_to_euler_angles


class TinyVLAPolicy:
    """Loads a TinyVLA (Llava-Pythia) checkpoint and predicts world-frame actions."""

    def __init__(self, policy_config):
        self.policy_config = policy_config
        self.load_policy(policy_config)
        self.to_tensor = transforms.ToTensor()
        self.all_time_actions = None
        self.action_dim = self.policy.config.action_dim
        self.policy.eval()

        stats_path = os.path.join(os.path.dirname(self.policy_config.model_path), 'dataset_stats.pkl')
        with open(stats_path, 'rb') as f:
            self.stats = pickle.load(f)

        if self.policy_config.action_head == 'act':
            self.unnormalize_action = lambda a: a * self.stats['action_std'] + self.stats['action_mean']
        elif self.policy_config.action_head == 'transformer_diffusion':
            self.unnormalize_action = lambda a: ((a + 1) / 2) * (self.stats['action_max'] - self.stats['action_min']) + self.stats['action_min']
        elif self.policy_config.action_head == 'droid_diffusion':
            self.unnormalize_action = lambda a: (((a + 1) / 2) * (self.stats['action_max'] - self.stats['action_min'])) + self.stats['action_min']
        else:
            raise NotImplementedError(f"Unknown action_head: {self.policy_config.action_head}")

    def load_policy(self, policy_config):
        model_base = policy_config.model_base if policy_config.enable_lora else None
        model_name = get_model_name_from_path(policy_config.model_path)
        self.tokenizer, self.policy, self.image_processor, self.context_len = load_pretrained_model(
            policy_config.model_path, model_base, model_name, False, False
        )
        self.config = LlavaPythiaConfig.from_pretrained(
            os.path.dirname(policy_config.model_path), trust_remote_code=False
        )

    def expand2square(self, pil_imgs, background_color):
        batch_size, channels, height, width = pil_imgs.shape
        max_dim = max(height, width)
        expanded_imgs = np.full((batch_size, max_dim, max_dim, channels), background_color, dtype=np.float32)

        if height == width:
            expanded_imgs = pil_imgs.permute(0, 2, 3, 1).cpu().numpy()
        elif height > width:
            offset = (max_dim - width) // 2
            expanded_imgs[:, :height, offset:offset + width, :] = pil_imgs.permute(0, 2, 3, 1).cpu().numpy()
        else:
            offset = (max_dim - height) // 2
            expanded_imgs[:, offset:offset + height, :width, :] = pil_imgs.permute(0, 2, 3, 1).cpu().numpy()
        expanded_imgs = torch.tensor(expanded_imgs).to(dtype=pil_imgs.dtype, device=pil_imgs.device)
        return expanded_imgs

    def process_batch_to_llava(self, curr_image, robo_state, raw_lang):
        self.conv = conv_templates[self.policy_config.conv_mode].copy()

        if len(curr_image.shape) == 5:
            curr_image = curr_image.squeeze(0)

        image, image_r = torch.chunk(curr_image, 2, dim=0)

        image = self.expand2square(image, tuple(x for x in self.image_processor.image_mean))
        image_tensor = self.image_processor.preprocess(
            image, return_tensors='pt', do_normalize=True, do_rescale=False, do_center_crop=False
        )['pixel_values']
        image_tensor = image_tensor.to(self.policy.device, dtype=self.policy.dtype)

        image_r = self.expand2square(image_r, tuple(x for x in self.image_processor.image_mean))
        image_tensor_r = self.image_processor.preprocess(
            image_r, return_tensors='pt', do_normalize=True, do_rescale=False, do_center_crop=False
        )['pixel_values']
        image_tensor_r = image_tensor_r.to(self.policy.device, dtype=self.policy.dtype)

        inp = raw_lang
        if self.policy.config.mm_use_im_start_end:
            inp = DEFAULT_IM_START_TOKEN + DEFAULT_IMAGE_TOKEN + DEFAULT_IM_END_TOKEN + '\n' + inp
        else:
            inp = DEFAULT_IMAGE_TOKEN + '\n' + inp
        self.conv.append_message(self.conv.roles[0], inp)
        self.conv.append_message(self.conv.roles[1], None)
        prompt = self.conv.get_prompt()
        prompt += " <|endoftext|>"

        input_ids = tokenizer_image_token(prompt, self.tokenizer, IMAGE_TOKEN_INDEX, return_tensors='pt').unsqueeze(0).cuda()
        attn_mask = input_ids.ne(self.tokenizer.pad_token_id)
        states = robo_state.to(self.policy.device, dtype=self.policy.dtype)

        return dict(
            input_ids=input_ids,
            attention_mask=attn_mask,
            images=image_tensor,
            images_r=image_tensor_r,
            states=states.unsqueeze(0),
        )

    def prepare_observation(self, obs, task_name, stats, gripper_closed):
        """Crop/resize the two camera views and build the normalized 8-dim proprio
        state [eef_pos(3), eef_euler_rpy(3), gripper_open, gripper_closed]."""
        front_image = crop_front_image(obs['camera_front_image'], task_name)

        eye_in_hand = cv2.flip(obs['eye_in_hand_image'], 1)
        eye_in_hand = cv2.resize(eye_in_hand, (224, 224))

        images = np.array([
            cv2.resize(front_image, (320, 180)),
            cv2.resize(eye_in_hand, (320, 180)),
        ])
        images = images / 255.0

        eef_mat = R_EE_TO_GRIPPER @ _quat2mat(obs['eef_quat'])
        eef_euler = np.array([normalize_angle(a) for a in _mat2euler_sxyz(eef_mat)])

        eef_pose = np.zeros(6, dtype=np.float32)
        eef_pose[0:3] = obs['eef_pos']
        eef_pose[3:6] = eef_euler

        gripper_state = np.array([0.0, 1.0], dtype=np.float32) if gripper_closed else np.array([0.0, 0.0], dtype=np.float32)
        state = np.zeros(8, dtype=np.float32)
        state[0:6] = eef_pose
        state[6:8] = gripper_state

        state = (state - stats["qpos_mean"]) / stats["qpos_std"]
        return images, state

    def convert_action(self, pred_action):
        """Decode the action head's [xyz(3), rot_6d(6), gripper(1)] output into
        [xyz(3), euler_rpy(3), gripper(1)]."""
        cur_xyz = pred_action[:3]
        cur_rot6d = pred_action[3:9]
        cur_gripper = np.expand_dims(pred_action[-1], axis=0)

        cur_rot6d = torch.from_numpy(cur_rot6d).unsqueeze(0)
        cur_euler = rot_6d_to_euler_angles(rot_6d=cur_rot6d, convention="XYZ").squeeze().numpy()
        return np.concatenate((cur_xyz, cur_euler, cur_gripper))

    def action_post_processing(self, obs, action_chunk, n_steps=-1):
        """Integrate a chunk of delta-actions against the current EEF pose to
        produce absolute world-frame actions [x, y, z, ax, ay, az, gripper]."""
        task_suite_name = getattr(self.policy_config, "task_suite_name", "")
        post_processed_actions = []
        for action in action_chunk:
            action = action * SCALE_FACTOR

            action_world = np.zeros(7)
            if 'abs_pose' in task_suite_name:
                action_world[0:3] = action[0:3]
            else:
                action_world[0:3] = obs['eef_pos'] + action[0:3]

                current_gripper_orientation = _mat2euler_sxyz(R_EE_TO_GRIPPER @ _quat2mat(obs['eef_quat']))
                current_gripper_orientation = [normalize_angle(a) for a in current_gripper_orientation]
                gripper_orientation_action = current_gripper_orientation + action[3:6]
                gripper_orientation_action = [normalize_angle(a) for a in gripper_orientation_action]
                action_world[3:6] = euler_to_axis_angle(gripper_orientation_action)
                action_world[6] = action[-1]

            post_processed_actions.append(action_world.copy())

        return post_processed_actions

    def compute_action(self, obs, gripper_closed, task_description, task_name='pick_place', n_steps=-1):
        """Run one TinyVLA forward pass (with ACT-style temporal aggregation across
        steps) and return a single-element list containing the world-frame action.

        Parameters
        ----------
        obs : dict with keys 'camera_front_image', 'eye_in_hand_image' (raw uint8
              RGB arrays) and 'eef_pos' (3,), 'eef_quat' (4,) (xyzw).
        gripper_closed : 0 or 1 - the currently-commanded gripper state.
        task_description : language goal string.
        task_name : used to select the front-camera crop window (see tinyvla_utils.TASK_CROP).
        n_steps : current control-loop step for this trajectory; pass 0 to (re)start
                  temporal aggregation and run the warm-up queries.

        Returns
        -------
        (actions, inf_time) : actions is a list with one 7-D numpy array
        [x, y, z, ax, ay, az, gripper]; inf_time is the model query latency in seconds.
        """
        max_timesteps = 200
        num_queries = self.policy.config.chunk_size

        if n_steps == 0:
            self.all_time_actions = torch.zeros(
                [max_timesteps, max_timesteps + num_queries, self.action_dim], dtype=torch.float32
            ).cuda()

        with torch.no_grad():
            traj_rgb_np, robot_state = self.prepare_observation(
                obs=obs, task_name=task_name, stats=self.stats, gripper_closed=gripper_closed
            )
            robot_state = torch.from_numpy(robot_state).float().cuda()

            curr_image = torch.stack([self.to_tensor(img).float().cuda() for img in traj_rgb_np], dim=0)
            if self.policy_config.action_head != 'act':
                # random-resized-crop-free eval-time equivalent used at training: a
                # fixed 0.95 center crop, then resize back to the original size.
                original_size = curr_image.shape[-2:]
                ratio = 0.95
                curr_image = curr_image[:, :,
                                         int(original_size[0] * (1 - ratio) / 2): int(original_size[0] * (1 + ratio) / 2),
                                         int(original_size[1] * (1 - ratio) / 2): int(original_size[1] * (1 + ratio) / 2)]
                curr_image = curr_image.squeeze(0)
                curr_image = transforms.Resize(original_size, antialias=True)(curr_image)
                curr_image = curr_image.unsqueeze(0)
            else:
                curr_image = curr_image.unsqueeze(0)

            if n_steps == 0:
                for _ in range(10):
                    batch = self.process_batch_to_llava(curr_image, robot_state, task_description)
                    self.policy(**batch, eval=True)

            time_start = time.time()
            batch = self.process_batch_to_llava(curr_image, robot_state, task_description)
            all_actions = self.policy(**batch, eval=True)
            inf_time = time.time() - time_start

            self.all_time_actions[[n_steps], n_steps:n_steps + num_queries] = all_actions
            actions_for_curr_step = self.all_time_actions[:, n_steps]
            actions_populated = torch.all(actions_for_curr_step != 0, axis=1)
            actions_for_curr_step = actions_for_curr_step[actions_populated]
            k = 0.01
            exp_weights = np.exp(-k * np.arange(len(actions_for_curr_step)))
            exp_weights = exp_weights / exp_weights.sum()
            exp_weights = torch.from_numpy(exp_weights).cuda().unsqueeze(dim=1)
            raw_action = (actions_for_curr_step * exp_weights).sum(dim=0, keepdim=True)

            raw_action = raw_action.squeeze(0).cpu().numpy()
            action = self.unnormalize_action(raw_action)
            action = self.convert_action(action)

            actions = self.action_post_processing(obs=obs, action_chunk=action[None, :], n_steps=n_steps)
            return actions, inf_time
