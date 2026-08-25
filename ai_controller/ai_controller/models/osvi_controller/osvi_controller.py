import importlib
import json
import os
import pickle
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np
import torch
import yaml
from PIL import Image as PILImage

from ai_controller.utils.ai_controller import AIController
from ai_controller.utils.utils import EEF_POS_NAME, EEF_QUAT_NAME, seed_everything


IMAGENET_MEAN = np.asarray([0.485, 0.456, 0.406], dtype=np.float32).reshape(3, 1, 1)
IMAGENET_STD = np.asarray([0.229, 0.224, 0.225], dtype=np.float32).reshape(3, 1, 1)
IDENTITY_QUAT_XYZW = np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
TOP_DOWN_QUAT_XYZW = np.asarray(
    [0.9994452044624775, 0.03161651380119412, 0.0021438049655468088, 0.010251021036213035],
    dtype=np.float64,
)

_THIS_DIR = Path(__file__).resolve().parent
_MODEL_ROOT = None
for candidate in (_THIS_DIR / "osvi_wm", _THIS_DIR):
    if (candidate / "models" / "model.py").is_file():
        _MODEL_ROOT = candidate
        if str(candidate) not in sys.path:
            sys.path.insert(0, str(candidate))
        break

if _MODEL_ROOT is None:
    raise ImportError(
        "Could not find OSVI model sources. Expected either "
        "osvi_controller/osvi_wm/models/model.py or osvi_controller/models/model.py."
    )

from models.model import StateSpaceModel


@dataclass
class OSVIConfig:
    checkpoint_path: str = "checkpoints/best.pt"
    training_config_path: str = "configs/training_config.yaml"
    projection_matrix_path: str = "configs/ur5e_zed_front_projection.yaml"
    device: str = "cuda"
    image: dict = field(default_factory=dict)
    context: dict = field(default_factory=dict)
    agent_observation: dict = field(default_factory=dict)
    waypoints: dict = field(default_factory=dict)
    projection: dict = field(default_factory=dict)
    control: dict = field(default_factory=dict)
    grasp_refinement: dict = field(default_factory=dict)
    safety: dict = field(default_factory=dict)


class TrajectoryUnpickler(pickle.Unpickler):
    """Resolve Trajectory objects saved by dataset_collector_pkg/savers.py."""

    def find_class(self, module, name):
        if name == "Trajectory":
            for module_name in ("savers", "scripts.savers"):
                try:
                    return importlib.import_module(module_name).Trajectory
                except Exception:
                    pass
        return super().find_class(module, name)


class OSVIController(AIController):
    """Controller wrapper for the trained OSVI-WM waypoint predictor.

    OSVI predicts image-normalized waypoints with layout [x_norm, y_norm, z, grip].
    This wrapper converts them to base_link waypoints, adds the fixed top-down
    TCP orientation used by the scripted controller, and returns actions in the
    same [x, y, z, qx, qy, qz, qw, gripper] format used by ai_controller_node.py.
    """

    def __init__(self, model_config: str, task_name: str = "pick_place"):
        self.task_name = task_name
        self.cfg: Optional[OSVIConfig] = None
        self.config_dir: Optional[Path] = None
        self.training_config = None
        self.projection_matrix = None
        self.context_tensor = None
        self.context_source = None
        self.current_eef_quat = None
        self.gripper_closed = False
        self.epoch = "unknown"
        self.global_step = "unknown"

        super().__init__(model_config)

        seed_everything(42)
        self.model.eval()

    def _resolve_path(self, value):
        path = Path(os.path.expanduser(str(value)))
        if not path.is_absolute():
            path = self.config_dir / path
        return path

    def load_model(self, model_config: str):
        self.config_dir = Path(model_config).expanduser().resolve().parent
        with open(model_config, "r") as f:
            config_dict = yaml.safe_load(f) or {}

        valid_fields = OSVIConfig.__dataclass_fields__.keys()
        self.cfg = OSVIConfig(**{k: v for k, v in config_dict.items() if k in valid_fields})

        requested_device = self.cfg.device
        if requested_device == "cuda" and not torch.cuda.is_available():
            print("[OSVIController] CUDA requested but unavailable; falling back to CPU.")
            requested_device = "cpu"
        self.device = torch.device(requested_device)

        self.checkpoint_path = self._resolve_path(self.cfg.checkpoint_path)
        self.training_config_path = self._resolve_path(self.cfg.training_config_path)
        self.projection_matrix_path = self._resolve_path(self.cfg.projection_matrix_path)

        with open(self.training_config_path, "r") as f:
            self.training_config = yaml.safe_load(f)
        with open(self.projection_matrix_path, "r") as f:
            projection_data = yaml.safe_load(f)

        matrix = projection_data["projection_matrix"] if isinstance(projection_data, dict) else projection_data
        self.projection_matrix = torch.as_tensor(matrix, dtype=torch.float32, device=self.device)
        if self.projection_matrix.shape != (4, 4):
            raise ValueError(f"projection_matrix must be 4x4, got {tuple(self.projection_matrix.shape)}")

        model_cfg = self.training_config["model"]
        model = StateSpaceModel(
            latent_dim=int(model_cfg["latent_dim"]),
            waypoints=int(model_cfg["waypoints"]),
            sub_waypoints=bool(model_cfg["sub_waypoints"]),
            metaworld=False,
        ).to(self.device)

        try:
            checkpoint = torch.load(self.checkpoint_path, map_location=self.device, weights_only=False)
        except TypeError:
            checkpoint = torch.load(self.checkpoint_path, map_location=self.device)

        state_dict = checkpoint.get("model_state_dict", checkpoint) if isinstance(checkpoint, dict) else checkpoint
        model.load_state_dict(state_dict)
        if isinstance(checkpoint, dict):
            self.epoch = checkpoint.get("epoch", "unknown")
            self.global_step = checkpoint.get("global_step", "unknown")
        model.eval()

        print(
            f"[OSVIController] Loaded checkpoint {self.checkpoint_path} "
            f"(epoch={self.epoch}, global_step={self.global_step})"
        )
        return model

    def move_model_to_device(self, device):
        self.device = torch.device(device)
        self.model = self.model.to(self.device)
        if self.projection_matrix is not None:
            self.projection_matrix = self.projection_matrix.to(self.device)
        if self.context_tensor is not None:
            self.context_tensor = self.context_tensor.to(self.device)

    def reset(self):
        self.context_tensor = None
        self.context_source = None
        self.current_eef_quat = None
        self.gripper_closed = False

    def load_command(self, demo_path: str, task_id: str, save_demo_frames=True, traj_cnt=0, save_path=None):
        task_folder = str(task_id)
        if not task_folder.startswith("task_"):
            task_folder = f"task_{task_folder.zfill(2)}"

        demo_dir = Path(os.path.expanduser(demo_path)) / task_folder
        demo_files = sorted(demo_dir.glob("*.pkl"))
        if not demo_files:
            raise FileNotFoundError(f"No demo .pkl files found in {demo_dir}")

        demo_file = demo_files[0]
        with demo_file.open("rb") as f:
            payload = TrajectoryUnpickler(f).load()
        traj = self._unwrap_trajectory(payload)

        frames = self._sample_context_frames(traj)
        processed = [self._preprocess_frame(frame, source="demo") for frame in frames]
        context_np = np.stack(processed, axis=0)
        self.context_tensor = torch.from_numpy(context_np).unsqueeze(0).float().to(self.device)
        self.context_source = str(demo_file)

        if save_demo_frames and save_path is not None:
            out_dir = Path(save_path) / task_folder / f"osvi_context_{int(traj_cnt):03d}"
            out_dir.mkdir(parents=True, exist_ok=True)
            for i, frame in enumerate(frames):
                PILImage.fromarray(self._as_uint8_rgb_for_preview(frame)).save(out_dir / f"context_raw_{i:02d}.png")
                PILImage.fromarray(self._chw_to_uint8(context_np[i])).save(out_dir / f"context_model_{i:02d}.png")

        print(f"[OSVIController] Loaded context from {demo_file}: {tuple(self.context_tensor.shape)}")

    def pre_process(self, input_data):
        if self.context_tensor is None:
            raise RuntimeError("OSVI context is missing. Call load_command() before inference().")

        images, states = input_data[0], input_data[1]
        image_cfg = self.cfg.image
        front_idx = int(image_cfg.get("front_camera_index", 0))
        if front_idx >= len(images):
            raise IndexError(f"front_camera_index={front_idx} but only {len(images)} image(s) were provided")

        live_frame = self._prepare_live_frame(images[front_idx])
        model_frame = self._preprocess_frame(live_frame, source="live")

        obs_cfg = self.cfg.agent_observation
        num_frames = int(obs_cfg.get("num_frames", 2))
        if bool(obs_cfg.get("repeat_current_image", True)):
            agent_np = np.stack([model_frame for _ in range(num_frames)], axis=0)
        else:
            raise NotImplementedError("OSVIController currently supports repeat_current_image only.")

        agent_tensor = torch.from_numpy(agent_np).unsqueeze(0).float().to(self.device)
        state = self._parse_state(states)
        return {
            "images": agent_tensor,
            "context": self.context_tensor,
            "state": state,
            "live_frame": live_frame,
        }

    def post_process(self, output_data):
        base_waypoints = np.asarray(output_data["base_waypoints"], dtype=np.float64)
        image_waypoints = np.asarray(output_data["image_waypoints"], dtype=np.float64)
        state = output_data.get("state", {})
        quat = self._orientation_for_action(state)

        control_cfg = self.cfg.control
        safety_cfg = self.cfg.safety
        close_threshold = float(control_cfg.get("gripper_close_threshold", 0.1))
        open_pos = float(control_cfg.get("gripper_open_position", 0.0))
        closed_pos = float(control_cfg.get("gripper_closed_position", 255.0))
        max_actions = int(control_cfg.get("max_actions_per_inference", len(base_waypoints)))
        min_dist = float(safety_cfg.get("min_waypoint_distance", 0.0))

        refine_cfg = self.cfg.grasp_refinement
        refine_enabled = bool(refine_cfg.get("enabled", False))
        refinement_done = False
        actions = []
        for base_wp, image_wp in zip(base_waypoints[:max_actions], image_waypoints[:max_actions]):
            xyz = self._apply_workspace_safety(base_wp[:3])
            raw_gripper = float(image_wp[3])
            should_close = raw_gripper >= close_threshold
            is_closing_transition = should_close and not self.gripper_closed

            if refine_enabled and is_closing_transition and not refinement_done:
                self._append_grasp_refinement(
                    actions=actions,
                    target_xyz=xyz,
                    quat=quat,
                    open_pos=open_pos,
                    closed_pos=closed_pos,
                    min_dist=min_dist,
                    state=state,
                )
                self.gripper_closed = True
                refinement_done = True
                if not bool(refine_cfg.get("resume_after_lift", True)):
                    break
                continue

            self.gripper_closed = should_close
            gripper_cmd = closed_pos if should_close else open_pos
            self._append_action(actions, xyz, quat, gripper_cmd, min_dist=min_dist)

        if not actions and len(base_waypoints):
            xyz = self._apply_workspace_safety(base_waypoints[-1, :3])
            raw_gripper = float(image_waypoints[-1, 3])
            gripper_cmd = closed_pos if raw_gripper >= close_threshold else open_pos
            self._append_action(actions, xyz, quat, gripper_cmd)

        return actions

    def _append_grasp_refinement(self, actions, target_xyz, quat, open_pos, closed_pos, min_dist, state):
        refine_cfg = self.cfg.grasp_refinement
        target_xyz = self._apply_workspace_safety(target_xyz)

        approach_xyz = target_xyz.copy()
        approach_xyz[2] += float(refine_cfg.get("approach_height", 0.04))
        approach_xyz = self._apply_workspace_safety(approach_xyz)

        start_xyz = actions[-1][:3] if actions else None
        if start_xyz is None and isinstance(state, dict):
            start_xyz = state.get("eef_pos")
        if start_xyz is None:
            start_xyz = approach_xyz

        approach_step = float(refine_cfg.get("approach_step", 0.03))
        descent_step = float(refine_cfg.get("descent_step", 0.01))
        self._append_linear_actions(actions, start_xyz, approach_xyz, quat, open_pos, approach_step, min_dist)
        self._append_linear_actions(actions, approach_xyz, target_xyz, quat, open_pos, descent_step, min_dist)
        self._append_action(actions, target_xyz, quat, closed_pos)

        if bool(refine_cfg.get("lift_after_close", True)):
            lift_xyz = target_xyz.copy()
            lift_xyz[2] += float(refine_cfg.get("lift_height", 0.15))
            lift_xyz = self._apply_workspace_safety(lift_xyz)
            lift_step = float(refine_cfg.get("lift_step", approach_step))
            self._append_linear_actions(actions, target_xyz, lift_xyz, quat, closed_pos, lift_step, min_dist)

    def _append_linear_actions(self, actions, start_xyz, end_xyz, quat, gripper_cmd, max_step, min_dist=0.0):
        start_xyz = np.asarray(start_xyz, dtype=np.float64)
        end_xyz = np.asarray(end_xyz, dtype=np.float64)
        distance = float(np.linalg.norm(end_xyz - start_xyz))
        if distance <= 1e-9:
            self._append_action(actions, end_xyz, quat, gripper_cmd, min_dist=min_dist)
            return

        max_step = max(float(max_step), 1e-6)
        n_steps = max(1, int(np.ceil(distance / max_step)))
        for i in range(1, n_steps + 1):
            alpha = i / n_steps
            xyz = start_xyz + (end_xyz - start_xyz) * alpha
            self._append_action(actions, xyz, quat, gripper_cmd, min_dist=min_dist)

    def _append_action(self, actions, xyz, quat, gripper_cmd, min_dist=0.0):
        xyz = self._apply_workspace_safety(xyz)
        if actions and min_dist > 0.0:
            same_pose = np.linalg.norm(xyz - actions[-1][:3]) < min_dist
            same_gripper = abs(float(actions[-1][-1]) - float(gripper_cmd)) < 1e-9
            if same_pose and same_gripper:
                return
        actions.append(np.concatenate([xyz, quat, [float(gripper_cmd)]]))

    def _apply_workspace_safety(self, xyz):
        xyz = np.asarray(xyz, dtype=np.float64)
        safety_cfg = self.cfg.safety
        if bool(safety_cfg.get("clamp_workspace", False)):
            xyz = np.clip(
                xyz,
                np.asarray(safety_cfg.get("workspace_min"), dtype=np.float64),
                np.asarray(safety_cfg.get("workspace_max"), dtype=np.float64),
            )
        return xyz

    def inference(self, input_data, t: int, save_path: Optional[str] = None):
        processed = self.pre_process(input_data)

        if save_path is not None:
            os.makedirs(save_path, exist_ok=True)
            PILImage.fromarray(processed["live_frame"]).save(
                os.path.join(save_path, f"osvi_input_front_t{t:03d}.png")
            )

        with torch.no_grad():
            out = self.model(processed["images"], processed["context"])
            all_waypoints = out["waypoints"].float()
            selected_waypoints = self._select_waypoints(all_waypoints)
            base_waypoints = self._project_waypoints_to_base(selected_waypoints)

        image_np = selected_waypoints[0].detach().cpu().numpy()
        base_np = base_waypoints[0].detach().cpu().numpy()

        if save_path is not None:
            with open(os.path.join(save_path, f"osvi_waypoints_t{t:03d}.json"), "w") as f:
                json.dump(
                    {
                        "image_waypoints": image_np.tolist(),
                        "base_waypoints": base_np.tolist(),
                        "context_source": self.context_source,
                    },
                    f,
                    indent=2,
                )

        print(f"[OSVIController] Inference t={t}: selected {len(base_np)} waypoint(s)")
        return self.post_process(
            {
                "image_waypoints": image_np,
                "base_waypoints": base_np,
                "state": processed["state"],
            }
        )

    def _select_waypoints(self, all_waypoints):
        wp_cfg = self.cfg.waypoints
        base_count = int(wp_cfg.get("base_count", 5))
        selected_count = int(wp_cfg.get("selected_count", base_count))
        selection = wp_cfg.get("selection", "last_group")
        sub_waypoints = bool(wp_cfg.get("sub_waypoints", False))

        if selection == "last_group" and sub_waypoints:
            start = base_count * (base_count - 1) // 2
        else:
            start = 0
        end = start + selected_count
        if end > all_waypoints.shape[1]:
            raise ValueError(
                f"Cannot select waypoints [{start}:{end}] from tensor with shape {tuple(all_waypoints.shape)}"
            )
        return all_waypoints[:, start:end]

    def _project_waypoints_to_base(self, waypoints):
        hom_im_coords = torch.cat(
            (waypoints[:, :, :2] * waypoints[:, :, 2:3], waypoints[:, :, 2:3]),
            dim=-1,
        )
        ones = torch.ones(*hom_im_coords.shape[:2], 1, device=waypoints.device)
        hom_im_4d = torch.cat((hom_im_coords, ones), dim=-1)
        projection = self.projection_matrix.unsqueeze(0).expand(waypoints.shape[0], -1, -1)
        trans_waypoints = torch.einsum("bwh,bdh->bwd", hom_im_4d, projection.float())
        return torch.cat((trans_waypoints[:, :, :3], waypoints[:, :, 3:]), dim=-1)

    def _sample_context_frames(self, traj):
        num_frames = int(self.cfg.context.get("num_frames", 10))
        length = self._traj_len(traj)
        indices = np.linspace(0, length - 1, num=num_frames, endpoint=True, dtype=int)
        frames = []
        for index in indices:
            step = self._traj_step(traj, int(index))
            obs = self._step_obs(step)
            frames.append(self._get_obs_image(obs, source="demo"))
        return frames

    def _preprocess_frame(self, frame, source):
        img = self._as_uint8_rgb_for_model(frame, source=source)
        img = self._crop(img, self.cfg.image.get("crop", [0, 0, 0, 0]))
        height = int(self.cfg.image.get("height", 240))
        width = int(self.cfg.image.get("width", 320))
        if img.shape[:2] != (height, width):
            img = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)
        chw = img.astype(np.float32).transpose(2, 0, 1) / 255.0
        if bool(self.cfg.image.get("normalize", True)):
            chw = (chw - IMAGENET_MEAN) / IMAGENET_STD
        return chw.astype(np.float32)

    def _prepare_live_frame(self, frame):
        img = self._decode_image(frame, convert_bgr_to_rgb=False)
        if not bool(self.cfg.image.get("live_images_are_rgb", True)):
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return img

    def _as_uint8_rgb_for_model(self, frame, source):
        if source == "demo":
            convert = bool(self.cfg.image.get("demo_convert_bgr_to_rgb", False))
            return self._decode_image(frame, convert_bgr_to_rgb=convert)
        return self._decode_image(frame, convert_bgr_to_rgb=False)

    def _as_uint8_rgb_for_preview(self, frame):
        return self._decode_image(frame, convert_bgr_to_rgb=bool(self.cfg.image.get("demo_convert_bgr_to_rgb", False)))

    def _decode_image(self, value, convert_bgr_to_rgb=False):
        img = value
        if isinstance(img, (bytes, bytearray)):
            img = np.frombuffer(img, dtype=np.uint8)
        if isinstance(img, np.ndarray) and img.ndim == 1:
            decoded = cv2.imdecode(img.astype(np.uint8), cv2.IMREAD_COLOR)
            if decoded is None:
                raise ValueError("cv2.imdecode failed for compressed image")
            img = decoded
        img = np.asarray(img)
        if img.ndim == 2:
            img = img[:, :, None]
        if img.ndim == 3 and img.shape[0] in (1, 3, 4) and img.shape[-1] not in (1, 3, 4):
            img = np.transpose(img, (1, 2, 0))
        if img.shape[-1] == 4:
            img = img[:, :, :3]
        img = img.astype(np.uint8)
        if convert_bgr_to_rgb and img.shape[-1] >= 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        if img.shape[-1] == 1:
            img = np.repeat(img, 3, axis=-1)
        return img

    def _chw_to_uint8(self, chw):
        arr = np.asarray(chw, dtype=np.float32)
        if bool(self.cfg.image.get("normalize", True)):
            arr = arr * IMAGENET_STD + IMAGENET_MEAN
        arr = np.transpose(arr, (1, 2, 0)) * 255.0
        return np.clip(arr, 0, 255).astype(np.uint8)

    def _crop(self, img, crop):
        top, bottom, left, right = [int(x) for x in crop]
        if top > 0:
            img = img[top:]
        if bottom > 0:
            img = img[:-bottom]
        if left > 0:
            img = img[:, left:]
        if right > 0:
            img = img[:, :-right]
        return img

    def _parse_state(self, states):
        state = {"eef_pos": None, "eef_quat": None}
        if states is None:
            return state

        if isinstance(states, dict):
            pos = states.get(EEF_POS_NAME)
            if pos is None:
                pos = states.get("eef_pos")
            quat = states.get(EEF_QUAT_NAME)
            if quat is None:
                quat = states.get("eef_quat")
        else:
            arr = np.asarray(states, dtype=np.float64).reshape(-1)
            pos = arr[:3] if arr.size >= 3 else None
            quat = arr[3:7] if arr.size >= 7 else None

        if pos is not None:
            state["eef_pos"] = np.asarray(pos, dtype=np.float64)
        if quat is not None:
            quat = np.asarray(quat, dtype=np.float64)
            norm = np.linalg.norm(quat)
            if norm > 1e-8:
                quat = quat / norm
            self.current_eef_quat = quat
            state["eef_quat"] = quat
        return state

    def _orientation_for_action(self, state):
        mode = self.cfg.control.get("orientation_mode", "current_eef_quat")
        if mode == "identity":
            return IDENTITY_QUAT_XYZW.copy()
        if mode in ("fixed_top_down", "top_down"):
            quat = np.asarray(
                self.cfg.control.get("fixed_orientation_xyzw", TOP_DOWN_QUAT_XYZW),
                dtype=np.float64,
            )
            norm = np.linalg.norm(quat)
            if norm <= 1e-8:
                raise ValueError("fixed_orientation_xyzw must be a non-zero xyzw quaternion")
            return quat / norm
        quat = state.get("eef_quat") if isinstance(state, dict) else None
        if quat is None:
            quat = self.current_eef_quat
        if quat is None:
            print("[OSVIController] Missing EEF quaternion; using identity orientation.")
            quat = IDENTITY_QUAT_XYZW
        return np.asarray(quat, dtype=np.float64)

    @staticmethod
    def _unwrap_trajectory(payload):
        if isinstance(payload, dict) and "traj" in payload:
            return payload["traj"]
        return payload

    @staticmethod
    def _traj_len(traj):
        return len(traj)

    @staticmethod
    def _traj_step(traj, index):
        if hasattr(traj, "get"):
            return traj.get(index)
        return traj[index]

    @staticmethod
    def _step_obs(step):
        if isinstance(step, dict) and "obs" in step:
            return step["obs"]
        return step

    def _get_obs_image(self, obs, source):
        if source == "demo":
            keys = ("camera_front_image", "front_camera_image")
        else:
            keys = ("camera_front_image", "front_camera_image")
        for key in keys:
            if key in obs:
                return obs[key]
        raise KeyError(f"None of image keys {keys} found in obs keys {list(obs.keys())}")
