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

try:
    import rclpy
    import rclpy.wait_for_message
    import tf2_ros
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.time import Time as RclpyTime
    from sensor_msgs.msg import CameraInfo
    from sensor_msgs.msg import Image as RosImage
except Exception:
    rclpy = None
    tf2_ros = None
    CvBridge = None
    Node = None
    RclpyTime = None
    CameraInfo = None
    RosImage = None


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


def _compute_crop_adjustment(crop_values, size_before):
    top, bottom, left, right = [int(x) for x in crop_values]
    rows_before, cols_before = [int(x) for x in size_before]
    rows_after = rows_before - (top + bottom)
    cols_after = cols_before - (left + right)
    if rows_before <= 0 or cols_before <= 0 or rows_after <= 0 or cols_after <= 0:
        raise ValueError(
            f"Invalid crop {crop_values} for source image size "
            f"(height={rows_before}, width={cols_before})"
        )

    to_01 = np.array(
        [
            [0.5, 0.0, 0.5, 0.0],
            [0.0, 0.5, 0.5, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    from_01 = np.linalg.inv(to_01)
    crop_adjustment = np.array(
        [
            [cols_after / cols_before, 0.0, left / cols_before, 0.0],
            [0.0, rows_after / rows_before, bottom / rows_before, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    return from_01 @ np.linalg.inv(crop_adjustment) @ to_01


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
    debug: dict = field(default_factory=dict)


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
        self.last_gripper_decisions = []
        self.epoch = "unknown"
        self.global_step = "unknown"
        self._depth_ros_node = None
        self._depth_bridge = None
        self._depth_tf_buffer = None
        self._depth_tf_listener = None
        self._depth_camera_matrix = None
        self._depth_camera_info_size = None
        self._depth_warning_printed = False

        super().__init__(model_config)
        self._maybe_init_gripper_depth_refinement()

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
        matrix = np.asarray(matrix, dtype=np.float64)
        if matrix.shape != (4, 4):
            raise ValueError(f"projection_matrix must be 4x4, got {tuple(matrix.shape)}")

        crop = [int(x) for x in self.cfg.image.get("crop", [0, 0, 0, 0])]
        apply_crop_adjustment = bool(self.cfg.projection.get("apply_crop_adjustment", True))
        if any(crop) and apply_crop_adjustment:
            source_size = self.cfg.projection.get("source_image_size")
            if source_size is None and isinstance(projection_data, dict):
                camera_cfg = projection_data.get("camera", {}) or {}
                source_size = (camera_cfg.get("image_height"), camera_cfg.get("image_width"))
            if source_size is None or source_size[0] is None or source_size[1] is None:
                raise ValueError(
                    "Non-zero image.crop requires projection.camera.image_height/image_width "
                    "or projection.source_image_size=[height, width] in the OSVI config."
                )
            crop_adjust = _compute_crop_adjustment(crop, source_size)
            matrix = matrix @ np.linalg.inv(crop_adjust)
            print(
                "[OSVIController] Applied crop adjustment to projection matrix: "
                f"crop={crop}, source_size={tuple(int(x) for x in source_size)}"
            )

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
        self.last_gripper_decisions = []

    def _gripper_depth_enabled(self):
        refine_cfg = self.cfg.grasp_refinement if self.cfg is not None else {}
        return bool(refine_cfg.get("enabled", False)) and bool(refine_cfg.get("use_gripper_depth", False))

    def _maybe_init_gripper_depth_refinement(self):
        if not self._gripper_depth_enabled() or self._depth_ros_node is not None:
            return

        missing = [
            name for name, value in (
                ("rclpy", rclpy),
                ("tf2_ros", tf2_ros),
                ("CvBridge", CvBridge),
                ("Node", Node),
                ("CameraInfo", CameraInfo),
                ("RosImage", RosImage),
                ("RclpyTime", RclpyTime),
            )
            if value is None
        ]
        if missing:
            self._print_depth_warning_once(
                f"Gripper depth refinement disabled: missing ROS dependency/dependencies {missing}."
            )
            return

        try:
            if not rclpy.ok():
                self._print_depth_warning_once(
                    "Gripper depth refinement disabled: rclpy is not initialized."
                )
                return

            refine_cfg = self.cfg.grasp_refinement
            node_name = str(refine_cfg.get("depth_ros_node_name", "osvi_gripper_depth_refinement"))
            self._depth_ros_node = Node(node_name)
            self._depth_bridge = CvBridge()
            self._depth_tf_buffer = tf2_ros.Buffer()
            self._depth_tf_listener = tf2_ros.TransformListener(
                self._depth_tf_buffer,
                self._depth_ros_node,
            )
            print(
                "[OSVIController] Gripper depth refinement enabled "
                f"on {self._gripper_depth_topic()}"
            )
        except Exception as exc:
            self._print_depth_warning_once(
                f"Gripper depth refinement disabled: could not create ROS helpers ({exc})."
            )
            if self._depth_ros_node is not None:
                try:
                    self._depth_ros_node.destroy_node()
                except Exception:
                    pass
            self._depth_ros_node = None
            self._depth_bridge = None
            self._depth_tf_buffer = None
            self._depth_tf_listener = None

    def _gripper_depth_topic(self):
        refine_cfg = self.cfg.grasp_refinement
        topic = refine_cfg.get("depth_topic")
        if topic:
            return str(topic)
        camera_name = str(refine_cfg.get("depth_camera_name", "zed_gripper"))
        camera_node = str(refine_cfg.get("depth_camera_node_name", "zed_node"))
        return f"/{camera_name}/{camera_node}/depth/depth_registered"

    def _gripper_camera_info_topic(self):
        refine_cfg = self.cfg.grasp_refinement
        topic = refine_cfg.get("camera_info_topic")
        if topic:
            return str(topic)
        camera_name = str(refine_cfg.get("depth_camera_name", "zed_gripper"))
        camera_node = str(refine_cfg.get("depth_camera_node_name", "zed_node"))
        return f"/{camera_name}/{camera_node}/rgb/color/rect/camera_info"

    def _ensure_gripper_depth_ros_ready(self):
        if not self._gripper_depth_enabled():
            return False
        if self._depth_ros_node is None:
            self._maybe_init_gripper_depth_refinement()
        return self._depth_ros_node is not None

    def _depth_refined_grasp_target(self, predicted_xyz):
        predicted_xyz = self._apply_workspace_safety(predicted_xyz)
        if not self._gripper_depth_enabled():
            return predicted_xyz

        depth_target = self._estimate_gripper_depth_target_base()
        if depth_target is None:
            return predicted_xyz

        refine_cfg = self.cfg.grasp_refinement
        refined_xyz = np.asarray(depth_target, dtype=np.float64)
        refined_xyz[2] += float(refine_cfg.get("depth_grasp_z_offset", 0.03))

        max_xy_correction = refine_cfg.get("max_depth_xy_correction", None)
        if max_xy_correction is not None:
            xy_delta = float(np.linalg.norm(refined_xyz[:2] - predicted_xyz[:2]))
            if xy_delta > float(max_xy_correction):
                print(
                    "[OSVIController] Skipping gripper-depth refinement: "
                    f"XY correction {xy_delta:.3f} m exceeds max_depth_xy_correction."
                )
                return predicted_xyz

        refined_xyz = self._apply_workspace_safety(refined_xyz)
        print(
            "[OSVIController] Gripper-depth refinement target: "
            f"predicted={predicted_xyz.tolist()} refined={refined_xyz.tolist()}"
        )
        return refined_xyz

    def _estimate_gripper_depth_target_base(self):
        if not self._ensure_gripper_depth_ros_ready():
            return None
        if not self._load_depth_camera_info():
            return None

        depth_m, frame_id = self._read_gripper_depth_image()
        if depth_m is None:
            return None

        centroid = self._find_depth_object_centroid(depth_m)
        if centroid is None:
            self._print_depth_warning_once(
                "Gripper depth refinement skipped: no foreground object found in depth image."
            )
            return None

        u, v = centroid
        depth = self._median_depth_at(depth_m, u, v)
        if depth is None:
            self._print_depth_warning_once(
                "Gripper depth refinement skipped: no valid depth at object centroid."
            )
            return None

        camera_matrix = self._scaled_depth_camera_matrix(depth_m.shape[:2])
        point_camera = self._deproject_pixel(u, v, depth, camera_matrix)
        return self._transform_depth_point_to_target_frame(point_camera, frame_id)

    def _load_depth_camera_info(self):
        if self._depth_camera_matrix is not None:
            return True

        refine_cfg = self.cfg.grasp_refinement
        topic = self._gripper_camera_info_topic()
        retries = max(1, int(refine_cfg.get("camera_info_retries", 3)))
        timeout = float(refine_cfg.get("camera_info_timeout_sec", 0.5))
        for _ in range(retries):
            self._spin_depth_ros_once()
            try:
                ok, msg = rclpy.wait_for_message.wait_for_message(
                    topic=topic,
                    msg_type=CameraInfo,
                    node=self._depth_ros_node,
                    time_to_wait=timeout,
                )
            except Exception as exc:
                self._print_depth_warning_once(
                    f"Gripper depth refinement skipped: CameraInfo read failed ({exc})."
                )
                return False
            if ok:
                camera_matrix = np.asarray(msg.k, dtype=np.float64).reshape((3, 3))
                if camera_matrix[0, 0] <= 0.0 or camera_matrix[1, 1] <= 0.0:
                    self._print_depth_warning_once(
                        f"Gripper depth refinement skipped: invalid CameraInfo K on {topic}."
                    )
                    return False
                self._depth_camera_matrix = camera_matrix
                self._depth_camera_info_size = (int(msg.width), int(msg.height))
                return True

        self._print_depth_warning_once(
            f"Gripper depth refinement skipped: no CameraInfo received on {topic}."
        )
        return False

    def _read_gripper_depth_image(self):
        refine_cfg = self.cfg.grasp_refinement
        topic = self._gripper_depth_topic()
        timeout = float(refine_cfg.get("depth_timeout_sec", 0.25))
        self._spin_depth_ros_once()
        try:
            ok, msg = rclpy.wait_for_message.wait_for_message(
                topic=topic,
                msg_type=RosImage,
                node=self._depth_ros_node,
                time_to_wait=timeout,
            )
        except Exception as exc:
            self._print_depth_warning_once(
                f"Gripper depth refinement skipped: depth read failed ({exc})."
            )
            return None, None
        if not ok:
            self._print_depth_warning_once(
                f"Gripper depth refinement skipped: no depth image received on {topic}."
            )
            return None, None

        frame_id = str(getattr(msg.header, "frame_id", "") or "").lstrip("/")
        if not frame_id:
            self._print_depth_warning_once(
                "Gripper depth refinement skipped: depth image has no frame_id."
            )
            return None, None

        try:
            raw_depth = self._depth_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self._print_depth_warning_once(
                f"Gripper depth refinement skipped: depth conversion failed ({exc})."
            )
            return None, None
        return self._depth_to_meters(raw_depth), frame_id

    def _depth_to_meters(self, depth_image):
        raw = np.asarray(depth_image)
        if raw.ndim == 3:
            raw = raw[:, :, 0]
        scale = float(self.cfg.grasp_refinement.get("depth_scale", 1.0))
        if np.issubdtype(raw.dtype, np.integer) and scale == 1.0:
            scale = 0.001
        return raw.astype(np.float64) * scale

    def _find_depth_object_centroid(self, depth_m):
        refine_cfg = self.cfg.grasp_refinement
        max_depth = float(refine_cfg.get("depth_max_range_m", 1.0))
        min_height = float(refine_cfg.get("depth_min_object_height_m", 0.01))
        min_area = int(refine_cfg.get("depth_min_component_area_px", 20))

        valid = np.isfinite(depth_m) & (depth_m > 0.0) & (depth_m <= max_depth)
        if int(valid.sum()) < min_area:
            return None

        floor_depth = float(np.median(depth_m[valid]))
        object_mask = valid & (depth_m <= floor_depth - min_height)
        if int(object_mask.sum()) < min_area:
            return None

        mask = object_mask.astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8))
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)

        h, w = depth_m.shape[:2]
        image_center = np.asarray([(w - 1) * 0.5, (h - 1) * 0.5], dtype=np.float64)
        best_centroid = None
        best_score = None
        for label in range(1, num_labels):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < min_area:
                continue
            centroid = np.asarray(centroids[label], dtype=np.float64)
            score = float(np.linalg.norm(centroid - image_center))
            if best_score is None or score < best_score:
                best_score = score
                best_centroid = centroid

        if best_centroid is None:
            return None
        u = int(np.clip(round(best_centroid[0]), 0, w - 1))
        v = int(np.clip(round(best_centroid[1]), 0, h - 1))
        return u, v

    def _median_depth_at(self, depth_m, u, v):
        window = max(1, int(self.cfg.grasp_refinement.get("depth_window_px", 5)))
        h, w = depth_m.shape[:2]
        half = window // 2
        u0, u1 = max(0, u - half), min(w, u + half + 1)
        v0, v1 = max(0, v - half), min(h, v + half + 1)
        patch = depth_m[v0:v1, u0:u1].reshape(-1)
        max_depth = float(self.cfg.grasp_refinement.get("depth_max_range_m", 1.0))
        valid = patch[np.isfinite(patch) & (patch > 0.0) & (patch <= max_depth)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _scaled_depth_camera_matrix(self, depth_shape):
        camera_matrix = np.asarray(self._depth_camera_matrix, dtype=np.float64).copy()
        if self._depth_camera_info_size is None:
            return camera_matrix

        info_w, info_h = self._depth_camera_info_size
        depth_h, depth_w = int(depth_shape[0]), int(depth_shape[1])
        if info_w > 0 and info_h > 0 and (info_w != depth_w or info_h != depth_h):
            sx = depth_w / float(info_w)
            sy = depth_h / float(info_h)
            camera_matrix[0, 0] *= sx
            camera_matrix[0, 2] *= sx
            camera_matrix[1, 1] *= sy
            camera_matrix[1, 2] *= sy
        return camera_matrix

    @staticmethod
    def _deproject_pixel(u, v, depth, camera_matrix):
        fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
        cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
        x = (float(u) - cx) * depth / fx
        y = (float(v) - cy) * depth / fy
        return np.asarray([x, y, depth], dtype=np.float64)

    def _transform_depth_point_to_target_frame(self, point_camera, camera_frame):
        refine_cfg = self.cfg.grasp_refinement
        target_frame = str(
            refine_cfg.get("depth_target_frame")
            or self.cfg.projection.get("output_frame", "base_link")
        ).lstrip("/")
        camera_frame = str(camera_frame).lstrip("/")
        if target_frame == camera_frame:
            return np.asarray(point_camera, dtype=np.float64)

        transform = self._lookup_depth_transform(target_frame, camera_frame)
        if transform is None:
            return None

        trans = transform.transform.translation
        rot = transform.transform.rotation
        translation = np.asarray([trans.x, trans.y, trans.z], dtype=np.float64)
        rotation = self._quat_xyzw_to_mat(np.asarray([rot.x, rot.y, rot.z, rot.w], dtype=np.float64))
        return rotation @ np.asarray(point_camera, dtype=np.float64) + translation

    def _lookup_depth_transform(self, target_frame, source_frame):
        refine_cfg = self.cfg.grasp_refinement
        timeout = float(refine_cfg.get("tf_timeout_sec", 0.35))
        spin_dt = 0.02
        attempts = max(1, int(np.ceil(timeout / spin_dt)))
        last_exc = None
        for _ in range(attempts):
            try:
                return self._depth_tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    RclpyTime(),
                )
            except Exception as exc:
                last_exc = exc
                self._spin_depth_ros_once(spin_dt)

        self._print_depth_warning_once(
            "Gripper depth refinement skipped: could not transform "
            f"{source_frame} -> {target_frame} ({last_exc})."
        )
        return None

    def _spin_depth_ros_once(self, timeout_sec=0.02):
        if self._depth_ros_node is None or rclpy is None:
            return
        try:
            rclpy.spin_once(self._depth_ros_node, timeout_sec=timeout_sec)
        except Exception:
            pass

    def _print_depth_warning_once(self, message):
        if not self._depth_warning_printed:
            print(f"[OSVIController] {message}")
            self._depth_warning_printed = True

    @staticmethod
    def _quat_xyzw_to_mat(quat_xyzw):
        quat_xyzw = np.asarray(quat_xyzw, dtype=np.float64)
        norm = np.linalg.norm(quat_xyzw)
        if norm <= 1e-8:
            return np.eye(3, dtype=np.float64)
        x, y, z, w = quat_xyzw / norm
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        return np.asarray(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
            dtype=np.float64,
        )

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
            "model_frame": model_frame,
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
        max_actions = min(
            int(control_cfg.get("max_actions_per_inference", len(base_waypoints))),
            len(base_waypoints),
            len(image_waypoints),
        )
        min_dist = float(safety_cfg.get("min_waypoint_distance", 0.0))
        close_transition_offset = int(control_cfg.get("gripper_close_transition_offset", 0))

        selected_base_waypoints = base_waypoints[:max_actions]
        selected_image_waypoints = image_waypoints[:max_actions]
        raw_grippers = [float(image_wp[3]) for image_wp in selected_image_waypoints]
        threshold_close_flags = [raw_gripper >= close_threshold for raw_gripper in raw_grippers]
        should_close_flags = self._offset_first_close_transition(
            threshold_close_flags,
            close_transition_offset,
        )

        refine_cfg = self.cfg.grasp_refinement
        refine_enabled = bool(refine_cfg.get("enabled", False))
        refinement_done = False
        actions = []
        self.last_gripper_decisions = []
        for waypoint_index, (base_wp, raw_gripper, threshold_close, should_close) in enumerate(
            zip(selected_base_waypoints, raw_grippers, threshold_close_flags, should_close_flags),
            start=1,
        ):
            xyz = self._apply_workspace_safety(base_wp[:3])
            is_closing_transition = should_close and not self.gripper_closed

            if refine_enabled and is_closing_transition and not refinement_done:
                grasp_target = self._depth_refined_grasp_target(xyz)
                self._append_grasp_refinement(
                    actions=actions,
                    target_xyz=grasp_target,
                    quat=quat,
                    open_pos=open_pos,
                    closed_pos=closed_pos,
                    min_dist=min_dist,
                    state=state,
                )
                self.gripper_closed = True
                refinement_done = True
                self.last_gripper_decisions.append(
                    {
                        "waypoint_index": waypoint_index,
                        "raw_gripper": raw_gripper,
                        "close_threshold": close_threshold,
                        "closed_by_threshold": bool(threshold_close),
                        "commanded_closed": True,
                        "command_position": closed_pos,
                        "close_transition_offset": close_transition_offset,
                        "grasp_refinement_inserted": True,
                    }
                )
                if not bool(refine_cfg.get("resume_after_lift", True)):
                    break
                continue

            self.gripper_closed = should_close
            gripper_cmd = closed_pos if should_close else open_pos
            self._append_action(actions, xyz, quat, gripper_cmd, min_dist=min_dist)
            self.last_gripper_decisions.append(
                {
                    "waypoint_index": waypoint_index,
                    "raw_gripper": raw_gripper,
                    "close_threshold": close_threshold,
                    "closed_by_threshold": bool(threshold_close),
                    "commanded_closed": bool(should_close),
                    "command_position": float(gripper_cmd),
                    "close_transition_offset": close_transition_offset,
                    "grasp_refinement_inserted": False,
                }
            )
        if not actions and len(base_waypoints):
            xyz = self._apply_workspace_safety(base_waypoints[-1, :3])
            raw_gripper = float(image_waypoints[-1, 3])
            gripper_cmd = closed_pos if raw_gripper >= close_threshold else open_pos
            self._append_action(actions, xyz, quat, gripper_cmd)

        return actions

    @staticmethod
    def _offset_first_close_transition(close_flags, offset):
        flags = [bool(flag) for flag in close_flags]
        if not flags or offset == 0:
            return flags

        close_index = None
        previous_closed = False
        for index, is_closed in enumerate(flags):
            if is_closed and not previous_closed:
                close_index = index
                break
            previous_closed = is_closed

        if close_index is None:
            return flags

        shifted_index = int(np.clip(close_index + int(offset), 0, len(flags) - 1))
        if shifted_index < close_index:
            for index in range(shifted_index, close_index):
                flags[index] = True
        elif shifted_index > close_index:
            for index in range(close_index, shifted_index):
                flags[index] = False
            flags[shifted_index] = True

        return flags


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
            PILImage.fromarray(self._chw_to_uint8(processed["model_frame"])).save(
                os.path.join(save_path, f"osvi_model_input_front_t{t:03d}.png")
            )

        with torch.no_grad():
            out = self.model(processed["images"], processed["context"])
            all_waypoints = out["waypoints"].float()
            selected_waypoints = self._select_waypoints(all_waypoints)
            base_waypoints = self._project_waypoints_to_base(selected_waypoints)
            if self.device.type == "cuda":
                torch.cuda.synchronize(self.device)

        image_np = selected_waypoints[0].detach().cpu().numpy()
        base_np = base_waypoints[0].detach().cpu().numpy()

        actions = self.post_process(
            {
                "image_waypoints": image_np,
                "base_waypoints": base_np,
                "state": processed["state"],
            }
        )

        if save_path is not None:
            with open(os.path.join(save_path, f"osvi_waypoints_t{t:03d}.json"), "w") as f:
                json.dump(
                    {
                        "image_waypoints": image_np.tolist(),
                        "base_waypoints": base_np.tolist(),
                        "actions": [action.tolist() for action in actions],
                        "gripper_decisions": self.last_gripper_decisions,
                        "context_source": self.context_source,
                    },
                    f,
                    indent=2,
                )

            overlay_frame = self._chw_to_uint8(processed["model_frame"])
            overlay = self._draw_waypoint_overlay(overlay_frame, image_np)
            if bool(self.cfg.debug.get("save_waypoint_overlay", True)):
                PILImage.fromarray(overlay).save(
                    os.path.join(save_path, f"osvi_waypoints_overlay_t{t:03d}.png")
                )
        else:
            overlay = None

        if bool(self.cfg.debug.get("show_waypoint_overlay", False)):
            if overlay is None:
                overlay_frame = self._chw_to_uint8(processed["model_frame"])
                overlay = self._draw_waypoint_overlay(overlay_frame, image_np)
            self._show_waypoint_overlay(overlay)


        print(f"[OSVIController] Inference t={t}: selected {len(base_np)} waypoint(s)")
        return actions

    def _draw_waypoint_overlay(self, rgb_frame, image_waypoints):
        overlay = np.ascontiguousarray(rgb_frame.copy())
        height, width = overlay.shape[:2]
        close_threshold = float(self.cfg.control.get("gripper_close_threshold", 0.1))
        close_transition_offset = int(self.cfg.control.get("gripper_close_transition_offset", 0))
        waypoints = np.asarray(image_waypoints)
        close_flags = [
            float(waypoint[3]) >= close_threshold
            for waypoint in waypoints
        ]
        close_flags = self._offset_first_close_transition(close_flags, close_transition_offset)
        for idx, (waypoint, is_close) in enumerate(zip(waypoints, close_flags), start=1):
            x_norm, y_norm, z_value, grip_value = [float(v) for v in waypoint[:4]]
            col = int(round((x_norm + 1.0) * 0.5 * (width - 1)))
            row = int(round((1.0 - y_norm) * 0.5 * (height - 1)))
            col = int(np.clip(col, 0, width - 1))
            row = int(np.clip(row, 0, height - 1))

            color = (255, 40, 40) if is_close else (40, 220, 255)
            radius = 7 if is_close else 5

            cv2.circle(overlay, (col, row), radius, color, thickness=-1)
            cv2.circle(overlay, (col, row), radius + 2, (255, 255, 255), thickness=1)
            cv2.putText(
                overlay,
                f"{idx} g={grip_value:.2f} z={z_value:.2f}",
                (min(col + 9, width - 1), max(row - 9, 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
                cv2.LINE_AA,
            )

        return overlay

    def _show_waypoint_overlay(self, rgb_overlay):
        window_name = str(self.cfg.debug.get("waypoint_overlay_window", "OSVI waypoint overlay"))
        wait_ms = int(self.cfg.debug.get("waypoint_overlay_wait_ms", 1))
        bgr_overlay = cv2.cvtColor(rgb_overlay, cv2.COLOR_RGB2BGR)
        cv2.imshow(window_name, bgr_overlay)
        key = cv2.waitKey(max(wait_ms, 1)) & 0xFF
        if key == 27:
            raise KeyboardInterrupt("OSVI waypoint overlay interrupted by ESC")


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
