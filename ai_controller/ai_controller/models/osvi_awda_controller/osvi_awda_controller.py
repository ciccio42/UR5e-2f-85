"""OSVI-AWDA adapter for the existing UR5e AI-controller runtime.

The robot-facing behavior will be shared with ``OSVIController`` without
importing its OSVI-WM model: camera decoding, calibrated image-to-``base_link``
projection, pose creation, workspace checks, gripper commands, optional depth
refinement, and debug output.

The AWDA-specific model loading, context sampling, preprocessing, and forward
pass live in this adapter.  Keeping a distinct class prevents an AWDA
experiment from changing the known OSVI-WM controller.
"""

import importlib
import json
import os
import pickle
import sys
import types
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional
import copy

import cv2
import numpy as np
import torch
import yaml
from PIL import Image as PILImage

from ai_controller.utils.ai_controller import AIController
from ai_controller.utils.utils import EEF_POS_NAME, EEF_QUAT_NAME

IMAGENET_MEAN = np.asarray([0.485, 0.456, 0.406], dtype=np.float32).reshape(3, 1, 1)
IMAGENET_STD = np.asarray([0.229, 0.224, 0.225], dtype=np.float32).reshape(3, 1, 1)
TO_UNIT_COORDS = np.asarray(
    [
        [0.5, 0.0, 0.5, 0.0],
        [0.0, 0.5, 0.5, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)
FROM_UNIT_COORDS = np.linalg.inv(TO_UNIT_COORDS)
IDENTITY_QUAT_XYZW = np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
TOP_DOWN_QUAT_XYZW = np.asarray(
    [0.9994452044624775, 0.03161651380119412, 0.0021438049655468088, 0.010251021036213035],
    dtype=np.float64,
)


def _compute_crop_adjustment(crop_values, size_before):
    """Mirror utils.projection_utils.compute_crop_adjustment from OSVI-AWDA."""
    top, bottom, left, right = [int(value) for value in crop_values]
    rows_before, columns_before = [int(value) for value in size_before]
    rows_after = rows_before - top - bottom
    columns_after = columns_before - left - right
    if rows_before <= 0 or columns_before <= 0 or rows_after <= 0 or columns_after <= 0:
        raise ValueError(
            f"Invalid crop {crop_values} for source size "
            f"(height={rows_before}, width={columns_before})"
        )

    crop_adjustment = np.asarray(
        [
            [columns_after / columns_before, 0.0, left / columns_before, 0.0],
            [0.0, rows_after / rows_before, bottom / rows_before, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    return FROM_UNIT_COORDS @ np.linalg.inv(crop_adjustment) @ TO_UNIT_COORDS


class TrajectoryUnpickler(pickle.Unpickler):
    """Resolve trajectory classes used by the real and simulated datasets."""

    def find_class(self, module, name):
        if name == "Trajectory":
            for module_name in (
                "ai_controller.models.osvi_awda_controller.datasets",
                "multi_task_il.datasets.savers",
                "savers",
                "scripts.savers",
            ):
                try:
                    return importlib.import_module(module_name).Trajectory
                except Exception:
                    pass
        return super().find_class(module, name)


@dataclass
class OSVIAWDAConfig:
    """Configuration fields that are specific to an OSVI-AWDA checkpoint."""

    checkpoint_dir: str = ""
    checkpoint_step: Optional[int] = None
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


class OSVIAWDAController(AIController):
    """OSVI-AWDA controller with the same action contract as OSVI-WM.

    Model loading and preprocessing follow the repository's real-data offline
    evaluator.  Robot postprocessing and the forward pass are added separately
    so each integration stage can be tested before robot motion is enabled.
    """

    def __init__(self, model_config: str, task_name: str = "pick_place"):
        self.task_name = task_name
        self.cfg: Optional[OSVIAWDAConfig] = None
        self.config_dir: Optional[Path] = None
        self.checkpoint_dir: Optional[Path] = None
        self.checkpoint_path: Optional[Path] = None
        self.training_config_path: Optional[Path] = None
        self.training_config = None
        self.device = None
        self.projection_matrix_path: Optional[Path] = None
        self.projection_matrix = None
        self.projection_matrix_numpy = None
        self.context_tensor = None
        self.context_source = None
        self.last_image_waypoints = None
        self.last_base_waypoints = None
        self.current_eef_quat = None
        self.gripper_closed = False
        self.last_gripper_decisions = []
        self._pending_grasp_plan = None
        self._selected_grasp_waypoint_index = None
        self.last_execution_phase = None
        self.epoch = "unknown"
        self.global_step = "unknown"
        # Populated each step by pre_process() from input_data[2] (see
        # ai_controller_node.py, which gathers this via ROS on this
        # controller's behalf - mirrors current_eef_pos/quat for
        # CODController). _depth_camera_matrix/_depth_camera_info_size keep
        # their names so _scaled_depth_camera_matrix() needs no changes.
        self._current_depth_data = None
        self._current_gripper_rgb = None
        self._depth_camera_matrix = None
        self._depth_camera_info_size = None
        self._depth_warning_printed = False
        self._waypoint_overlay_window_initialized = False
        self._waypoint_overlay_display_failed = False
        self._last_waypoint_overlay_bgr = None
        self._last_gripper_centroid_overlay_bgr = None
        self._last_gripper_centroid_uv = None
        self._last_depth_centroid_overlay_bgr = None
        self._depth_debug_root = Path(
            "/home/ros2_ws/src/"
            "ai_controller/ai_controller/models/osvi_awda_controller/"
            "debug/test_controller"
        )

        self._depth_debug_capture_index = 0
        self._current_depth_debug_dir = None
        super().__init__(model_config)

    def _resolve_path(self, value: str) -> Path:
        path = Path(os.path.expanduser(str(value)))
        if not path.is_absolute():
            path = self.config_dir / path
        return path.resolve()

    @staticmethod
    def _require_file(path: Path, description: str) -> None:
        if not path.is_file():
            raise FileNotFoundError(f"{description} not found: {path}")

    def _load_training_config(self) -> dict:
        self._require_file(self.training_config_path, "AWDA training config")
        with self.training_config_path.open("r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}

        policy = config.get("policy")
        if not isinstance(policy, dict):
            raise ValueError(
                f"AWDA training config has no 'policy' mapping: {self.training_config_path}"
            )
        if not bool(config.get("image_waypoints", False)):
            raise ValueError(
                "This controller requires an AWDA checkpoint trained with image_waypoints=true."
            )
        return config

    def _validate_runtime_config(self) -> None:
        dataset = self.training_config.get("dataset", {})
        policy = self.training_config["policy"]

        # ---------------------------------------------------------
        # HARD CONSTRAINTS
        #
        # These must match the checkpoint/training configuration.
        # ---------------------------------------------------------

        checks = {
            "dataset.T_context": (
                int(dataset.get("T_context", -1)),
                int(self.cfg.context.get("num_frames", 10)),
            ),
            "dataset.height": (
                int(dataset.get("height", -1)),
                int(self.cfg.image.get("height", 100)),
            ),
            "dataset.width": (
                int(dataset.get("width", -1)),
                int(self.cfg.image.get("width", 180)),
            ),
            "policy.waypoints": (
                int(policy.get("waypoints", -1)),
                int(self.cfg.waypoints.get("base_count", 5)),
            ),
            "policy.sub_waypoints": (
                bool(policy.get("sub_waypoints", False)),
                bool(self.cfg.waypoints.get("sub_waypoints", False)),
            ),
        }

        mismatches = [
            f"{name}: checkpoint={actual!r}, runtime={expected!r}"
            for name, (actual, expected) in checks.items()
            if actual != expected
        ]

        if mismatches:
            raise ValueError(
                "OSVI-AWDA runtime config does not match its training config: "
                + "; ".join(mismatches)
            )

        # ---------------------------------------------------------
        # RUNTIME CROP
        #
        # The runtime crop is allowed to differ from the training crop.
        # This is intentional because the deployed camera framing may
        # require a different ROI.
        #
        # _preprocess_frame() uses the runtime crop.
        # _load_projection_matrix() also compensates using the runtime
        # crop, so image-coordinate projection remains geometrically
        # consistent with the actual runtime preprocessing.
        # ---------------------------------------------------------

        training_crop = list(
            dataset.get(
                "crop",
                [],
            )
        )

        runtime_crop = list(
            self.cfg.image.get(
                "crop",
                [],
            )
        )

        if training_crop != runtime_crop:
            print(
                "[OSVIAWDAController] WARNING: "
                "runtime image crop differs from training crop: "
                f"training={training_crop}, "
                f"runtime={runtime_crop}. "
                "Runtime preprocessing and projection will use the "
                "runtime crop."
            )

        # ---------------------------------------------------------
        # STATE CONFIGURATION
        # ---------------------------------------------------------

        if bool(policy.get("concat_state", True)):
            raise ValueError(
                "This adapter currently supports AWDA checkpoints "
                "with policy.concat_state=false."
            )

    def _import_model_class(self):
        package_name = f"{__package__}.models"
        models_package = importlib.import_module(package_name)
        module = importlib.import_module(f"{package_name}.inverse_module")
        module_file = Path(module.__file__).resolve()
        expected_models_dir = Path(__file__).resolve().parent / "models"
        if expected_models_dir not in module_file.parents:
            raise ImportError(
                "Imported the OSVI-AWDA inverse model from an unexpected location: "
                f"{module_file}; expected it below {expected_models_dir}"
            )

        # The original trainer stores the complete nn.Module with torch.save().
        # Its pickle therefore refers to the old repository names hem.models.*.
        # Point those names at the bundled copies before torch.load() runs.
        hem_package = sys.modules.get("hem")
        if hem_package is None:
            hem_package = types.ModuleType("hem")
            hem_package.__path__ = []
            sys.modules["hem"] = hem_package
        setattr(hem_package, "models", models_package)
        sys.modules["hem.models"] = models_package

        aliases = {
            "hem.models.inverse_module": module,
            "hem.models.basic_embedding": importlib.import_module(
                f"{package_name}.basic_embedding"
            ),
            "hem.models.traj_embed": importlib.import_module(
                f"{package_name}.traj_embed"
            ),
            "hem.models.discrete_logistic": importlib.import_module(
                f"{package_name}.discrete_logistic"
            ),
        }
        sys.modules.update(aliases)

        positional_module = importlib.import_module(
            f"{package_name}.positional_encoding"
        )
        archs_package = sys.modules.get("archs")
        if archs_package is None:
            archs_package = types.ModuleType("archs")
            archs_package.__path__ = []
            sys.modules["archs"] = archs_package
        setattr(archs_package, "PositionalEncoding", positional_module)
        sys.modules["archs.PositionalEncoding"] = positional_module
        return module.InverseImitation

    def _load_projection_matrix(self) -> None:
        self.projection_matrix_path = self._resolve_path(self.cfg.projection_matrix_path)
        self._require_file(self.projection_matrix_path, "AWDA real-camera projection config")
        with self.projection_matrix_path.open("r", encoding="utf-8") as stream:
            projection_config = yaml.safe_load(stream) or {}

        matrix = np.asarray(projection_config.get("projection_matrix"), dtype=np.float64)
        if matrix.shape != (4, 4):
            raise ValueError(
                f"projection_matrix must be 4x4, got {matrix.shape} in "
                f"{self.projection_matrix_path}"
            )

        camera = projection_config.get("camera", {})
        source_size = (camera.get("image_height"), camera.get("image_width"))
        if source_size[0] is None or source_size[1] is None:
            raise ValueError(
                "AWDA projection config must define camera.image_height and camera.image_width."
            )

        crop = self.cfg.image.get("crop", [0, 0, 0, 0])
        crop_adjustment = _compute_crop_adjustment(crop, source_size)
        # Exact no-augmentation branch of
        # camera_projection_real.build_sample_projection().
        matrix = matrix @ np.linalg.inv(crop_adjustment)
        self.projection_matrix_numpy = matrix.astype(np.float32)
        self.projection_matrix = torch.as_tensor(
            self.projection_matrix_numpy,
            dtype=torch.float32,
            device=self.device,
        )

        output_frame = str(self.cfg.projection.get("output_frame", "base_link"))
        if output_frame != "base_link":
            raise ValueError(
                f"OSVI-AWDA projection must output 'base_link', got {output_frame!r}."
            )

    def load_model(self, model_config: str):
        self.config_dir = Path(model_config).expanduser().resolve().parent
        with open(model_config, "r", encoding="utf-8") as stream:
            config_dict = yaml.safe_load(stream) or {}

        valid_fields = OSVIAWDAConfig.__dataclass_fields__.keys()
        self.cfg = OSVIAWDAConfig(
            **{key: value for key, value in config_dict.items() if key in valid_fields}
        )

        if not self.cfg.checkpoint_dir:
            raise ValueError("checkpoint_dir must be set in the OSVI-AWDA runtime config.")
        if self.cfg.checkpoint_step is None:
            raise ValueError("checkpoint_step must be set in the OSVI-AWDA runtime config.")

        requested_device = self.cfg.device
        if requested_device == "cuda" and not torch.cuda.is_available():
            print("[OSVIAWDAController] CUDA requested but unavailable; falling back to CPU.")
            requested_device = "cpu"
        self.device = torch.device(requested_device)

        self.checkpoint_dir = self._resolve_path(self.cfg.checkpoint_dir)
        self.training_config_path = self.checkpoint_dir / "config.yaml"
        self.checkpoint_path = (
            self.checkpoint_dir / f"model_save-{int(self.cfg.checkpoint_step)}.pt"
        )
        self._require_file(self.checkpoint_path, "AWDA checkpoint")

        self.training_config = self._load_training_config()
        self._validate_runtime_config()
        self._load_projection_matrix()
        model_class = self._import_model_class()
        policy_config = dict(self.training_config["policy"])
        policy_config["vis"] = dict(policy_config.get("vis", {}))
        # All visual weights are replaced immediately by the checkpoint. Avoid
        # torchvision attempting an unnecessary ImageNet download at startup.
        policy_config["vis"]["scratch"] = True
        model = model_class(**policy_config)

        try:
            loaded = torch.load(self.checkpoint_path, map_location="cpu", weights_only=False)
        except TypeError:
            loaded = torch.load(self.checkpoint_path, map_location="cpu")

        if hasattr(loaded, "state_dict"):
            state_dict = loaded.state_dict()
        elif isinstance(loaded, dict) and "model_state_dict" in loaded:
            state_dict = loaded["model_state_dict"]
        elif isinstance(loaded, dict) and "state_dict" in loaded:
            state_dict = loaded["state_dict"]
        else:
            state_dict = loaded

        model.load_state_dict(state_dict)
        model = model.to(self.device).eval()
        self.global_step = int(self.cfg.checkpoint_step)

        print(
            f"[OSVIAWDAController] Loaded checkpoint {self.checkpoint_path} "
            f"(global_step={self.global_step}, device={self.device})"
        )
        return model

    def move_model_to_device(self, device):
        self.device = torch.device(device)
        self.model = self.model.to(self.device)
        if self.projection_matrix is not None:
            self.projection_matrix = self.projection_matrix.to(self.device)
        if self.context_tensor is not None:
            self.context_tensor = self.context_tensor.to(self.device)

    def _reset_waypoint_visualization(self):
        """
        Reset all OpenCV debug state between trajectories.

        The window is explicitly destroyed so that the next trajectory
        recreates it and displays its new scene/waypoints.
        """
        debug_cfg = (
            self.cfg.debug
            if self.cfg is not None
            else {}
        )

        window_name = str(
            debug_cfg.get(
                "waypoint_overlay_window",
                "OSVI-AWDA waypoint overlay",
            )
        )

        if self._waypoint_overlay_window_initialized:
            try:
                cv2.destroyWindow(window_name)

                # Let OpenCV process the destroy event.
                cv2.waitKey(1)

            except cv2.error:
                pass

        self._waypoint_overlay_window_initialized = False
        self._waypoint_overlay_display_failed = False

        self._last_waypoint_overlay_bgr = None
        self._last_gripper_centroid_overlay_bgr = None
        self._last_depth_centroid_overlay_bgr = None
        self._last_gripper_centroid_uv = None

        self._depth_warning_printed = False

    def reset(self):
        """
        reset the controller's internal state between trajectories.
        This is called by the AIController before each new trajectory.
        """
        self.context_tensor = None
        self.context_source = None
        self.last_image_waypoints = None
        self.last_base_waypoints = None
        self.current_eef_quat = None
        self.gripper_closed = False
        self.last_gripper_decisions = []
        self._pending_grasp_plan = None
        self.last_execution_phase = None
        self._selected_grasp_waypoint_index = None
        self._reset_waypoint_visualization()

    def _gripper_depth_enabled(self):
        """
        Return true if the runtime config enables eye-in-hand depth refinement.
        """
        refine_cfg = self.cfg.grasp_refinement if self.cfg is not None else {}
        return bool(refine_cfg.get("enabled", False)) and bool(
            refine_cfg.get("use_gripper_depth", True)
        )

    def _depth_debug_grayscale(
        self,
        depth_m,
        min_depth=0.01,
        max_depth=0.10,
    ):
        """
        Convert metric depth to uint8 grayscale using a FIXED range:

            0.01 m -> 0
            0.10 m -> 255

        Invalid depth values are mapped to min_depth -> black.

        This is ONLY a visualization conversion.
        The metric depth array itself is not modified.
        """

        depth_m = np.asarray(
            copy.deepcopy(depth_m),
            dtype=np.float64,
        )

        depth_safe = copy.deepcopy(depth_m)

        invalid = (
            ~np.isfinite(depth_safe)
            | (depth_safe <= 0.0)
        )

        depth_safe[invalid] = min_depth

        depth_clipped = np.clip(
            depth_safe,
            min_depth,
            max_depth,
        )

        depth_normalized = (
            (depth_clipped - min_depth)
            / (max_depth - min_depth)
        )

        return (
            depth_normalized * 255.0
        ).astype(np.uint8)


    def _create_depth_debug_capture_dir(self):

        capture_dir = (
            self._depth_debug_root
            / f"capture_{self._depth_debug_capture_index:04d}"
        )

        self._depth_debug_capture_index += 1

        capture_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        self._current_depth_debug_dir = capture_dir

        print(
            "[DEPTH DEBUG] Saving refinement images to "
            f"{capture_dir}"
        )

        return capture_dir


    @staticmethod
    def _save_debug_image(
        directory,
        filename,
        image,
    ):
        if directory is None or image is None:
            return

        path = Path(directory) / filename

        ok = cv2.imwrite(
            str(path),
            image,
        )

        if not ok:
            print(
                f"[DEPTH DEBUG] Failed to save {path}"
            )

    def should_refine_grasp_action(self, action_index):
        """Return true for the coarse waypoint that starts AWDA's grasp primitive."""
        if not bool(self.cfg.grasp_refinement.get("enabled", False)):
            return False
        return self.get_action_primitive(action_index) == "grasp"

    def get_action_primitive(self, action_index):
        if action_index < 0 or action_index >= len(self.last_gripper_decisions):
            return None
        return self.last_gripper_decisions[action_index].get("primitive")

    def build_free_space_actions(self, coarse_action, current_xyz):
        """Build AWDA free_space_primitive's approach-then-target motion."""
        target = self._validated_robot_action(coarse_action).copy()
        current_xyz = np.asarray(current_xyz, dtype=np.float64).reshape(-1)
        if current_xyz.shape != (3,) or not np.isfinite(current_xyz).all():
            raise ValueError(f"Current EEF position must be a finite 3-vector, got {current_xyz}.")

        approach = target.copy()
        approach[:3] = target[:3]
        approach[2] = max(float(target[2]), float(current_xyz[2]))
        approach[:3] = self._apply_workspace_safety(approach[:3])
        if np.allclose(approach[:3], target[:3], atol=1e-9, rtol=0.0):
            return [target]
        return [approach, target]

    def build_drop_actions(self, coarse_action):
        """Reach the predicted drop target while holding, then open there."""
        release = self._validated_robot_action(coarse_action).copy()
        release[2]+=0.02
        release[:3] = self._apply_workspace_safety(release[:3])
        release[-1] = float(self.cfg.control.get("gripper_open_position", 0.0))
        carry_to_target = release.copy()
        carry_to_target[-1] = float(
            self.cfg.control.get("gripper_closed_position", 255.0)
        )
        return [carry_to_target, release]

    def build_grasp_hover_action(self, coarse_action):
        """Build AWDA grasp_primitive's first move (hint + 10 cm, gripper open)."""
        action = self._validated_robot_action(coarse_action).copy()
        hover_height = float(self.cfg.grasp_refinement.get("hover_height_m", 0.05))
        action[1]-=0.04
        action[:3] = self._apply_workspace_safety(
            action[:3] + np.asarray([0.0, 0.0, hover_height], dtype=np.float64)
        )
        action[-1] = float(self.cfg.control.get("gripper_open_position", 0.0))
        return action

    def build_post_hover_grasp_actions(self, coarse_action):
        """Localize after hover, then return approach/descend/close/lift actions.

        This is the real-robot equivalent of OSVI-AWDA's ``grasp_primitive``.
        The original simulator function reads MuJoCo intrinsics/extrinsics and
        hard-codes its tabletop z.  Here the same depth segmentation is followed
        by CameraInfo deprojection and a TF transform to ``base_link``.
        """
        coarse_action = self._validated_robot_action(coarse_action)
        quaternion = coarse_action[3:7]
        open_position = float(self.cfg.control.get("gripper_open_position", 0.0))
        closed_position = float(self.cfg.control.get("gripper_closed_position", 255.0))
        refine_cfg = self.cfg.grasp_refinement

        target = self._depth_refined_grasp_target(coarse_action[:3])
        approach = target.copy()
        approach[2] += float(refine_cfg.get("approach_height_m", 0.05))
        approach = self._apply_workspace_safety(approach)

        descend = self._apply_workspace_safety(target)
        lift = descend.copy()
        lift[2] += float(refine_cfg.get("lift_height_m", 0.15))
        lift = self._apply_workspace_safety(lift)

        return [
            self._make_robot_action(approach, quaternion, open_position),
            self._make_robot_action(descend, quaternion, open_position),
            self._make_robot_action(descend, quaternion, closed_position),
            self._make_robot_action(lift, quaternion, closed_position),
        ]

    def _execution_actions_from_coarse(self, coarse_actions, robot_state):
        """Expand a new model plan until completion or the first grasp hover."""
        coarse_actions = [
            self._validated_robot_action(action).copy() for action in coarse_actions
        ]
        decisions = [dict(decision) for decision in self.last_gripper_decisions]
        if len(coarse_actions) != len(decisions):
            raise RuntimeError(
                "AWDA coarse actions and primitive decisions are not aligned: "
                f"{len(coarse_actions)} action(s), {len(decisions)} decision(s)."
            )
        if not coarse_actions:
            raise RuntimeError("AWDA produced an empty coarse action plan.")

        parsed_state = self._parse_robot_state(robot_state)
        current_xyz = parsed_state.get("eef_pos")
        if current_xyz is None:
            current_xyz = coarse_actions[0][:3]

        self._pending_grasp_plan = None
        actions = self._expand_awda_primitives(
            coarse_actions,
            decisions,
            current_xyz=np.asarray(current_xyz, dtype=np.float64),
        )
        self.last_execution_phase = (
            "plan_to_grasp_hover"
            if self._pending_grasp_plan is not None
            else "complete_plan"
        )
        return actions

    def _expand_awda_primitives(
        self,
        coarse_actions,
        decisions,
        current_xyz,
        prefix_actions=None,
    ):
        """Translate AWDA primitive labels into the node's standard action list."""
        if len(coarse_actions) != len(decisions):
            raise RuntimeError(
                "Cannot expand an AWDA plan with mismatched actions and decisions."
            )

        execution_actions = [
            self._validated_robot_action(action).copy()
            for action in (prefix_actions or [])
        ]
        current_xyz = np.asarray(current_xyz, dtype=np.float64)

        for index, (coarse_action, decision) in enumerate(
            zip(coarse_actions, decisions)
        ):
            coarse_action = self._validated_robot_action(coarse_action).copy()
            primitive = decision.get("primitive")

            if primitive in ("free_space", "carry"):
                primitive_actions = self.build_free_space_actions(
                    coarse_action,
                    current_xyz,
                )
            elif primitive == "drop":
                primitive_actions = self.build_drop_actions(coarse_action)
            elif primitive == "grasp":
                hover_action = self.build_grasp_hover_action(coarse_action)
                execution_actions.append(hover_action)
                current_xyz = hover_action[:3]

                if bool(self.cfg.grasp_refinement.get("enabled", False)):
                    # Stop the returned plan here. The unchanged ROS node executes
                    # the hover normally; its next control-loop iteration supplies
                    # the observation from the new eye-in-hand pose.
                    self._pending_grasp_plan = {
                        "grasp_action": coarse_action.copy(),
                        "remaining_actions": [
                            self._validated_robot_action(action).copy()
                            for action in coarse_actions[index + 1 :]
                        ],
                        "remaining_decisions": [
                            dict(item) for item in decisions[index + 1 :]
                        ],
                    }
                    return execution_actions

                primitive_actions = self.build_post_hover_grasp_actions(
                    coarse_action
                )
            else:
                primitive_actions = [coarse_action]

            for primitive_action in primitive_actions:
                primitive_action = self._validated_robot_action(primitive_action).copy()
                execution_actions.append(primitive_action)
                current_xyz = primitive_action[:3]

        self._pending_grasp_plan = None
        return execution_actions

    def _resume_pending_grasp(self):
        """Read post-hover depth and finish the deferred AWDA primitive plan."""
        pending = self._pending_grasp_plan
        if pending is None:
            raise RuntimeError("No AWDA grasp plan is pending.")

        # Clear the current pending state before expansion. If the remaining plan
        # contains another grasp, _expand_awda_primitives will install a new one.
        self._pending_grasp_plan = None
        post_hover_actions = self.build_post_hover_grasp_actions(
            pending["grasp_action"]
        )
        current_xyz = post_hover_actions[-1][:3]
        actions = self._expand_awda_primitives(
            pending["remaining_actions"],
            pending["remaining_decisions"],
            current_xyz=current_xyz,
            prefix_actions=post_hover_actions,
        )
        self.last_execution_phase = (
            "grasp_resume_to_next_hover"
            if self._pending_grasp_plan is not None
            else "grasp_resume_complete"
        )
        return actions

    @staticmethod
    def _validated_robot_action(action):
        action = np.asarray(action, dtype=np.float64).reshape(-1)
        if action.shape != (8,) or not np.isfinite(action).all():
            raise ValueError(
                "AWDA robot action must be finite [x,y,z,qx,qy,qz,qw,gripper], "
                f"got shape {action.shape}."
            )
        return action

    @staticmethod
    def _make_robot_action(xyz, quaternion, gripper):
        return np.concatenate(
            [
                np.asarray(xyz, dtype=np.float64),
                np.asarray(quaternion, dtype=np.float64),
                np.asarray([float(gripper)], dtype=np.float64),
            ]
        )

    def _depth_refined_grasp_target(self, predicted_xyz):
        predicted_xyz = np.asarray(predicted_xyz, dtype=np.float64)

        print("\n" + "=" * 80)
        print("[DEPTH REFINEMENT DEBUG] START")
        print(f"predicted_xyz RAW             = {predicted_xyz.tolist()}")

        predicted_after_safety = self._apply_workspace_safety(predicted_xyz)

        print(f"predicted_xyz after safety    = {predicted_after_safety.tolist()}")
        print(
            f"workspace_min                 = "
            f"{self.cfg.safety.get('workspace_min')}"
        )
        print(
            f"workspace_max                 = "
            f"{self.cfg.safety.get('workspace_max')}"
        )

        predicted_xyz = predicted_after_safety

        if not self._gripper_depth_enabled():
            print("[DEPTH REFINEMENT DEBUG] Depth disabled -> using AWDA prediction")
            print("=" * 80 + "\n")
            return predicted_xyz

        # ------------------------------------------------------------
        # 1. TARGET GREZZO PRODOTTO DALLA DEPTH + TF
        # ------------------------------------------------------------
        depth_target = self._estimate_gripper_depth_target_base()

        if depth_target is None:
            print(
                "[DEPTH REFINEMENT DEBUG] depth_target=None -> "
                "using predicted AWDA target"
            )
            print("=" * 80 + "\n")
            return predicted_xyz

        depth_target = np.asarray(depth_target, dtype=np.float64)

        print(f"depth_target RAW base_link    = {depth_target.tolist()}")

        # ------------------------------------------------------------
        # 2. OFFSET Z
        # ------------------------------------------------------------
        refine_cfg = self.cfg.grasp_refinement

        z_offset = float(
            refine_cfg.get("depth_grasp_z_offset_m", 0.0)
        )

        refined_xyz = depth_target.copy()

        print(f"depth_grasp_z_offset_m        = {z_offset:+.6f}")

        refined_xyz[2] += z_offset

        print(f"after Z offset                = {refined_xyz.tolist()}")

        # ------------------------------------------------------------
        # 3. DIFFERENZA RISPETTO AD AWDA
        # ------------------------------------------------------------
        delta_xyz = refined_xyz - predicted_xyz
        xy_delta = float(
            np.linalg.norm(refined_xyz[:2] - predicted_xyz[:2])
        )

        print(f"delta depth-AWDA XYZ          = {delta_xyz.tolist()}")
        print(f"XY correction norm            = {xy_delta:.6f} m")

        max_xy_correction = refine_cfg.get(
            "max_depth_xy_correction_m"
        )

        print(
            f"max_depth_xy_correction_m     = "
            f"{max_xy_correction}"
        )

        if max_xy_correction is not None:
            if xy_delta > float(max_xy_correction):
                print(
                    "[DEPTH REFINEMENT DEBUG] REJECTED: "
                    f"{xy_delta:.6f} > "
                    f"{float(max_xy_correction):.6f}"
                )
                print("=" * 80 + "\n")
                return predicted_xyz
            else:
                print(
                    "[DEPTH REFINEMENT DEBUG] XY correction accepted"
                )

        # ------------------------------------------------------------
        # 4. CLAMP WORKSPACE
        # ------------------------------------------------------------
        before_clip = refined_xyz.copy()

        refined_xyz = self._apply_workspace_safety(refined_xyz)

        print(f"before workspace clamp        = {before_clip.tolist()}")
        print(f"after workspace clamp         = {refined_xyz.tolist()}")

        clip_delta = refined_xyz - before_clip
        print(f"workspace clamp correction    = {clip_delta.tolist()}")

        if not np.allclose(before_clip, refined_xyz):
            print(
                "[DEPTH REFINEMENT DEBUG] *** WORKSPACE CLAMP ACTIVE ***"
            )
        else:
            print(
                "[DEPTH REFINEMENT DEBUG] workspace clamp did not modify target"
            )

        print(
            "[DEPTH REFINEMENT DEBUG] FINAL TARGET = "
            f"{refined_xyz.tolist()}"
        )
        print("=" * 80 + "\n")

        return refined_xyz

    def _estimate_gripper_depth_target_base(self):

        depth_data = self._current_depth_data
        if not depth_data or depth_data.get("transform") is None:
            self._print_depth_warning_once(
                "Eye-in-hand localization skipped: depth/transform not available "
                "(ai_controller_node.py couldn't get CameraInfo or the depth_camera -> "
                f"{self.cfg.projection.get('output_frame', 'base_link')} transform)."
            )
            return None

        depth_m = depth_data.get("image")
        if depth_m is None:
            self._print_depth_warning_once(
                "Eye-in-hand localization skipped: no gripper depth frame received yet."
            )
            return None

        self._depth_camera_matrix = depth_data.get("camera_matrix")
        self._depth_camera_info_size = depth_data.get("camera_info_size")
        if self._depth_camera_matrix is None:
            self._print_depth_warning_once(
                "Eye-in-hand localization skipped: gripper CameraInfo not available."
            )
            return None

        # ---------------------------------------------------------
        # CREATE DEBUG CAPTURE
        # ---------------------------------------------------------

        debug_dir = self._create_depth_debug_capture_dir()

        # ---------------------------------------------------------
        # SAVE RAW DEPTH AS FIXED 0.1-1.0 m GRAYSCALE
        # ---------------------------------------------------------

        depth_raw_gray = self._depth_debug_grayscale(
            copy.deepcopy(depth_m),
            min_depth=0.01,
            max_depth=1.00,
        )

        self._save_debug_image(
            debug_dir,
            "00_depth_raw_gray.png",
            depth_raw_gray,
        )

        # ---------------------------------------------------------
        # READ + SAVE RAW GRIPPER RGB
        # ---------------------------------------------------------

        gripper_rgb = self._current_gripper_rgb

        if gripper_rgb is not None:

            self._save_debug_image(
                debug_dir,
                "01_gripper_rgb_raw.png",
                gripper_rgb,
            )

        # ---------------------------------------------------------
        # DEPTH PROCESSING
        # ---------------------------------------------------------

        centroid = self._find_depth_object_centroid(
            copy.deepcopy(depth_m),
            debug_dir=debug_dir,
        )

        if centroid is None:

            self._print_depth_warning_once(
                "Eye-in-hand localization skipped: "
                "no foreground object found in depth image."
            )

            return None

        u, v = centroid

        # ---------------------------------------------------------
        # DEPTH DEBUG AROUND CENTROID
        # ---------------------------------------------------------

        r = 3

        patch = depth_m[
            max(0, v-r):min(depth_m.shape[0], v+r+1),
            max(0, u-r):min(depth_m.shape[1], u+r+1),
        ]

        print(
            "[DEPTH DEBUG] "
            f"centroid=(u={u}, v={v})\n"
            f"center_depth={depth_m[v,u]}\n"
            f"patch=\n{patch}"
        )

        # ---------------------------------------------------------
        # DEPTH USED FOR DEPROJECTION
        # ---------------------------------------------------------

        depth = self._depth_at_centroid(
            copy.deepcopy(depth_m),
            u,
            v,
        )

        if depth is None:

            self._print_depth_warning_once(
                "Eye-in-hand localization skipped: "
                "no valid depth at object centroid."
            )

            return None

        print(
            "[DEPTH GEOMETRY DEBUG] "
            f"selected_depth={depth:.6f} m"
        )

        # ---------------------------------------------------------
        # RGB CENTROID OVERLAY
        # ---------------------------------------------------------

        self._last_gripper_centroid_overlay_bgr = (
            self._build_gripper_centroid_overlay(
                centroid=(u, v),
                depth_shape=depth_m.shape[:2],
                rgb=gripper_rgb,
            )
        )

        if self._last_gripper_centroid_overlay_bgr is not None:

            self._save_debug_image(
                debug_dir,
                "09_gripper_rgb_centroid.png",
                self._last_gripper_centroid_overlay_bgr,
            )

        # ---------------------------------------------------------
        # DEPROJECTION
        # ---------------------------------------------------------

        camera_matrix = self._scaled_depth_camera_matrix(
            copy.deepcopy(depth_m).shape[:2]
        )

        point_camera = self._deproject_pixel(
            u,
            v,
            depth,
            camera_matrix,
        )

        return self._transform_depth_point_to_target_frame(point_camera)

    def _find_depth_object_centroid(self, depth_m, debug_dir=None):
        """Find the centroid of the closest object contour in the depth image.

        Pipeline:
            metric depth
            -> horizontal ROI crop
            -> fixed-range grayscale normalization
            -> Gaussian blur
            -> Canny edge detection
            -> gripper/border exclusion mask
            -> morphological closing
            -> contour extraction
            -> minimum-area filtering
            -> minimum valid depth inside each contour
            -> select contour closest to camera
            -> contour centroid
            -> conversion back to full-image pixel coordinates

        The returned pixel coordinates (u, v) refer to the ORIGINAL
        uncropped depth image, so they can be used directly for
        camera deprojection.
        """

        refine_cfg = self.cfg.grasp_refinement

        # ---------------------------------------------------------
        # EDGE PARAMETERS
        # ---------------------------------------------------------

        min_depth = float(
            refine_cfg.get(
                "depth_edge_min_depth_m",
                0.01,
            )
        )

        max_depth = float(
            refine_cfg.get(
                "depth_edge_max_depth_m",
                0.5,
            )
        )

        canny_low = int(
            refine_cfg.get(
                "depth_edge_canny_low",
                5,
            )
        )

        canny_high = int(
            refine_cfg.get(
                "depth_edge_canny_high",
                15,
            )
        )

        blur_kernel = int(
            refine_cfg.get(
                "depth_edge_blur_kernel_px",
                3,
            )
        )

        close_kernel_size = int(
            refine_cfg.get(
                "depth_edge_close_kernel_px",
                11,
            )
        )

        close_iterations = int(
            refine_cfg.get(
                "depth_edge_close_iterations",
                1,
            )
        )

        min_contour_area = float(
            refine_cfg.get(
                "depth_edge_min_contour_area_px",
                100,
            )
        )

        # ---------------------------------------------------------
        # DEPTH CROP PARAMETERS
        # ---------------------------------------------------------

        crop_left = int(
            refine_cfg.get(
                "depth_crop_left_px",
                281,
            )
        )

        crop_right = int(
            refine_cfg.get(
                "depth_crop_right_px",
                431,
            )
        )

        # Gaussian and morphology kernels must be positive and odd.
        blur_kernel = max(
            1,
            blur_kernel,
        )

        if blur_kernel % 2 == 0:
            blur_kernel += 1

        close_kernel_size = max(
            1,
            close_kernel_size,
        )

        if close_kernel_size % 2 == 0:
            close_kernel_size += 1

        # ---------------------------------------------------------
        # DEPTH IMAGE
        # ---------------------------------------------------------

        depth_full_m = np.asarray(
            copy.deepcopy(depth_m),
            dtype=np.float64,
        )

        full_height, full_width = depth_full_m.shape[:2]

        # Clamp crop limits to the valid image domain.
        crop_left = int(
            np.clip(
                crop_left,
                0,
                full_width - 1,
            )
        )

        crop_right = int(
            np.clip(
                crop_right,
                crop_left,
                full_width - 1,
            )
        )

        # crop_right is considered INCLUSIVE.
        depth_m = depth_full_m[
            :,
            crop_left:crop_right + 1,
        ]

        height, width = depth_m.shape[:2]

        # ---------------------------------------------------------
        # GRIPPER / BORDER EXCLUSION MASK
        # ---------------------------------------------------------

        margins = refine_cfg.get(
            "depth_ignore_margins_px",
            {},
        )

        top = max(
            0,
            int(margins.get("top", 0)),
        )

        bottom = max(
            0,
            int(margins.get("bottom", 0)),
        )

        left = max(
            0,
            int(margins.get("left", 0)),
        )

        right = max(
            0,
            int(margins.get("right", 0)),
        )

        keep_mask = np.ones(
            (height, width),
            dtype=np.uint8,
        )

        if top > 0:
            keep_mask[
                :min(top, height),
                :
            ] = 0

        if bottom > 0:
            keep_mask[
                max(0, height - bottom):,
                :
            ] = 0

        if left > 0:
            keep_mask[
                :,
                :min(left, width)
            ] = 0

        if right > 0:
            keep_mask[
                :,
                max(0, width - right):
            ] = 0

        # ---------------------------------------------------------
        # METRIC DEPTH -> GRAYSCALE [0,255]
        # ---------------------------------------------------------

        depth_safe = depth_m.copy()

        invalid = (
            ~np.isfinite(depth_safe)
            | (depth_safe <= 0.0)
        )

        depth_safe[invalid] = min_depth

        depth_clipped = np.clip(
            depth_safe,
            min_depth,
            max_depth,
        )

        depth_normalized = (
            (depth_clipped - min_depth)
            / (max_depth - min_depth)
        )

        depth_gray = (
            depth_normalized * 255.0
        ).astype(np.uint8)

        self._save_debug_image(
            debug_dir,
            "02_depth_crop_gray.png",
            depth_gray,
        )

        # ---------------------------------------------------------
        # GAUSSIAN BLUR
        # ---------------------------------------------------------

        depth_blurred = cv2.GaussianBlur(
            depth_gray,
            (
                blur_kernel,
                blur_kernel,
            ),
            0,
        )

        # ---------------------------------------------------------
        # CANNY EDGE DETECTION
        # ---------------------------------------------------------

        edges_raw = cv2.Canny(
            depth_blurred,
            canny_low,
            canny_high,
        )

        self._save_debug_image(
            debug_dir,
            "04_canny_raw.png",
            edges_raw,
        )

        edges = (
            (edges_raw > 0)
            & (keep_mask > 0)
        ).astype(np.uint8) * 255

        self._save_debug_image(
            debug_dir,
            "05_canny_masked.png",
            edges,
        )

        if not np.any(edges):
            return None

        # ---------------------------------------------------------
        # CLOSE SMALL GAPS IN EDGE CONTOURS
        # ---------------------------------------------------------

        close_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (
                close_kernel_size,
                close_kernel_size,
            ),
        )

        edges_closed = cv2.morphologyEx(
            edges,
            cv2.MORPH_CLOSE,
            close_kernel,
            iterations=close_iterations,
        )

        edges_closed = (
            (edges_closed > 0)
            & (keep_mask > 0)
        ).astype(np.uint8) * 255

        self._save_debug_image(
            debug_dir,
            "06_edges_closed.png",
            edges_closed,
        )

        if not np.any(edges_closed):
            return None

        # ---------------------------------------------------------
        # FIND CONTOURS
        # ---------------------------------------------------------

        contours, _ = cv2.findContours(
            edges_closed,
            cv2.RETR_LIST,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        all_contours_vis = cv2.cvtColor(
            depth_gray,
            cv2.COLOR_GRAY2BGR,
        )

        if not contours:
            return None

        best_centroid = None
        best_depth = None
        best_contour = None
        best_rect = None

        # ---------------------------------------------------------
        # FILTER CONTOURS + SELECT CLOSEST ONE
        # ---------------------------------------------------------

        for contour in contours:

            area = float(
                cv2.contourArea(
                    contour
                )
            )

            if area < min_contour_area:
                continue

            cv2.drawContours(
                all_contours_vis,
                [contour],
                -1,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )

            # -----------------------------------------------------
            # MINIMUM-AREA ROTATED RECTANGLE
            #
            # Instead of using the area centroid of the irregular
            # depth contour, approximate the object's top surface
            # with the minimum-area oriented rectangle.
            #
            # The rectangle center is used as the grasp pixel.
            # -----------------------------------------------------

            rect = cv2.minAreaRect(
                contour
            )

            (u, v), (rect_width, rect_height), rect_angle = rect

            # Reject degenerate rectangles.
            if (
                rect_width <= 1e-6
                or rect_height <= 1e-6
            ):
                continue

            centroid = np.asarray(
                [
                    float(u),
                    float(v),
                ],
                dtype=np.float64,
            )

            # Diagnostic only:
            # since the real object is a cube, the top surface is
            # expected to be approximately square, although perspective
            # projection can make it rectangular in the image.
            aspect_ratio = (
                max(rect_width, rect_height)
                / min(rect_width, rect_height)
            )

            print(
                "[DEPTH RECT DEBUG] "
                f"center=({u:.2f}, {v:.2f}), "
                f"size=({rect_width:.2f}, {rect_height:.2f}), "
                f"aspect_ratio={aspect_ratio:.3f}, "
                f"angle={rect_angle:.2f}"
            )

            # -----------------------------------------------------
            # DEPTH VALUES INSIDE THE CONTOUR
            # -----------------------------------------------------

            contour_mask = np.zeros(
                (height, width),
                dtype=np.uint8,
            )

            cv2.drawContours(
                contour_mask,
                [contour],
                contourIdx=-1,
                color=255,
                thickness=cv2.FILLED,
            )

            valid_pixels = (
                (contour_mask > 0)
                & (keep_mask > 0)
                & np.isfinite(depth_m)
                & (depth_m > 0.0)
                & (depth_m >= min_depth)
                & (depth_m <= max_depth)
            )

            contour_depth_values = depth_m[
                valid_pixels
            ]

            if contour_depth_values.size == 0:
                continue

            # Smallest metric depth = point closest to camera.
            contour_min_depth = float(
                np.min(
                    contour_depth_values
                )
            )

            # -----------------------------------------------------
            # SELECT CONTOUR CLOSEST TO CAMERA
            # -----------------------------------------------------

            if (
                best_depth is None
                or contour_min_depth < best_depth
            ):
                best_depth = contour_min_depth
                best_centroid = centroid
                best_contour = contour.copy()
                best_rect = rect

        self._save_debug_image(
            debug_dir,
            "07_all_contours.png",
            all_contours_vis,
        )

    
        if best_centroid is None:
            return None

        # ---------------------------------------------------------
        # CROPPED CENTROID -> ORIGINAL IMAGE PIXEL COORDINATES
        # ---------------------------------------------------------

        u_crop = int(
            np.clip(
                round(best_centroid[0]),
                0,
                width - 1,
            )
        )

        v = int(
            np.clip(
                round(best_centroid[1]),
                0,
                height - 1,
            )
        )

        # Reintroduce horizontal crop offset.
        u = crop_left + u_crop

        u = int(
            np.clip(
                u,
                0,
                full_width - 1,
            )
        )

        self._last_depth_centroid_overlay_bgr = (
            self._build_depth_centroid_overlay(
                depth_full_m=depth_full_m,
                crop_left=crop_left,
                crop_right=crop_right,
                depth_gray_crop=depth_gray,
                edges_closed=edges_closed,
                best_contour=best_contour,
                best_centroid=best_centroid,
                best_depth=best_depth,
                best_rect=best_rect,
            )
        )

        self._save_debug_image(
            debug_dir,
            "08_selected_contour.png",
            self._last_depth_centroid_overlay_bgr,
        )

        return u, v

    def _depth_at_centroid(self, depth_m, u, v):
        window = max(1, int(self.cfg.grasp_refinement.get("depth_window_px", 1)))
        height, width = depth_m.shape[:2]
        half = window // 2
        u0, u1 = max(0, u - half), min(width, u + half + 1)
        v0, v1 = max(0, v - half), min(height, v + half + 1)
        values = depth_m[v0:v1, u0:u1].reshape(-1)
        max_depth = float(self.cfg.grasp_refinement.get("depth_max_range_m", 1.0))
        values = values[np.isfinite(values) & (values > 0.0) & (values < max_depth)]
        if values.size == 0:
            return None
        return float(np.median(values))

    def _scaled_depth_camera_matrix(self, depth_shape):
        camera_matrix = np.asarray(self._depth_camera_matrix, dtype=np.float64).copy()
        if self._depth_camera_info_size is None:
            return camera_matrix

        info_width, info_height = self._depth_camera_info_size
        depth_height, depth_width = int(depth_shape[0]), int(depth_shape[1])
        if (
            info_width > 0
            and info_height > 0
            and (info_width != depth_width or info_height != depth_height)
        ):
            scale_x = depth_width / float(info_width)
            scale_y = depth_height / float(info_height)
            camera_matrix[0, 0] *= scale_x
            camera_matrix[0, 2] *= scale_x
            camera_matrix[1, 1] *= scale_y
            camera_matrix[1, 2] *= scale_y
        return camera_matrix

    @staticmethod
    def _deproject_pixel(u, v, depth, camera_matrix):
        focal_x, focal_y = camera_matrix[0, 0], camera_matrix[1, 1]
        center_x, center_y = camera_matrix[0, 2], camera_matrix[1, 2]
        x = (float(u) - center_x) * depth / focal_x
        y = (float(v) - center_y) * depth / focal_y
        return np.asarray([x, y, depth], dtype=np.float64)

    def _transform_depth_point_to_target_frame(self, point_camera):
        """Applies the depth_camera -> target_frame transform ai_controller_node.py
        already looked up this step (self._current_depth_data['transform']), instead
        of doing a TF lookup here."""
        transform = self._current_depth_data.get("transform") if self._current_depth_data else None
        if transform is None:
            return None
        rotation = transform["rotation"]
        translation = transform["translation"]
        return rotation @ np.asarray(point_camera, dtype=np.float64) + translation

    def _print_depth_warning_once(self, message):
        if not self._depth_warning_printed:
            print(f"[OSVIAWDAController] {message}")
            self._depth_warning_printed = True

    def _update_runtime_sensor_inputs(self, input_data):
        """Cache runtime sensor inputs supplied by the caller.

        The controller intentionally has no ROS dependency: depth, camera
        intrinsics and the camera->target-frame transform are gathered by the
        runtime (AIControllerNode on the real robot) and passed here as plain
        Python/NumPy data.

        This helper is also called when resuming a pending grasp so that the
        depth/RGB used for refinement are the *new* post-hover observations.
        """
        if input_data is None or len(input_data) < 1:
            raise ValueError("OSVI-AWDA inference requires at least an image list.")

        images = input_data[0]

        # Optional payload: callers such as offline tests may provide only
        # (images, robot_state).
        self._current_depth_data = input_data[2] if len(input_data) > 2 else None

        gripper_index = int(self.cfg.grasp_refinement.get("gripper_camera_index", 3))
        self._current_gripper_rgb = (
            cv2.cvtColor(images[gripper_index], cv2.COLOR_RGB2BGR)
            if 0 <= gripper_index < len(images)
            else None
        )

    def pre_process(self, input_data):
        if self.context_tensor is None:
            raise RuntimeError(
                "OSVI-AWDA context is missing. Call load_command() before inference()."
            )

        images, robot_state = input_data[0], input_data[1]
        self._update_runtime_sensor_inputs(input_data)

        front_index = int(self.cfg.image.get("front_camera_index", 0))
        if front_index >= len(images):
            raise IndexError(
                f"front_camera_index={front_index} but only {len(images)} image(s) were provided"
            )

        live_frame = self._prepare_live_frame(images[front_index])
        model_frame = self._preprocess_frame(live_frame, source="live")

        observation_config = self.cfg.agent_observation
        frame_count = int(observation_config.get("num_frames", 2))
        if not bool(observation_config.get("repeat_current_image", True)):
            raise NotImplementedError(
                "OSVI-AWDA currently supports repeat_current_image=true only."
            )

        agent_array = np.stack([model_frame for _ in range(frame_count)], axis=0)
        agent_tensor = torch.from_numpy(agent_array).unsqueeze(0).float().to(self.device)

        # This checkpoint has concat_state=false.  The offline AWDA evaluator still
        # supplies a [B,T,1] zero tensor, so the live adapter mirrors it exactly.
        state_tensor = torch.zeros((1, frame_count, 1), device=self.device)
        entities = torch.zeros((1,), dtype=torch.long, device=self.device)
        return {
            "images": agent_tensor,
            "context": self.context_tensor,
            "states": state_tensor,
            "entities": entities,
            "robot_state": robot_state,
            "live_frame": live_frame,
            "model_frame": model_frame,
        }

    def post_process(self, output_data):
        """Convert projected waypoints and classify AWDA primitive transitions.

        This stage creates the coarse UR5e actions. Primitive expansion remains
        inside this controller and is applied by _execution_actions_from_coarse.
        """
        base_waypoints = np.asarray(output_data["base_waypoints"], dtype=np.float64)
        image_waypoints = np.asarray(output_data["image_waypoints"], dtype=np.float64)
        if base_waypoints.shape != image_waypoints.shape or base_waypoints.ndim != 2:
            raise ValueError(
                "image_waypoints and base_waypoints must have the same [W,4] shape; "
                f"got {image_waypoints.shape} and {base_waypoints.shape}."
            )
        if base_waypoints.shape[1] != 4:
            raise ValueError(f"Expected waypoint shape [W,4], got {base_waypoints.shape}.")

        robot_state = self._parse_robot_state(output_data.get("robot_state"))
        quaternion = self._orientation_for_action(robot_state)
        control = self.cfg.control
        safety = self.cfg.safety
        threshold = float(control.get("gripper_close_threshold", 0.1))
        open_position = float(control.get("gripper_open_position", 0.0))
        closed_position = float(control.get("gripper_closed_position", 255.0))
        action_limit = min(
            int(control.get("max_actions_per_inference", len(base_waypoints))),
            len(base_waypoints),
        )
        min_distance = float(safety.get("min_waypoint_distance", 0.0))

        base_waypoints = base_waypoints[:action_limit]
        image_waypoints = image_waypoints[:action_limit]
        raw_attributes = image_waypoints[:, 3]
        threshold_flags = raw_attributes > threshold

        if bool(control.get("fixed_gripper_schedule", False)):

            # ---------------------------------------------------------
            # Dynamically select the grasp waypoint.
            #
            # The LAST waypoint is always reserved for DROP and is
            # therefore excluded from grasp selection.
            #
            # Among all previous waypoints, choose the one with the
            # minimum projected Z coordinate in base_link.
            # ---------------------------------------------------------

            if len(base_waypoints) < 2:
                raise ValueError(
                    "At least 2 waypoints are required: "
                    "one grasp candidate and one final drop waypoint."
                )

            # Last waypoint is always the drop.
            drop_idx = len(base_waypoints) - 1

            # Grasp candidates:
            #
            # WP1 ... WP(N-1)
            #
            # Explicitly exclude WP(N), i.e. the final drop waypoint.
            candidate_z = np.asarray(
                base_waypoints[:drop_idx, 2],
                dtype=np.float64,
            )

            if candidate_z.size == 0:
                raise ValueError(
                    "No waypoint available for grasp selection."
                )

            if not np.isfinite(candidate_z).all():
                raise ValueError(
                    "Cannot select grasp waypoint because candidate "
                    f"Z values are invalid: {candidate_z.tolist()}"
                )

            # np.argmin returns the first minimum in case of ties.
            grasp_idx = int(
                np.argmin(candidate_z)
            )

            # Save it for debugging / visualization.
            self._selected_grasp_waypoint_index = grasp_idx

            # Human-readable 1-based waypoint numbers.
            grasp_number = grasp_idx + 1
            drop_number = drop_idx + 1

            # ---------------------------------------------------------
            # Build gripper schedule.
            #
            # Before grasp:
            #     OPEN
            #
            # At grasp waypoint:
            #     grasp transition
            #
            # Between grasp and drop:
            #     CLOSED / carry
            #
            # Final waypoint:
            #     DROP / OPEN
            # ---------------------------------------------------------

            command_flags = np.zeros(
                len(image_waypoints),
                dtype=bool,
            )

            command_flags[
                grasp_idx:drop_idx
            ] = True

            print(
                "[OSVIAWDAController] Dynamic grasp selection: "
                f"candidate_z={candidate_z.tolist()}, "
                f"grasp_waypoint={grasp_number}, "
                f"grasp_z={float(candidate_z[grasp_idx]):.6f}, "
                f"drop_waypoint={drop_number}, "
                f"command_flags={command_flags.tolist()}"
            )

            print(
                "[OSVIAWDAController] Model grasp attributes "
                f"(diagnostic only)={raw_attributes.tolist()}, "
                f"threshold_flags={threshold_flags.tolist()}"
            )

        else:
            self._selected_grasp_waypoint_index = None

            command_flags = self._repository_grasp_flags(
                threshold_flags
            )

        actions = []
        self.last_gripper_decisions = []

        if bool(control.get("fixed_gripper_schedule", False)):
            # The fixed schedule assumes the trajectory starts with the gripper open.
            holding = False
        else:
            holding = bool(self.gripper_closed)
        for index, (base_waypoint, raw_attribute, threshold_flag, command_flag) in enumerate(
            zip(base_waypoints, raw_attributes, threshold_flags, command_flags),
            start=1,
        ):
            if command_flag and not holding:
                primitive = "grasp"
            elif not command_flag and holding:
                primitive = "drop"
            elif command_flag:
                primitive = "carry"
            else:
                primitive = "free_space"

            raw_xyz = np.asarray(base_waypoint[:3], dtype=np.float64)
            xyz = self._apply_workspace_safety(raw_xyz)
            command_position = closed_position if command_flag else open_position
            action_appended = self._append_action(
                actions,
                xyz,
                quaternion,
                command_position,
                min_distance=min_distance,
            )
            if not action_appended:
                holding = bool(command_flag)
                continue
            self.last_gripper_decisions.append(
                {
                    "waypoint_index": index,
                    "raw_grasp_attribute": float(raw_attribute),
                    "grasp_threshold": threshold,
                    "threshold_closed": bool(threshold_flag),
                    "commanded_closed": bool(command_flag),
                    "command_position": command_position,
                    "primitive": primitive,
                    "grasp_refinement_pending": bool(
                        primitive == "grasp"
                        and self.cfg.grasp_refinement.get("enabled", False)
                    ),
                    "workspace_clamped": not np.allclose(raw_xyz, xyz),
                }
            )
            holding = bool(command_flag)

        self.gripper_closed = holding
        return actions

    def _repository_grasp_flags(self, threshold_flags):
        """Apply the one-waypoint left shift from AWDA's rollout code."""
        flags = np.asarray(threshold_flags, dtype=bool)
        if not bool(self.cfg.control.get("repository_grasp_shift_left", True)):
            return flags
        if flags.size == 0:
            return flags
        return np.concatenate([flags[1:], np.asarray([False])])

    def _parse_robot_state(self, state):
        parsed = {"eef_pos": None, "eef_quat": None}
        if state is None:
            return parsed

        if isinstance(state, dict):
            position = state.get(EEF_POS_NAME)
            if position is None:
                position = state.get("eef_pos")
            quaternion = state.get(EEF_QUAT_NAME)
            if quaternion is None:
                quaternion = state.get("eef_quat")
        else:
            values = np.asarray(state, dtype=np.float64).reshape(-1)
            position = values[:3] if values.size >= 3 else None
            quaternion = values[3:7] if values.size >= 7 else None

        if position is not None:
            parsed["eef_pos"] = np.asarray(position, dtype=np.float64)
        if quaternion is not None:
            quaternion = np.asarray(quaternion, dtype=np.float64)
            norm = float(np.linalg.norm(quaternion))
            if norm <= 1e-8:
                raise ValueError("EEF quaternion must be non-zero.")
            quaternion = quaternion / norm
            parsed["eef_quat"] = quaternion
            self.current_eef_quat = quaternion
        return parsed

    def _orientation_for_action(self, robot_state):
        mode = str(self.cfg.control.get("orientation_mode", "fixed_top_down"))
        if mode == "identity":
            return IDENTITY_QUAT_XYZW.copy()
        if mode in ("fixed_top_down", "top_down"):
            quaternion = np.asarray(
                self.cfg.control.get("fixed_orientation_xyzw", TOP_DOWN_QUAT_XYZW),
                dtype=np.float64,
            )
        elif mode == "current_eef_quat":
            quaternion = robot_state.get("eef_quat")
            if quaternion is None:
                quaternion = self.current_eef_quat
            if quaternion is None:
                raise ValueError(
                    "orientation_mode=current_eef_quat requires an EEF quaternion."
                )
            quaternion = np.asarray(quaternion, dtype=np.float64)
        else:
            raise ValueError(f"Unsupported orientation_mode: {mode!r}")

        norm = float(np.linalg.norm(quaternion))
        if norm <= 1e-8:
            raise ValueError("Action quaternion must be non-zero.")
        return quaternion / norm

    def _apply_workspace_safety(self, xyz):
        xyz = np.asarray(xyz, dtype=np.float64)
        if not np.isfinite(xyz).all():
            raise ValueError("Projected AWDA waypoint contains NaN or infinite coordinates.")
        if not bool(self.cfg.safety.get("clamp_workspace", False)):
            return xyz

        minimum = np.asarray(self.cfg.safety.get("workspace_min"), dtype=np.float64)
        maximum = np.asarray(self.cfg.safety.get("workspace_max"), dtype=np.float64)
        if minimum.shape != (3,) or maximum.shape != (3,) or np.any(minimum >= maximum):
            raise ValueError("safety.workspace_min/max must define valid 3D bounds.")
        return np.clip(xyz, minimum, maximum)

    def _append_action(self, actions, xyz, quaternion, gripper, min_distance=0.0):
        action = np.concatenate(
            [
                np.asarray(xyz, dtype=np.float64),
                np.asarray(quaternion, dtype=np.float64),
                np.asarray([float(gripper)], dtype=np.float64),
            ]
        )
        if actions and min_distance > 0.0:
            same_pose = np.linalg.norm(action[:3] - actions[-1][:3]) < min_distance
            same_gripper = abs(float(action[-1] - actions[-1][-1])) < 1e-9
            if same_pose and same_gripper:
                return False
        actions.append(action)
        return True

    @staticmethod
    def _image_waypoint_to_pixel(waypoint, width, height):
        """
        Convert AWDA normalized image coordinates [-1, 1]
        to OpenCV pixel coordinates.

        Coordinate conventions:

            AWDA:
                u = -1 -> left
                u = +1 -> right

                v = +1 -> top
                v = -1 -> bottom

            OpenCV:
                x = 0        -> left
                x = width-1  -> right

                y = 0        -> top
                y = height-1 -> bottom

        Therefore the vertical axis must be flipped.
        """
        waypoint = np.asarray(
            waypoint,
            dtype=np.float64,
        )

        u_norm = float(waypoint[0])
        v_norm = float(waypoint[1])

        # Horizontal axis: same direction.
        x = (
            (u_norm + 1.0)
            * 0.5
            * (width - 1)
        )

        # Vertical axis: AWDA and OpenCV have opposite directions.
        y = (
            (1.0 - v_norm)
            * 0.5
            * (height - 1)
        )

        x = int(
            np.clip(
                round(x),
                0,
                width - 1,
            )
        )

        y = int(
            np.clip(
                round(y),
                0,
                height - 1,
            )
        )

        return x, y
    
    def _build_waypoint_overlay(self, model_frame, image_waypoints):
        """
        Draw the five selected AWDA waypoints on the exact image
        seen by the model.

        This visualization is PRE grasp-refinement.
        """
        image_waypoints = np.asarray(
            image_waypoints,
            dtype=np.float64,
        )

        # model_frame is normalized CHW float.
        # Convert it back to RGB uint8.
        overlay_rgb = self._chw_to_uint8(model_frame)

        # OpenCV visualization uses BGR.
        overlay = cv2.cvtColor(
            overlay_rgb,
            cv2.COLOR_RGB2BGR,
        )

        height, width = overlay.shape[:2]

        points = [
            self._image_waypoint_to_pixel(
                waypoint,
                width,
                height,
            )
            for waypoint in image_waypoints
        ]

        # Retrieve the primitive actually assigned by post_process().
        primitive_by_waypoint = {
            int(decision["waypoint_index"]):
                str(decision.get("primitive", "?"))
            for decision in self.last_gripper_decisions
        }

        primitive_colors = {
            "free_space": (255, 255, 0),  # cyan
            "grasp":      (0, 0, 255),    # red
            "carry":      (0, 165, 255),  # orange
            "drop":       (0, 255, 0),    # green
        }

        # Connect the five predicted waypoints.
        for index in range(len(points) - 1):
            cv2.line(
                overlay,
                points[index],
                points[index + 1],
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        for index, (point, waypoint) in enumerate(
            zip(points, image_waypoints),
            start=1,
        ):
            primitive = primitive_by_waypoint.get(
                index,
                "?",
            )

            color = primitive_colors.get(
                primitive,
                (255, 255, 255),
            )

            radius = 5 if primitive == "grasp" else 3

            cv2.circle(
                overlay,
                point,
                radius,
                color,
                -1,
                cv2.LINE_AA,
            )

            # Keep text compact because model image is only 180x100.
            label = f"{index}:{primitive}"

            text_position = (
                min(point[0] + 4, width - 1),
                max(point[1] - 4, 10),
            )

            cv2.putText(
                overlay,
                label,
                text_position,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.30,
                color,
                1,
                cv2.LINE_AA,
            )

        return overlay

    def _handle_waypoint_overlay(
        self,
        model_frame,
        image_waypoints,
        t,
        save_path=None,
    ):
        debug_cfg = self.cfg.debug

        show_overlay = bool(
            debug_cfg.get(
                "show_waypoint_overlay",
                False,
            )
        )

        save_overlay = bool(
            debug_cfg.get(
                "save_waypoint_overlay",
                False,
            )
        )

        if not show_overlay and not save_overlay:
            return

        overlay = self._build_waypoint_overlay(
            model_frame,
            image_waypoints,
        )

        self._last_waypoint_overlay_bgr = (
            overlay.copy()
        )

        # At t=0 the right panel will contain the
        # "waiting for refinement" placeholder.
        self._refresh_debug_visualization(
            t=t,
            save_path=save_path,
        )

    def _build_gripper_centroid_overlay(
        self,
        centroid,
        depth_shape,
        rgb=None,
    ):
        """
        Read the current gripper RGB image and draw an X at the
        centroid selected by the depth connected-component stage.

        centroid is expressed in depth-image coordinates (u, v).
        Registered depth and RGB normally share the same geometry,
        but scaling is handled if their resolutions differ.
        """
        if rgb is None:
            rgb = self._current_gripper_rgb

        if rgb is None:
            return None

        rgb = np.asarray(
            rgb,
            dtype=np.uint8,
        )

        u_depth, v_depth = centroid

        depth_height = int(depth_shape[0])
        depth_width = int(depth_shape[1])

        rgb_height, rgb_width = rgb.shape[:2]

        # Depth is registered to RGB, so normally these sizes match.
        # Still handle a possible resolution difference robustly.
        if depth_width > 1:
            u_rgb = int(
                round(
                    float(u_depth)
                    * (rgb_width - 1)
                    / (depth_width - 1)
                )
            )
        else:
            u_rgb = 0

        if depth_height > 1:
            v_rgb = int(
                round(
                    float(v_depth)
                    * (rgb_height - 1)
                    / (depth_height - 1)
                )
            )
        else:
            v_rgb = 0

        u_rgb = int(
            np.clip(
                u_rgb,
                0,
                rgb_width - 1,
            )
        )

        v_rgb = int(
            np.clip(
                v_rgb,
                0,
                rgb_height - 1,
            )
        )

        overlay = rgb.copy()

        cross_size = max(
            10,
            int(
                min(
                    rgb_width,
                    rgb_height,
                )
                * 0.03
            ),
        )

        thickness = max(
            2,
            int(cross_size / 5),
        )

        color = (0, 0, 255)  # red BGR

        # Draw an actual X, not a +.
        cv2.line(
            overlay,
            (
                u_rgb - cross_size,
                v_rgb - cross_size,
            ),
            (
                u_rgb + cross_size,
                v_rgb + cross_size,
            ),
            color,
            thickness,
            cv2.LINE_AA,
        )

        cv2.line(
            overlay,
            (
                u_rgb - cross_size,
                v_rgb + cross_size,
            ),
            (
                u_rgb + cross_size,
                v_rgb - cross_size,
            ),
            color,
            thickness,
            cv2.LINE_AA,
        )

        cv2.circle(
            overlay,
            (u_rgb, v_rgb),
            4,
            color,
            -1,
            cv2.LINE_AA,
        )

        cv2.putText(
            overlay,
            f"centroid ({u_depth}, {v_depth})",
            (
                max(5, u_rgb + cross_size + 5),
                max(20, v_rgb),
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
            cv2.LINE_AA,
        )

        self._last_gripper_centroid_uv = (
            int(u_depth),
            int(v_depth),
        )

        return overlay

    def _build_depth_centroid_overlay(
        self,
        depth_full_m,
        crop_left,
        crop_right,
        depth_gray_crop,
        edges_closed=None,
        best_contour=None,
        best_centroid=None,
        best_depth=None,
        best_rect=None,
    ):
        """
        Build a live visualization of the depth-processing pipeline.

        Visualization:
            - full normalized depth image;
            - area outside horizontal ROI darkened;
            - crop boundaries;
            - final Canny/closing edges inside ROI;
            - selected contour;
            - selected contour centroid;
            - contour minimum depth and centroid depth.

        best_contour and best_centroid are expressed in CROPPED-image
        coordinates. The overlay itself uses ORIGINAL full-image coordinates.
        """

        refine_cfg = self.cfg.grasp_refinement

        min_depth = float(
            refine_cfg.get(
                "depth_edge_min_depth_m",
                0.01,
            )
        )

        max_depth = float(
            refine_cfg.get(
                "depth_edge_max_depth_m",
                1.0,
            )
        )

        depth_full_m = np.asarray(
            depth_full_m,
            dtype=np.float64,
        )

        height, width = depth_full_m.shape[:2]

        # ---------------------------------------------------------
        # FULL DEPTH -> GRAYSCALE
        # ---------------------------------------------------------

        depth_safe = depth_full_m.copy()

        invalid = (
            ~np.isfinite(depth_safe)
            | (depth_safe <= 0.0)
        )

        depth_safe[invalid] = min_depth

        depth_clipped = np.clip(
            depth_safe,
            min_depth,
            max_depth,
        )

        depth_normalized = (
            (depth_clipped - min_depth)
            / (max_depth - min_depth)
        )

        depth_gray_full = (
            depth_normalized * 255.0
        ).astype(np.uint8)

        overlay = cv2.cvtColor(
            depth_gray_full,
            cv2.COLOR_GRAY2BGR,
        )

        # ---------------------------------------------------------
        # DARKEN EVERYTHING OUTSIDE THE ACTIVE CROP
        # ---------------------------------------------------------

        if crop_left > 0:
            overlay[
                :,
                :crop_left,
            ] = (
                overlay[
                    :,
                    :crop_left,
                ].astype(np.float32)
                * 0.20
            ).astype(np.uint8)

        if crop_right + 1 < width:
            overlay[
                :,
                crop_right + 1:,
            ] = (
                overlay[
                    :,
                    crop_right + 1:,
                ].astype(np.float32)
                * 0.20
            ).astype(np.uint8)

        # ---------------------------------------------------------
        # CROP BOUNDARIES
        # ---------------------------------------------------------

        cv2.line(
            overlay,
            (crop_left, 0),
            (crop_left, height - 1),
            (255, 255, 255),
            2,
        )

        cv2.line(
            overlay,
            (crop_right, 0),
            (crop_right, height - 1),
            (255, 255, 255),
            2,
        )

        # ---------------------------------------------------------
        # FINAL EDGES AFTER CLOSING
        # ---------------------------------------------------------

        if edges_closed is not None:

            roi = overlay[
                :,
                crop_left:crop_right + 1,
            ]

            edge_mask = edges_closed > 0

            # Yellow edges.
            roi[
                edge_mask
            ] = (
                0,
                255,
                255,
            )

        # ---------------------------------------------------------
        # SELECTED CONTOUR + CENTROID
        # ---------------------------------------------------------

        if (
            best_contour is not None
            and best_centroid is not None
        ):

            # Convert contour coordinates from crop -> full image.
            contour_full = best_contour.copy()

            contour_full[
                :,
                :,
                0,
            ] += crop_left

            # Selected contour = red.
            cv2.drawContours(
                overlay,
                [contour_full],
                -1,
                (0, 0, 255),
                3,
                cv2.LINE_AA,
            )

            # -----------------------------------------------------
            # MIN-AREA RECTANGLE
            # -----------------------------------------------------

            if best_rect is not None:

                # boxPoints returns the four corners in CROPPED
                # depth-image coordinates.
                box = cv2.boxPoints(
                    best_rect
                ).astype(np.float64)

                # Convert crop coordinates back to the ORIGINAL
                # full depth-image coordinates.
                box[:, 0] += crop_left

                box = np.round(
                    box
                ).astype(np.int32)

                box = box.reshape(
                    (-1, 1, 2)
                )

                # Magenta = minimum-area rotated rectangle.
                cv2.drawContours(
                    overlay,
                    [box],
                    0,
                    (255, 0, 255),
                    2,
                    cv2.LINE_AA,
                )

            u_crop = int(
                np.clip(
                    round(best_centroid[0]),
                    0,
                    depth_gray_crop.shape[1] - 1,
                )
            )

            v = int(
                np.clip(
                    round(best_centroid[1]),
                    0,
                    height - 1,
                )
            )

            u_full = int(
                np.clip(
                    crop_left + u_crop,
                    0,
                    width - 1,
                )
            )

            # Actual depth at centroid.
            centroid_depth = depth_full_m[
                v,
                u_full,
            ]

            # -----------------------------------------------------
            # CENTROID MARKER
            # -----------------------------------------------------

            marker_size = 15

            cv2.drawMarker(
                overlay,
                (
                    u_full,
                    v,
                ),
                (0, 255, 0),
                markerType=cv2.MARKER_CROSS,
                markerSize=marker_size,
                thickness=3,
                line_type=cv2.LINE_AA,
            )

            cv2.circle(
                overlay,
                (
                    u_full,
                    v,
                ),
                5,
                (0, 255, 0),
                -1,
                cv2.LINE_AA,
            )

            # -----------------------------------------------------
            # TEXT
            # -----------------------------------------------------

            text_x = min(
                u_full + 15,
                max(5, width - 260),
            )

            text_y = max(
                30,
                v - 25,
            )

            cv2.putText(
                overlay,
                f"rect center=({u_full},{v})",
                (
                    text_x,
                    text_y,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

            if np.isfinite(centroid_depth):

                cv2.putText(
                    overlay,
                    f"center depth={centroid_depth:.4f} m",
                    (
                        text_x,
                        text_y + 25,
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.50,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

            if best_depth is not None:

                cv2.putText(
                    overlay,
                    f"contour min={best_depth:.4f} m",
                    (
                        text_x,
                        text_y + 50,
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.50,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )

        else:

            cv2.putText(
                overlay,
                "NO VALID CONTOUR",
                (
                    20,
                    40,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        return overlay

    def _build_combined_debug_view(self):
        """
        Build the live two-panel debug view:

            LEFT:
                AWDA front-camera model input + predicted waypoints

            RIGHT:
                gripper-camera RGB + X on selected depth centroid
        """
        if self._last_waypoint_overlay_bgr is None:
            return None

        front = self._last_waypoint_overlay_bgr.copy()

        if self._last_gripper_centroid_overlay_bgr is None:
            gripper = np.zeros_like(front)

            cv2.putText(
                gripper,
                "GRIPPER: waiting for refinement",
                (5, max(15, gripper.shape[0] // 2)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        else:
            gripper = (
                self._last_gripper_centroid_overlay_bgr
                .copy()
            )

        if self._last_depth_centroid_overlay_bgr is None:

            depth_debug = np.zeros_like(front)

            cv2.putText(
                depth_debug,
                "DEPTH: waiting for refinement",
                (
                    5,
                    max(
                        15,
                        depth_debug.shape[0] // 2,
                    ),
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        else:

            depth_debug = (
                self._last_depth_centroid_overlay_bgr
                .copy()
            )

        debug_cfg = self.cfg.debug

        panel_height = int(
            debug_cfg.get(
                "waypoint_overlay_panel_height",
                400,
            )
        )

        def resize_to_height(image, target_height):
            height, width = image.shape[:2]

            scale = (
                float(target_height)
                / float(height)
            )

            target_width = max(
                1,
                int(round(width * scale)),
            )

            return cv2.resize(
                image,
                (
                    target_width,
                    target_height,
                ),
                interpolation=cv2.INTER_NEAREST,
            )

        front = resize_to_height(
            front,
            panel_height,
        )

        gripper = resize_to_height(
            gripper,
            panel_height,
        )

        depth_debug = resize_to_height(
            depth_debug,
            panel_height,
        )

        cv2.putText(
            front,
            "FRONT - AWDA waypoints",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            gripper,
            "GRIPPER - depth centroid",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            depth_debug,
            "DEPTH - processed + selected centroid",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        separator = np.zeros(
            (
                panel_height,
                4,
                3,
            ),
            dtype=np.uint8,
        )

        return np.hstack(
            [
                front,
                separator,
                gripper,
                separator.copy(),
                depth_debug,
            ]
        )
    def _refresh_debug_visualization(
        self,
        t,
        save_path=None,
    ):
        debug_cfg = self.cfg.debug

        show_overlay = bool(
            debug_cfg.get(
                "show_waypoint_overlay",
                False,
            )
        )

        save_overlay = bool(
            debug_cfg.get(
                "save_waypoint_overlay",
                False,
            )
        )

        if not show_overlay and not save_overlay:
            return

        combined = self._build_combined_debug_view()

        if combined is None:
            return

        if save_overlay and save_path is not None:
            os.makedirs(
                save_path,
                exist_ok=True,
            )

            output_path = os.path.join(
                save_path,
                f"osvi_awda_debug_overlay_t{t:03d}.png",
            )

            cv2.imwrite(
                output_path,
                combined,
            )

        if (
            show_overlay
            and not self._waypoint_overlay_display_failed
        ):
            window_name = str(
                debug_cfg.get(
                    "waypoint_overlay_window",
                    "OSVI-AWDA waypoint overlay",
                )
            )

            wait_ms = max(
                1,
                int(
                    debug_cfg.get(
                        "waypoint_overlay_wait_ms",
                        1,
                    )
                ),
            )

            try:
                # Also handle a window manually closed by the user.
                if self._waypoint_overlay_window_initialized:
                    try:
                        visible = cv2.getWindowProperty(
                            window_name,
                            cv2.WND_PROP_VISIBLE,
                        )

                        if visible < 1:
                            self._waypoint_overlay_window_initialized = False

                    except cv2.error:
                        self._waypoint_overlay_window_initialized = False

                if not self._waypoint_overlay_window_initialized:
                    cv2.namedWindow(
                        window_name,
                        cv2.WINDOW_NORMAL,
                    )

                    cv2.resizeWindow(
                        window_name,
                        combined.shape[1],
                        combined.shape[0],
                    )

                    self._waypoint_overlay_window_initialized = True

                cv2.imshow(
                    window_name,
                    combined,
                )

                cv2.waitKey(wait_ms)

            except cv2.error as exc:
                print(
                    "[OSVIAWDAController] "
                    "Debug overlay display disabled: "
                    f"{exc}"
                )

                self._waypoint_overlay_display_failed = True

    def inference(self, input_data, t: int = 0, save_path=None):
        """Return an execution-ready AWDA plan using one or two control-loop calls.

        A normal call runs the model and expands primitives up to a possible
        grasp hover. If a grasp is pending, the next call reads post-hover depth
        and returns the remainder without running the model a second time.
        """
        if self._pending_grasp_plan is not None:
            # The previous call ended at grasp-hover. The runtime has now
            # executed that action and supplied a fresh observation. Cache those
            # NEW sensor values before resuming, otherwise refinement would reuse
            # the pre-hover depth/RGB from the previous inference call.
            self._update_runtime_sensor_inputs(input_data)
            actions = self._resume_pending_grasp()
            # _resume_pending_grasp() has just run the wrist-depth
            # localization. Therefore the gripper RGB panel now contains
            # the X on the centroid actually selected for grasping.
            self._refresh_debug_visualization(
                t=t,
                save_path=save_path,
            )
            if save_path is not None:
                os.makedirs(save_path, exist_ok=True)
                with open(
                    os.path.join(
                        save_path,
                        f"osvi_awda_pending_grasp_actions_t{t:03d}.json",
                    ),
                    "w",
                    encoding="utf-8",
                ) as stream:
                    json.dump(
                        {
                            "execution_phase": self.last_execution_phase,
                            "actions": [action.tolist() for action in actions],
                        },
                        stream,
                        indent=2,
                    )
            print(
                f"[OSVIAWDAController] Resumed grasp t={t}: "
                f"phase={self.last_execution_phase}, actions={len(actions)}"
            )
            return actions

        processed = self.pre_process(input_data)

        if save_path is not None:
            os.makedirs(save_path, exist_ok=True)
            PILImage.fromarray(processed["live_frame"]).save(
                os.path.join(save_path, f"osvi_awda_input_front_t{t:03d}.png")
            )
            PILImage.fromarray(self._chw_to_uint8(processed["model_frame"])).save(
                os.path.join(save_path, f"osvi_awda_model_input_front_t{t:03d}.png")
            )

        # Keep this call aligned with
        # mtlfd_adaptation/eval_zero_shot_real.py::predict_waypoints.
        with torch.no_grad():
            output = self.model(
                processed["states"],
                processed["images"],
                processed["context"],
                ret_dist=False,
                ents=processed["entities"],
            )

        all_waypoints = self._validate_waypoint_output(output)
        selected_waypoints = (
            all_waypoints[0]
            .detach()
            .cpu()
            .numpy()[-5:]
        )

        base_waypoints = self._project_waypoints_to_base(
            selected_waypoints
        )

        self.last_image_waypoints = selected_waypoints.copy()
        self.last_base_waypoints = base_waypoints.copy()

        coarse_actions = self.post_process(
            {
                "image_waypoints": selected_waypoints,
                "base_waypoints": base_waypoints,
                "robot_state": processed["robot_state"],
            }
        )

        # ---------------------------------------------------------
        # ONLINE WAYPOINT VISUALIZATION
        #
        # Must happen AFTER post_process(), because only now
        # last_gripper_decisions contains:
        # free_space / grasp / carry / drop.
        #
        # The displayed waypoints are the ORIGINAL AWDA prediction,
        # before eye-in-hand grasp refinement.
        # ---------------------------------------------------------
        self._handle_waypoint_overlay(
            model_frame=processed["model_frame"],
            image_waypoints=selected_waypoints,
            t=t,
            save_path=save_path,
        )

        actions = self._execution_actions_from_coarse(
            coarse_actions,
            processed["robot_state"],
        )

        if save_path is not None:
            with open(
                os.path.join(save_path, f"osvi_awda_raw_waypoints_t{t:03d}.json"),
                "w",
                encoding="utf-8",
            ) as stream:
                json.dump(
                    {
                        "all_waypoints": all_waypoints[0].detach().cpu().tolist(),
                        "selected_waypoints": selected_waypoints.tolist(),
                        "base_waypoints": base_waypoints.tolist(),
                        "coarse_actions": [
                            action.tolist() for action in coarse_actions
                        ],
                        "actions": [action.tolist() for action in actions],
                        "gripper_decisions": self.last_gripper_decisions,
                        "context_source": self.context_source,
                        "execution_phase": self.last_execution_phase,
                        "format": ["u_norm", "v_norm", "depth", "grasp_attr"],
                    },
                    stream,
                    indent=2,
                )

        print(
            f"[OSVIAWDAController] Forward t={t}: "
            f"raw_shape={tuple(all_waypoints.shape)}, selected_shape={selected_waypoints.shape}, "
            f"base_shape={base_waypoints.shape}, phase={self.last_execution_phase}, "
            f"actions={len(actions)}"
        )
        return actions

    def _project_waypoints_to_base(self, image_waypoints):
        """Mirror camera_projection_real.project_normalized_depth_to_world."""
        if self.projection_matrix_numpy is None:
            raise RuntimeError("AWDA projection matrix is not loaded.")

        waypoints = np.asarray(image_waypoints, dtype=np.float64)
        if waypoints.ndim != 2 or waypoints.shape[1] != 4:
            raise ValueError(
                f"Expected image waypoints with shape [W,4], got {waypoints.shape}."
            )
        if not np.isfinite(waypoints).all():
            raise ValueError("Cannot project AWDA waypoints containing NaN or infinite values.")

        uvz = waypoints[:, :3]
        homogeneous = np.concatenate(
            [uvz[:, :2] * uvz[:, 2:3], uvz[:, 2:3], np.ones_like(uvz[:, :1])],
            axis=-1,
        )
        world_homogeneous = homogeneous @ self.projection_matrix_numpy.astype(np.float64).T
        return np.concatenate([world_homogeneous[:, :3], waypoints[:, 3:4]], axis=-1)

    def _validate_waypoint_output(self, output):
        if not isinstance(output, dict) or "waypoints" not in output:
            keys = list(output.keys()) if isinstance(output, dict) else type(output).__name__
            raise ValueError(f"AWDA model output has no 'waypoints' tensor: {keys}")

        waypoints = output["waypoints"]
        if not torch.is_tensor(waypoints):
            raise TypeError(
                f"AWDA output['waypoints'] must be a torch.Tensor, got {type(waypoints).__name__}"
            )

        base_count = int(self.training_config["policy"]["waypoints"])
        sub_waypoints = bool(self.training_config["policy"].get("sub_waypoints", False))
        expected_count = (
            (base_count + 1) * base_count // 2 if sub_waypoints else base_count
        )
        expected_shape = (1, expected_count, 4)
        if tuple(waypoints.shape) != expected_shape:
            raise ValueError(
                f"Unexpected AWDA waypoint shape {tuple(waypoints.shape)}; "
                f"expected {expected_shape} from the checkpoint policy."
            )
        if not bool(torch.isfinite(waypoints).all()):
            raise ValueError("AWDA waypoint output contains NaN or infinite values.")
        return waypoints

    def load_command(self, demo_path: str, task_id: str, **kwargs):
        save_demo_frames = bool(kwargs.get("save_demo_frames", True))
        trajectory_count = int(kwargs.get("traj_cnt", 0))
        save_path = kwargs.get("save_path")

        task_folder = str(task_id)
        if not task_folder.startswith("task_"):
            task_folder = f"task_{task_folder.zfill(2)}"

        demo_directory = Path(os.path.expanduser(demo_path)) / task_folder
        demo_files = sorted(demo_directory.glob("*.pkl"))
        if not demo_files:
            raise FileNotFoundError(f"No demo .pkl files found in {demo_directory}")

        demo_file = demo_files[0]
        with demo_file.open("rb") as stream:
            payload = TrajectoryUnpickler(stream).load()
        trajectory = self._unwrap_trajectory(payload)

        frames = self._sample_context_frames(trajectory)
        processed = [self._preprocess_frame(frame, source="demo") for frame in frames]
        context_array = np.stack(processed, axis=0)
        self.context_tensor = (
            torch.from_numpy(context_array).unsqueeze(0).float().to(self.device)
        )
        self.context_source = str(demo_file)

        if save_demo_frames and save_path is not None:
            output_directory = (
                Path(save_path)
                / task_folder
                / f"osvi_awda_context_{trajectory_count:03d}"
            )
            output_directory.mkdir(parents=True, exist_ok=True)
            for index, (raw_frame, model_frame) in enumerate(zip(frames, processed)):
                raw_preview = self._as_uint8_rgb_for_model(raw_frame, source="demo")
                PILImage.fromarray(raw_preview).save(
                    output_directory / f"context_raw_{index:02d}.png"
                )
                PILImage.fromarray(self._chw_to_uint8(model_frame)).save(
                    output_directory / f"context_model_{index:02d}.png"
                )

        print(
            f"[OSVIAWDAController] Loaded context from {demo_file}: "
            f"{tuple(self.context_tensor.shape)}"
        )

    def _sample_context_indices(self, trajectory_length: int):
        if trajectory_length <= 0:
            raise ValueError("Cannot sample context from an empty trajectory.")

        frame_count = int(self.cfg.context.get("num_frames", 10))
        if frame_count <= 0:
            raise ValueError("context.num_frames must be positive.")

        sample_sides = bool(self.cfg.context.get("sample_sides", True))
        per_bracket = max(trajectory_length / frame_count, 1)

        def clip(index):
            return int(max(0, min(index, trajectory_length - 1)))

        indices = []
        for index in range(frame_count):
            sampled = clip(int((index + 0.5) * per_bracket))
            if sample_sides and index == 0:
                sampled = 0
            elif sample_sides and index == frame_count - 1:
                sampled = trajectory_length - 1
            indices.append(sampled)
        return indices

    def _sample_context_frames(self, trajectory):
        frames = []
        for index in self._sample_context_indices(self._trajectory_length(trajectory)):
            step = self._trajectory_step(trajectory, index)
            observation = self._step_observation(step)
            frames.append(self._get_observation_image(observation))
        return frames

    def _preprocess_frame(self, frame, source: str):
        image = self._as_uint8_rgb_for_model(frame, source=source)
        image = self._crop(image, self.cfg.image.get("crop", [0, 0, 0, 0]))
        height = int(self.cfg.image.get("height", 100))
        width = int(self.cfg.image.get("width", 180))
        if image.shape[:2] != (height, width):
            interpolation = cv2.INTER_AREA
            if np.prod(image.shape[:2]) > height * width:
                interpolation = cv2.INTER_LINEAR
            image = cv2.resize(image, (width, height), interpolation=interpolation)

        chw = image.astype(np.float32).transpose(2, 0, 1) / 255.0
        if bool(self.cfg.image.get("normalize", True)):
            chw = (chw - IMAGENET_MEAN) / IMAGENET_STD
        return chw.astype(np.float32)

    def _prepare_live_frame(self, frame):
        image = self._decode_image(frame)
        if not bool(self.cfg.image.get("live_images_are_rgb", True)):
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return image

    def _as_uint8_rgb_for_model(self, frame, source: str):
        convert_bgr = source == "demo" and bool(
            self.cfg.image.get("demo_convert_bgr_to_rgb", False)
        )
        return self._decode_image(frame, convert_bgr_to_rgb=convert_bgr)

    @staticmethod
    def _decode_image(value: Any, convert_bgr_to_rgb: bool = False):
        image = value
        if isinstance(image, (bytes, bytearray)):
            image = np.frombuffer(image, dtype=np.uint8)
        if isinstance(image, np.ndarray) and image.ndim == 1:
            decoded = cv2.imdecode(image.astype(np.uint8), cv2.IMREAD_COLOR)
            if decoded is None:
                raise ValueError("cv2.imdecode failed for compressed image")
            image = decoded

        image = np.asarray(image)
        if image.ndim == 2:
            image = image[:, :, None]
        if image.ndim != 3:
            raise ValueError(f"Expected an image with 2 or 3 dimensions, got {image.shape}")
        if image.shape[0] in (1, 3, 4) and image.shape[-1] not in (1, 3, 4):
            image = np.transpose(image, (1, 2, 0))
        if image.shape[-1] == 4:
            image = image[:, :, :3]
        if image.shape[-1] == 1:
            image = np.repeat(image, 3, axis=-1)
        image = image.astype(np.uint8)
        if convert_bgr_to_rgb:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return image

    def _chw_to_uint8(self, chw):
        array = np.asarray(chw, dtype=np.float32)
        if bool(self.cfg.image.get("normalize", True)):
            array = array * IMAGENET_STD + IMAGENET_MEAN
        array = np.transpose(array, (1, 2, 0)) * 255.0
        return np.clip(array, 0, 255).astype(np.uint8)

    @staticmethod
    def _crop(image, crop):
        top, bottom, left, right = [int(value) for value in crop]
        if top > 0:
            image = image[top:]
        if bottom > 0:
            image = image[:-bottom]
        if left > 0:
            image = image[:, left:]
        if right > 0:
            image = image[:, :-right]
        if image.size == 0:
            raise ValueError(f"Crop {crop} produced an empty image.")
        return image

    @staticmethod
    def _unwrap_trajectory(payload):
        if isinstance(payload, dict) and "traj" in payload:
            return payload["traj"]
        return payload

    @staticmethod
    def _trajectory_length(trajectory):
        return len(trajectory)

    @staticmethod
    def _trajectory_step(trajectory, index):
        if hasattr(trajectory, "get"):
            return trajectory.get(index)
        return trajectory[index]

    @staticmethod
    def _step_observation(step):
        if isinstance(step, dict) and "obs" in step:
            return step["obs"]
        return step

    @staticmethod
    def _get_observation_image(observation):
        for key in ("image", "camera_front_image", "front_camera_image"):
            if key in observation:
                return observation[key]
        raise KeyError(
            "No front-camera image found; available observation keys: "
            f"{list(observation.keys())}"
        )
