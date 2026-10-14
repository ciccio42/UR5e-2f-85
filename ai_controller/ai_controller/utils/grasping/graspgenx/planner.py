from __future__ import annotations

from pathlib import Path

import numpy as np
import json

from ai_controller.utils.grasping.graspgenx.client import (
    GraspGenXRemoteClient,
)
from ai_controller.utils.grasping.graspmolmo.client import (
    GraspMolmoClient,
)

from ai_controller.utils.grasping.graspgenx.artifacts import (
    save_graspgenx_artifacts,
)

from dataclasses import dataclass
from scipy.spatial.transform import Rotation

@dataclass(frozen=True)
class GraspGenXPlanResult:
    grasp_instruction: str
    semantic_point_px: np.ndarray
    tcp_position_base: np.ndarray
    tcp_orientation_base: np.ndarray
    approach_direction_base: np.ndarray
    confidence: float

class GraspGenXPlanner:
    """
    SeeDo grasp-planning backend backed by the standalone
    GraspGen-X inference server.

    GraspGen-X runs in its own Python environment and is
    accessed through GraspGenXRemoteClient.
    """

    backend_name = "graspgenx"

    ROBOTIQ_SWEEP_VOLUME = {
        "extents_open": [
            0.085,
            0.032,
            0.036,
        ],
        "offset_open": [
            0.0,
            0.0,
            0.130,
        ],
        "extents_mid": [
            0.046,
            0.032,
            0.036,
        ],
        "offset_mid": [
            0.0,
            0.0,
            0.143,
        ],
        "gripper_type": 0,
        "fingertip_depth": 0.136,
    }

    DEFAULT_SAM_CHECKPOINT = (
        "/opt/checkpoints/seedo/sam/"
        "sam_vit_h_4b8939.pth"
    )

    ROBOTIQ_BBOX_MIN = np.array(
        [
            -0.07421507382649573,
            -0.037499999999747013,
            -0.004,
        ],
        dtype=np.float64,
    )

    ROBOTIQ_BBOX_MAX = np.array(
        [
            0.07421507382649821,
            0.037499999999747013,
            0.14907399999999998,
        ],
        dtype=np.float64,
    )

    FINGERTIP_DEPTH = 0.136

    TABLE_SAFETY_MARGIN = 0.005

    # Scene point-cloud collision validation. Table safety is already
    # handled independently above, so 2 mm here is an intersection-like
    # threshold rather than an extra table margin.
    SCENE_COLLISION_THRESHOLD_M = 0.002
    TARGET_COLLISION_THRESHOLD_M = 0.002
    NUM_COLLISION_SAMPLES = 2000

    # Retained only for compatibility with the current client/worker CLI.
    # The full-resolution collision worker does NOT subsample the scene.
    MAX_SCENE_COLLISION_POINTS = 8192

    MIN_DEPTH = 0.15
    MAX_DEPTH = 0.60

    def __init__(
        self,
        server_host: str = "127.0.0.1",
        server_port: int = 8782,
        gripper_name: str = "robotiq_2f_85",
        num_grasps: int = 200,
        grasp_threshold: float = -1.0,
        topk_num_grasps: int = 100,
        graspmolmo_config_path: str | Path | None = None,
        seed: int = 42,
        sam_checkpoint_path: str | Path = DEFAULT_SAM_CHECKPOINT,
        table_surface_offset_m: float = 0.0,
        semantic_radius_px: float = 50.0,
        grasp_depth_offset_m: float = 0.005,
    ) -> None:

        self.num_grasps = int(
            num_grasps
        )

        self.grasp_threshold = float(
            grasp_threshold
        )

        self.topk_num_grasps = int(
            topk_num_grasps
        )

        self.seed = int(
            seed
        )

        self.client = GraspGenXRemoteClient(
            host=server_host,
            port=server_port,
            gripper_name=gripper_name,
        )

        self.graspmolmo_client = GraspMolmoClient(
            config_path=(
                None
                if graspmolmo_config_path is None
                else str(graspmolmo_config_path)
            )
        )

        self.sam_checkpoint_path = Path(
            sam_checkpoint_path
        )

        self._sam_predictor = None

        self.table_surface_offset_m = float(
            table_surface_offset_m
        )

        self.semantic_radius_px = float(
            semantic_radius_px
        )

        # Positive values move every GraspGen-X candidate farther along
        # its local +Z approach axis before any safety/semantic checks.
        # For the top-down OBB grasps used here, +Z points downward,
        # therefore 0.01 m means a 10 mm deeper grasp.
        self.grasp_depth_offset_m = float(
            grasp_depth_offset_m
        )

    def _get_semantic_grasp_point_base(
        self,
        grasp_base: np.ndarray,
        target_point_cloud_base: np.ndarray,
    ) -> np.ndarray | None:
        grasp_base = np.asarray(
            grasp_base,
            dtype=np.float64,
        )

        point_cloud = np.asarray(
            target_point_cloud_base,
            dtype=np.float64,
        )

        rotation = grasp_base[
            :3,
            :3,
        ]

        translation = grasp_base[
            :3,
            3,
        ]

        sweep_center_local = np.asarray(
            self.ROBOTIQ_SWEEP_VOLUME[
                "offset_open"
            ],
            dtype=np.float64,
        )

        sweep_extents = np.asarray(
            self.ROBOTIQ_SWEEP_VOLUME[
                "extents_open"
            ],
            dtype=np.float64,
        )

        sweep_min = (
            sweep_center_local
            - sweep_extents / 2.0
        )

        sweep_max = (
            sweep_center_local
            + sweep_extents / 2.0
        )

        # base -> grasp local
        local_points = (
            point_cloud
            - translation
        ) @ rotation

        inside = np.all(
            (
                local_points
                >= sweep_min
            )
            & (
                local_points
                <= sweep_max
            ),
            axis=1,
        )

        # Center of the actual Robotiq sweep/contact volume,
        # expressed in base_link.
        reference_base = (
            translation
            + rotation
            @ sweep_center_local
        )

        # The target must physically enter the intended open
        # sweep/contact volume. Do not accept a grasp floating above
        # or beside the target via a nearest-point fallback.
        if not np.any(
            inside
        ):
            return None

        candidate_points = (
            point_cloud[
                inside
            ]
        )

        squared_distances = np.sum(
            (
                candidate_points
                - reference_base
            )
            ** 2,
            axis=1,
        )

        return candidate_points[
            int(
                np.argmin(
                    squared_distances
                )
            )
        ].copy()

    def _project_base_point_to_image(
        self,
        point_base: np.ndarray,
        T_base_camera: np.ndarray,
        camera_matrix: np.ndarray,
    ) -> np.ndarray | None:

        point_base = np.asarray(
            point_base,
            dtype=np.float64,
        )

        T_base_camera = np.asarray(
            T_base_camera,
            dtype=np.float64,
        )

        K = np.asarray(
            camera_matrix,
            dtype=np.float64,
        )

        T_camera_base = np.linalg.inv(
            T_base_camera
        )

        point_camera = (
            T_camera_base[
                :3,
                :3,
            ]
            @ point_base
            + T_camera_base[
                :3,
                3,
            ]
        )

        if (
            not np.isfinite(
                point_camera
            ).all()
            or point_camera[2] <= 0.0
        ):
            return None

        u = (
            K[0, 0]
            * point_camera[0]
            / point_camera[2]
            + K[0, 2]
        )

        v = (
            K[1, 1]
            * point_camera[1]
            / point_camera[2]
            + K[1, 2]
        )

        if not np.isfinite(
            [u, v]
        ).all():
            return None

        return np.asarray(
            [
                u,
                v,
            ],
            dtype=np.float64,
        )

    def _get_sam_predictor(self):
        if self._sam_predictor is not None:
            return self._sam_predictor

        import torch

        from segment_anything import (
            SamPredictor,
            build_sam,
        )

        device = torch.device(
            "cuda:0"
            if torch.cuda.is_available()
            else "cpu"
        )

        sam = build_sam(
            checkpoint=str(
                self.sam_checkpoint_path
            )
        )

        sam.to(
            device=device
        )

        self._sam_predictor = SamPredictor(
            sam
        )

        return self._sam_predictor

    @classmethod
    def _select_sam_mask_index(
        cls,
        masks: np.ndarray,
        scores: np.ndarray,
    ) -> int:

        masks = np.asarray(
            masks,
            dtype=bool,
        )

        scores = np.asarray(
            scores,
            dtype=np.float64,
        )

        if len(masks) == 0:
            raise RuntimeError(
                "SAM returned no masks."
            )

        if len(masks) != len(scores):
            raise ValueError(
                "SAM masks and scores must have "
                "the same length."
            )

        # Standard SAM behavior:
        # choose the mask with the highest predicted quality.
        return int(
            np.argmax(scores)
        )

    def _segment_target(
        self,
        rgb_image: np.ndarray,
        semantic_point_px: np.ndarray,
    ) -> np.ndarray:

        predictor = self._get_sam_predictor()

        predictor.set_image(
            np.asarray(
                rgb_image,
                dtype=np.uint8,
            )
        )

        point = np.asarray(
            [
                [
                    float(
                        semantic_point_px[0]
                    ),
                    float(
                        semantic_point_px[1]
                    ),
                ]
            ],
            dtype=np.float32,
        )

        label = np.asarray(
            [1],
            dtype=np.int32,
        )

        masks, scores, logits = predictor.predict(
            point_coords=point,
            point_labels=label,
            multimask_output=True,
        )

        if len(masks) == 0:
            raise RuntimeError(
                "SAM returned no masks."
            )

        best_index = (
            self._select_sam_mask_index(
                masks=masks,
                scores=scores,
            )
        )

        # Keep SAM diagnostics available for
        # runtime artifact generation.
        self._last_sam_masks = np.asarray(
            masks
        ).copy()

        self._last_sam_scores = np.asarray(
            scores,
            dtype=np.float64,
        ).copy()

        self._last_sam_logits = np.asarray(
            logits
        ).copy()

        self._last_sam_best_index = int(
            best_index
        )

        return np.asarray(
            masks[
                best_index
            ],
            dtype=bool,
        )

    def _mask_to_camera_point_cloud(
        self,
        mask: np.ndarray,
        depth_image: np.ndarray,
        camera_matrix: np.ndarray,
    ) -> np.ndarray:

        mask = np.asarray(
            mask,
            dtype=bool,
        )

        depth = np.asarray(
            depth_image,
            dtype=np.float32,
        )

        K = np.asarray(
            camera_matrix,
            dtype=np.float64,
        )

        if mask.shape != depth.shape:
            raise ValueError(
                "SAM mask and depth image must "
                "have the same shape."
            )

        valid = (
            mask
            & np.isfinite(depth)
            & (depth >= self.MIN_DEPTH)
            & (depth <= self.MAX_DEPTH)
        )

        v, u = np.where(
            valid
        )

        if len(u) == 0:
            raise RuntimeError(
                "Target mask contains no valid "
                "depth points."
            )

        z = depth[
            v,
            u,
        ].astype(
            np.float32
        )

        fx = float(
            K[0, 0]
        )
        fy = float(
            K[1, 1]
        )
        cx = float(
            K[0, 2]
        )
        cy = float(
            K[1, 2]
        )

        x = (
            (
                u.astype(
                    np.float32
                )
                - cx
            )
            * z
            / fx
        )

        y = (
            (
                v.astype(
                    np.float32
                )
                - cy
            )
            * z
            / fy
        )

        return np.stack(
            [
                x,
                y,
                z,
            ],
            axis=1,
        ).astype(
            np.float32
        )

    @staticmethod
    def _bbox_corners(
        minimum: np.ndarray,
        maximum: np.ndarray,
    ) -> np.ndarray:

        return np.asarray(
            [
                [x, y, z]
                for x in (
                    minimum[0],
                    maximum[0],
                )
                for y in (
                    minimum[1],
                    maximum[1],
                )
                for z in (
                    minimum[2],
                    maximum[2],
                )
            ],
            dtype=np.float64,
        )

    def _select_safe_grasp(
        self,
        grasps_base: np.ndarray,
        confidences: np.ndarray,
        branch_tags: np.ndarray,
        T_base_table: np.ndarray,
        semantic_point_px: np.ndarray,
        T_base_camera: np.ndarray,
        camera_matrix: np.ndarray,
        target_point_cloud_base: np.ndarray,
        background_scene_point_cloud_base: np.ndarray,
    ) -> int:

        grasps_base = np.asarray(
            grasps_base,
            dtype=np.float64,
        )

        confidences = np.asarray(
            confidences,
            dtype=np.float64,
        )

        branch_tags = np.asarray(
            branch_tags,
            dtype=str,
        )

        T_base_table = np.asarray(
            T_base_table,
            dtype=np.float64,
        )

        if len(grasps_base) == 0:
            raise RuntimeError(
                "GraspGen-X returned zero grasps."
            )

        if T_base_table.shape != (4, 4):
            raise ValueError(
                "T_base_table must have shape (4, 4)."
            )

        # Table +Z expressed in base_link.
        normal = T_base_table[
            :3,
            2,
        ].copy()

        normal /= np.linalg.norm(
            normal
        )

        # Make the table normal point upward
        # in base_link.
        if normal[2] < 0.0:
            normal = -normal

        table_origin = T_base_table[
            :3,
            3,
        ]

        # table_0 represents the physical tabletop plane.
        # An optional offset can be applied along table +Z if needed.
        table_surface_origin = (
            table_origin
            + self.table_surface_offset_m
            * normal
        )

        plane_d = -float(
            np.dot(
                normal,
                table_surface_origin,
            )
        )

        local_bbox = self._bbox_corners(
            self.ROBOTIQ_BBOX_MIN,
            self.ROBOTIQ_BBOX_MAX,
        )

        safe_indices = []

        for index, grasp in enumerate(
            grasps_base
        ):
            rotation = grasp[
                :3,
                :3,
            ]

            translation = grasp[
                :3,
                3,
            ]

            bbox_base = (
                local_bbox
                @ rotation.T
                + translation
            )

            clearances = (
                bbox_base
                @ normal
                + plane_d
            )

            minimum_clearance = float(
                clearances.min()
            )

            if (
                minimum_clearance
                >= self.TABLE_SAFETY_MARGIN
            ):
                safe_indices.append(
                    index
                )

        if not safe_indices:
            raise RuntimeError(
                "No GraspGen-X candidate satisfies "
                "the table-clearance constraint."
            )

        # Prefer the GraspMoE OBB branch because it is
        # explicitly world/base-aligned top-down.
        safe_obb = [
            index
            for index in safe_indices
            if branch_tags[index] == "obb"
        ]

        candidates = (
            safe_obb
            if safe_obb
            else safe_indices
        )

        semantic_point_px = np.asarray(
            semantic_point_px,
            dtype=np.float64,
        )

        semantic_candidates = []

        semantic_distances_px = {}

        target_contact_candidates = []

        grasp_points_base = {}

        # ---------------------------------------------------------
        # 1. Require real target geometry inside the grasp sweep.
        # ---------------------------------------------------------
        for index in candidates:
            grasp = grasps_base[
                index
            ]

            grasp_point_base = (
                self._get_semantic_grasp_point_base(
                    grasp_base=grasp,
                    target_point_cloud_base=(
                        target_point_cloud_base
                    ),
                )
            )

            if grasp_point_base is None:
                continue

            target_contact_candidates.append(
                index
            )

            grasp_points_base[
                index
            ] = grasp_point_base

            grasp_point_px = (
                self._project_base_point_to_image(
                    point_base=grasp_point_base,
                    T_base_camera=T_base_camera,
                    camera_matrix=camera_matrix,
                )
            )

            if grasp_point_px is None:
                continue

            distance_px = float(
                np.linalg.norm(
                    grasp_point_px
                    - semantic_point_px
                )
            )

            semantic_distances_px[
                index
            ] = distance_px

            if (
                distance_px
                <= self.semantic_radius_px
            ):
                semantic_candidates.append(
                    index
                )

        print(
            "[GRASP] Table-safe OBB candidates:",
            len(candidates),
        )

        print(
            "[GRASP] Target-contact candidates:",
            len(target_contact_candidates),
        )

        print(
            "[GRASP] Semantic candidates before collision:",
            len(semantic_candidates),
        )

        if not semantic_candidates:
            nearest_distance = (
                min(
                    semantic_distances_px.values()
                )
                if semantic_distances_px
                else float("inf")
            )

            raise RuntimeError(
                "No target-contact GraspGen-X candidate lies "
                f"within {self.semantic_radius_px:.1f} px "
                "of the GraspMolmo point. "
                f"Nearest candidate: "
                f"{nearest_distance:.2f} px."
            )

        # ---------------------------------------------------------
        # 2. Check only semantic candidates against the real
        #    Robotiq collision mesh.
        # ---------------------------------------------------------
        (
            collision_free_mask,
            background_free_mask,
            target_free_mask,
        ) = self.client.filter_collision_free_grasps(
            grasps_base=(
                grasps_base[
                    semantic_candidates
                ]
            ),
            background_scene_point_cloud_base=(
                background_scene_point_cloud_base
            ),
            target_point_cloud_base=(
                target_point_cloud_base
            ),
            sweep_volume_params=(
                self.ROBOTIQ_SWEEP_VOLUME
            ),
            collision_threshold=(
                self.SCENE_COLLISION_THRESHOLD_M
            ),
            target_collision_threshold=(
                self.TARGET_COLLISION_THRESHOLD_M
            ),
            num_collision_samples=(
                self.NUM_COLLISION_SAMPLES
            ),
            max_scene_points=(
                self.MAX_SCENE_COLLISION_POINTS
            ),
            timeout=180.0,
        )

        background_free_candidates = [
            index
            for index, keep in zip(
                semantic_candidates,
                background_free_mask,
                strict=True,
            )
            if keep
        ]

        target_self_free_candidates = [
            index
            for index, keep in zip(
                semantic_candidates,
                target_free_mask,
                strict=True,
            )
            if keep
        ]

        collision_free_candidates = [
            index
            for index, keep in zip(
                semantic_candidates,
                collision_free_mask,
                strict=True,
            )
            if keep
        ]

        print(
            "[GRASP] Background-collision-free semantic candidates:",
            len(background_free_candidates),
        )

        print(
            "[GRASP] Target-self-collision-free semantic candidates:",
            len(target_self_free_candidates),
        )

        print(
            "[GRASP] Combined-collision-free semantic candidates:",
            len(collision_free_candidates),
        )

        if not collision_free_candidates:
            raise RuntimeError(
                "All semantic GraspGen-X candidates collide "
                "with scene or target geometry."
            )

        semantic_candidates = (
            collision_free_candidates
        )

        selected_index = max(
            semantic_candidates,
            key=lambda index: float(
                confidences[
                    index
                ]
            ),
        )

        print(
            "[GRASP] Selected semantic distance:",
            semantic_distances_px[
                selected_index
            ],
            "px",
        )

        return selected_index

    @staticmethod
    def _resolve_parallel_gripper_symmetry(
        grasp_rotation_base: np.ndarray,
        current_tcp_orientation: np.ndarray,
    ) -> np.ndarray:
        grasp_rotation_base = np.asarray(
            grasp_rotation_base,
            dtype=np.float64,
        )

        current_tcp_orientation = np.asarray(
            current_tcp_orientation,
            dtype=np.float64,
        )

        if grasp_rotation_base.shape != (3, 3):
            raise ValueError(
                "grasp_rotation_base must have shape (3, 3)."
            )

        if current_tcp_orientation.shape != (4,):
            raise ValueError(
                "current_tcp_orientation must have shape (4,)."
            )

        current_rotation = Rotation.from_quat(
            current_tcp_orientation
        )

        # Parallel-jaw symmetry:
        # rotating 180 degrees around local approach axis (+Z)
        # swaps the two fingers but leaves the physical grasp equivalent.
        symmetry_rotation = Rotation.from_euler(
            "z",
            180.0,
            degrees=True,
        ).as_matrix()

        candidate_a = grasp_rotation_base

        candidate_b = (
            grasp_rotation_base
            @ symmetry_rotation
        )

        def angular_distance(
            candidate: np.ndarray,
        ) -> float:
            relative = (
                current_rotation.inv()
                * Rotation.from_matrix(
                    candidate
                )
            )

            return float(
                relative.magnitude()
            )

        distance_a = angular_distance(
            candidate_a
        )

        distance_b = angular_distance(
            candidate_b
        )

        return (
            candidate_b
            if distance_b < distance_a
            else candidate_a
        )

    def _generate_grasp_candidates_base(
        self,
        target_point_cloud_camera: np.ndarray,
        T_base_camera: np.ndarray,
    ) -> tuple[
        np.ndarray,
        np.ndarray,
        np.ndarray,
        np.ndarray,
    ]:

        target_point_cloud_camera = np.asarray(
            target_point_cloud_camera,
            dtype=np.float32,
        )

        T_base_camera = np.asarray(
            T_base_camera,
            dtype=np.float64,
        )

        if (
            target_point_cloud_camera.ndim != 2
            or target_point_cloud_camera.shape[1] != 3
        ):
            raise ValueError(
                "target_point_cloud_camera must have "
                "shape (N, 3)."
            )

        if len(target_point_cloud_camera) == 0:
            raise ValueError(
                "target_point_cloud_camera must not be empty."
            )

        if T_base_camera.shape != (4, 4):
            raise ValueError(
                "T_base_camera must have shape (4, 4)."
            )

        if not np.isfinite(
            target_point_cloud_camera
        ).all():
            raise ValueError(
                "target_point_cloud_camera contains "
                "non-finite values."
            )

        if not np.isfinite(
            T_base_camera
        ).all():
            raise ValueError(
                "T_base_camera contains non-finite values."
            )

        target_point_cloud_base = (
            target_point_cloud_camera.astype(
                np.float64
            )
            @ T_base_camera[
                :3,
                :3,
            ].T
            + T_base_camera[
                :3,
                3,
            ]
        ).astype(
            np.float32
        )

        (
            grasps_base,
            confidences,
            branch_tags,
        ) = self.client.predict_object_grasps(
            point_cloud=target_point_cloud_base,
            sweep_volume_params=(
                self.ROBOTIQ_SWEEP_VOLUME
            ),
            planner="graspmoe",
            num_grasps=self.num_grasps,
            grasp_threshold=(
                self.grasp_threshold
            ),

            # Important:
            # keep every candidate so later stages
            # can apply geometric filtering.
            topk_num_grasps=-1,

            # Official GraspMoE dense OBB sweep.
            obb_density="dense",
            obb_position_spacing_m=0.01,

            timeout=180.0,
        )

        return (
            grasps_base,
            confidences,
            branch_tags,
            target_point_cloud_base,
        )

    def _semantic_point_to_camera_point(
        self,
        semantic_point_px: np.ndarray,
        depth_image: np.ndarray,
        camera_matrix: np.ndarray,
        search_radius_px: int = 3,
    ) -> np.ndarray:

        semantic_point_px = np.asarray(
            semantic_point_px,
            dtype=np.float32,
        )

        depth_image = np.asarray(
            depth_image,
            dtype=np.float32,
        )

        K = np.asarray(
            camera_matrix,
            dtype=np.float64,
        )

        if semantic_point_px.shape != (2,):
            raise ValueError(
                "semantic_point_px must have shape (2,)."
            )

        if depth_image.ndim != 2:
            raise ValueError(
                "depth_image must have shape (H, W)."
            )

        if K.shape != (3, 3):
            raise ValueError(
                "camera_matrix must have shape (3, 3)."
            )

        height, width = depth_image.shape

        u = int(
            round(
                float(
                    semantic_point_px[0]
                )
            )
        )

        v = int(
            round(
                float(
                    semantic_point_px[1]
                )
            )
        )

        if (
            u < 0
            or u >= width
            or v < 0
            or v >= height
        ):
            raise RuntimeError(
                "Semantic point lies outside the depth image: "
                f"({u}, {v})."
            )

        radius = int(
            search_radius_px
        )

        u_min = max(
            0,
            u - radius,
        )

        u_max = min(
            width,
            u + radius + 1,
        )

        v_min = max(
            0,
            v - radius,
        )

        v_max = min(
            height,
            v + radius + 1,
        )

        depth_patch = depth_image[
            v_min:v_max,
            u_min:u_max,
        ]

        valid_depths = depth_patch[
            np.isfinite(
                depth_patch
            )
            & (
                depth_patch > 0.0
            )
        ]

        if len(valid_depths) == 0:
            raise RuntimeError(
                "No valid depth found around "
                "the semantic point."
            )

        # Median is more robust than taking exactly one
        # depth pixel, especially near object boundaries.
        z = float(
            np.median(
                valid_depths
            )
        )

        fx = float(
            K[0, 0]
        )

        fy = float(
            K[1, 1]
        )

        cx = float(
            K[0, 2]
        )

        cy = float(
            K[1, 2]
        )

        x = (
            float(u) - cx
        ) * z / fx

        y = (
            float(v) - cy
        ) * z / fy

        return np.asarray(
            [
                x,
                y,
                z,
            ],
            dtype=np.float32,
        )

    def _predict_semantic_point(
        self,
        rgb_image: np.ndarray,
        grasp_instruction: str,
    ) -> np.ndarray:

        rgb_image = np.asarray(
            rgb_image,
            dtype=np.uint8,
        )

        if (
            rgb_image.ndim != 3
            or rgb_image.shape[2] != 3
        ):
            raise ValueError(
                "rgb_image must have shape (H, W, 3)."
            )

        grasp_instruction = str(
            grasp_instruction
        ).strip()

        if not grasp_instruction:
            raise ValueError(
                "grasp_instruction cannot be empty."
            )

        semantic_point = (
            self.graspmolmo_client.predict_point(
                rgb=rgb_image,
                task=grasp_instruction,
                verbosity=1,
                timeout=60.0,
                seed=self.seed,
            )
        )

        if semantic_point is None:
            raise RuntimeError(
                "GraspMolmo returned no grasp point."
            )

        return np.asarray(
            semantic_point,
            dtype=np.float32,
        )

    def plan(
        self,
        rgb_image: np.ndarray,
        depth_image: np.ndarray,
        camera_matrix: np.ndarray,
        T_base_camera: np.ndarray,
        T_base_table: np.ndarray,
        current_tcp_position: np.ndarray,
        current_tcp_orientation: np.ndarray,
        grasp_instruction: str,
        artifacts_dir: str | Path | None = None,
    ) -> GraspGenXPlanResult:

        rgb_image = np.asarray(
            rgb_image,
            dtype=np.uint8,
        )

        depth_image = np.asarray(
            depth_image,
            dtype=np.float32,
        )

        camera_matrix = np.asarray(
            camera_matrix,
            dtype=np.float64,
        )

        T_base_camera = np.asarray(
            T_base_camera,
            dtype=np.float64,
        )

        T_base_table = np.asarray(
            T_base_table,
            dtype=np.float64,
        )

        if (
            rgb_image.ndim != 3
            or rgb_image.shape[2] != 3
        ):
            raise ValueError(
                "rgb_image must have shape (H, W, 3)."
            )

        if depth_image.shape != rgb_image.shape[:2]:
            raise ValueError(
                "depth_image and rgb_image must "
                "have matching image dimensions."
            )

        if camera_matrix.shape != (3, 3):
            raise ValueError(
                "camera_matrix must have shape (3, 3)."
            )

        if T_base_camera.shape != (4, 4):
            raise ValueError(
                "T_base_camera must have shape (4, 4)."
            )

        semantic_point_px = (
            self._predict_semantic_point(
                rgb_image=rgb_image,
                grasp_instruction=(
                    grasp_instruction
                ),
            )
        )

        target_mask = self._segment_target(
            rgb_image=rgb_image,
            semantic_point_px=(
                semantic_point_px
            ),
        )

        semantic_point_camera = (
            self._semantic_point_to_camera_point(
                semantic_point_px=semantic_point_px,
                depth_image=depth_image,
                camera_matrix=camera_matrix,
            )
        )

        semantic_point_base = (
            semantic_point_camera.astype(
                np.float64
            )
            @ T_base_camera[
                :3,
                :3,
            ].T
            + T_base_camera[
                :3,
                3,
            ]
        )

        target_point_cloud_camera = (
            self._mask_to_camera_point_cloud(
                mask=target_mask,
                depth_image=depth_image,
                camera_matrix=camera_matrix,
            )
        )

        # Use the FULL eye-in-hand depth image as background collision
        # geometry. Only pixels belonging to the selected SAM target mask
        # are removed here; the target itself is checked separately by the
        # target-self-collision path below.
        #
        # No image-space stride/subsampling is applied: every valid depth
        # pixel in [MIN_DEPTH, MAX_DEPTH] outside the target mask becomes a
        # 3D scene point.
        background_scene_mask = (
            ~target_mask
        )

        background_scene_point_cloud_camera = (
            self._mask_to_camera_point_cloud(
                mask=background_scene_mask,
                depth_image=depth_image,
                camera_matrix=camera_matrix,
            )
        )

        background_scene_point_cloud_base = (
            background_scene_point_cloud_camera.astype(
                np.float64
            )
            @ T_base_camera[
                :3,
                :3,
            ].T
            + T_base_camera[
                :3,
                3,
            ]
        ).astype(
            np.float32
        )

        print(
            "[GRASP] Full-resolution background points:",
            len(
                background_scene_point_cloud_base
            ),
        )

        (
            grasps_base,
            confidences,
            branch_tags,
            target_point_cloud_base,
        ) = self._generate_grasp_candidates_base(
            target_point_cloud_camera=(
                target_point_cloud_camera
            ),
            T_base_camera=T_base_camera,
        )

        # ---------------------------------------------------------
        # DEBUG: target / table height consistency
        # ---------------------------------------------------------
        table_normal = np.asarray(
            T_base_table[:3, 2],
            dtype=np.float64,
        ).copy()

        table_normal /= np.linalg.norm(
            table_normal
        )

        if table_normal[2] < 0.0:
            table_normal = -table_normal

        table_origin = np.asarray(
            T_base_table[:3, 3],
            dtype=np.float64,
        )

        target_heights = (
            np.asarray(
                target_point_cloud_base,
                dtype=np.float64,
            )
            - table_origin
        ) @ table_normal

        print(
            "[DEPTH DEBUG] Target height above table [mm]: "
            f"min={np.min(target_heights) * 1000.0:.2f}, "
            f"p10={np.percentile(target_heights, 10) * 1000.0:.2f}, "
            f"median={np.median(target_heights) * 1000.0:.2f}, "
            f"p90={np.percentile(target_heights, 90) * 1000.0:.2f}, "
            f"max={np.max(target_heights) * 1000.0:.2f}"
        )

        background_heights = (
            np.asarray(
                background_scene_point_cloud_base,
                dtype=np.float64,
            )
            - table_origin
        ) @ table_normal

        near_table = background_heights[
            np.abs(background_heights) < 0.020
        ]

        print(
            "[DEPTH DEBUG] Background points within +/-20 mm of table:",
            len(near_table),
        )

        if len(near_table) > 0:
            print(
                "[DEPTH DEBUG] Near-table height [mm]: "
                f"median={np.median(near_table) * 1000.0:.2f}, "
                f"mean={np.mean(near_table) * 1000.0:.2f}, "
                f"std={np.std(near_table) * 1000.0:.2f}"
            )

        # Apply the desired extra grasp depth BEFORE every downstream
        # geometric check. This is important: the table-clearance,
        # target-contact, collision and semantic filters must validate
        # the same pose that will eventually be sent to the Motion Layer.
        grasps_base = np.asarray(
            grasps_base,
            dtype=np.float64,
        ).copy()

        if self.grasp_depth_offset_m != 0.0:
            approach_axes = (
                grasps_base[
                    :,
                    :3,
                    2,
                ]
            )

            approach_norms = np.linalg.norm(
                approach_axes,
                axis=1,
                keepdims=True,
            )

            if np.any(
                approach_norms <= 1e-12
            ):
                raise RuntimeError(
                    "GraspGen-X returned a candidate with "
                    "an invalid approach axis."
                )

            approach_axes = (
                approach_axes
                / approach_norms
            )

            grasps_base[
                :,
                :3,
                3,
            ] += (
                self.grasp_depth_offset_m
                * approach_axes
            )

        print(
            "[GRASP] Depth offset:",
            f"{self.grasp_depth_offset_m * 1000.0:.1f} mm",
        )

        selected_index = (
            self._select_safe_grasp(
                grasps_base=grasps_base,
                confidences=confidences,
                branch_tags=branch_tags,
                T_base_table=T_base_table,
                semantic_point_px=semantic_point_px,
                T_base_camera=T_base_camera,
                camera_matrix=camera_matrix,
                target_point_cloud_base=(
                    target_point_cloud_base
                ),
                background_scene_point_cloud_base=(
                    background_scene_point_cloud_base
                ),
            )
        )

        selected_grasp = np.asarray(
            grasps_base[
                selected_index
            ],
            dtype=np.float64,
        )

        approach_direction_base = (
            selected_grasp[
                :3,
                2,
            ].copy()
        )

        approach_direction_base /= (
            np.linalg.norm(
                approach_direction_base
            )
        )

        # GraspGen-X pose origin is the Robotiq
        # base frame. Motion Layer commands tcp_link.
        #
        # URDF:
        # robotiq_85_base_link -> tcp_link
        # = +0.136 m along local +Z.
        tcp_position_base = (
            selected_grasp[
                :3,
                3,
            ]
            + self.FINGERTIP_DEPTH
            * approach_direction_base
        )

        selected_rotation_base = (
            self._resolve_parallel_gripper_symmetry(
                grasp_rotation_base=(
                    selected_grasp[
                        :3,
                        :3,
                    ]
                ),
                current_tcp_orientation=(
                    current_tcp_orientation
                ),
            )
        )

        tcp_orientation_base = (
            Rotation.from_matrix(
                selected_rotation_base
            ).as_quat()
        )

        confidence = float(
            confidences[
                selected_index
            ]
        )

        if artifacts_dir is not None:
            save_graspgenx_artifacts(
                artifacts_dir=(
                    artifacts_dir
                ),
                rgb_image=(
                    rgb_image
                ),
                depth_image=(
                    depth_image
                ),
                camera_matrix=(
                    camera_matrix
                ),
                T_base_camera=(
                    T_base_camera
                ),
                T_base_table=(
                    T_base_table
                ),
                semantic_point_px=(
                    semantic_point_px
                ),
                target_mask=(
                    target_mask
                ),
                target_point_cloud_camera=(
                    target_point_cloud_camera
                ),
                target_point_cloud_base=(
                    target_point_cloud_base
                ),
                grasps_base=(
                    grasps_base
                ),
                confidences=(
                    confidences
                ),
                branch_tags=(
                    branch_tags
                ),
                selected_index=(
                    selected_index
                ),
                tcp_position_base=(
                    tcp_position_base
                ),
                tcp_orientation_base=(
                    tcp_orientation_base
                ),
                approach_direction_base=(
                    approach_direction_base
                ),
                fingertip_depth=(
                    self.FINGERTIP_DEPTH
                ),
                gripper_opening=(
                    self.ROBOTIQ_SWEEP_VOLUME[
                        "extents_open"
                    ][0]
                ),
                bbox_min=(
                    self.ROBOTIQ_BBOX_MIN
                ),
                bbox_max=(
                    self.ROBOTIQ_BBOX_MAX
                ),
                sam_masks=getattr(
                    self,
                    "_last_sam_masks",
                    None,
                ),
                sam_scores=getattr(
                    self,
                    "_last_sam_scores",
                    None,
                ),
                sam_logits=getattr(
                    self,
                    "_last_sam_logits",
                    None,
                ),
                sam_best_index=getattr(
                    self,
                    "_last_sam_best_index",
                    None,
                ),
            )

        result = GraspGenXPlanResult(
            grasp_instruction=str(
                grasp_instruction
            ),
            semantic_point_px=np.asarray(
                semantic_point_px,
                dtype=np.float32,
            ),
            tcp_position_base=np.asarray(
                tcp_position_base,
                dtype=np.float64,
            ),
            tcp_orientation_base=np.asarray(
                tcp_orientation_base,
                dtype=np.float64,
            ),
            approach_direction_base=np.asarray(
                approach_direction_base,
                dtype=np.float64,
            ),
            confidence=confidence,
        )

        if artifacts_dir is not None:
            artifacts_path = (
                Path(artifacts_dir)
                .expanduser()
                .resolve()
            )

            artifacts_path.mkdir(
                parents=True,
                exist_ok=True,
            )

            payload = {
                "backend": self.backend_name,
                "grasp_instruction": (
                    result.grasp_instruction
                ),
                "semantic_point_px": (
                    result.semantic_point_px.tolist()
                ),
                "tcp_position_base": (
                    result.tcp_position_base.tolist()
                ),
                "tcp_orientation_base": (
                    result.tcp_orientation_base.tolist()
                ),
                "approach_direction_base": (
                    result.approach_direction_base.tolist()
                ),
                "confidence": float(
                    result.confidence
                ),
                "selected_grasp_index": int(
                    selected_index
                ),
                "selected_branch": str(
                    branch_tags[
                        selected_index
                    ]
                ),
                "fingertip_depth_m": float(
                    self.FINGERTIP_DEPTH
                ),
                "grasp_depth_offset_m": float(
                    self.grasp_depth_offset_m
                ),
            }

            with (
                artifacts_path
                / "grasp_plan.json"
            ).open(
                "w",
                encoding="utf-8",
            ) as stream:
                json.dump(
                    payload,
                    stream,
                    indent=2,
                )

        return result