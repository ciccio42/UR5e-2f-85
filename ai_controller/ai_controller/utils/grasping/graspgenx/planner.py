from __future__ import annotations

from pathlib import Path

import numpy as np

from ai_controller.utils.grasping.graspgenx.client import (
    GraspGenXRemoteClient,
)
from ai_controller.utils.grasping.graspmolmo.client import (
    GraspMolmoClient,
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

        masks, scores, _ = predictor.predict(
            point_coords=point,
            point_labels=label,
            multimask_output=True,
        )

        if len(masks) == 0:
            raise RuntimeError(
                "SAM returned no masks."
            )

        best_index = int(
            np.argmax(
                scores
            )
        )

        return np.asarray(
            masks[best_index],
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

        plane_d = -float(
            np.dot(
                normal,
                table_origin,
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

        return max(
            candidates,
            key=lambda index: float(
                confidences[index]
            ),
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

        target_point_cloud_camera = (
            self._mask_to_camera_point_cloud(
                mask=target_mask,
                depth_image=depth_image,
                camera_matrix=camera_matrix,
            )
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

        selected_index = (
            self._select_safe_grasp(
                grasps_base=grasps_base,
                confidences=confidences,
                branch_tags=branch_tags,
                T_base_table=T_base_table,
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

        tcp_orientation_base = (
            Rotation.from_matrix(
                selected_grasp[
                    :3,
                    :3,
                ]
            ).as_quat()
        )

        confidence = float(
            confidences[
                selected_index
            ]
        )

        return GraspGenXPlanResult(
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