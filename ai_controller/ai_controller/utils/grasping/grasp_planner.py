from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from PIL import Image

from ai_controller.utils.grasping.graspmolmo.client import (
    GraspMolmoClient,
)
from ai_controller.utils.grasping.m2t2.client import (
    M2T2Client,
)

from ai_controller.utils.grasping.grasping_utils import (
    _depth_to_point_cloud,
    _draw_combined_result,
    _draw_m2t2_candidates,
    _estimate_table_z_offset,
    _get_finger_collision_counts,
    _get_grasp_points,
    _pose_to_transform,
    _project_points,
    _transform_points,
    _transform_poses,
    _save_depth_preview,
    _approach_tilt_deg,
    _wrist_rotation_deg,
)

@dataclass
class GraspPlan:
    """
    Result of task-oriented 6-DoF grasp planning.

    grasp_pose_base transforms points from the selected M2T2 grasp
    frame into base_link.
    """

    grasp_instruction: str

    semantic_point_px: np.ndarray

    grasp_pose_base: np.ndarray
    grasp_position_base: np.ndarray
    grasp_orientation_base: np.ndarray
    approach_direction_base: np.ndarray

    selected_m2t2_index: int
    confidence: float
    semantic_distance_px: float
    approach_tilt_deg: float
    wrist_rotation_deg: float

class GraspPlanner:
    """
    Task-oriented grasp planner combining:

        GraspMolmo semantic 2-D grasp selection
                    +
        M2T2 generic 6-DoF grasp proposals

    The selection policy intentionally mirrors the already validated
    test_grasp_pipeline.py behavior.
    """

    def __init__(
        self,
        m2t2_config_path: str | Path | None = None,
        graspmolmo_config_path: str | Path | None = None,
        num_runs: int = 20,
        seed: int = 42,
        confidence_threshold: float = 0.40,
        semantic_radius_px: float = 50.0,
        max_approach_tilt_deg: float = 45.0,
        max_wrist_rotation_deg: float = 45.0,
        depth_min: float = 0.15,
        depth_max: float = 0.60,
        bottom_ignore_px: int = 30,
        surface_snap: float = 0.0,
        finger_collision_check: bool = True,
        gripper_opening_m: float = 0.085,
        finger_thickness_m: float = 0.031214,
        finger_width_m: float = 0.027000,
        finger_z_min_m: float = 0.0483,
        finger_z_max_m: float = 0.1053,
        collision_margin_m: float = 0.002,
        collision_min_points: int = 3,
    ) -> None:
        self.num_runs = int(
            num_runs
        )

        self.seed = int(
            seed
        )

        self.confidence_threshold = float(
            confidence_threshold
        )

        self.semantic_radius_px = float(
            semantic_radius_px
        )

        self.max_approach_tilt_deg = float(
            max_approach_tilt_deg
        )

        self.max_wrist_rotation_deg = float(
            max_wrist_rotation_deg
        )

        self.depth_min = float(
            depth_min
        )

        self.depth_max = float(
            depth_max
        )

        self.bottom_ignore_px = int(
            bottom_ignore_px
        )

        self.surface_snap = float(
            surface_snap
        )

        self.finger_collision_check = bool(
            finger_collision_check
        )

        self.gripper_opening_m = float(
            gripper_opening_m
        )

        self.finger_thickness_m = float(
            finger_thickness_m
        )

        self.finger_width_m = float(
            finger_width_m
        )

        self.finger_z_min_m = float(
            finger_z_min_m
        )

        self.finger_z_max_m = float(
            finger_z_max_m
        )

        self.collision_margin_m = float(
            collision_margin_m
        )

        self.collision_min_points = int(
            collision_min_points
        )

        self.m2t2_client = M2T2Client(
            config_path=(
                None
                if m2t2_config_path is None
                else str(
                    m2t2_config_path
                )
            )
        )

        self.graspmolmo_client = (
            GraspMolmoClient(
                config_path=(
                    None
                    if graspmolmo_config_path is None
                    else str(
                        graspmolmo_config_path
                    )
                )
            )
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
    ) -> GraspPlan:
        grasp_instruction = str(
            grasp_instruction
        ).strip()

        if not grasp_instruction:
            raise ValueError(
                "grasp_instruction cannot be empty."
            )

        rgb_image = np.asarray(
            rgb_image,
            dtype=np.uint8,
        )

        depth_image = np.asarray(
            depth_image,
            dtype=np.float32,
        )

        K = np.asarray(
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

        if depth_image.ndim != 2:
            raise ValueError(
                "depth_image must have shape (H, W)."
            )

        if K.shape != (3, 3):
            raise ValueError(
                "camera_matrix must have shape (3, 3)."
            )

        if T_base_camera.shape != (4, 4):
            raise ValueError(
                "T_base_camera must have shape (4, 4)."
            )

        if T_base_table.shape != (4, 4):
            raise ValueError(
                "T_base_table must have shape (4, 4)."
            )

        T_base_tcp = _pose_to_transform(
            current_tcp_position,
            current_tcp_orientation,
        )

        artifact_dir = None

        if artifacts_dir is not None:
            artifact_dir = (
                Path(
                    artifacts_dir
                )
                .expanduser()
                .resolve()
            )

            artifact_dir.mkdir(
                parents=True,
                exist_ok=True,
            )

            _save_depth_preview(
                depth_image,
                artifact_dir
                / "input_depth_preview.png",
            )

        # ---------------------------------------------------------
        # Eye-in-hand depth -> camera point cloud
        # ---------------------------------------------------------

        point_cloud_camera = (
            _depth_to_point_cloud(
                depth=depth_image,
                K=K,
                min_depth=self.depth_min,
                max_depth=self.depth_max,
                bottom_ignore_px=(
                    self.bottom_ignore_px
                ),
            )
        )

        if len(point_cloud_camera) == 0:
            raise RuntimeError(
                "Eye-in-hand depth produced an empty point cloud."
            )

        # ---------------------------------------------------------
        # Camera -> table
        # ---------------------------------------------------------

        T_table_base = np.linalg.inv(
            T_base_table
        )

        T_table_camera_raw = (
            T_table_base
            @ T_base_camera
        )

        point_cloud_table_raw = (
            _transform_points(
                point_cloud_camera,
                T_table_camera_raw,
            )
        )

        # Same workspace used by the validated offline test.
        workspace_x_min = -0.45
        workspace_x_max = 0.45
        workspace_y_min = -0.35
        workspace_y_max = 0.30

        table_z_offset = (
            _estimate_table_z_offset(
                point_cloud_table_raw,
                x_min=workspace_x_min,
                x_max=workspace_x_max,
                y_min=workspace_y_min,
                y_max=workspace_y_max,
            )
        )

        # ---------------------------------------------------------
        # Align table surface with Z = 0
        # ---------------------------------------------------------

        T_aligned_table = np.eye(
            4,
            dtype=np.float64,
        )

        T_aligned_table[
            2,
            3,
        ] = -table_z_offset

        T_table_aligned = np.linalg.inv(
            T_aligned_table
        )

        T_aligned_camera = (
            T_aligned_table
            @ T_table_camera_raw
        )

        T_camera_aligned = np.linalg.inv(
            T_aligned_camera
        )

        T_base_aligned = (
            T_base_table
            @ T_table_aligned
        )

        T_aligned_base = np.linalg.inv(
            T_base_aligned
        )

        T_aligned_tcp = (
            T_aligned_base
            @ T_base_tcp
        )

        point_cloud_aligned = (
            _transform_points(
                point_cloud_camera,
                T_aligned_camera,
            )
            .astype(
                np.float32
            )
        )

        # ---------------------------------------------------------
        # Workspace crop for M2T2
        # ---------------------------------------------------------

        workspace_mask = (
            (point_cloud_aligned[:, 0] >= workspace_x_min)
            & (point_cloud_aligned[:, 0] <= workspace_x_max)
            & (point_cloud_aligned[:, 1] >= workspace_y_min)
            & (point_cloud_aligned[:, 1] <= workspace_y_max)
            & (point_cloud_aligned[:, 2] >= -0.03)
            & (point_cloud_aligned[:, 2] <= 0.25)
        )

        point_cloud_m2t2 = (
            point_cloud_aligned[
                workspace_mask
            ]
            .copy()
        )

        if len(point_cloud_m2t2) == 0:
            raise RuntimeError(
                "Workspace crop removed all points."
            )

        if self.surface_snap > 0.0:
            surface_mask = (
                np.abs(
                    point_cloud_m2t2[:, 2]
                )
                < self.surface_snap
            )

            point_cloud_m2t2[
                surface_mask,
                2,
            ] = 0.0

        # ---------------------------------------------------------
        # M2T2
        # ---------------------------------------------------------

        (
            grasps_aligned,
            contacts_aligned,
            confidence,
        ) = self.m2t2_client.predict_grasps(
            point_cloud=point_cloud_m2t2,
            num_runs=self.num_runs,
            seed=self.seed,
        )

        if len(grasps_aligned) == 0:
            raise RuntimeError(
                "M2T2 returned zero grasps."
            )

        grasps_camera = (
            _transform_poses(
                grasps_aligned,
                T_camera_aligned,
            )
        )

        contacts_camera = (
            _transform_points(
                contacts_aligned,
                T_camera_aligned,
            )
            .astype(
                np.float32
            )
        )

        grasps_base = (
            _transform_poses(
                grasps_aligned,
                T_base_aligned,
            )
        )

        if artifact_dir is not None:
            input_image = Image.fromarray(
                rgb_image
            )

            m2t2_image = (
                _draw_m2t2_candidates(
                    image=input_image,
                    contacts_camera=contacts_camera,
                    confidence=confidence,
                    K=K,
                )
            )

            m2t2_image.save(
                artifact_dir
                / "m2t2_candidates.png"
            )

        # ---------------------------------------------------------
        # GraspMolmo semantic point
        # ---------------------------------------------------------

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

        semantic_point = np.asarray(
            semantic_point,
            dtype=np.float32,
        )

        # ---------------------------------------------------------
        # 1. Confidence filter
        # ---------------------------------------------------------

        confidence_mask = (
            confidence
            >= self.confidence_threshold
        )

        candidate_indices = np.flatnonzero(
            confidence_mask
        )

        if len(candidate_indices) == 0:
            raise RuntimeError(
                "No M2T2 candidate survived "
                "the confidence threshold."
            )

        filtered_grasps_camera = (
            grasps_camera[
                candidate_indices
            ]
        )

        filtered_grasps_aligned = (
            grasps_aligned[
                candidate_indices
            ]
        )

        filtered_confidence = (
            confidence[
                candidate_indices
            ]
        )

        # ---------------------------------------------------------
        # 2. Geometric filter
        # ---------------------------------------------------------

        wrist_rotations = np.asarray(
            [
                _wrist_rotation_deg(
                    grasp,
                    T_aligned_tcp,
                )
                for grasp
                in filtered_grasps_aligned
            ],
            dtype=np.float32,
        )

        approach_tilts = np.asarray(
            [
                _approach_tilt_deg(
                    grasp
                )
                for grasp
                in filtered_grasps_aligned
            ],
            dtype=np.float32,
        )

        geometric_mask = (
            (
                approach_tilts
                <= self.max_approach_tilt_deg
            )
            & (
                wrist_rotations
                <= self.max_wrist_rotation_deg
            )
        )

        geometric_local_indices = (
            np.flatnonzero(
                geometric_mask
            )
        )

        if len(
            geometric_local_indices
        ) == 0:
            raise RuntimeError(
                "No M2T2 candidate survived "
                "the geometric constraints."
            )

        # ---------------------------------------------------------
        # 3. Semantic projection
        # ---------------------------------------------------------

        geometric_grasp_points_camera = (
            _get_grasp_points(
                point_cloud_camera,
                filtered_grasps_camera[
                    geometric_local_indices
                ],
            )
        )

        geometric_grasp_pixels = (
            _project_points(
                geometric_grasp_points_camera,
                K,
            )
        )

        grasp_pixels = np.full(
            (
                len(candidate_indices),
                2,
            ),
            np.nan,
            dtype=np.float64,
        )

        grasp_pixels[
            geometric_local_indices
        ] = geometric_grasp_pixels

        geometric_distances = np.linalg.norm(
            geometric_grasp_pixels
            - semantic_point[
                None,
                :
            ],
            axis=1,
        )

        valid_projection = (
            np.isfinite(
                geometric_grasp_pixels
            ).all(
                axis=1
            )
            & (
                geometric_grasp_points_camera[
                    :,
                    2
                ]
                > 0.0
            )
        )

        geometric_distances[
            ~valid_projection
        ] = np.inf

        semantic_local_mask = (
            valid_projection
            & (
                geometric_distances
                <= self.semantic_radius_px
            )
        )

        semantic_geometry_indices = (
            np.flatnonzero(
                semantic_local_mask
            )
        )

        if len(
            semantic_geometry_indices
        ) == 0:
            finite_distances = (
                geometric_distances[
                    np.isfinite(
                        geometric_distances
                    )
                ]
            )

            nearest = (
                float(
                    finite_distances.min()
                )
                if len(finite_distances)
                else float("inf")
            )

            raise RuntimeError(
                "No semantic-compatible M2T2 grasp. "
                f"Nearest candidate is {nearest:.2f} px "
                "from the GraspMolmo point."
            )

        # Convert indexes from the geometric subset back to the
        # confidence-filtered candidate array.
        viable_indices = (
            geometric_local_indices[
                semantic_geometry_indices
            ]
        )

        # ---------------------------------------------------------
        # 4. Robotiq 2F-85 finger collision filtering
        # ---------------------------------------------------------

        semantic_indices = viable_indices

        finger_left_collision_counts = np.zeros(
            len(candidate_indices),
            dtype=np.int32,
        )

        finger_right_collision_counts = np.zeros(
            len(candidate_indices),
            dtype=np.int32,
        )

        collision_free_mask = np.zeros(
            len(candidate_indices),
            dtype=bool,
        )

        if self.finger_collision_check:
            (
                semantic_left_counts,
                semantic_right_counts,
            ) = _get_finger_collision_counts(
                point_cloud=point_cloud_m2t2,
                grasps=filtered_grasps_aligned[
                    semantic_indices
                ],
                opening_m=self.gripper_opening_m,
                finger_thickness_m=(
                    self.finger_thickness_m
                ),
                finger_width_m=(
                    self.finger_width_m
                ),
                finger_z_min_m=(
                    self.finger_z_min_m
                ),
                finger_z_max_m=(
                    self.finger_z_max_m
                ),
                margin_m=(
                    self.collision_margin_m
                ),
            )

            finger_left_collision_counts[
                semantic_indices
            ] = semantic_left_counts

            finger_right_collision_counts[
                semantic_indices
            ] = semantic_right_counts

            semantic_collision_free = (
                (
                    semantic_left_counts
                    < self.collision_min_points
                )
                & (
                    semantic_right_counts
                    < self.collision_min_points
                )
            )

            collision_free_mask[
                semantic_indices
            ] = semantic_collision_free

            collision_free_indices = (
                semantic_indices[
                    semantic_collision_free
                ]
            )

        else:
            collision_free_mask[
                semantic_indices
            ] = True

            collision_free_indices = (
                semantic_indices.copy()
            )

        if len(collision_free_indices) == 0:
            raise RuntimeError(
                "Semantic-compatible M2T2 grasps were found, "
                "but no collision-free grasp remained."
            )

        viable_indices = collision_free_indices

        # ---------------------------------------------------------
        # 4. Ranking
        #
        # Primary:   minimum wrist rotation
        # Secondary: maximum M2T2 confidence
        # Tertiary:  minimum semantic distance
        # ---------------------------------------------------------

        semantic_distances = np.full(
            len(candidate_indices),
            np.inf,
            dtype=np.float64,
        )

        semantic_distances[
            geometric_local_indices
        ] = geometric_distances

        ranking = np.lexsort(
            (
                semantic_distances[
                    viable_indices
                ],
                -filtered_confidence[
                    viable_indices
                ],
                wrist_rotations[
                    viable_indices
                ],
            )
        )

        selected_local_index = int(
            viable_indices[
                ranking[0]
            ]
        )

        selected_original_index = int(
            candidate_indices[
                selected_local_index
            ]
        )

        selected_grasp_camera = (
            grasps_camera[
                selected_original_index
            ]
        )

        selected_grasp_base = np.asarray(
            grasps_base[
                selected_original_index
            ],
            dtype=np.float64,
        )

        selected_position = (
            selected_grasp_base[
                :3,
                3,
            ].copy()
        )

        selected_orientation = (
            Rotation
            .from_matrix(
                selected_grasp_base[
                    :3,
                    :3,
                ]
            )
            .as_quat()
        )

        selected_approach_direction = (
            selected_grasp_base[
                :3,
                2,
            ].copy()
        )

        if artifact_dir is not None:
            combined_image = (
                _draw_combined_result(
                    image=Image.fromarray(
                        rgb_image
                    ),
                    semantic_point=(
                        semantic_point
                    ),
                    grasp_pixels=(
                        grasp_pixels
                    ),
                    selected_index=(
                        selected_local_index
                    ),
                    selected_grasp_camera=(
                        selected_grasp_camera
                    ),
                    K=K,
                    semantic_radius=(
                        self.semantic_radius_px
                    ),
                )
            )

            combined_image.save(
                artifact_dir
                / "combined_candidates.png"
            )

        # ---------------------------------------------------------
        # Result
        # ---------------------------------------------------------

        result = GraspPlan(
            grasp_instruction=(
                grasp_instruction
            ),
            semantic_point_px=(
                semantic_point.copy()
            ),
            grasp_pose_base=(
                selected_grasp_base
            ),
            grasp_position_base=(
                selected_position
            ),
            grasp_orientation_base=(
                selected_orientation
            ),
            approach_direction_base=(
                selected_approach_direction
            ),
            selected_m2t2_index=(
                selected_original_index
            ),
            confidence=float(
                filtered_confidence[
                    selected_local_index
                ]
            ),
            semantic_distance_px=float(
                semantic_distances[
                    selected_local_index
                ]
            ),
            approach_tilt_deg=float(
                approach_tilts[
                    selected_local_index
                ]
            ),
            wrist_rotation_deg=float(
                wrist_rotations[
                    selected_local_index
                ]
            ),
        )

        self._save_artifact(
            result=result,
            artifacts_dir=artifacts_dir,
            table_z_offset=table_z_offset,
            num_m2t2_candidates=(
                len(
                    grasps_aligned
                )
            ),
            num_confidence_candidates=(
                len(
                    candidate_indices
                )
            ),
            num_geometric_candidates=(
                len(
                    geometric_local_indices
                )
            ),
            num_semantic_candidates=(
                len(
                    viable_indices
                )
            ),
            num_collision_free_candidates=(
                len(
                    collision_free_indices
                )
            ),
        )

        return result

    def _save_artifact(
        self,
        result: GraspPlan,
        artifacts_dir: str | Path | None,
        table_z_offset: float,
        num_m2t2_candidates: int,
        num_confidence_candidates: int,
        num_geometric_candidates: int,
        num_semantic_candidates: int,
        num_collision_free_candidates: int,
    ) -> None:
        if artifacts_dir is None:
            return

        output_dir = (
            Path(
                artifacts_dir
            )
            .expanduser()
            .resolve()
        )

        output_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        payload = {
            "grasp_instruction": (
                result.grasp_instruction
            ),
            "semantic_point_px": (
                result.semantic_point_px.tolist()
            ),
            "selected_m2t2_index": (
                result.selected_m2t2_index
            ),
            "confidence": (
                result.confidence
            ),
            "semantic_distance_px": (
                result.semantic_distance_px
            ),
            "approach_tilt_deg": (
                result.approach_tilt_deg
            ),
            "wrist_rotation_deg": (
                result.wrist_rotation_deg
            ),
            "grasp_pose_base": (
                result.grasp_pose_base.tolist()
            ),
            "grasp_position_base": (
                result.grasp_position_base.tolist()
            ),
            "grasp_orientation_base": (
                result.grasp_orientation_base.tolist()
            ),
            "approach_direction_base": (
                result.approach_direction_base.tolist()
            ),
            "table_z_offset_m": float(
                table_z_offset
            ),
            "m2t2_candidates": int(
                num_m2t2_candidates
            ),
            "confidence_candidates": int(
                num_confidence_candidates
            ),
            "geometric_candidates": int(
                num_geometric_candidates
            ),
            "semantic_candidates": int(
                num_semantic_candidates
            ),
            "finger_collision_check": bool(
                self.finger_collision_check
            ),
            "collision_free_candidates": int(
                num_collision_free_candidates
            ),
        }

        with (
            output_dir
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