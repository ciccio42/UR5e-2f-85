from __future__ import annotations

import torch
import numpy as np
import cv2
import json
import base64
import os
import re
from scipy.optimize import least_squares

from collections import Counter
from openai import OpenAI

from pathlib import Path
from typing import Any
from GroundingDINO.groundingdino.util import box_ops
from ai_controller.models.seedo_controller.utils import (
    load_camera_calibration,
    robust_depth_at,
    deproject_pixel,
    camera_point_to_aruco,
    aruco_point_to_table0,
    crop_image_for_grounding_dino,
    remap_grounding_dino_boxes_to_full_frame,
    ARUCO_TO_TABLE0_ROTATION,
    ARUCO_TO_TABLE0_TRANSLATION,
)
from results import (
    DetectedObject,
    RawSceneObject,
    RawSceneState,
    ScenePerceptionResult,
)
from GroundingDINO.groundingdino.util.inference import (
    load_image_from_array,
    predict,
)
from segment_anything import (
    SamPredictor,
    build_sam,
)
from track_objects import (
    GENERALIZED_DISCOVERY_SYSTEM_PROMPT,
    GENERALIZED_DISCOVERY_USER_PROMPT,
    parse_structured_object_list,
    detect_generalized_objects,
    filter_oversized_storage_bin_detections,
    load_groundingdino_model,
)

from ai_controller.models.seedo_controller.timing_utils import TIMING


class ScenePerceiver:
    """Runtime RGB-D perception used by SeeDo.

    The baseline performs perception only once, at timestep 0.

    This first implementation provides the geometric conversion from an
    RGB-image pixel to a 3D point expressed in base_link. Object detection
    and segmentation will be added on top of this component later.
    """

    def __init__(
        self,
        camera_calibration_path: str | Path,
        camera_name: str = "zed_front",
        grounding_config: str | Path | None = None,
        grounding_checkpoint: str | Path | None = None,
        bert_model: str | Path | None = None,
        sam_checkpoint: str | Path | None = None,
        detector_labels: list[str] | tuple[str, ...] | None = None,
        camera_pose_noise_level: str = "baseline",
        translation_noise_std_mm: float = 0.0,
        rotation_noise_std_deg: float = 0.0,
        perception_mode: str = "generalized",
        grounding_dino_crop: dict[str, int] | None = None,
    ) -> None:
        self.camera_calibration_path = (
            Path(camera_calibration_path)
            .expanduser()
            .resolve()
        )

        if not self.camera_calibration_path.is_file():
            raise FileNotFoundError(
                "Camera calibration file does not exist: "
                f"{self.camera_calibration_path}"
            )

        self.camera_name = str(camera_name)

        # Geometry of the most recent runtime perception.
        #
        # Peg-axis refinement is executed only after semantic/task resolution,
        # therefore it must reuse exactly the same camera calibration and
        # base/table transform used by run().
        self._last_runtime_camera_calibration: dict[str, np.ndarray] | None = None
        self._last_base_to_table_rotation: np.ndarray | None = None
        self._last_base_to_table_translation: np.ndarray | None = None

        self.perception_mode = str(
            perception_mode
        ).strip().lower()

        if self.perception_mode not in {
            "prior_guided",
            "generalized",
        }:
            raise ValueError(
                "Invalid perception_mode: "
                f"{self.perception_mode!r}"
            )

        crop_config = grounding_dino_crop or {}

        self.grounding_dino_crop = {
            "top_px": crop_config.get(
                "top_px",
                80,
            ),
            "bottom_px": crop_config.get(
                "bottom_px",
                0,
            ),
            "left_px": crop_config.get(
                "left_px",
                0,
            ),
            "right_px": crop_config.get(
                "right_px",
                0,
            ),
        }
        
        self.camera_calibration = load_camera_calibration(
            str(self.camera_calibration_path)
        )

        if self.camera_name not in self.camera_calibration:
            raise KeyError(
                f"Camera '{self.camera_name}' is not present in "
                f"{self.camera_calibration_path}."
            )

        self.grounding_config = (
            Path(grounding_config).expanduser().resolve()
            if grounding_config is not None
            else None
        )

        self.grounding_checkpoint = (
            Path(grounding_checkpoint).expanduser().resolve()
            if grounding_checkpoint is not None
            else None
        )

        self.bert_model = (
            Path(bert_model).expanduser().resolve()
            if bert_model is not None
            else None
        )

        self.sam_checkpoint = (
            Path(sam_checkpoint).expanduser().resolve()
            if sam_checkpoint is not None
            else None
        )

        required_model_paths = {
            "GroundingDINO config": self.grounding_config,
            "GroundingDINO checkpoint": self.grounding_checkpoint,
            "BERT model": self.bert_model,
            "SAM checkpoint": self.sam_checkpoint,
        }

        for name, path in required_model_paths.items():
            if path is None:
                raise ValueError(
                    f"{name} has not been configured."
                )

            if not path.exists():
                raise FileNotFoundError(
                    f"{name} does not exist: {path}"
                )
        
        if detector_labels is None:
            raise ValueError(
                "At least one detector label must be configured."
            )

        self.detector_labels = tuple(
            str(label).strip()
            for label in detector_labels
            if str(label).strip()
        )

        if not self.detector_labels:
            raise ValueError(
                "At least one non-empty detector label must be configured."
            )

        self.camera_pose_noise_level = str(
            camera_pose_noise_level
        ).strip().lower()

        self.translation_noise_std_mm = float(
            translation_noise_std_mm
        )

        self.rotation_noise_std_deg = float(
            rotation_noise_std_deg
        )

        if self.translation_noise_std_mm < 0.0:
            raise ValueError(
                "translation_noise_std_mm cannot be negative."
            )

        if self.rotation_noise_std_deg < 0.0:
            raise ValueError(
                "rotation_noise_std_deg cannot be negative."
            )

        # A new camera-pose perturbation is sampled once per ScenePerceiver.run().
        self._noise_rng = np.random.default_rng(42)

        self.device = torch.device(
            "cuda:0"
            if torch.cuda.is_available()
            else "cpu"
        )

        if self.device.type == "cuda":
            torch.cuda.set_device(self.device)

        # Models are loaded lazily when runtime scene perception is first used.
        # This avoids keeping a second GroundingDINO + SAM instance on the GPU
        # while the demonstration video is being processed.
        self.grounding_model = None
        self.sam = None
        self.sam_predictor = None

    def _sample_camera_pose_noise(
        self,
    ) -> dict[str, np.ndarray]:
        """Sample one camera extrinsic perturbation for the current run."""

        translation_std_m = (
            self.translation_noise_std_mm / 1000.0
        )

        rotation_std_rad = np.deg2rad(
            self.rotation_noise_std_deg
        )

        delta_translation = self._noise_rng.normal(
            loc=0.0,
            scale=translation_std_m,
            size=3,
        ).astype(np.float64)

        delta_rotation_vector = self._noise_rng.normal(
            loc=0.0,
            scale=rotation_std_rad,
            size=3,
        ).astype(np.float64)

        delta_rotation_matrix, _ = cv2.Rodrigues(
            delta_rotation_vector.reshape(3, 1)
        )

        return {
            "translation_m": delta_translation,
            "rotation_vector_rad": delta_rotation_vector,
            "rotation_matrix": delta_rotation_matrix,
        }


    def _build_noisy_camera_calibration(
        self,
        noise_sample: dict[str, np.ndarray],
    ) -> dict[str, np.ndarray]:
        """Apply the sampled perturbation to the camera->ArUco extrinsic."""

        original = self.camera_calibration[
            self.camera_name
        ]

        original_position = np.asarray(
            original["position"],
            dtype=np.float64,
        )

        original_rotation = np.asarray(
            original["orientation_matrix"],
            dtype=np.float64,
        )

        # Keep baseline exactly on the original calibration path.
        if (
            self.translation_noise_std_mm == 0.0
            and self.rotation_noise_std_deg == 0.0
        ):
            return {
                "position": original_position.copy(),
                "orientation_matrix": original_rotation.copy(),
            }

        noisy_position = (
            original_position
            + noise_sample["translation_m"]
        )

        # The random rotation is expressed in the ArUco/world frame.
        noisy_rotation = (
            noise_sample["rotation_matrix"]
            @ original_rotation
        )

        return {
            "position": noisy_position,
            "orientation_matrix": noisy_rotation,
        }

    def pixel_to_base_link(
        self,
        pixel: tuple[int, int],
        depth_image: np.ndarray,
        camera_matrix: np.ndarray,
        base_to_table_rotation: np.ndarray,
        base_to_table_translation: np.ndarray,
        camera_calibration_override: dict[str, np.ndarray] | None = None,
    ) -> tuple[float, float, float]:
        """Convert an RGB pixel into a 3D point in base_link."""

        u, v = int(pixel[0]), int(pixel[1])

        if depth_image.ndim != 2:
            raise ValueError(
                "Depth image must be a 2D array."
            )

        if camera_matrix.shape != (3, 3):
            raise ValueError(
                "Camera matrix must have shape (3, 3)."
            )

        base_to_table_rotation = np.asarray(
            base_to_table_rotation,
            dtype=np.float64,
        )

        base_to_table_translation = np.asarray(
            base_to_table_translation,
            dtype=np.float64,
        )

        if base_to_table_rotation.shape != (3, 3):
            raise ValueError(
                "base_to_table_rotation must have shape (3, 3)."
            )

        if base_to_table_translation.shape != (3,):
            raise ValueError(
                "base_to_table_translation must have shape (3,)."
            )

        depth = robust_depth_at(
            depth_image,
            u,
            v,
        )

        point_camera = deproject_pixel(
            u,
            v,
            depth,
            camera_matrix,
        )

        if camera_calibration_override is None:
            camera_calibration = self.camera_calibration[
                self.camera_name
            ]
        else:
            camera_calibration = camera_calibration_override

        point_aruco = camera_point_to_aruco(
            point_camera,
            camera_calibration,
        )

        point_table = aruco_point_to_table0(
            point_aruco
        )

        point_base = (
            base_to_table_rotation
            @ np.asarray(point_table, dtype=np.float64)
            + base_to_table_translation
        )

        return (
            float(point_base[0]),
            float(point_base[1]),
            float(point_base[2]),
        )

    def refine_nut_hole(
        self,
        *,
        mask: np.ndarray,
        depth_image: np.ndarray,
        camera_matrix: np.ndarray,
        min_hole_area_px: float = 40.0,
        top_surface_band_m: float = 0.008,
        plane_inlier_threshold_m: float = 0.003,
        min_plane_points: int = 80,
        support_annulus_inner_px: int = 5,
        support_annulus_outer_px: int = 35,
        support_z_tolerance_m: float = 0.008,
        min_support_points: int = 80,
        max_plane_rms_m: float = 0.003,
        max_support_mad_m: float = 0.003,
        min_thickness_m: float = 0.002,
        max_thickness_m: float = 0.080,
        max_points: int = 12000,
    ) -> dict[str, Any]:
        """
        Estimate the center and axis of the internal hole of one
        already-selected ring/nut.

        The method does NOT know the hole diameter.

        The hole is identified topologically from the selected object's
        segmentation mask. Its 3D center is obtained by intersecting the
        image ray through the fitted hole center with the observed top
        surface of the object.

        The object's thickness is estimated from the distance between
        its top surface and the locally observed supporting surface.
        """

        if self._last_runtime_camera_calibration is None:
            raise RuntimeError(
                "Nut-hole refinement requires a preceding "
                "ScenePerceiver.run() call."
            )

        if self._last_base_to_table_rotation is None:
            raise RuntimeError(
                "Missing table-to-base rotation."
            )

        if self._last_base_to_table_translation is None:
            raise RuntimeError(
                "Missing table-to-base translation."
            )

        mask = np.asarray(
            mask,
            dtype=bool,
        )

        depth = np.asarray(
            depth_image,
            dtype=np.float64,
        )

        K = np.asarray(
            camera_matrix,
            dtype=np.float64,
        )

        if mask.ndim != 2:
            raise ValueError(
                "Nut mask must be 2D."
            )

        if depth.ndim != 2:
            raise ValueError(
                "Depth image must be 2D."
            )

        if mask.shape != depth.shape:
            raise ValueError(
                "Nut mask and depth image must have "
                "identical shape."
            )

        if K.shape != (3, 3):
            raise ValueError(
                "camera_matrix must have shape (3, 3)."
            )

        fx = float(K[0, 0])
        fy = float(K[1, 1])
        cx = float(K[0, 2])
        cy = float(K[1, 2])

        if (
            not np.isfinite(fx)
            or not np.isfinite(fy)
            or fx <= 0.0
            or fy <= 0.0
        ):
            raise ValueError(
                "Invalid camera focal lengths."
            )

        if min_hole_area_px <= 0.0:
            raise ValueError(
                "min_hole_area_px must be positive."
            )

        if top_surface_band_m <= 0.0:
            raise ValueError(
                "top_surface_band_m must be positive."
            )

        if plane_inlier_threshold_m <= 0.0:
            raise ValueError(
                "plane_inlier_threshold_m must be positive."
            )

        if min_plane_points < 3:
            raise ValueError(
                "min_plane_points must be at least 3."
            )

        if (
            support_annulus_outer_px
            <= support_annulus_inner_px
        ):
            raise ValueError(
                "Invalid support annulus."
            )

        # ---------------------------------------------------------
        # 1. Find a CLOSED INTERNAL CAVITY in the selected mask.
        #
        # No expected diameter or radius is provided.
        # ---------------------------------------------------------

        mask_uint8 = (
            mask.astype(np.uint8)
            * 255
        )

        contours, hierarchy = cv2.findContours(
            mask_uint8,
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_NONE,
        )

        if hierarchy is None or not contours:
            raise RuntimeError(
                "Nut-hole refinement could not extract "
                "mask contours."
            )

        hierarchy = hierarchy[0]

        outer_candidates = [
            index
            for index, item in enumerate(hierarchy)
            if item[3] == -1
        ]

        if not outer_candidates:
            raise RuntimeError(
                "Nut mask contains no external contour."
            )

        outer_index = max(
            outer_candidates,
            key=lambda index: cv2.contourArea(
                contours[index]
            ),
        )

        hole_candidates: list[
            tuple[int, float]
        ] = []

        for index, item in enumerate(hierarchy):

            parent_index = int(
                item[3]
            )

            if parent_index != outer_index:
                continue

            contour_area = float(
                cv2.contourArea(
                    contours[index]
                )
            )

            if contour_area < min_hole_area_px:
                continue

            if len(contours[index]) < 5:
                continue

            hole_candidates.append(
                (
                    index,
                    contour_area,
                )
            )

        if not hole_candidates:
            raise RuntimeError(
                "The selected nut/ring mask does not contain "
                "a sufficiently large closed internal cavity. "
                "No hole diameter prior is used; the hole must be "
                "observable in the segmentation mask."
            )

        hole_index, hole_area_px = max(
            hole_candidates,
            key=lambda item: item[1],
        )

        hole_contour = contours[
            hole_index
        ]

        ellipse = cv2.fitEllipse(
            hole_contour
        )

        (
            (hole_u, hole_v),
            (ellipse_width_px, ellipse_height_px),
            ellipse_angle_deg,
        ) = ellipse

        if (
            not np.isfinite(hole_u)
            or not np.isfinite(hole_v)
        ):
            raise RuntimeError(
                "Invalid fitted hole center."
            )

        if (
            ellipse_width_px <= 0.0
            or ellipse_height_px <= 0.0
        ):
            raise RuntimeError(
                "Invalid fitted hole ellipse."
            )

        ellipse_area_px = float(
            np.pi
            * 0.25
            * ellipse_width_px
            * ellipse_height_px
        )

        hole_fit_fill_ratio = (
            hole_area_px
            / ellipse_area_px
            if ellipse_area_px > 1e-9
            else 0.0
        )

        if not (
            0.40
            <= hole_fit_fill_ratio
            <= 1.60
        ):
            raise RuntimeError(
                "Internal cavity is poorly represented by an "
                "ellipse: "
                f"fill_ratio={hole_fit_fill_ratio:.3f}."
            )

        # ---------------------------------------------------------
        # Runtime camera geometry.
        # ---------------------------------------------------------

        calibration = (
            self._last_runtime_camera_calibration
        )

        R_camera_aruco = np.asarray(
            calibration["orientation_matrix"],
            dtype=np.float64,
        )

        t_camera_aruco = np.asarray(
            calibration["position"],
            dtype=np.float64,
        )

        R_camera_table = (
            ARUCO_TO_TABLE0_ROTATION
            @ R_camera_aruco
        )

        camera_origin_table = (
            ARUCO_TO_TABLE0_ROTATION
            @ t_camera_aruco
            + ARUCO_TO_TABLE0_TRANSLATION
        )

        # ---------------------------------------------------------
        # 2. Build the point cloud belonging to the ring/nut.
        # ---------------------------------------------------------

        valid_object = (
            mask
            & np.isfinite(depth)
            & (depth > 0.0)
        )

        object_indices = np.flatnonzero(
            valid_object
        )

        if len(object_indices) < min_plane_points:
            raise RuntimeError(
                "Too few valid depth points on the nut/ring: "
                f"{len(object_indices)}."
            )

        if len(object_indices) > max_points:

            sample_ids = np.linspace(
                0,
                len(object_indices) - 1,
                max_points,
                dtype=np.int64,
            )

            object_indices = (
                object_indices[
                    sample_ids
                ]
            )

        object_v, object_u = np.unravel_index(
            object_indices,
            depth.shape,
        )

        z_camera = depth[
            object_v,
            object_u,
        ]

        x_camera = (
            (
                object_u.astype(np.float64)
                - cx
            )
            * z_camera
            / fx
        )

        y_camera = (
            (
                object_v.astype(np.float64)
                - cy
            )
            * z_camera
            / fy
        )

        points_camera = np.column_stack(
            (
                x_camera,
                y_camera,
                z_camera,
            )
        )

        points_aruco = (
            points_camera
            @ R_camera_aruco.T
            + t_camera_aruco
        )

        points_table = (
            points_aruco
            @ ARUCO_TO_TABLE0_ROTATION.T
            + ARUCO_TO_TABLE0_TRANSLATION
        )

        # ---------------------------------------------------------
        # 3. Select the upper surface.
        #
        # Vertical edges and lower surfaces are excluded by taking
        # only a thin band near the upper depth population.
        # ---------------------------------------------------------

        object_z = (
            points_table[:, 2]
        )

        top_seed_z = float(
            np.percentile(
                object_z,
                90.0,
            )
        )

        top_band_mask = (
            object_z
            >= (
                top_seed_z
                - top_surface_band_m
            )
        )

        top_points = points_table[
            top_band_mask
        ]

        if len(top_points) < min_plane_points:
            raise RuntimeError(
                "Too few points on the estimated upper "
                "nut/ring surface."
            )

        def fit_plane(
            points: np.ndarray,
        ) -> tuple[np.ndarray, np.ndarray]:

            centroid = np.mean(
                points,
                axis=0,
            )

            centered = (
                points
                - centroid
            )

            _, _, vh = np.linalg.svd(
                centered,
                full_matrices=False,
            )

            normal = np.asarray(
                vh[-1],
                dtype=np.float64,
            )

            norm = float(
                np.linalg.norm(
                    normal
                )
            )

            if norm <= 1e-12:
                raise RuntimeError(
                    "Degenerate plane fit."
                )

            normal /= norm

            return (
                centroid,
                normal,
            )

        (
            top_plane_point,
            top_plane_normal,
        ) = fit_plane(
            top_points
        )

        # The top-face normal must point away from the support surface.
        if top_plane_normal[2] < 0.0:
            top_plane_normal = (
                -top_plane_normal
            )

        top_distances = np.abs(
            (
                points_table
                - top_plane_point
            )
            @ top_plane_normal
        )

        plane_inliers = (
            top_distances
            <= plane_inlier_threshold_m
        )

        inlier_points = points_table[
            plane_inliers
        ]

        if len(inlier_points) < min_plane_points:
            raise RuntimeError(
                "Too few inliers for the nut/ring "
                "upper-surface plane."
            )

        (
            top_plane_point,
            top_plane_normal,
        ) = fit_plane(
            inlier_points
        )

        if top_plane_normal[2] < 0.0:
            top_plane_normal = (
                -top_plane_normal
            )

        final_plane_residuals = (
            (
                inlier_points
                - top_plane_point
            )
            @ top_plane_normal
        )

        top_plane_rms_m = float(
            np.sqrt(
                np.mean(
                    final_plane_residuals ** 2
                )
            )
        )

        if top_plane_rms_m > max_plane_rms_m:
            raise RuntimeError(
                "Nut/ring top-surface plane is too noisy: "
                f"RMS={top_plane_rms_m * 1000.0:.2f} mm."
            )

        # ---------------------------------------------------------
        # 4. Ray through the fitted image-space hole center.
        #
        # Its intersection with the observed upper surface gives
        # the 3D center of the hole.
        # ---------------------------------------------------------

        ray_camera = np.array(
            [
                (hole_u - cx) / fx,
                (hole_v - cy) / fy,
                1.0,
            ],
            dtype=np.float64,
        )

        ray_camera /= np.linalg.norm(
            ray_camera
        )

        ray_table = (
            R_camera_table
            @ ray_camera
        )

        ray_table /= np.linalg.norm(
            ray_table
        )

        ray_plane_denominator = float(
            np.dot(
                top_plane_normal,
                ray_table,
            )
        )

        if (
            abs(ray_plane_denominator)
            <= 1e-8
        ):
            raise RuntimeError(
                "Hole-center camera ray is nearly parallel "
                "to the nut surface."
            )

        ray_distance = float(
            np.dot(
                top_plane_normal,
                (
                    top_plane_point
                    - camera_origin_table
                ),
            )
            / ray_plane_denominator
        )

        if ray_distance <= 0.0:
            raise RuntimeError(
                "Estimated nut-hole center lies behind "
                "the runtime camera."
            )

        hole_center_table = (
            camera_origin_table
            + ray_distance
            * ray_table
        )

        object_distances_from_hole = np.linalg.norm(
            points_table
            - hole_center_table.reshape(1, 3),
            axis=1,
        )

        object_radius_from_hole_m = float(
            np.percentile(
                object_distances_from_hole,
                99.5,
            )
        )

        if (
            not np.isfinite(
                object_radius_from_hole_m
            )
            or object_radius_from_hole_m <= 0.005
            or object_radius_from_hole_m >= 0.30
        ):
            raise RuntimeError(
                "Invalid observed nut/ring extent around "
                "the hole center: "
                f"{object_radius_from_hole_m:.4f} m."
            )

        # ---------------------------------------------------------
        # 5. Estimate the local supporting surface.
        # ---------------------------------------------------------

        outer_kernel_size = (
            2 * support_annulus_outer_px
            + 1
        )

        outer_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (
                outer_kernel_size,
                outer_kernel_size,
            ),
        )

        outer_region = (
            cv2.dilate(
                mask.astype(np.uint8),
                outer_kernel,
            )
            > 0
        )

        if support_annulus_inner_px > 0:

            inner_kernel_size = (
                2 * support_annulus_inner_px
                + 1
            )

            inner_kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                (
                    inner_kernel_size,
                    inner_kernel_size,
                ),
            )

            inner_region = (
                cv2.dilate(
                    mask.astype(np.uint8),
                    inner_kernel,
                )
                > 0
            )

        else:
            inner_region = mask.copy()

        support_region = (
            outer_region
            & ~inner_region
        )

        support_valid = (
            support_region
            & np.isfinite(depth)
            & (depth > 0.0)
        )

        support_indices = np.flatnonzero(
            support_valid
        )

        if len(support_indices) < min_support_points:
            raise RuntimeError(
                "Too few depth points around the nut/ring "
                "to estimate the supporting surface."
            )

        if len(support_indices) > max_points:

            sample_ids = np.linspace(
                0,
                len(support_indices) - 1,
                max_points,
                dtype=np.int64,
            )

            support_indices = (
                support_indices[
                    sample_ids
                ]
            )

        support_v, support_u = np.unravel_index(
            support_indices,
            depth.shape,
        )

        support_z_camera = depth[
            support_v,
            support_u,
        ]

        support_x_camera = (
            (
                support_u.astype(np.float64)
                - cx
            )
            * support_z_camera
            / fx
        )

        support_y_camera = (
            (
                support_v.astype(np.float64)
                - cy
            )
            * support_z_camera
            / fy
        )

        support_points_camera = np.column_stack(
            (
                support_x_camera,
                support_y_camera,
                support_z_camera,
            )
        )

        support_points_aruco = (
            support_points_camera
            @ R_camera_aruco.T
            + t_camera_aruco
        )

        support_points_table = (
            support_points_aruco
            @ ARUCO_TO_TABLE0_ROTATION.T
            + ARUCO_TO_TABLE0_TRANSLATION
        )

        support_z = (
            support_points_table[:, 2]
        )

        support_seed_z = float(
            np.percentile(
                support_z,
                20.0,
            )
        )

        support_inliers = (
            np.abs(
                support_z
                - support_seed_z
            )
            <= support_z_tolerance_m
        )

        support_inlier_points = (
            support_points_table[
                support_inliers
            ]
        )

        if (
            len(support_inlier_points)
            < min_support_points
        ):
            raise RuntimeError(
                "Could not obtain enough support-surface "
                "inliers around the nut/ring."
            )

        support_median_z = float(
            np.median(
                support_inlier_points[:, 2]
            )
        )

        support_mad_m = float(
            np.median(
                np.abs(
                    support_inlier_points[:, 2]
                    - support_median_z
                )
            )
        )

        if support_mad_m > max_support_mad_m:
            raise RuntimeError(
                "Local nut support surface is too noisy: "
                f"MAD={support_mad_m * 1000.0:.2f} mm."
            )

        (
            support_plane_point,
            support_plane_normal,
        ) = fit_plane(
            support_inlier_points
        )

        if support_plane_normal[2] < 0.0:
            support_plane_normal = (
                -support_plane_normal
            )

        # ---------------------------------------------------------
        # 6. Intersect the hole axis with the support plane.
        #
        # This gives the bottom-center projection and therefore an
        # observed object thickness. Again: no thickness prior.
        # ---------------------------------------------------------

        thickness_denominator = float(
            np.dot(
                support_plane_normal,
                top_plane_normal,
            )
        )

        if abs(thickness_denominator) <= 1e-6:
            raise RuntimeError(
                "Nut top plane and support plane geometry "
                "are inconsistent."
            )

        thickness_m = float(
            np.dot(
                support_plane_normal,
                (
                    hole_center_table
                    - support_plane_point
                ),
            )
            / thickness_denominator
        )

        if not (
            min_thickness_m
            <= thickness_m
            <= max_thickness_m
        ):
            raise RuntimeError(
                "Observed nut/ring thickness is outside "
                "the admissible sanity interval: "
                f"{thickness_m * 1000.0:.2f} mm."
            )

        hole_bottom_center_table = (
            hole_center_table
            - thickness_m
            * top_plane_normal
        )

        # ---------------------------------------------------------
        # 7. table_0 -> base_link.
        # ---------------------------------------------------------

        R_table_base = np.asarray(
            self._last_base_to_table_rotation,
            dtype=np.float64,
        )

        t_table_base = np.asarray(
            self._last_base_to_table_translation,
            dtype=np.float64,
        )

        hole_center_base = (
            R_table_base
            @ hole_center_table
            + t_table_base
        )

        hole_bottom_center_base = (
            R_table_base
            @ hole_bottom_center_table
            + t_table_base
        )

        hole_axis_base = (
            R_table_base
            @ top_plane_normal
        )

        hole_axis_base /= np.linalg.norm(
            hole_axis_base
        )

        print(
            "[ScenePerceiver] Nut-hole refinement | "
            f"center_base="
            f"[{hole_center_base[0]:.5f}, "
            f"{hole_center_base[1]:.5f}, "
            f"{hole_center_base[2]:.5f}] | "
            f"thickness="
            f"{thickness_m * 1000.0:.2f} mm | "
            f"plane_rms="
            f"{top_plane_rms_m * 1000.0:.2f} mm | "
            f"hole_area={hole_area_px:.1f} px | "
            f"ellipse="
            f"({ellipse_width_px:.1f}, "
            f"{ellipse_height_px:.1f}) px",
            flush=True,
        )

        return {
            "accepted": True,

            "hole_center_px": (
                float(hole_u),
                float(hole_v),
            ),

            "hole_center_base": (
                float(hole_center_base[0]),
                float(hole_center_base[1]),
                float(hole_center_base[2]),
            ),

            "hole_bottom_center_base": (
                float(hole_bottom_center_base[0]),
                float(hole_bottom_center_base[1]),
                float(hole_bottom_center_base[2]),
            ),

            "hole_axis_base": (
                float(hole_axis_base[0]),
                float(hole_axis_base[1]),
                float(hole_axis_base[2]),
            ),

            "nut_thickness_estimate_m": float(
                thickness_m
            ),

            "hole_area_px": float(
                hole_area_px
            ),

            "hole_ellipse_axes_px": (
                float(ellipse_width_px),
                float(ellipse_height_px),
            ),

            "hole_ellipse_angle_deg": float(
                ellipse_angle_deg
            ),

            "hole_fit_fill_ratio": float(
                hole_fit_fill_ratio
            ),

            "object_radius_from_hole_m": float(
                object_radius_from_hole_m
            ),

            "top_plane_rms_m": float(
                top_plane_rms_m
            ),

            "support_mad_m": float(
                support_mad_m
            ),

            "top_plane_point_table": (
                float(top_plane_point[0]),
                float(top_plane_point[1]),
                float(top_plane_point[2]),
            ),

            "top_plane_normal_table": (
                float(top_plane_normal[0]),
                float(top_plane_normal[1]),
                float(top_plane_normal[2]),
            ),
        }

    def refine_peg_axis(
        self,
        *,
        mask: np.ndarray,
        depth_image: np.ndarray,
        camera_matrix: np.ndarray,
        peg_radius_m: float = 0.015,
        radial_tolerance_m: float = 0.003,
        min_points: int = 70,
        min_inlier_fraction: float = 0.65,
        max_rms_m: float = 0.0025,
        min_arc_coverage_deg: float = 60.0,
        top_exclusion_m: float = 0.015,
        side_band_height_m: float = 0.080,
        max_points: int = 12000,
        support_annulus_inner_px: int = 5,
        support_annulus_outer_px: int = 30,
        support_z_tolerance_m: float = 0.008,
        min_support_points: int = 80,
        max_support_mad_m: float = 0.003,
    ) -> dict[str, Any]:
        """
        Estimate the axis of one already-selected vertical peg.

        This method does NOT detect peg objects and does NOT decide
        which peg is the assembly target.

        The caller must provide the SAM mask of the specific object
        that has already been selected by the semantic/task pipeline.

        Geometry is estimated in table_0, where the peg is modeled
        as a vertical cylinder with known radius. The resulting axis
        is then transformed to base_link.

        The method is intentionally independent from ScenePerceiver.run():
        it only refines the geometry of an object that has already
        been semantically resolved.
        """

        # ---------------------------------------------------------
        # Runtime geometry must already exist.
        # ---------------------------------------------------------

        if self._last_runtime_camera_calibration is None:
            raise RuntimeError(
                "Peg-axis refinement requires a preceding "
                "ScenePerceiver.run() call."
            )

        if self._last_base_to_table_rotation is None:
            raise RuntimeError(
                "Missing base-to-table rotation from runtime perception."
            )

        if self._last_base_to_table_translation is None:
            raise RuntimeError(
                "Missing base-to-table translation from runtime perception."
            )

        # ---------------------------------------------------------
        # Input validation.
        # ---------------------------------------------------------

        mask = np.asarray(
            mask,
            dtype=bool,
        )

        depth = np.asarray(
            depth_image,
            dtype=np.float64,
        )

        K = np.asarray(
            camera_matrix,
            dtype=np.float64,
        )

        if mask.ndim != 2:
            raise ValueError(
                f"Peg mask must be 2D, received shape {mask.shape}."
            )

        if depth.ndim != 2:
            raise ValueError(
                f"Depth image must be 2D, received shape {depth.shape}."
            )

        if mask.shape != depth.shape:
            raise ValueError(
                "Peg mask and depth image must have identical shape."
            )

        if K.shape != (3, 3):
            raise ValueError(
                "camera_matrix must have shape (3, 3)."
            )

        if not np.all(np.isfinite(K)):
            raise ValueError(
                "camera_matrix contains non-finite values."
            )

        if (
            not np.isfinite(peg_radius_m)
            or peg_radius_m <= 0.0
        ):
            raise ValueError(
                "peg_radius_m must be finite and positive."
            )

        if (
            not np.isfinite(radial_tolerance_m)
            or radial_tolerance_m <= 0.0
        ):
            raise ValueError(
                "radial_tolerance_m must be finite and positive."
            )

        if min_points < 3:
            raise ValueError(
                "min_points must be at least 3."
            )

        if (
            min_inlier_fraction < 0.0
            or min_inlier_fraction > 1.0
        ):
            raise ValueError(
                "min_inlier_fraction must be in [0, 1]."
            )

        if max_rms_m <= 0.0:
            raise ValueError(
                "max_rms_m must be positive."
            )

        if (
            min_arc_coverage_deg < 0.0
            or min_arc_coverage_deg > 360.0
        ):
            raise ValueError(
                "min_arc_coverage_deg must be in [0, 360]."
            )

        if top_exclusion_m < 0.0:
            raise ValueError(
                "top_exclusion_m cannot be negative."
            )

        if side_band_height_m <= 0.0:
            raise ValueError(
                "side_band_height_m must be positive."
            )

        if max_points < min_points:
            raise ValueError(
                "max_points cannot be smaller than min_points."
            )

        if support_annulus_inner_px < 0:
            raise ValueError(
                "support_annulus_inner_px cannot be negative."
            )

        if (
            support_annulus_outer_px
            <= support_annulus_inner_px
        ):
            raise ValueError(
                "support_annulus_outer_px must be greater than "
                "support_annulus_inner_px."
            )

        if support_z_tolerance_m <= 0.0:
            raise ValueError(
                "support_z_tolerance_m must be positive."
            )

        if min_support_points < 3:
            raise ValueError(
                "min_support_points must be at least 3."
            )

        if max_support_mad_m <= 0.0:
            raise ValueError(
                "max_support_mad_m must be positive."
            )

        fx = float(K[0, 0])
        fy = float(K[1, 1])
        cx = float(K[0, 2])
        cy = float(K[1, 2])

        if (
            not np.isfinite(fx)
            or not np.isfinite(fy)
            or fx <= 0.0
            or fy <= 0.0
        ):
            raise ValueError(
                "Camera focal lengths must be finite and positive."
            )

        # ---------------------------------------------------------
        # Extract valid depth samples belonging to the selected peg.
        # ---------------------------------------------------------

        valid = (
            mask
            & np.isfinite(depth)
            & (depth > 0.0)
        )

        flat_indices = np.flatnonzero(
            valid
        )

        total_valid_points = int(
            len(flat_indices)
        )

        if total_valid_points < min_points:
            raise RuntimeError(
                "Peg-axis refinement has too few valid depth points: "
                f"{total_valid_points} < {min_points}."
            )

        # Deterministic subsampling.
        #
        # We deliberately avoid random sampling so that real/offline
        # diagnostics remain reproducible.
        if len(flat_indices) > max_points:
            sample_ids = np.linspace(
                0,
                len(flat_indices) - 1,
                max_points,
                dtype=np.int64,
            )

            flat_indices = flat_indices[
                sample_ids
            ]

        v, u = np.unravel_index(
            flat_indices,
            depth.shape,
        )

        z_camera = depth[
            v,
            u,
        ]

        # ---------------------------------------------------------
        # Vectorized deprojection: image -> camera optical frame.
        # ---------------------------------------------------------

        x_camera = (
            (u.astype(np.float64) - cx)
            * z_camera
            / fx
        )

        y_camera = (
            (v.astype(np.float64) - cy)
            * z_camera
            / fy
        )

        points_camera = np.column_stack(
            (
                x_camera,
                y_camera,
                z_camera,
            )
        )

        # ---------------------------------------------------------
        # Camera -> raw ArUco frame.
        # ---------------------------------------------------------

        runtime_calibration = (
            self._last_runtime_camera_calibration
        )

        camera_to_aruco_rotation = np.asarray(
            runtime_calibration[
                "orientation_matrix"
            ],
            dtype=np.float64,
        )

        camera_to_aruco_translation = np.asarray(
            runtime_calibration[
                "position"
            ],
            dtype=np.float64,
        )

        points_aruco = (
            points_camera
            @ camera_to_aruco_rotation.T
            + camera_to_aruco_translation
        )

        # ---------------------------------------------------------
        # Raw ArUco frame -> table_0.
        #
        # Fit the cylinder in this frame because its physical axis is
        # assumed to be parallel to table_0 Z.
        # ---------------------------------------------------------

        points_table = (
            points_aruco
            @ ARUCO_TO_TABLE0_ROTATION.T
            + ARUCO_TO_TABLE0_TRANSLATION
        )

        # ---------------------------------------------------------
        # Keep an upper lateral band but remove the top face.
        #
        # A robust percentile is preferable to max(Z), because one
        # erroneous depth sample must not define the top of the peg.
        # ---------------------------------------------------------

        z_table = points_table[
            :,
            2,
        ]

        z_top_table = float(
            np.percentile(
                z_table,
                95.0,
            )
        )

        # ---------------------------------------------------------
        # Estimate the local supporting surface around the peg.
        #
        # Do NOT assume that table_0 z=0 corresponds exactly to
        # the physical support surface. Instead, use depth pixels
        # surrounding the selected peg mask.
        #
        # The annulus deliberately excludes the peg itself.
        # ---------------------------------------------------------

        mask_uint8 = mask.astype(
            np.uint8
        )

        outer_kernel_size = (
            2 * support_annulus_outer_px + 1
        )

        outer_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (
                outer_kernel_size,
                outer_kernel_size,
            ),
        )

        outer_region = (
            cv2.dilate(
                mask_uint8,
                outer_kernel,
            )
            > 0
        )

        if support_annulus_inner_px > 0:

            inner_kernel_size = (
                2 * support_annulus_inner_px + 1
            )

            inner_kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                (
                    inner_kernel_size,
                    inner_kernel_size,
                ),
            )

            inner_region = (
                cv2.dilate(
                    mask_uint8,
                    inner_kernel,
                )
                > 0
            )

        else:
            inner_region = mask.copy()

        support_region = (
            outer_region
            & ~inner_region
        )

        support_valid = (
            support_region
            & np.isfinite(depth)
            & (depth > 0.0)
        )

        support_flat_indices = np.flatnonzero(
            support_valid
        )

        if len(support_flat_indices) < min_support_points:
            raise RuntimeError(
                "Peg-axis refinement has too few local "
                "support-surface depth points: "
                f"{len(support_flat_indices)} "
                f"< {min_support_points}."
            )

        if len(support_flat_indices) > max_points:

            support_sample_ids = np.linspace(
                0,
                len(support_flat_indices) - 1,
                max_points,
                dtype=np.int64,
            )

            support_flat_indices = (
                support_flat_indices[
                    support_sample_ids
                ]
            )

        support_v, support_u = np.unravel_index(
            support_flat_indices,
            depth.shape,
        )

        support_z_camera = depth[
            support_v,
            support_u,
        ]

        support_x_camera = (
            (
                support_u.astype(np.float64)
                - cx
            )
            * support_z_camera
            / fx
        )

        support_y_camera = (
            (
                support_v.astype(np.float64)
                - cy
            )
            * support_z_camera
            / fy
        )

        support_points_camera = np.column_stack(
            (
                support_x_camera,
                support_y_camera,
                support_z_camera,
            )
        )

        support_points_aruco = (
            support_points_camera
            @ camera_to_aruco_rotation.T
            + camera_to_aruco_translation
        )

        support_points_table = (
            support_points_aruco
            @ ARUCO_TO_TABLE0_ROTATION.T
            + ARUCO_TO_TABLE0_TRANSLATION
        )

        support_z_candidates = (
            support_points_table[:, 2]
        )

        # The support surface is normally the lowest dominant
        # surface around a vertical peg. Starting from a low
        # percentile prevents neighboring elevated objects from
        # pulling the estimate upward.
        support_seed_z = float(
            np.percentile(
                support_z_candidates,
                20.0,
            )
        )

        support_inliers = (
            np.abs(
                support_z_candidates
                - support_seed_z
            )
            <= support_z_tolerance_m
        )

        support_inlier_count = int(
            np.count_nonzero(
                support_inliers
            )
        )

        if (
            support_inlier_count
            < min_support_points
        ):
            raise RuntimeError(
                "Could not identify enough points belonging "
                "to the local support surface: "
                f"{support_inlier_count} "
                f"< {min_support_points}."
            )

        support_inlier_z = (
            support_z_candidates[
                support_inliers
            ]
        )

        support_z_table = float(
            np.median(
                support_inlier_z
            )
        )

        support_mad_m = float(
            np.median(
                np.abs(
                    support_inlier_z
                    - support_z_table
                )
            )
        )

        if support_mad_m > max_support_mad_m:
            raise RuntimeError(
                "Local support-surface estimate is too noisy: "
                f"MAD={support_mad_m * 1000.0:.2f} mm."
            )

        peg_height_estimate_m = (
            z_top_table
            - support_z_table
        )

        if (
            not np.isfinite(
                peg_height_estimate_m
            )
            or peg_height_estimate_m <= 0.01
            or peg_height_estimate_m >= 0.50
        ):
            raise RuntimeError(
                "Invalid runtime peg-height estimate: "
                f"{peg_height_estimate_m:.4f} m."
            )

        side_upper_z = (
            z_top_table
            - top_exclusion_m
        )

        side_lower_z = (
            side_upper_z
            - side_band_height_m
        )

        side_mask = (
            (z_table <= side_upper_z)
            & (z_table >= side_lower_z)
        )

        side_points_table = points_table[
            side_mask
        ]

        side_point_count = int(
            len(side_points_table)
        )

        if side_point_count < min_points:
            raise RuntimeError(
                "Peg-axis refinement has too few points after "
                "top-face exclusion and side-band selection: "
                f"{side_point_count} < {min_points}."
            )

        xy = side_points_table[
            :,
            :2,
        ]

        # ---------------------------------------------------------
        # Circle fitting.
        #
        # Radius is fixed; only center (cx, cy) is estimated.
        #
        # residual_i =
        #
        #   ||point_i - center|| - peg_radius
        #
        # We use several deterministic initializations because only
        # one visible arc of the cylinder may be available.
        # ---------------------------------------------------------

        def radial_residuals(
            center_xy: np.ndarray,
            points_xy: np.ndarray,
        ) -> np.ndarray:
            distances = np.linalg.norm(
                points_xy
                - center_xy.reshape(1, 2),
                axis=1,
            )

            return (
                distances
                - peg_radius_m
            )

        mean_xy = np.mean(
            xy,
            axis=0,
        )

        median_xy = np.median(
            xy,
            axis=0,
        )

        initial_centers: list[np.ndarray] = [
            mean_xy,
            median_xy,
        ]

        # ---------------------------------------------------------
        # Camera-aware initialization.
        #
        # The visible surface of the peg normally lies on the
        # camera-facing side. Moving approximately one peg radius
        # away from the camera gives a useful center hypothesis.
        # ---------------------------------------------------------

        camera_origin_aruco = (
            camera_to_aruco_translation
        )

        camera_origin_table = (
            ARUCO_TO_TABLE0_ROTATION
            @ camera_origin_aruco
            + ARUCO_TO_TABLE0_TRANSLATION
        )

        camera_to_surface_xy = (
            mean_xy
            - camera_origin_table[:2]
        )

        camera_direction_norm = float(
            np.linalg.norm(
                camera_to_surface_xy
            )
        )

        if camera_direction_norm > 1e-9:
            camera_direction_xy = (
                camera_to_surface_xy
                / camera_direction_norm
            )

            initial_centers.append(
                mean_xy
                + peg_radius_m
                * camera_direction_xy
            )

        # ---------------------------------------------------------
        # Algebraic circle initialization.
        #
        # This is used only as another starting point. The final fit
        # still enforces peg_radius_m explicitly.
        # ---------------------------------------------------------

        x = xy[:, 0]
        y = xy[:, 1]

        algebraic_A = np.column_stack(
            (
                2.0 * x,
                2.0 * y,
                np.ones_like(x),
            )
        )

        algebraic_b = (
            x * x
            + y * y
        )

        try:
            algebraic_solution, _, _, _ = (
                np.linalg.lstsq(
                    algebraic_A,
                    algebraic_b,
                    rcond=None,
                )
            )

            algebraic_center = np.asarray(
                algebraic_solution[:2],
                dtype=np.float64,
            )

            if np.all(
                np.isfinite(
                    algebraic_center
                )
            ):
                initial_centers.append(
                    algebraic_center
                )

        except np.linalg.LinAlgError:
            pass

        # ---------------------------------------------------------
        # Robust nonlinear fit from each initialization.
        # ---------------------------------------------------------

        best_fit = None
        best_cost = np.inf

        for initial_center in initial_centers:

            initial_center = np.asarray(
                initial_center,
                dtype=np.float64,
            )

            if (
                initial_center.shape != (2,)
                or not np.all(
                    np.isfinite(
                        initial_center
                    )
                )
            ):
                continue

            fit = least_squares(
                radial_residuals,
                x0=initial_center,
                args=(xy,),
                loss="soft_l1",
                f_scale=radial_tolerance_m,
                max_nfev=300,
            )

            if not np.all(
                np.isfinite(
                    fit.x
                )
            ):
                continue

            if float(fit.cost) < best_cost:
                best_cost = float(
                    fit.cost
                )

                best_fit = fit

        if best_fit is None:
            raise RuntimeError(
                "Peg-axis circle fitting failed for every initialization."
            )

        center_xy_table = np.asarray(
            best_fit.x,
            dtype=np.float64,
        )

        # ---------------------------------------------------------
        # Radial inliers.
        # ---------------------------------------------------------

        residuals = radial_residuals(
            center_xy_table,
            xy,
        )

        radial_inliers = (
            np.abs(residuals)
            <= radial_tolerance_m
        )

        initial_inlier_count = int(
            np.count_nonzero(
                radial_inliers
            )
        )

        # Refit once using only points compatible with the expected
        # cylindrical surface.
        if initial_inlier_count >= 3:

            refined_fit = least_squares(
                radial_residuals,
                x0=center_xy_table,
                args=(xy[radial_inliers],),
                loss="soft_l1",
                f_scale=radial_tolerance_m,
                max_nfev=200,
            )

            if np.all(
                np.isfinite(
                    refined_fit.x
                )
            ):
                center_xy_table = np.asarray(
                    refined_fit.x,
                    dtype=np.float64,
                )

                residuals = radial_residuals(
                    center_xy_table,
                    xy,
                )

                radial_inliers = (
                    np.abs(residuals)
                    <= radial_tolerance_m
                )

        inlier_count = int(
            np.count_nonzero(
                radial_inliers
            )
        )

        inlier_fraction = (
            float(inlier_count)
            / float(side_point_count)
        )

        # ---------------------------------------------------------
        # Fit error.
        # ---------------------------------------------------------

        if inlier_count > 0:

            inlier_residuals = residuals[
                radial_inliers
            ]

            rms_m = float(
                np.sqrt(
                    np.mean(
                        inlier_residuals ** 2
                    )
                )
            )

            median_abs_residual_m = float(
                np.median(
                    np.abs(
                        inlier_residuals
                    )
                )
            )

        else:
            rms_m = float("inf")
            median_abs_residual_m = float(
                "inf"
            )

        # ---------------------------------------------------------
        # Angular coverage of the observed cylinder surface.
        #
        # Coverage is:
        #
        #   360 deg - largest empty angular gap
        #
        # so a visible 90-degree cylindrical arc reports roughly
        # 90 degrees rather than 270.
        # ---------------------------------------------------------

        if inlier_count >= 2:

            inlier_xy = xy[
                radial_inliers
            ]

            centered_xy = (
                inlier_xy
                - center_xy_table.reshape(1, 2)
            )

            angles = np.mod(
                np.arctan2(
                    centered_xy[:, 1],
                    centered_xy[:, 0],
                ),
                2.0 * np.pi,
            )

            angles = np.sort(
                angles
            )

            wrapped_angles = np.concatenate(
                (
                    angles,
                    [
                        angles[0]
                        + 2.0 * np.pi
                    ],
                )
            )

            angular_gaps = np.diff(
                wrapped_angles
            )

            largest_gap = float(
                np.max(
                    angular_gaps
                )
            )

            arc_coverage_deg = float(
                np.degrees(
                    2.0 * np.pi
                    - largest_gap
                )
            )

        else:
            arc_coverage_deg = 0.0

        # ---------------------------------------------------------
        # Choose one reference point on the fitted axis.
        #
        # The cylinder axis is:
        #
        #   [center_x, center_y, z]
        #
        # in table_0.
        #
        # Use the median Z of the radial inliers only to represent one
        # concrete point on this line.
        # ---------------------------------------------------------

        if inlier_count > 0:
            reference_z_table = float(
                np.median(
                    side_points_table[
                        radial_inliers,
                        2,
                    ]
                )
            )
        else:
            reference_z_table = float(
                np.median(
                    side_points_table[
                        :,
                        2,
                    ]
                )
            )

        axis_point_table = np.array(
            [
                center_xy_table[0],
                center_xy_table[1],
                reference_z_table,
            ],
            dtype=np.float64,
        )

        peg_tip_point_table = np.array(
            [
                center_xy_table[0],
                center_xy_table[1],
                z_top_table,
            ],
            dtype=np.float64,
        )

        peg_support_point_table = np.array(
            [
                center_xy_table[0],
                center_xy_table[1],
                support_z_table,
            ],
            dtype=np.float64,
        )

        # ---------------------------------------------------------
        # table_0 -> base_link.
        # ---------------------------------------------------------

        table_to_base_rotation = np.asarray(
            self._last_base_to_table_rotation,
            dtype=np.float64,
        )

        table_to_base_translation = np.asarray(
            self._last_base_to_table_translation,
            dtype=np.float64,
        )

        axis_point_base = (
            table_to_base_rotation
            @ axis_point_table
            + table_to_base_translation
        )

        peg_tip_point_base = (
            table_to_base_rotation
            @ peg_tip_point_table
            + table_to_base_translation
        )

        peg_support_point_base = (
            table_to_base_rotation
            @ peg_support_point_table
            + table_to_base_translation
        )

        axis_direction_base = (
            table_to_base_rotation
            @ np.array(
                [0.0, 0.0, 1.0],
                dtype=np.float64,
            )
        )

        axis_direction_norm = float(
            np.linalg.norm(
                axis_direction_base
            )
        )

        measured_axis_extent = (
            peg_tip_point_base
            - peg_support_point_base
        )

        projected_height_m = float(
            np.dot(
                measured_axis_extent,
                axis_direction_base,
            )
        )

        lateral_extent_m = float(
            np.linalg.norm(
                measured_axis_extent
                - projected_height_m
                * axis_direction_base
            )
        )

        if projected_height_m <= 0.0:
            raise RuntimeError(
                "Peg tip lies behind the support surface "
                "along the estimated peg axis."
            )

        if lateral_extent_m > 0.005:
            raise RuntimeError(
                "Peg tip/support geometry is inconsistent "
                "with the estimated axis: "
                f"lateral error="
                f"{lateral_extent_m * 1000.0:.2f} mm."
            )

        if axis_direction_norm <= 1e-12:
            raise RuntimeError(
                "Invalid table Z axis after transformation to base_link."
            )

        axis_direction_base = (
            axis_direction_base
            / axis_direction_norm
        )

        # ---------------------------------------------------------
        # Quality gate.
        #
        # Do not silently trust every geometric fit. The caller will
        # later decide whether to use this refinement or fall back to
        # the normal ScenePerceiver position.
        # ---------------------------------------------------------

        rejection_reasons: list[str] = []

        if inlier_count < min_points:
            rejection_reasons.append(
                "too_few_radial_inliers"
            )

        if (
            inlier_fraction
            < min_inlier_fraction
        ):
            rejection_reasons.append(
                "low_inlier_fraction"
            )

        if rms_m > max_rms_m:
            rejection_reasons.append(
                "high_radial_rms"
            )

        if (
            arc_coverage_deg
            < min_arc_coverage_deg
        ):
            rejection_reasons.append(
                "insufficient_arc_coverage"
            )

        accepted = (
            len(rejection_reasons) == 0
        )

        # ---------------------------------------------------------
        # Diagnostics.
        # ---------------------------------------------------------

        print(
            "[ScenePerceiver] Peg-axis refinement | "
            f"accepted={accepted} | "
            f"axis_xy_base="
            f"[{axis_point_base[0]:.5f}, "
            f"{axis_point_base[1]:.5f}] | "
            f"inliers={inlier_count}/{side_point_count} "
            f"({inlier_fraction:.3f}) | "
            f"rms={rms_m * 1000.0:.2f} mm | "
            f"arc={arc_coverage_deg:.1f} deg | "
            f"reasons={rejection_reasons}",
            f"height={peg_height_estimate_m * 1000.0:.1f} mm | ",
            f"support_mad={support_mad_m * 1000.0:.2f} mm | ",
            flush=True,
        )

        return {
            "accepted": accepted,

            "axis_xy_base": (
                float(axis_point_base[0]),
                float(axis_point_base[1]),
            ),

            "axis_point_base": (
                float(axis_point_base[0]),
                float(axis_point_base[1]),
                float(axis_point_base[2]),
            ),

            "axis_direction_base": (
                float(axis_direction_base[0]),
                float(axis_direction_base[1]),
                float(axis_direction_base[2]),
            ),

            "axis_xy_table": (
                float(center_xy_table[0]),
                float(center_xy_table[1]),
            ),

            "peg_tip_point_base": (
                float(peg_tip_point_base[0]),
                float(peg_tip_point_base[1]),
                float(peg_tip_point_base[2]),
            ),

            "peg_support_point_base": (
                float(peg_support_point_base[0]),
                float(peg_support_point_base[1]),
                float(peg_support_point_base[2]),
            ),

            "peg_tip_point_table": (
                float(peg_tip_point_table[0]),
                float(peg_tip_point_table[1]),
                float(peg_tip_point_table[2]),
            ),

            "peg_support_point_table": (
                float(peg_support_point_table[0]),
                float(peg_support_point_table[1]),
                float(peg_support_point_table[2]),
            ),

            "support_z_table_m": float(
                support_z_table
            ),

            "peg_height_estimate_m": float(
                peg_height_estimate_m
            ),

            "support_inlier_points": int(
                support_inlier_count
            ),

            "support_mad_m": float(
                support_mad_m
            ),

            "reference_z_table_m": float(
                reference_z_table
            ),

            "peg_radius_m": float(
                peg_radius_m
            ),

            "radial_tolerance_m": float(
                radial_tolerance_m
            ),

            "valid_depth_points": (
                total_valid_points
            ),

            "sampled_depth_points": int(
                len(points_table)
            ),

            "side_band_points": (
                side_point_count
            ),

            "inlier_points": (
                inlier_count
            ),

            "inlier_fraction": float(
                inlier_fraction
            ),

            "rms_m": float(
                rms_m
            ),

            "median_abs_residual_m": float(
                median_abs_residual_m
            ),

            "arc_coverage_deg": float(
                arc_coverage_deg
            ),

            "z_top_table_m": float(
                z_top_table
            ),

            "side_band_z_table_m": (
                float(side_lower_z),
                float(side_upper_z),
            ),

            "rejection_reasons": tuple(
                rejection_reasons
            ),
        }
    
    def _ensure_models_loaded(self) -> None:
        """Load GroundingDINO and SAM only when scene perception is needed."""

        if (
            self.grounding_model is not None
            and self.sam is not None
            and self.sam_predictor is not None
        ):
            return

        print(
            "[ScenePerceiver] Loading GroundingDINO and SAM...",
            flush=True,
        )

        self.grounding_model = load_groundingdino_model(
            str(self.grounding_config),
            str(self.grounding_checkpoint),
            device=str(self.device),
            text_encoder_path=str(self.bert_model),
        )

        self.sam = build_sam(
            checkpoint=str(self.sam_checkpoint)
        )

        self.sam.to(
            device=self.device
        )

        self.sam_predictor = SamPredictor(
            self.sam
        )

        print(
            "[ScenePerceiver] GroundingDINO and SAM loaded.",
            flush=True,
        )

    def camera_matrix_from_info(
        self,
        camera_info: dict,
    ) -> np.ndarray:
        """Build the 3x3 intrinsic camera matrix from CameraInfo data."""

        if not isinstance(camera_info, dict):
            raise TypeError(
                "camera_info must be a dictionary."
            )

        if "k" not in camera_info:
            raise ValueError(
                "camera_info does not contain the intrinsic matrix 'k'."
            )

        k = np.asarray(
            camera_info["k"],
            dtype=np.float64,
        )

        if k.size != 9:
            raise ValueError(
                "camera_info['k'] must contain exactly 9 values."
            )

        return k.reshape(3, 3)

    def refine_post_grasp_nut_hole(
        self,
        *,
        mask: np.ndarray,
        depth_image: np.ndarray,
        camera_matrix: np.ndarray,
        T_base_camera: np.ndarray,
        T_base_table: np.ndarray,
        known_thickness_m: float,
        artifacts_dir: str | Path | None = None,
        min_hole_area_px: float = 40.0,
        top_surface_band_m: float = 0.008,
        plane_inlier_threshold_m: float = 0.003,
        min_plane_points: int = 80,
        max_plane_rms_m: float = 0.003,
        hole_support_tolerance_m: float = 0.006,
        min_thickness_m: float = 0.002,
        max_thickness_m: float = 0.080,
        max_points: int = 12000,
    ) -> dict[str, Any]:
        """
        Re-estimate the grasped ring/nut hole before LIFT_UP.

        SAM provides only the external target region.

        The hole itself is recovered geometrically from depth:
        pixels inside the target external silhouette whose 3D position lies
        approximately one previously-measured object thickness below the
        observed top plane are interpreted as support/table visible through
        the internal hole.
        """

        mask = np.asarray(
            mask,
            dtype=bool,
        )

        depth = np.asarray(
            depth_image,
            dtype=np.float64,
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

        known_thickness_m = float(
            known_thickness_m
        )

        if mask.ndim != 2:
            raise ValueError(
                "Post-grasp mask must be 2D."
            )

        if depth.ndim != 2:
            raise ValueError(
                "Post-grasp depth image must be 2D."
            )

        if mask.shape != depth.shape:
            raise ValueError(
                "Post-grasp mask and depth must "
                "have identical shape."
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

        if not (
            min_thickness_m
            <= known_thickness_m
            <= max_thickness_m
        ):
            raise ValueError(
                "Invalid pre-grasp measured thickness: "
                f"{known_thickness_m * 1000.0:.2f} mm."
            )

        fx = float(K[0, 0])
        fy = float(K[1, 1])
        cx = float(K[0, 2])
        cy = float(K[1, 2])

        if (
            not np.isfinite(fx)
            or not np.isfinite(fy)
            or fx <= 0.0
            or fy <= 0.0
        ):
            raise ValueError(
                "Invalid eye-in-hand intrinsics."
            )

        # ---------------------------------------------------------
        # base <- camera
        # base <- table
        #
        # therefore:
        #
        # table <- camera =
        # inv(base <- table) @ (base <- camera)
        # ---------------------------------------------------------

        try:
            T_table_base = np.linalg.inv(
                T_base_table
            )
        except np.linalg.LinAlgError as exc:
            raise RuntimeError(
                "T_base_table is not invertible."
            ) from exc

        T_table_camera = (
            T_table_base
            @ T_base_camera
        )

        R_table_camera = np.asarray(
            T_table_camera[:3, :3],
            dtype=np.float64,
        )

        t_table_camera = np.asarray(
            T_table_camera[:3, 3],
            dtype=np.float64,
        )

        # ---------------------------------------------------------
        # 1. Build the FILLED EXTERNAL SILHOUETTE.
        #
        # SAM may fill the real internal hole, therefore we deliberately
        # ignore SAM's internal topology.
        # ---------------------------------------------------------

        mask_uint8 = (
            mask.astype(np.uint8)
            * 255
        )

        external_contours, _ = (
            cv2.findContours(
                mask_uint8,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_NONE,
            )
        )

        if not external_contours:
            raise RuntimeError(
                "Post-grasp SAM mask contains no "
                "external contour."
            )

        outer_contour = max(
            external_contours,
            key=cv2.contourArea,
        )

        silhouette_mask = np.zeros(
            mask.shape,
            dtype=np.uint8,
        )

        cv2.drawContours(
            silhouette_mask,
            [outer_contour],
            -1,
            255,
            thickness=cv2.FILLED,
        )

        silhouette = (
            silhouette_mask > 0
        )

        valid_region = (
            silhouette
            & np.isfinite(depth)
            & (depth > 0.0)
        )

        all_indices = np.flatnonzero(
            valid_region
        )

        if len(
            all_indices
        ) < min_plane_points:
            raise RuntimeError(
                "Too few valid depth points inside "
                "the post-grasp target silhouette."
            )

        all_v, all_u = np.unravel_index(
            all_indices,
            depth.shape,
        )

        all_z_camera = depth[
            all_v,
            all_u,
        ]

        all_x_camera = (
            (
                all_u.astype(
                    np.float64
                )
                - cx
            )
            * all_z_camera
            / fx
        )

        all_y_camera = (
            (
                all_v.astype(
                    np.float64
                )
                - cy
            )
            * all_z_camera
            / fy
        )

        all_points_camera = np.column_stack(
            (
                all_x_camera,
                all_y_camera,
                all_z_camera,
            )
        )

        all_points_table = (
            all_points_camera
            @ R_table_camera.T
            + t_table_camera
        )

        # ---------------------------------------------------------
        # 2. Estimate the top plane.
        #
        # Even if SAM fills the hole, the support/table samples are
        # lower than the ring top. The upper percentile therefore
        # isolates the actual upper ring surface.
        # ---------------------------------------------------------

        if len(
            all_points_table
        ) > max_points:

            sample_ids = np.linspace(
                0,
                len(all_points_table) - 1,
                max_points,
                dtype=np.int64,
            )

            fitting_points = (
                all_points_table[
                    sample_ids
                ]
            )

        else:
            fitting_points = (
                all_points_table
            )

        object_z = fitting_points[
            :,
            2,
        ]

        top_seed_z = float(
            np.percentile(
                object_z,
                90.0,
            )
        )

        top_points = fitting_points[
            object_z
            >= (
                top_seed_z
                - top_surface_band_m
            )
        ]

        if len(top_points) < min_plane_points:
            raise RuntimeError(
                "Too few points on the post-grasp "
                "ring upper surface."
            )

        def fit_plane(
            points: np.ndarray,
        ) -> tuple[
            np.ndarray,
            np.ndarray,
        ]:

            centroid = np.mean(
                points,
                axis=0,
            )

            centered = (
                points
                - centroid
            )

            _, _, vh = np.linalg.svd(
                centered,
                full_matrices=False,
            )

            normal = np.asarray(
                vh[-1],
                dtype=np.float64,
            )

            norm = float(
                np.linalg.norm(
                    normal
                )
            )

            if norm <= 1e-12:
                raise RuntimeError(
                    "Degenerate post-grasp top plane."
                )

            normal /= norm

            return centroid, normal

        (
            top_plane_point,
            top_plane_normal,
        ) = fit_plane(
            top_points
        )

        if top_plane_normal[2] < 0.0:
            top_plane_normal = (
                -top_plane_normal
            )

        top_distances = np.abs(
            (
                fitting_points
                - top_plane_point
            )
            @ top_plane_normal
        )

        plane_inliers = fitting_points[
            top_distances
            <= plane_inlier_threshold_m
        ]

        if len(
            plane_inliers
        ) < min_plane_points:
            raise RuntimeError(
                "Too few post-grasp top-plane inliers."
            )

        (
            top_plane_point,
            top_plane_normal,
        ) = fit_plane(
            plane_inliers
        )

        if top_plane_normal[2] < 0.0:
            top_plane_normal = (
                -top_plane_normal
            )

        residuals = (
            (
                plane_inliers
                - top_plane_point
            )
            @ top_plane_normal
        )

        top_plane_rms_m = float(
            np.sqrt(
                np.mean(
                    residuals ** 2
                )
            )
        )

        if (
            top_plane_rms_m
            > max_plane_rms_m
        ):
            raise RuntimeError(
                "Post-grasp top plane is too noisy: "
                f"{top_plane_rms_m * 1000.0:.2f} mm."
            )

        # ---------------------------------------------------------
        # 3. Find depth samples approximately one ring thickness
        #    BELOW the top plane.
        #
        # Those samples are the table/support seen THROUGH the hole.
        # ---------------------------------------------------------

        signed_plane_distance = (
            (
                all_points_table
                - top_plane_point
            )
            @ top_plane_normal
        )

        depth_below_top = (
            -signed_plane_distance
        )

        effective_hole_tolerance_m = max(
            float(
                hole_support_tolerance_m
            ),
            float(
                known_thickness_m
                * 0.25
            ),
        )

        support_like = (
            np.abs(
                depth_below_top
                - known_thickness_m
            )
            <= effective_hole_tolerance_m
        )

        hole_candidate_mask = np.zeros(
            depth.shape,
            dtype=np.uint8,
        )

        hole_candidate_mask[
            all_v[
                support_like
            ],
            all_u[
                support_like
            ],
        ] = 255

        # Remove isolated depth noise.
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (3, 3),
        )

        hole_candidate_mask = cv2.morphologyEx(
            hole_candidate_mask,
            cv2.MORPH_OPEN,
            kernel,
        )

        hole_candidate_mask = cv2.morphologyEx(
            hole_candidate_mask,
            cv2.MORPH_CLOSE,
            kernel,
        )

        # ---------------------------------------------------------
        # 4. Reject support-like regions touching the OUTER target
        #    boundary.
        #
        # The actual ring hole is enclosed inside the object.
        # Side/background leakage normally reaches the external boundary.
        # ---------------------------------------------------------

        eroded_silhouette = cv2.erode(
            silhouette_mask,
            kernel,
            iterations=1,
        )

        silhouette_boundary = (
            (silhouette_mask > 0)
            & (eroded_silhouette == 0)
        )

        (
            component_count,
            component_labels,
            component_stats,
            _,
        ) = cv2.connectedComponentsWithStats(
            (
                hole_candidate_mask > 0
            ).astype(np.uint8),
            connectivity=8,
        )

        component_candidates = []

        for label_index in range(
            1,
            component_count,
        ):

            area = int(
                component_stats[
                    label_index,
                    cv2.CC_STAT_AREA,
                ]
            )

            if area < min_hole_area_px:
                continue

            component = (
                component_labels
                == label_index
            )

            if np.any(
                component
                & silhouette_boundary
            ):
                continue

            component_candidates.append(
                (
                    label_index,
                    area,
                )
            )

        if not component_candidates:
            raise RuntimeError(
                "Depth-based post-grasp refinement could not "
                "find an enclosed support region corresponding "
                "to the ring hole."
            )

        hole_component_label, _ = max(
            component_candidates,
            key=lambda item:
            item[1],
        )

        hole_mask = (
            component_labels
            == hole_component_label
        )

        hole_contours, _ = cv2.findContours(
            hole_mask.astype(
                np.uint8
            ),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_NONE,
        )

        if not hole_contours:
            raise RuntimeError(
                "Depth-based hole region has no contour."
            )

        hole_contour = max(
            hole_contours,
            key=cv2.contourArea,
        )

        hole_area_px = float(
            cv2.contourArea(
                hole_contour
            )
        )

        if len(hole_contour) < 5:
            raise RuntimeError(
                "Depth-based hole contour has too "
                "few points for ellipse fitting."
            )

        ellipse = cv2.fitEllipse(
            hole_contour
        )

        (
            (hole_u, hole_v),
            (
                ellipse_width_px,
                ellipse_height_px,
            ),
            ellipse_angle_deg,
        ) = ellipse

        ellipse_area_px = float(
            np.pi
            * 0.25
            * ellipse_width_px
            * ellipse_height_px
        )

        hole_fit_fill_ratio = (
            hole_area_px
            / ellipse_area_px
            if ellipse_area_px > 1e-9
            else 0.0
        )

        if not (
            0.40
            <= hole_fit_fill_ratio
            <= 1.60
        ):
            raise RuntimeError(
                "Depth-based hole candidate is poorly "
                "represented by an ellipse: "
                f"fill_ratio={hole_fit_fill_ratio:.3f}."
            )

        # ---------------------------------------------------------
        # 5. Hole center = image ray intersected with current top plane.
        # ---------------------------------------------------------

        ray_camera = np.array(
            [
                (hole_u - cx) / fx,
                (hole_v - cy) / fy,
                1.0,
            ],
            dtype=np.float64,
        )

        ray_camera /= np.linalg.norm(
            ray_camera
        )

        ray_table = (
            R_table_camera
            @ ray_camera
        )

        ray_table /= np.linalg.norm(
            ray_table
        )

        camera_origin_table = (
            t_table_camera
        )

        denominator = float(
            np.dot(
                top_plane_normal,
                ray_table,
            )
        )

        if abs(
            denominator
        ) <= 1e-8:
            raise RuntimeError(
                "Post-grasp hole-center ray is "
                "parallel to the ring top plane."
            )

        ray_distance = float(
            np.dot(
                top_plane_normal,
                (
                    top_plane_point
                    - camera_origin_table
                ),
            )
            / denominator
        )

        if ray_distance <= 0.0:
            raise RuntimeError(
                "Post-grasp hole center lies "
                "behind the eye-in-hand camera."
            )

        hole_center_table = (
            camera_origin_table
            + ray_distance
            * ray_table
        )

        hole_bottom_center_table = (
            hole_center_table
            - known_thickness_m
            * top_plane_normal
        )

        # ---------------------------------------------------------
        # 6. table_0 -> base_link.
        # ---------------------------------------------------------

        R_base_table = np.asarray(
            T_base_table[:3, :3],
            dtype=np.float64,
        )

        t_base_table = np.asarray(
            T_base_table[:3, 3],
            dtype=np.float64,
        )

        hole_center_base = (
            R_base_table
            @ hole_center_table
            + t_base_table
        )

        hole_bottom_center_base = (
            R_base_table
            @ hole_bottom_center_table
            + t_base_table
        )

        hole_axis_base = (
            R_base_table
            @ top_plane_normal
        )

        hole_axis_base /= np.linalg.norm(
            hole_axis_base
        )

        # ---------------------------------------------------------
        # Diagnostics.
        # ---------------------------------------------------------

        if artifacts_dir is not None:

            output_dir = (
                Path(artifacts_dir)
                .expanduser()
                .resolve()
            )

            output_dir.mkdir(
                parents=True,
                exist_ok=True,
            )

            cv2.imwrite(
                str(
                    output_dir
                    / "post_grasp_depth_hole_mask.png"
                ),
                (
                    hole_mask.astype(
                        np.uint8
                    )
                    * 255
                ),
            )

            cv2.imwrite(
                str(
                    output_dir
                    / "post_grasp_depth_candidates.png"
                ),
                hole_candidate_mask,
            )

            overlay = np.zeros(
                (
                    depth.shape[0],
                    depth.shape[1],
                    3,
                ),
                dtype=np.uint8,
            )

            overlay[
                silhouette
            ] = (
                80,
                80,
                80,
            )

            overlay[
                hole_mask
            ] = (
                255,
                255,
                255,
            )

            cv2.ellipse(
                overlay,
                ellipse,
                (0, 255, 0),
                2,
            )

            cv2.imwrite(
                str(
                    output_dir
                    / "post_grasp_depth_hole_overlay.png"
                ),
                overlay,
            )

        print(
            "[POST-GRASP] Depth hole refinement | "
            f"center_base="
            f"[{hole_center_base[0]:.5f}, "
            f"{hole_center_base[1]:.5f}, "
            f"{hole_center_base[2]:.5f}] | "
            f"axis_base="
            f"[{hole_axis_base[0]:.5f}, "
            f"{hole_axis_base[1]:.5f}, "
            f"{hole_axis_base[2]:.5f}] | "
            f"thickness="
            f"{known_thickness_m * 1000.0:.2f} mm | "
            f"hole_area="
            f"{hole_area_px:.1f} px | "
            f"plane_rms="
            f"{top_plane_rms_m * 1000.0:.2f} mm",
            flush=True,
        )

        return {
            "accepted": True,

            "hole_center_px": (
                float(hole_u),
                float(hole_v),
            ),

            "hole_center_base": tuple(
                float(value)
                for value
                in hole_center_base
            ),

            "hole_bottom_center_base": tuple(
                float(value)
                for value
                in hole_bottom_center_base
            ),

            "hole_axis_base": tuple(
                float(value)
                for value
                in hole_axis_base
            ),

            "nut_thickness_estimate_m": float(
                known_thickness_m
            ),

            "thickness_source": (
                "initial_pre_grasp_refinement"
            ),

            "hole_area_px": float(
                hole_area_px
            ),

            "hole_ellipse_axes_px": (
                float(
                    ellipse_width_px
                ),
                float(
                    ellipse_height_px
                ),
            ),

            "hole_ellipse_angle_deg": float(
                ellipse_angle_deg
            ),

            "hole_fit_fill_ratio": float(
                hole_fit_fill_ratio
            ),

            "top_plane_rms_m": float(
                top_plane_rms_m
            ),

            "depth_hole_tolerance_m": float(
                effective_hole_tolerance_m
            ),

            "top_plane_point_table": tuple(
                float(value)
                for value
                in top_plane_point
            ),

            "top_plane_normal_table": tuple(
                float(value)
                for value
                in top_plane_normal
            ),
        }

    def detect_post_grasp_target_mask(
        self,
        *,
        rgb_image: np.ndarray,
        target_label: str,
        artifacts_dir: str | Path | None = None,
        box_threshold: float = 0.20,
        text_threshold: float = 0.25,
    ) -> dict[str, Any]:
        """
        Detect and segment only the already-known grasped Nut-Assembly object.

        IMPORTANT:
        SAM is used only to delimit the target object.

        The internal hole is NOT inferred from SAM topology because SAM can
        legitimately fill the visible hole. Hole geometry is recovered later
        from RGB-D geometry.
        """

        if not isinstance(
            rgb_image,
            np.ndarray,
        ):
            raise TypeError(
                "Post-grasp RGB image must be a numpy array."
            )

        if rgb_image.ndim != 3:
            raise ValueError(
                "Post-grasp RGB image must have shape (H, W, C)."
            )

        target_label = str(
            target_label
        ).strip()

        if not target_label:
            raise ValueError(
                "Post-grasp target label cannot be empty."
            )

        self._ensure_models_loaded()

        normalized_label = (
            target_label
            .strip()
            .lower()
            .replace("_", " ")
        )

        normalized_label = re.sub(
            r"\s+\d+$",
            "",
            normalized_label,
        ).strip()

        if not normalized_label:
            raise RuntimeError(
                "Could not derive a GroundingDINO query "
                "from the post-grasp target label."
            )

        object_type = (
            normalized_label
            .split()[-1]
        )

        queries = [
            normalized_label
        ]

        if object_type != normalized_label:
            queries.append(
                object_type
            )

        image_source = np.asarray(
            rgb_image,
            dtype=np.uint8,
        )

        _, image = load_image_from_array(
            image_source
        )

        height, width = (
            image_source.shape[:2]
        )

        image_area = float(
            height * width
        )

        selected_candidate = None

        for query in queries:

            timing_query = (
                query.replace(
                    " ",
                    "_",
                )
            )

            with TIMING.measure(
                f"post_grasp.grounding_dino.{timing_query}",
                cuda=True,
            ):
                boxes, logits, _ = predict(
                    model=self.grounding_model,
                    image=image,
                    caption=query,
                    box_threshold=box_threshold,
                    text_threshold=text_threshold,
                    device=self.device,
                )

            print(
                "[POST-GRASP] GroundingDINO | "
                f"query={query!r} | "
                f"candidates={len(boxes)}",
                flush=True,
            )

            if len(boxes) == 0:
                continue

            boxes_xyxy = (
                box_ops.box_cxcywh_to_xyxy(
                    boxes
                )
                * torch.tensor(
                    [
                        width,
                        height,
                        width,
                        height,
                    ],
                    dtype=boxes.dtype,
                    device=boxes.device,
                )
            )

            with TIMING.measure(
                "post_grasp.sam",
                cuda=True,
            ):
                self.sam_predictor.set_image(
                    image_source
                )

                transformed_boxes = (
                    self.sam_predictor
                    .transform
                    .apply_boxes_torch(
                        boxes_xyxy,
                        image_source.shape[:2],
                    )
                    .to(
                        self.device
                    )
                )

                masks, _, _ = (
                    self.sam_predictor.predict_torch(
                        point_coords=None,
                        point_labels=None,
                        boxes=transformed_boxes,
                        multimask_output=False,
                    )
                )

            masks_np = (
                masks
                .detach()
                .cpu()
                .numpy()
            )

            logits_np = (
                logits
                .detach()
                .cpu()
                .numpy()
            )

            candidates = []

            for candidate_index, (
                mask,
                confidence,
            ) in enumerate(
                zip(
                    masks_np,
                    logits_np,
                )
            ):
                object_mask = (
                    mask[0] > 0
                )

                mask_area = int(
                    np.count_nonzero(
                        object_mask
                    )
                )

                if mask_area <= 0:
                    continue

                mask_fraction = (
                    float(mask_area)
                    / image_area
                )

                # Eye-in-hand targets can occupy a large fraction of
                # the image. Only reject obviously degenerate masks.
                if (
                    mask_fraction <= 0.005
                    or mask_fraction >= 0.80
                ):
                    print(
                        "[POST-GRASP] Candidate rejected | "
                        f"query={query!r} | "
                        f"index={candidate_index} | "
                        f"mask_fraction={mask_fraction:.4f}",
                        flush=True,
                    )

                    continue

                candidates.append(
                    {
                        "query": query,
                        "candidate_index": (
                            int(candidate_index)
                        ),
                        "confidence": float(
                            confidence
                        ),
                        "mask": object_mask,
                        "mask_fraction": (
                            mask_fraction
                        ),
                    }
                )

            if candidates:
                selected_candidate = max(
                    candidates,
                    key=lambda candidate: (
                        candidate[
                            "confidence"
                        ],
                        candidate[
                            "mask_fraction"
                        ],
                    ),
                )

                break

        if selected_candidate is None:
            raise RuntimeError(
                "Post-grasp target detection could not "
                f"segment {normalized_label!r}."
            )

        selected_mask = np.asarray(
            selected_candidate[
                "mask"
            ],
            dtype=bool,
        )

        print(
            "[POST-GRASP] Target mask accepted | "
            f"label={normalized_label!r} | "
            f"query={selected_candidate['query']!r} | "
            f"confidence="
            f"{selected_candidate['confidence']:.4f} | "
            f"mask_fraction="
            f"{selected_candidate['mask_fraction']:.4f}",
            flush=True,
        )

        if artifacts_dir is not None:

            output_dir = (
                Path(artifacts_dir)
                .expanduser()
                .resolve()
            )

            output_dir.mkdir(
                parents=True,
                exist_ok=True,
            )

            cv2.imwrite(
                str(
                    output_dir
                    / "post_grasp_rgb.png"
                ),
                cv2.cvtColor(
                    image_source,
                    cv2.COLOR_RGB2BGR,
                ),
            )

            cv2.imwrite(
                str(
                    output_dir
                    / "post_grasp_target_mask.png"
                ),
                (
                    selected_mask.astype(
                        np.uint8
                    )
                    * 255
                ),
            )

            overlay = image_source.copy()

            contours, _ = cv2.findContours(
                selected_mask.astype(
                    np.uint8
                ),
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )

            cv2.drawContours(
                overlay,
                contours,
                -1,
                (255, 255, 255),
                2,
            )

            cv2.imwrite(
                str(
                    output_dir
                    / "post_grasp_target_overlay.png"
                ),
                cv2.cvtColor(
                    overlay,
                    cv2.COLOR_RGB2BGR,
                ),
            )

        return {
            "mask": selected_mask,
            "target_label": normalized_label,
            "query": selected_candidate[
                "query"
            ],
            "confidence": float(
                selected_candidate[
                    "confidence"
                ]
            ),
            "mask_fraction": float(
                selected_candidate[
                    "mask_fraction"
                ]
            ),
        }

    def pixel_to_base_link_from_info(
        self,
        pixel: tuple[int, int],
        depth_image: np.ndarray,
        camera_info: dict,
        base_to_table_rotation: np.ndarray,
        base_to_table_translation: np.ndarray,
    ) -> tuple[float, float, float]:
        camera_matrix = self.camera_matrix_from_info(
            camera_info
        )

        return self.pixel_to_base_link(
            pixel=pixel,
            depth_image=depth_image,
            camera_matrix=camera_matrix,
            base_to_table_rotation=base_to_table_rotation,
            base_to_table_translation=base_to_table_translation,
        )

    def _discover_detector_labels(
        self,
        rgb_image: np.ndarray,
    ) -> tuple[list[str], dict[str, dict]]:
        """Discover semantic detector labels from the runtime RGB scene."""

        if not isinstance(rgb_image, np.ndarray):
            raise TypeError(
                "rgb_image must be a numpy array."
            )

        if not os.environ.get("OPENAI_API_KEY"):
            raise ValueError(
                "OPENAI_API_KEY is not configured."
            )

        # rgb_image is RGB, while OpenCV encoding expects BGR.
        image_bgr = cv2.cvtColor(
            rgb_image,
            cv2.COLOR_RGB2BGR,
        )

        ok, encoded_image = cv2.imencode(
            ".jpg",
            image_bgr,
            [
                cv2.IMWRITE_JPEG_QUALITY,
                95,
            ],
        )

        if not ok:
            raise RuntimeError(
                "Could not encode runtime RGB image."
            )

        image_base64 = base64.b64encode(
            encoded_image.tobytes()
        ).decode("ascii")

        # ---------------------------------------------------------
        # Generalized object-discovery prompt
        # ---------------------------------------------------------

        generalized_messages = [
            {
                "role": "system",
                "content": GENERALIZED_DISCOVERY_SYSTEM_PROMPT,
            },
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": GENERALIZED_DISCOVERY_USER_PROMPT,
                    },
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": (
                                "data:image/jpeg;base64,"
                                + image_base64
                            ),
                            "detail": "high",
                        },
                    },
                ],
            },
        ]

        # ---------------------------------------------------------
        # Prior-guided object-discovery prompt
        # ---------------------------------------------------------

        prior_guided_messages = [
            {
                "role": "system",
                "content": (
                    "You are a visual object detector whose output will be used "
                    "directly as text queries for GroundingDINO. "
                    "The scene contains colored cubes and storage bins. "
                    "For every cube, include its visible color in the detector "
                    "label using the exact form '<color> cube'. "
                    "For every storage bin, always use the exact detector label "
                    "'storage bin' without adding color, position, material, "
                    "or other attributes."
                ),
            },
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": (
                            "Inspect the physical objects visible on the table "
                            "and classify them using these rules:\n"
                            "1. For every visible cube or graspable colored block, "
                            "identify its visible color and return '<color> cube'.\n"
                            "The only valid cube colors in this benchmark are: "
                            "red, green, blue, and yellow. "
                            "For every cube, the detector label MUST therefore be exactly one of: "
                            "'red cube', 'green cube', 'blue cube', or 'yellow cube'. "
                            "Do not use any other cube color.\n"
                            "2. Examples are 'red cube', 'green cube', "
                            "'blue cube', and 'yellow cube'.\n"
                            "3. For every bin, box, tray, container, or receptacle, "
                            "return exactly 'storage bin'.\n"
                            "4. Count every physical instance separately.\n"
                            "5. Repeat 'storage bin' once for every visible bin.\n"
                            "6. Do not include the robot, gripper, table, "
                            "or background objects.\n"
                            "7. Never use spatial descriptions such as "
                            "'first bin from the left'.\n"
                            "8. Do not add objects that are not visible.\n\n"
                            "Return exactly two lines and no additional explanation:\n"
                            "Number: <total number of instances>\n"
                            "Objects: <comma-separated detector labels, "
                            "repeated once per instance>"
                        ),
                    },
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": (
                                "data:image/jpeg;base64,"
                                + image_base64
                            ),
                            "detail": "high",
                        },
                    },
                ],
            },
        ]

        if self.perception_mode == "generalized":
            messages = generalized_messages
        else:
            messages = prior_guided_messages

        print(
            "[ScenePerceiver] Perception mode: "
            f"{self.perception_mode}"
        )

        params = {
            "model": "gpt-4o-2024-08-06",
            "messages": messages,
            "temperature": 0,
            "max_tokens": 300,
        }

        if self.perception_mode == "generalized":
            params.update(
                {
                    "model": "gpt-4o",
                    "max_tokens": 800,
                    "response_format": {
                        "type": "json_object",
                    },
                }
            )

        response = OpenAI().chat.completions.create(
            **params
        )

        raw_output = (
            response
            .choices[0]
            .message
            .content
        )

        print(
            "[ScenePerceiver] Raw VLM output:\n"
            f"{raw_output}"
        )

        if not raw_output:
            raise RuntimeError(
                "Runtime object discovery returned no output."
            )

        if self.perception_mode == "generalized":

            (
                num,
                detector_labels,
                metadata_by_label,
            ) = parse_structured_object_list(
                raw_output
            )

            print(
                "[ScenePerceiver] Structured object discovery: "
                f"{json.dumps(metadata_by_label, sort_keys=True)}"
            )

            print(
                "[ScenePerceiver] Discovered objects: "
                f"{num}"
            )

            return detector_labels, metadata_by_label

        number_match = re.search(
            r"Number:\s*(\d+)",
            raw_output,
        )

        objects_match = re.search(
            r"Objects:\s*(.+)",
            raw_output,
        )

        if objects_match is None:
            raise RuntimeError(
                "Could not parse object list from VLM output: "
                f"{raw_output!r}"
            )

        detector_labels = [
            label.strip().lower()
            for label in objects_match.group(1).split(",")
            if label.strip()
        ]

        if not detector_labels:
            raise RuntimeError(
                "VLM object discovery returned an empty object list."
            )

        reported_count = (
            int(number_match.group(1))
            if number_match is not None
            else None
        )

        print(
            "[ScenePerceiver] Discovered detector labels: "
            f"{detector_labels}"
        )

        if (
            reported_count is not None
            and reported_count != len(detector_labels)
        ):
            print(
                "[ScenePerceiver] WARNING: "
                f"VLM reported {reported_count} objects but "
                f"returned {len(detector_labels)} labels."
            )

        return detector_labels, {}

    def _detect_objects(
        self,
        rgb_image: np.ndarray,
        artifacts_dir: str | Path | None = None,
    ) -> list[DetectedObject]:
        """Detect and segment configured object categories in one RGB frame."""
        
        self._ensure_models_loaded()

        # Keep the complete RGB frame for VLM discovery, SAM,
        # depth lookup and all downstream geometry.
        image_source = rgb_image

        # GroundingDINO sees only the configured ROI.
        (
            dino_image_source,
            dino_crop_info,
        ) = crop_image_for_grounding_dino(
            image_source,
            top_px=self.grounding_dino_crop[
                "top_px"
            ],
            bottom_px=self.grounding_dino_crop[
                "bottom_px"
            ],
            left_px=self.grounding_dino_crop[
                "left_px"
            ],
            right_px=self.grounding_dino_crop[
                "right_px"
            ],
        )

        if artifacts_dir is not None:
            dino_artifacts_dir = (
                Path(artifacts_dir)
                .expanduser()
                .resolve()
            )

            dino_artifacts_dir.mkdir(
                parents=True,
                exist_ok=True,
            )

            dino_input_path = (
                dino_artifacts_dir
                / "groundingdino_input.png"
            )

            saved = cv2.imwrite(
                str(dino_input_path),
                cv2.cvtColor(
                    dino_image_source,
                    cv2.COLOR_RGB2BGR,
                ),
            )

            if not saved:
                raise RuntimeError(
                    "Could not save GroundingDINO input image: "
                    f"{dino_input_path}"
                )

            print(
                "[ScenePerceiver] GroundingDINO input saved to: "
                f"{dino_input_path}"
            )

        # GroundingDINO preprocessing is performed only on the cropped
        # image. The returned image_source is intentionally ignored.
        _, image = load_image_from_array(
            dino_image_source
        )

        print(
            "[ScenePerceiver] GroundingDINO crop: "
            f"top={dino_crop_info['top_px']}px | "
            f"bottom={dino_crop_info['bottom_px']}px | "
            f"left={dino_crop_info['left_px']}px | "
            f"right={dino_crop_info['right_px']}px | "
            f"full={dino_crop_info['full_width']}x"
            f"{dino_crop_info['full_height']} | "
            f"dino={dino_crop_info['crop_width']}x"
            f"{dino_crop_info['crop_height']}"
        )

        (
            detector_labels,
            metadata_by_label,
        ) = self._discover_detector_labels(
            rgb_image
        )

        object_counts = Counter(
            detector_labels
        )

        semantic_object_box_threshold  = 0.20
        default_box_threshold = 0.30
        text_threshold = 0.25

        detected_boxes: list[torch.Tensor] = []
        detected_labels: list[str] = []
        detected_confidences: list[float] = []

        semantic_candidates: list[dict[str, Any]] = []

        if self.perception_mode == "generalized":

            (
                selected_boxes,
                selected_logits,
                _,
                selected_labels,
                grounding_counts,
                width_diagnostics,
            ) = detect_generalized_objects(
                image=image,
                groundingdino_model=self.grounding_model,
                device=self.device,
                object_counts=object_counts,
                metadata_by_label=metadata_by_label,
                box_threshold=semantic_object_box_threshold,
                text_threshold=text_threshold,
            )

            detected_boxes = selected_boxes
            detected_labels = selected_labels

            detected_confidences = [
                float(logit.detach().cpu().item())
                for logit in selected_logits
            ]

            print(
                "[ScenePerceiver] Grounding counts: "
                f"{json.dumps(grounding_counts, sort_keys=True)}"
            )

            print(
                "[ScenePerceiver] Width filter: "
                f"{json.dumps(width_diagnostics, sort_keys=True)}"
            )

        else:
            for detector_label, requested_count in object_counts.items():

                timing_label = detector_label.replace(" ", "_")
                with TIMING.measure(
                    f"scene.grounding_dino.{timing_label}",
                    cuda=True,
                ):

                    is_storage_bin = (
                        detector_label.strip().lower() == "storage bin"
                    )

                    current_box_threshold = (
                        default_box_threshold
                        if detector_label.strip().lower() == "storage bin"
                        else semantic_object_box_threshold
                    )

                    boxes, logits, phrases = predict(
                        model=self.grounding_model,
                        image=image,
                        caption=detector_label,
                        box_threshold=current_box_threshold,
                        text_threshold=text_threshold,
                        device=self.device,
                    )

                    print(
                        f"\n[DINO] {detector_label!r}: "
                        f"{len(boxes)} candidates"
                    )

                    for i, (box, logit) in enumerate(
                        zip(boxes, logits)
                    ):
                        print(
                            f"  [{i}] "
                            f"box={box.detach().cpu().tolist()} "
                            f"score={float(logit.detach().cpu().item()):.4f}"
                        )


                if detector_label.strip().lower() != "storage bin":
                    box_areas = boxes[:, 2] * boxes[:, 3]
                    keep_mask = box_areas < 0.1

                    boxes = boxes[keep_mask]
                    logits = logits[keep_mask]

                    keep_values = keep_mask.detach().cpu().tolist()
                    phrases = [
                        phrase
                        for phrase, keep in zip(phrases, keep_values)
                        if keep
                    ]

                # Task-specific correction already implemented in SeeDo.
                # For every other label this function is effectively a no-op.
                boxes, logits, phrases, diagnostics = (
                    filter_oversized_storage_bin_detections(
                        boxes=boxes,
                        logits=logits,
                        phrases=phrases,
                        detector_label=detector_label,
                    )
                )

                if diagnostics["removed_count"]:
                    print(
                        "[ScenePerceiver] Oversized detections removed: "
                        f"{diagnostics}"
                    )
                
                is_storage_bin = (
                    detector_label.strip().lower()
                    == "storage bin"
                )

                if not is_storage_bin:
                    for box, logit in zip(
                        boxes,
                        logits,
                    ):
                        semantic_candidates.append(
                            {
                                "label": detector_label,
                                "box": box,
                                "confidence": float(
                                    logit
                                    .detach()
                                    .cpu()
                                    .item()
                                ),
                            }
                        )

                    continue


                # Storage bins keep the existing behaviour.
                selected_count = min(
                    requested_count,
                    int(boxes.shape[0]),
                )

                boxes = boxes[:selected_count]
                logits = logits[:selected_count]
                phrases = phrases[:selected_count]

                for box, logit in zip(
                    boxes,
                    logits,
                ):
                    detected_boxes.append(
                        box.unsqueeze(0)
                    )

                    detected_labels.append(
                        detector_label
                    )

                    detected_confidences.append(
                        float(
                            logit
                            .detach()
                            .cpu()
                            .item()
                        )
                    )

            # Resolve competing semantic detections.
            #
            # Different semantic queries may ground the same physical object.
            # Process candidates globally by confidence and allow each physical
            # region to be assigned only once.
            semantic_candidates.sort(
                key=lambda candidate: candidate[
                    "confidence"
                ],
                reverse=True,
            )

            accepted_semantic: list[
                dict[str, Any]
            ] = []

            accepted_counts: dict[str, int] = {
                label: 0
                for label in object_counts
                if label.strip().lower()
                != "storage bin"
            }

            semantic_iou_threshold = 0.7

            for candidate in semantic_candidates:
                label = candidate["label"]

                requested_count = object_counts[
                    label
                ]

                # The VLM determines how many physical instances
                # of this semantic label are expected.
                if (
                    accepted_counts[label]
                    >= requested_count
                ):
                    continue

                candidate_box_xyxy = (
                    box_ops.box_cxcywh_to_xyxy(
                        candidate["box"].unsqueeze(0)
                    )
                )

                conflicting_detection = None
                conflicting_iou = 0.0

                for accepted in accepted_semantic:
                    accepted_box_xyxy = (
                        box_ops.box_cxcywh_to_xyxy(
                            accepted["box"].unsqueeze(0)
                        )
                    )

                    iou_matrix, _ = box_ops.box_iou(
                        candidate_box_xyxy,
                        accepted_box_xyxy,
                    )

                    iou = float(
                        iou_matrix[0, 0]
                        .detach()
                        .cpu()
                        .item()
                    )

                    if iou >= semantic_iou_threshold:
                        conflicting_detection = (
                            accepted
                        )
                        conflicting_iou = iou
                        break

                if conflicting_detection is not None:
                    print(
                        "[ScenePerceiver] Semantic IoU conflict: "
                        f"{label!r} "
                        f"score={candidate['confidence']:.4f} "
                        "rejected because it overlaps "
                        f"{conflicting_detection['label']!r} "
                        "with "
                        f"score="
                        f"{conflicting_detection['confidence']:.4f}, "
                        f"IoU={conflicting_iou:.4f}"
                    )

                    continue

                accepted_semantic.append(
                    candidate
                )

                accepted_counts[label] += 1


            # Try progressively less restrictive GroundingDINO fallbacks
            # for semantic labels that could not be assigned uniquely.
            #
            # Example:
            #
            #   "blue cube"
            #       -> "dark blue cube"
            #       -> "light blue cube"
            #       -> "cube"
            #
            # The original semantic label is always preserved.
            for label, requested_count in object_counts.items():
                if (
                    label.strip().lower()
                    == "storage bin"
                ):
                    continue

                resolved_count = accepted_counts[
                    label
                ]

                missing_count = (
                    requested_count
                    - resolved_count
                )

                if missing_count <= 0:
                    continue

                print(
                    "[ScenePerceiver] Unresolved semantic label: "
                    f"{label!r}, "
                    f"requested={requested_count}, "
                    f"resolved={resolved_count}"
                )

                label_parts = (
                    label
                    .strip()
                    .lower()
                    .split()
                )

                if len(label_parts) < 2:
                    raise RuntimeError(
                        "Semantic detector label does not follow "
                        "the expected '<color> <object type>' format: "
                        f"{label!r}"
                    )

                color = label_parts[0]
                object_type = label_parts[-1]

                fallback_queries = [
                    f"dark {color} {object_type}",
                    f"light {color} {object_type}",
                    object_type,
                ]

                for fallback_query in fallback_queries:
                    if missing_count <= 0:
                        break

                    print(
                        "[ScenePerceiver] Trying fallback: "
                        f"{label!r} -> {fallback_query!r}"
                    )

                    timing_query = (
                        fallback_query
                        .replace(" ", "_")
                    )

                    with TIMING.measure(
                        f"scene.grounding_dino.fallback_{timing_query}",
                        cuda=True,
                    ):
                        (
                            fallback_boxes,
                            fallback_logits,
                            fallback_phrases,
                        ) = predict(
                            model=self.grounding_model,
                            image=image,
                            caption=fallback_query,
                            box_threshold=semantic_object_box_threshold,
                            text_threshold=text_threshold,
                            device=self.device,
                        )

                    print(
                        f"[DINO FALLBACK] {label!r} "
                        f"using {fallback_query!r}: "
                        f"{len(fallback_boxes)} candidates"
                    )

                    for i, (box, logit) in enumerate(
                        zip(
                            fallback_boxes,
                            fallback_logits,
                        )
                    ):
                        print(
                            f"  [{i}] "
                            f"box={box.detach().cpu().tolist()} "
                            f"score="
                            f"{float(logit.detach().cpu().item()):.4f}"
                        )

                    # Same small-object sanity filter used
                    # for semantic non-bin objects.
                    if len(fallback_boxes) > 0:
                        box_areas = (
                            fallback_boxes[:, 2]
                            * fallback_boxes[:, 3]
                        )

                        keep_mask = (
                            box_areas < 0.1
                        )

                        fallback_boxes = (
                            fallback_boxes[
                                keep_mask
                            ]
                        )

                        fallback_logits = (
                            fallback_logits[
                                keep_mask
                            ]
                        )

                        keep_values = (
                            keep_mask
                            .detach()
                            .cpu()
                            .tolist()
                        )

                        fallback_phrases = [
                            phrase
                            for phrase, keep in zip(
                                fallback_phrases,
                                keep_values,
                            )
                            if keep
                        ]

                    fallback_candidates = [
                        {
                            # IMPORTANT:
                            # preserve the original semantic label.
                            "label": label,
                            "box": box,
                            "confidence": float(
                                logit
                                .detach()
                                .cpu()
                                .item()
                            ),
                        }
                        for box, logit in zip(
                            fallback_boxes,
                            fallback_logits,
                        )
                    ]

                    fallback_candidates.sort(
                        key=lambda candidate: candidate[
                            "confidence"
                        ],
                        reverse=True,
                    )

                    for candidate in fallback_candidates:
                        if missing_count <= 0:
                            break

                        candidate_box_xyxy = (
                            box_ops.box_cxcywh_to_xyxy(
                                candidate["box"].unsqueeze(0)
                            )
                        )

                        conflict = False

                        # Protect semantic objects that have
                        # already been assigned.
                        for accepted in accepted_semantic:
                            accepted_box_xyxy = (
                                box_ops.box_cxcywh_to_xyxy(
                                    accepted[
                                        "box"
                                    ].unsqueeze(0)
                                )
                            )

                            iou_matrix, _ = box_ops.box_iou(
                                candidate_box_xyxy,
                                accepted_box_xyxy,
                            )

                            iou = float(
                                iou_matrix[0, 0]
                                .detach()
                                .cpu()
                                .item()
                            )

                            if (
                                iou
                                >= semantic_iou_threshold
                            ):
                                print(
                                    "[ScenePerceiver] Fallback candidate rejected: "
                                    f"{label!r} "
                                    f"query={fallback_query!r} "
                                    f"score={candidate['confidence']:.4f} "
                                    f"overlaps semantic object "
                                    f"{accepted['label']!r}, "
                                    f"IoU={iou:.4f}"
                                )

                                conflict = True
                                break

                        if conflict:
                            continue

                        # Protect storage-bin regions as well.
                        for existing_box, existing_label in zip(
                            detected_boxes,
                            detected_labels,
                        ):
                            if (
                                existing_label
                                .strip()
                                .lower()
                                != "storage bin"
                            ):
                                continue

                            existing_box_xyxy = (
                                box_ops.box_cxcywh_to_xyxy(
                                    existing_box
                                )
                            )

                            iou_matrix, _ = box_ops.box_iou(
                                candidate_box_xyxy,
                                existing_box_xyxy,
                            )

                            iou = float(
                                iou_matrix[0, 0]
                                .detach()
                                .cpu()
                                .item()
                            )

                            if (
                                iou
                                >= semantic_iou_threshold
                            ):
                                print(
                                    "[ScenePerceiver] Fallback candidate rejected: "
                                    f"{label!r} "
                                    f"query={fallback_query!r} "
                                    f"score={candidate['confidence']:.4f} "
                                    "overlaps storage bin, "
                                    f"IoU={iou:.4f}"
                                )

                                conflict = True
                                break

                        if conflict:
                            continue

                        print(
                            "[ScenePerceiver] Fallback candidate accepted: "
                            f"{label!r} "
                            f"localized using {fallback_query!r}, "
                            f"score={candidate['confidence']:.4f}"
                        )

                        accepted_semantic.append(
                            candidate
                        )

                        accepted_counts[
                            label
                        ] += 1

                        missing_count -= 1

            for candidate in accepted_semantic:
                detected_boxes.append(
                    candidate["box"].unsqueeze(0)
                )

                detected_labels.append(
                    candidate["label"]
                )

                detected_confidences.append(
                    candidate["confidence"]
                )

        if not detected_boxes:
            return []

        boxes = torch.cat(
            detected_boxes,
            dim=0,
        )

        # GroundingDINO boxes are normalized with respect to the
        # cropped detector image. Convert them back to full-frame
        # normalized coordinates before SAM uses them.
        boxes = remap_grounding_dino_boxes_to_full_frame(
            boxes,
            crop_info=dino_crop_info,
        )

        height, width, _ = image_source.shape

        boxes_xyxy = (
            box_ops.box_cxcywh_to_xyxy(boxes)
            * torch.tensor(
                [
                    width,
                    height,
                    width,
                    height,
                ],
                dtype=boxes.dtype,
                device=boxes.device,
            )
        )
        
        with TIMING.measure(
            "scene.sam",
            cuda=True,
        ):

            self.sam_predictor.set_image(
                image_source
            )

            transformed_boxes = (
                self.sam_predictor.transform
                .apply_boxes_torch(
                    boxes_xyxy,
                    image_source.shape[:2],
                )
                .to(self.device)
            )

            masks, _, _ = self.sam_predictor.predict_torch(
                point_coords=None,
                point_labels=None,
                boxes=transformed_boxes,
                multimask_output=False,
            )

        masks_np = (
            masks
            .detach()
            .cpu()
            .numpy()
        )

        detections: list[DetectedObject] = []

        image_area = height * width

        label_counters: dict[str, int] = {
            label: 0
            for label in object_counts
        }

        for detector_label, confidence, mask in zip(
            detected_labels,
            detected_confidences,
            masks_np,
        ):
            object_mask = mask[0] > 0

            mask_area = int(
                np.count_nonzero(
                    object_mask
                )
            )

            # Same sanity check used by SeeDo visual prompting.
            if mask_area > image_area * 0.3:
                continue

            mask_indices = np.argwhere(
                object_mask
            )

            if len(mask_indices) == 0:
                continue

            avg_y, avg_x = np.mean(
                mask_indices,
                axis=0,
            )

            pixel = (
                int(avg_x),
                int(avg_y),
            )

            object_index = label_counters[
                detector_label
            ]

            label_counters[
                detector_label
            ] += 1

            normalized_label = (
                detector_label
                .strip()
                .lower()
                .replace(" ", "_")
            )

            object_id = (
                f"{normalized_label}_{object_index}"
            )

            metadata = (
                metadata_by_label.get(detector_label)
                if self.perception_mode == "generalized"
                else None
            )

            if (
                self.perception_mode == "generalized"
                and metadata is None
            ):
                raise RuntimeError(
                    "Missing discovery metadata for detected label: "
                    f"{detector_label!r}"
                )

            detections.append(
                DetectedObject(
                    object_id=object_id,
                    label=detector_label,
                    pixel_coordinates=pixel,
                    mask=object_mask,
                    confidence=confidence,
                    category=(
                        metadata["category"]
                        if metadata is not None
                        else None
                    ),
                    attributes=(
                        dict(metadata["attributes"])
                        if metadata is not None
                        else {}
                    ),
                )
            )

        return detections

    def _parse_base_to_table_transform(
        self,
        transform: dict[str, Any],
    ) -> tuple[np.ndarray, np.ndarray]:
        if not isinstance(transform, dict):
            raise TypeError(
                "base_to_table_transform must be a dictionary."
            )

        if "rotation" not in transform:
            raise ValueError(
                "base_to_table_transform is missing 'rotation'."
            )

        if "translation" not in transform:
            raise ValueError(
                "base_to_table_transform is missing 'translation'."
            )

        rotation = np.asarray(
            transform["rotation"],
            dtype=np.float64,
        )

        translation = np.asarray(
            transform["translation"],
            dtype=np.float64,
        )

        if rotation.shape != (3, 3):
            raise ValueError(
                "Transform rotation must have shape (3, 3)."
            )

        if translation.shape != (3,):
            raise ValueError(
                "Transform translation must have shape (3,)."
            )

        return rotation, translation

    def _build_raw_scene_overlay(
        self,
        rgb_image: np.ndarray,
        raw_scene: RawSceneState,
    ) -> np.ndarray:
        if not isinstance(rgb_image, np.ndarray):
            raise TypeError(
                "rgb_image must be a numpy array."
            )

        prompted = rgb_image.copy()

        for obj in raw_scene.objects:
            mask = obj.mask

            if mask is None:
                continue

            mask_bool = np.asarray(mask).astype(bool)

            if mask_bool.ndim != 2:
                raise ValueError(
                    f"Invalid mask shape for {obj.object_id}: "
                    f"{mask_bool.shape}"
                )

            if mask_bool.shape != prompted.shape[:2]:
                raise ValueError(
                    f"Mask shape {mask_bool.shape} does not match "
                    f"RGB shape {prompted.shape[:2]} "
                    f"for {obj.object_id}."
                )

            contours, _ = cv2.findContours(
                mask_bool.astype(np.uint8),
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )

            cv2.drawContours(
                prompted,
                contours,
                -1,
                (255, 255, 255),
                2,
            )

            u, v = obj.pixel_coordinates

            cv2.circle(
                prompted,
                (int(u), int(v)),
                4,
                (255, 255, 255),
                -1,
            )

            label = obj.object_id

            cv2.putText(
                prompted,
                label,
                (
                    int(u) + 6,
                    int(v) - 6,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        return prompted

    def run(
        self,
        rgb_image: np.ndarray,
        depth_image: np.ndarray,
        camera_info: dict[str, Any],
        base_to_table_transform: dict[str, Any],
        artifacts_dir: Path | None = None,
    ) -> ScenePerceptionResult:
        """Perceive the initial robot scene and return a frozen RawSceneState."""

        if not isinstance(rgb_image, np.ndarray):
            raise TypeError(
                "rgb_image must be a numpy array."
            )

        if rgb_image.ndim != 3:
            raise ValueError(
                "rgb_image must have shape (H, W, C)."
            )

        if not isinstance(depth_image, np.ndarray):
            raise TypeError(
                "depth_image must be a numpy array."
            )

        if depth_image.ndim != 2:
            raise ValueError(
                "depth_image must have shape (H, W)."
            )

        if rgb_image.shape[:2] != depth_image.shape:
            raise ValueError(
                "RGB and depth images must have the same spatial resolution."
            )

        camera_matrix = self.camera_matrix_from_info(
            camera_info
        )

        rotation, translation = (
            self._parse_base_to_table_transform(
                base_to_table_transform
            )
        )

        #
        # Sample camera calibration noise ONCE for this complete run.
        # The same perturbed camera pose is therefore used for every
        # object detected in the scene.
        #
        camera_pose_noise = self._sample_camera_pose_noise()

        runtime_camera_calibration = (
            self._build_noisy_camera_calibration(
                camera_pose_noise
            )
        )

        # Preserve the exact geometry used for this perception pass.
        # A later task-specific refinement must not resample camera noise
        # or reconstruct a potentially different transform.
        self._last_runtime_camera_calibration = {
            "position": np.asarray(
                runtime_camera_calibration["position"],
                dtype=np.float64,
            ).copy(),
            "orientation_matrix": np.asarray(
                runtime_camera_calibration["orientation_matrix"],
                dtype=np.float64,
            ).copy(),
        }

        self._last_base_to_table_rotation = np.asarray(
            rotation,
            dtype=np.float64,
        ).copy()

        self._last_base_to_table_translation = np.asarray(
            translation,
            dtype=np.float64,
        ).copy()

        sampled_translation_mm = (
            camera_pose_noise["translation_m"] * 1000.0
        )

        sampled_rotation_deg = np.rad2deg(
            camera_pose_noise["rotation_vector_rad"]
        )

        print(
            "[ScenePerceiver] Camera pose noise "
            f"level='{self.camera_pose_noise_level}' | "
            f"translation [mm]={sampled_translation_mm.tolist()} | "
            f"rotation [deg]={sampled_rotation_deg.tolist()}",
            flush=True,
        )

        detections = self._detect_objects(
            rgb_image,
            artifacts_dir=artifacts_dir,
        )

        if not detections:
            raise RuntimeError(
                "No objects were detected in the runtime scene."
            )

        raw_objects: list[RawSceneObject] = []

        with TIMING.measure(
            "scene.geometry"
        ):

            for detection in detections:

                object_id = detection.object_id
                label = detection.label
                pixel = detection.pixel_coordinates

                position_base = self.pixel_to_base_link(
                    pixel=pixel,
                    depth_image=depth_image,
                    camera_matrix=camera_matrix,
                    base_to_table_rotation=rotation,
                    base_to_table_translation=translation,
                    camera_calibration_override=runtime_camera_calibration,
                )

                depth = robust_depth_at(
                    depth_image,
                    pixel[0],
                    pixel[1],
                )

                if depth is None:
                    raise RuntimeError(
                        f"No valid depth for object '{object_id}' "
                        f"at pixel {pixel}."
                    )

                position_camera_np = deproject_pixel(
                    pixel[0],
                    pixel[1],
                    depth,
                    camera_matrix,
                )

                position_camera = tuple(
                    float(value)
                    for value in position_camera_np
                )

                raw_objects.append(
                    RawSceneObject(
                        object_id=object_id,
                        label=label,
                        pixel_coordinates=pixel,
                        position_camera=position_camera,
                        position_base=position_base,
                        mask=detection.mask,
                        confidence=detection.confidence,
                        category=detection.category,
                        attributes=dict(detection.attributes),
                    )
                )

        raw_scene = RawSceneState(
            objects=tuple(raw_objects)
        )

        overlay_image_path = None
        raw_scene_json_path = None

        if artifacts_dir is not None:

            with TIMING.measure(
                "io.scene_perception_artifacts"
            ):

                artifacts_dir = (
                    Path(artifacts_dir)
                    .expanduser()
                    .resolve()
                )

                artifacts_dir.mkdir(
                    parents=True,
                    exist_ok=True,
                )

                original_camera_calibration = (
                    self.camera_calibration[
                        self.camera_name
                    ]
                )

                camera_pose_noise_json_path = (
                    artifacts_dir
                    / "camera_pose_noise.json"
                )

                with camera_pose_noise_json_path.open(
                    "w",
                    encoding="utf-8",
                ) as stream:
                    json.dump(
                        {
                            "level": self.camera_pose_noise_level,

                            "distribution": {
                                "translation_mean_mm": 0.0,
                                "translation_std_mm_per_axis": (
                                    self.translation_noise_std_mm
                                ),
                                "rotation_mean_deg": 0.0,
                                "rotation_std_deg_per_axis": (
                                    self.rotation_noise_std_deg
                                ),
                            },

                            "sampled_translation_mm": (
                                sampled_translation_mm.tolist()
                            ),

                            "sampled_rotation_deg": (
                                sampled_rotation_deg.tolist()
                            ),

                            "translation_error_norm_mm": float(
                                np.linalg.norm(
                                    sampled_translation_mm
                                )
                            ),

                            "rotation_error_norm_deg": float(
                                np.linalg.norm(
                                    sampled_rotation_deg
                                )
                            ),

                            "original_calibration": {
                                "position_m": (
                                    np.asarray(
                                        original_camera_calibration[
                                            "position"
                                        ]
                                    ).tolist()
                                ),
                                "orientation_matrix": (
                                    np.asarray(
                                        original_camera_calibration[
                                            "orientation_matrix"
                                        ]
                                    ).tolist()
                                ),
                            },

                            "perturbed_calibration": {
                                "position_m": (
                                    runtime_camera_calibration[
                                        "position"
                                    ].tolist()
                                ),
                                "orientation_matrix": (
                                    runtime_camera_calibration[
                                        "orientation_matrix"
                                    ].tolist()
                                ),
                            },
                        },
                        stream,
                        indent=2,
                        ensure_ascii=False,
                    )

                overlay = self._build_raw_scene_overlay(
                    rgb_image=rgb_image,
                    raw_scene=raw_scene,
                )

                overlay_image_path = (
                    artifacts_dir
                    / "raw_scene_overlay.png"
                )

                saved = cv2.imwrite(
                    str(overlay_image_path),
                    cv2.cvtColor(
                        overlay,
                        cv2.COLOR_RGB2BGR,
                    ),
                )

                if not saved:
                    raise RuntimeError(
                        f"Could not save raw scene overlay: "
                        f"{overlay_image_path}"
                    )

                raw_scene_json_path = (
                    artifacts_dir
                    / "raw_scene_state.json"
                )

                with raw_scene_json_path.open(
                    "w",
                    encoding="utf-8",
                ) as stream:
                    json.dump(
                        {
                            "objects": [
                                {
                                    "object_id": obj.object_id,
                                    "label": obj.label,
                                    "pixel_coordinates": list(
                                        obj.pixel_coordinates
                                    ),
                                    "confidence": obj.confidence,
                                    "position_camera": list(
                                        obj.position_camera
                                    ),
                                    "position_base": list(
                                        obj.position_base
                                    ),
                                    "label": obj.label,
                                    **(
                                        {
                                            "category": obj.category,
                                            "attributes": obj.attributes,
                                        }
                                        if self.perception_mode == "generalized"
                                        else {}
                                    ),
                                }
                                for obj in raw_scene.objects
                            ]
                        },
                        stream,
                        indent=2,
                        ensure_ascii=False,
                    )

                # -------------------------------------------------
                # Raw SAM masks.
                #
                # Keep the exact segmentation masks produced during
                # this perception pass so that task-specific geometry
                # and offline diagnostics can be replayed without
                # invoking GroundingDINO/SAM again.
                # -------------------------------------------------

                masks_dir = (
                    artifacts_dir
                    / "masks"
                )

                masks_dir.mkdir(
                    parents=True,
                    exist_ok=True,
                )

                mask_manifest: dict[
                    str,
                    str,
                ] = {}

                for obj in raw_scene.objects:

                    if obj.mask is None:
                        continue

                    mask = np.asarray(
                        obj.mask,
                        dtype=bool,
                    )

                    if mask.ndim != 2:
                        raise RuntimeError(
                            "Invalid raw SAM mask shape for "
                            f"{obj.object_id!r}: "
                            f"{mask.shape}."
                        )

                    if (
                        mask.shape
                        != rgb_image.shape[:2]
                    ):
                        raise RuntimeError(
                            "Raw SAM mask shape does not match "
                            "the runtime RGB image for "
                            f"{obj.object_id!r}: "
                            f"mask={mask.shape}, "
                            f"rgb={rgb_image.shape[:2]}."
                        )

                    safe_object_id = re.sub(
                        r"[^A-Za-z0-9_.-]+",
                        "_",
                        str(
                            obj.object_id
                        ),
                    ).strip(
                        "._"
                    )

                    if not safe_object_id:
                        raise RuntimeError(
                            "Could not build a valid artifact "
                            "filename for raw object "
                            f"{obj.object_id!r}."
                        )

                    mask_path = (
                        masks_dir
                        / (
                            f"{safe_object_id}"
                            ".png"
                        )
                    )

                    mask_image = (
                        mask.astype(
                            np.uint8
                        )
                        * 255
                    )

                    saved = cv2.imwrite(
                        str(
                            mask_path
                        ),
                        mask_image,
                    )

                    if not saved:
                        raise RuntimeError(
                            "Could not save raw SAM mask: "
                            f"{mask_path}"
                        )

                    mask_manifest[
                        obj.object_id
                    ] = str(
                        mask_path.relative_to(
                            artifacts_dir
                        )
                    )

                mask_manifest_path = (
                    artifacts_dir
                    / "mask_manifest.json"
                )

                with mask_manifest_path.open(
                    "w",
                    encoding="utf-8",
                ) as stream:
                    json.dump(
                        mask_manifest,
                        stream,
                        indent=2,
                        ensure_ascii=False,
                    )

        return ScenePerceptionResult(
            raw_scene=raw_scene,
            overlay_image_path=overlay_image_path,
            raw_scene_json_path=raw_scene_json_path,
        )