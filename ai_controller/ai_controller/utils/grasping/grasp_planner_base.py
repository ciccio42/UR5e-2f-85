from __future__ import annotations

from pathlib import Path
from typing import Protocol

import numpy as np


class GraspPlanResult(Protocol):
    """
    Common result exposed by every SeeDo grasp-planning backend.

    Backend-specific planners may expose additional fields, but
    SeeDoController must depend only on the fields declared here.
    """

    grasp_instruction: str
    semantic_point_px: np.ndarray

    tcp_position_base: np.ndarray
    tcp_orientation_base: np.ndarray
    approach_direction_base: np.ndarray

    confidence: float


class GraspPlannerBackend(Protocol):
    """
    Common interface implemented by every SeeDo grasp-planning backend.
    """

    backend_name: str

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
    ) -> GraspPlanResult:
        ...