"""
OpenVLA policy wrapper.

Builds and queries the OpenVLA model the same way VLA-Bench's
robosuite_test/models/openvla.py (`open_vla_policy`) does: model / processor /
action-head / proprio-projector / noisy-action-projector construction all go
through openvla_utils, and inference goes through openvla_utils.get_vla_action.
"""

import os
import sys
from typing import Any, Dict, List, Union

import numpy as np
import torch

# openvla_utils.py lives alongside this file, same as in the reference repo
# where it's dropped onto PYTHONPATH next to robosuite_test/models/openvla.py.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from openvla_utils import (
    check_unnorm_key,
    get_action_head,
    get_model,
    get_noisy_action_projector,
    get_processor,
    get_proprio_projector,
    get_vla_action,
)


class OpenVLAPolicy:
    """Loads an OpenVLA checkpoint and predicts actions from observations."""

    def __init__(self, cfg: Any):
        self.cfg = cfg

        self.model = get_model(cfg)

        self.proprio_projector = None
        if cfg.use_proprio:
            self.proprio_projector = get_proprio_projector(
                cfg, self.model.llm_dim, proprio_dim=cfg.proprio_dim
            )

        self.action_head = None
        if cfg.use_l1_regression or cfg.use_diffusion:
            self.action_head = get_action_head(cfg, self.model.llm_dim)

        self.noisy_action_projector = None
        if cfg.use_diffusion:
            self.noisy_action_projector = get_noisy_action_projector(cfg, self.model.llm_dim)

        self.processor = get_processor(cfg)
        check_unnorm_key(cfg, self.model)

    def get_action(self, obs: Dict[str, Any], task_label: str) -> Union[List[np.ndarray], np.ndarray]:
        """Query the model to get action predictions for one observation."""
        with torch.no_grad():
            return get_vla_action(
                cfg=self.cfg,
                vla=self.model,
                processor=self.processor,
                obs=obs,
                task_label=task_label,
                action_head=self.action_head,
                proprio_projector=self.proprio_projector,
                noisy_action_projector=self.noisy_action_projector,
                use_film=self.cfg.use_film,
            )
