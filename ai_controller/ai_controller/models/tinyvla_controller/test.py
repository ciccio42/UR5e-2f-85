"""
Unit test for TinyVLAController: model loading + one inference pass.

This is an integration-style test: it loads the real TinyVLA (Llava-Pythia)
checkpoint referenced by tinyvla_config.yaml and runs a forward pass on
synthetic camera images, so it needs the checkpoint files (base model + LoRA
weights + dataset_stats.pkl) present on disk plus a GPU.

Usage
-----
    python test.py                              # uses ./tinyvla_config.yaml
    python test.py --config /path/to/other.yaml  # use a different config
    python -m unittest test.py                   # via unittest runner

Env vars
--------
    TINYVLA_TEST_CONFIG   overrides the default config path.
"""

import argparse
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tinyvla_controller import TinyVLAController
from tinyvla_utils import TINYVLA_IMAGE_SIZE

DEFAULT_CONFIG_PATH = os.environ.get(
    "TINYVLA_TEST_CONFIG",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "tinyvla_config.yaml"),
)


def make_dummy_image(height: int = 480, width: int = 640) -> np.ndarray:
    """Random RGB uint8 image, shaped like a real camera frame."""
    rng = np.random.default_rng(0)
    return rng.integers(0, 256, size=(height, width, 3), dtype=np.uint8)


def make_dummy_state() -> np.ndarray:
    """7-D [eef_pos(3), eef_quat(4)] vector, matching _build_tinyvla_state's output."""
    return np.array([0.3, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0])


class TestTinyVLAController(unittest.TestCase):
    """Loads the model once for the whole test class (expensive step)."""

    config_path = DEFAULT_CONFIG_PATH
    controller = None

    @classmethod
    def setUpClass(cls):
        if not os.path.isfile(cls.config_path):
            raise unittest.SkipTest(f"Config file not found: {cls.config_path}")
        try:
            cls.controller = TinyVLAController(model_config=cls.config_path)
        except FileNotFoundError as exc:
            raise unittest.SkipTest(f"Checkpoint files not available: {exc}")

    def test_model_loaded(self):
        self.assertIsNotNone(self.controller.model)
        self.assertFalse(self.controller.model.training, "Model should be in eval() mode")

    def test_pre_process_packs_obs_dict(self):
        images = [make_dummy_image(), make_dummy_image()]
        states = make_dummy_state()

        obs = self.controller.pre_process([images, states])

        self.assertIn("camera_front_image", obs)
        self.assertIn("eye_in_hand_image", obs)
        # pre_process crops (TASK_CROP) + resizes both views to the model's
        # 224x224 input, so they no longer match the raw camera frames exactly.
        expected_shape = (TINYVLA_IMAGE_SIZE, TINYVLA_IMAGE_SIZE, 3)
        self.assertEqual(obs["camera_front_image"].shape, expected_shape)
        self.assertEqual(obs["eye_in_hand_image"].shape, expected_shape)
        self.assertIn("eef_pos", obs)
        self.assertIn("eef_quat", obs)
        self.assertEqual(obs["eef_pos"].shape, (3,))
        self.assertEqual(obs["eef_quat"].shape, (4,))

    def test_inference_returns_valid_action(self):
        self.controller.reset()
        images = [make_dummy_image(), make_dummy_image()]
        states = make_dummy_state()

        actions = self.controller.inference([images, states], t=0, save_path='./test_outputs')

        self.assertIsInstance(actions, list)
        self.assertEqual(len(actions), 1)
        action = np.asarray(actions[0]).flatten()
        self.assertEqual(len(action), 7)
        self.assertTrue(np.all(np.isfinite(action)))
        self.assertIn(action[-1], (0.0, 255.0))


def _parse_args_and_configure():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG_PATH,
        help="Path to the TinyVLA YAML config to load for testing.",
    )
    args, remaining = parser.parse_known_args()
    TestTinyVLAController.config_path = args.config
    sys.argv = [sys.argv[0]] + remaining


if __name__ == "__main__":
    _parse_args_and_configure()
    unittest.main()
