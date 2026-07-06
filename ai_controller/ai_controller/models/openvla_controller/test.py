"""
Unit test for OpenVLAController: model loading + one inference pass.

This is an integration-style test: it loads the real OpenVLA model (downloads
~14 GB of weights from HuggingFace on first run unless already cached) and
runs a forward pass on synthetic camera images, so it takes a while and needs
a GPU (or a lot of RAM/patience on CPU) plus network access on first run.

Usage
-----
    python test.py                              # uses ./openvla_config.yaml
    python test.py --config /path/to/other.yaml  # use a different config
    python -m unittest test.py                   # via unittest runner

Env vars
--------
    OPENVLA_TEST_CONFIG   overrides the default config path.
"""

import argparse
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from openvla_controller import OpenVLAController

DEFAULT_CONFIG_PATH = os.environ.get(
    "OPENVLA_TEST_CONFIG",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "openvla_config.yaml"),
)


def make_dummy_image(height: int = 480, width: int = 640) -> np.ndarray:
    """Random RGB uint8 image, shaped like a real camera frame."""
    rng = np.random.default_rng(0)
    return rng.integers(0, 256, size=(height, width, 3), dtype=np.uint8)


class TestOpenVLAController(unittest.TestCase):
    """Loads the model once for the whole test class (expensive step)."""

    config_path = DEFAULT_CONFIG_PATH
    controller = None

    @classmethod
    def setUpClass(cls):
        if not os.path.isfile(cls.config_path):
            raise unittest.SkipTest(f"Config file not found: {cls.config_path}")
        cls.controller = OpenVLAController(model_config=cls.config_path)

    def test_model_loaded(self):
        self.assertIsNotNone(self.controller.model)
        self.assertFalse(self.controller.model.training, "Model should be in eval() mode")

        model_device = next(self.controller.model.parameters()).device
        self.assertEqual(model_device.type, self.controller.device.type)

    def test_pre_process_packs_obs_dict(self):
        # pre_process only packs raw images/state into the obs dict; resizing and
        # center-cropping happen downstream in openvla_utils.get_vla_action.
        cfg = self.controller.cfg
        num_cams = 2 if cfg.num_images_in_input > 1 else 1
        images = [make_dummy_image() for _ in range(num_cams)]
        states = np.zeros(cfg.proprio_dim) if cfg.use_proprio else None

        obs, returned_states = self.controller.pre_process([images, states])

        self.assertIn("full_image", obs)
        np.testing.assert_array_equal(obs["full_image"], images[0])
        if cfg.num_images_in_input > 1:
            self.assertIn("camera_gripper_image", obs)
            np.testing.assert_array_equal(obs["camera_gripper_image"], images[1])
        if cfg.use_proprio:
            self.assertIn("state", obs)
            self.assertEqual(obs["state"].shape, (cfg.proprio_dim,))

    def test_inference_returns_valid_actions(self):
        cfg = self.controller.cfg
        num_cams = 2 if cfg.num_images_in_input > 1 else 1
        images = [make_dummy_image() for _ in range(num_cams)]
        states = np.zeros(cfg.proprio_dim) if cfg.use_proprio else None

        actions = self.controller.inference([images, states], 
                                            t=0,
                                            save_path='./test_outputs')

        self.assertIsInstance(actions, list)
        self.assertGreater(len(actions), 0)
        self.assertLessEqual(len(actions), cfg.num_open_loop_steps)
        for action in actions:
            self.assertEqual(len(np.asarray(action).flatten()), 7)
            self.assertTrue(np.all(np.isfinite(np.asarray(action))))


def _parse_args_and_configure():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG_PATH,
        help="Path to the OpenVLA YAML config to load for testing.",
    )
    args, remaining = parser.parse_known_args()
    TestOpenVLAController.config_path = args.config
    sys.argv = [sys.argv[0]] + remaining


if __name__ == "__main__":
    _parse_args_and_configure()
    unittest.main()
