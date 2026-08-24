"""
Integration test for MimicVideoController: model loading, temporal input and inference.

The test loads the real Cosmos Video2World and Action Head checkpoints configured
in mimic_video_config.yaml. It therefore requires the Mimic Video environment,
the checkpoint files and a CUDA GPU.

Usage
-----
    python test.py
    python test.py --config /path/to/config.yaml --task-id 01
    python test.py --config /path/to/runtime_t5.yaml --command "Pick the blue box"

Environment variables
---------------------
    MIMIC_VIDEO_TEST_CONFIG   overrides the default config path.
    MIMIC_VIDEO_TEST_TASK_ID  selects a precomputed embedding (default: 01).
    MIMIC_VIDEO_TEST_COMMAND  provides the command for runtime T5.
"""

import argparse
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mimic_video_controller import MimicVideoController


DEFAULT_CONFIG_PATH = os.environ.get(
    "MIMIC_VIDEO_TEST_CONFIG",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "mimic_video_config.yaml"),
)
DEFAULT_TASK_ID = os.environ.get("MIMIC_VIDEO_TEST_TASK_ID", "01")
DEFAULT_COMMAND = os.environ.get(
    "MIMIC_VIDEO_TEST_COMMAND",
    "Pick the blue box and place it into the first bin",
)


def make_dummy_image(seed: int, height: int = 480, width: int = 640) -> np.ndarray:
    """Create one deterministic RGB uint8 frame with the camera input shape."""
    rng = np.random.default_rng(seed)
    return rng.integers(0, 256, size=(height, width, 3), dtype=np.uint8)


def make_dummy_state() -> np.ndarray:
    """Return [EEF position, XYZW quaternion, gripper state] in the expected 8D format."""
    return np.array([0.3, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0, 0.0], dtype=np.float64)


class TestMimicVideoController(unittest.TestCase):
    """Load the expensive Mimic Video pipelines once for the whole test class."""

    config_path = DEFAULT_CONFIG_PATH
    task_id = DEFAULT_TASK_ID
    command = DEFAULT_COMMAND
    controller = None

    @classmethod
    def setUpClass(cls):
        if not os.path.isfile(cls.config_path):
            raise unittest.SkipTest(f"Config file not found: {cls.config_path}")
        try:
            cls.controller = MimicVideoController(model_config=cls.config_path)
        except (FileNotFoundError, EnvironmentError) as exc:
            raise unittest.SkipTest(f"Mimic Video assets not available: {exc}") from exc

    def setUp(self):
        self.controller.reset()

    def _load_command(self):
        if self.controller.cfg.language_conditioning == "runtime_t5":
            self.controller.load_command("", task_id=None, command=self.command)
        else:
            self.controller.load_command("", task_id=self.task_id)

    def test_model_loaded(self):
        self.assertIsNotNone(self.controller.model)
        self.assertFalse(self.controller.model.training, "Model should be in eval() mode")

    def test_pre_process_builds_five_frame_input(self):
        state = make_dummy_state()
        processed = None

        for frame_index in range(5):
            processed = self.controller.pre_process(
                [[make_dummy_image(frame_index)], state]
            )
            self.assertEqual(tuple(processed["state"].shape), (1, 1, 10))
            self.assertTrue(np.all(np.isfinite(processed["state"].numpy())))
            if frame_index < 4:
                self.assertIsNone(processed["video"])

        self.assertEqual(tuple(processed["video"].shape), (1, 3, 5, 480, 640))
        self.assertTrue(np.all(np.isfinite(processed["video"].numpy())))

    def test_inference_returns_one_absolute_action(self):
        self._load_command()
        state = make_dummy_state()

        for frame_index in range(4):
            self.controller.pre_process([[make_dummy_image(frame_index)], state])

        actions = self.controller.inference(
            [[make_dummy_image(4)], state],
            t=0,
            save_path="./test_outputs",
        )

        self.assertIsInstance(actions, list)
        self.assertEqual(len(actions), 1)

        action = np.asarray(actions[0]).flatten()
        self.assertEqual(action.shape, (8,))
        self.assertTrue(np.all(np.isfinite(action)))
        self.assertAlmostEqual(float(np.linalg.norm(action[3:7])), 1.0, places=4)
        self.assertIn(float(action[7]), (0.0, 255.0))


def _parse_args_and_configure():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG_PATH,
        help="Path to the Mimic Video YAML config used for testing.",
    )
    parser.add_argument(
        "--task-id",
        default=DEFAULT_TASK_ID,
        help="Task ID used when language_conditioning is precomputed.",
    )
    parser.add_argument(
        "--command",
        default=DEFAULT_COMMAND,
        help="Text command used when language_conditioning is runtime_t5.",
    )
    args, remaining = parser.parse_known_args()
    TestMimicVideoController.config_path = args.config
    TestMimicVideoController.task_id = args.task_id
    TestMimicVideoController.command = args.command
    sys.argv = [sys.argv[0]] + remaining


if __name__ == "__main__":
    _parse_args_and_configure()
    unittest.main()
