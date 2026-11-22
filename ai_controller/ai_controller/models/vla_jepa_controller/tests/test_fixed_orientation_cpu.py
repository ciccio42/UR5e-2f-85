"""Check YAML quaternion override through the actual post_process source.

No robot, checkpoint, ROS, Qwen or GPU required. A small AST extraction is used
because importing the whole ROS/LeRobot controller would load heavy dependencies.
Run: python -m unittest discover -s tests -v
"""
from __future__ import annotations

import ast
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml

CTRL_DIR = Path(__file__).resolve().parents[1]
CONTROLLER_SOURCE = CTRL_DIR / "vla_jepa_controller.py"
FIXED = np.asarray([
    0.9994452044624775,
    0.03161651380119412,
    0.0021438049655468088,
    -0.010251021036213035,
], dtype=np.float32)


def load_functions():
    tree = ast.parse(CONTROLLER_SOURCE.read_text(encoding="utf-8"))
    parse_fn = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                    and node.name == "_parse_fixed_orientation_xyzw")
    controller = next(node for node in tree.body if isinstance(node, ast.ClassDef)
                      and node.name == "VLAJEPAController")
    post_process = next(node for node in controller.body if isinstance(node, ast.FunctionDef)
                        and node.name == "post_process")
    # Load the original function bodies without importing the robot runtime.
    parse_fn.returns = None
    context = {"np": np}
    exec(compile(ast.Module(body=[parse_fn, post_process], type_ignores=[]),
                 str(CONTROLLER_SOURCE), "exec"), context)
    return context


class TestFixedOrientation(unittest.TestCase):
    def setUp(self):
        self.ns = load_functions()
        self.parse = self.ns["_parse_fixed_orientation_xyzw"]
        self.post_process = self.ns["post_process"]
        self.ref_quat = np.array([0, 0, 0, 1], dtype=np.float32)

        self.ns["validate_vla_jepa_action"] = lambda x: np.asarray(x, dtype=np.float32)
        self.ns["gripper_binary_to_moveit"] = (lambda gripper_state, open_position,
                    closed_position: closed_position if gripper_state else open_position)

    def fake_controller(self, fixed=None, grip=0):
        self.ns["delta_action_to_absolute_target"] = (
            lambda **kwargs: (np.array([0.1, 0.2, 0.3], dtype=np.float32),
                              self.ref_quat.copy(), grip)
        )
        return SimpleNamespace(
            fixed_orientation_xyzw=fixed,
            dataset_action_scale=0.05,
            grasp_z_offset_m=-0.01,
            _grasp_z_offset_applied=False,
            gripper_open_position=0.0,
            gripper_closed_position=255.0,
        )

    def run_post(self, controller, current_closed=False):
        return self.post_process(controller, {
            "action": np.zeros(7, dtype=np.float32),
            "reference_position": np.array([0.1, 0.2, 0.3], dtype=np.float32),
            "reference_quaternion": self.ref_quat,
            "current_gripper_closed": current_closed,
        })

    def test_yaml_commented_by_default_in_both_modes(self):
        for name in ("vla_jepa_config.yaml", "vla_jepa_config_wm_conditioned.yaml"):
            source = (CTRL_DIR / name).read_text(encoding="utf-8")
            doc = yaml.safe_load(source)
            self.assertNotIn("fixed_orientation_xyzw", doc)
            self.assertIn("# fixed_orientation_xyzw:", source)
            enabled_source = source.replace("# fixed_orientation_xyzw:",
                                            "fixed_orientation_xyzw:")
            for line in ("0.9994452044624775", "0.03161651380119412",
                         "0.0021438049655468088", "-0.010251021036213035"):
                enabled_source = enabled_source.replace("#   - " + line, "  - " + line)
            active = yaml.safe_load(enabled_source)
            np.testing.assert_allclose(self.parse(active["fixed_orientation_xyzw"]),
                                       FIXED, atol=1e-7, rtol=0.0)

    def test_default_preserves_model_quaternion(self):
        controller = self.fake_controller(fixed=self.parse(None))
        out = self.run_post(controller)
        self.assertEqual(out.shape, (8,))
        np.testing.assert_array_equal(out[3:7], self.ref_quat)
        np.testing.assert_allclose(out[:3], [0.1, 0.2, 0.3])
        self.assertEqual(float(out[7]), 0.0)

    def test_enabled_overrides_only_quaternion_each_action(self):
        controller = self.fake_controller(fixed=self.parse(FIXED))
        for i in range(7):  # seven actions, including those from the same cached chunk
            out = self.run_post(controller)
            np.testing.assert_array_equal(out[3:7], FIXED)
            np.testing.assert_allclose(out[:3], [0.1, 0.2, 0.3])
            self.assertEqual(float(out[7]), 0.0)

    def test_enabled_preserves_grasp_offset_and_gripper(self):
        controller = self.fake_controller(fixed=self.parse(FIXED), grip=1)
        out = self.run_post(controller)
        np.testing.assert_allclose(out[:3], [0.1, 0.18, 0.29], atol=1e-7)
        np.testing.assert_array_equal(out[3:7], FIXED)
        self.assertEqual(float(out[7]), 255.0)
        self.assertTrue(controller._grasp_z_offset_applied)

    def test_invalid_quaternions_fail_closed(self):
        for v in ([1, 0, 0], [0, 0, 0, 0], [2, 0, 0, 0],
                  [float("nan"), 0, 0, 1], ["bad", 0, 0, 1]):
            with self.subTest(value=v):
                with self.assertRaises(ValueError):
                    self.parse(v)


if __name__ == "__main__":
    unittest.main()
