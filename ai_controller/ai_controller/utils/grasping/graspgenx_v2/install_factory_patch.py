"""Patch ONLY the SeeDo planner factory to register 'graspgenx_v2'.

Usage: python3 install_factory_patch.py [absolute_factory_path]
Does not edit URDFs, ROS driver, or motion layer.
"""
from __future__ import annotations
from pathlib import Path
import sys

path = (Path(sys.argv[1]) if len(sys.argv) > 1 else
        Path('ai_controller/ai_controller/utils/grasping/grasp_planner_factory.py'))
source = path.read_text(encoding='utf-8')

marker = 'from ai_controller.utils.grasping.graspgenx_v2.backend import GraspGenXV2Planner\n'
if marker not in source:
    needle = 'from ai_controller.utils.grasping.graspgenx.planner import (\n    GraspGenXPlanner,\n)\n'
    if source.count(needle) != 1:
        raise RuntimeError('Factory import structure changed; refused to edit')
    source = source.replace(needle, needle + marker, 1)

original = '    if backend == "graspgenx":\n        return GraspGenXPlanner(\n'
replacement = '''    if backend in ("graspgenx", "graspgenx_v2"):
        is_v2 = backend == "graspgenx_v2"
        planner_class = GraspGenXV2Planner if is_v2 else GraspGenXPlanner
        v2_options = ({
            "v2_driver_urdf": str(grasp_config.get(
                "graspgenx_v2_driver_urdf",
                "/test_real/urdf_audit/ur5e_driver_installed_audit.urdf",
            )),
            "v2_robotiq_mesh_share": str(grasp_config.get(
                "graspgenx_v2_robotiq_mesh_share",
                "/home/ros2_ws/install/robotiq_description/share/robotiq_description",
            )),
            "v2_worker_python": str(grasp_config.get(
                "graspgenx_v2_worker_python", "/opt/graspgenx_venv/bin/python",
            )),
            "v2_gripper_open_q": float(grasp_config.get("graspgenx_v2_gripper_open_q", 0.1)),
            "v2_table_margin_mm": float(grasp_config.get("graspgenx_v2_table_margin_mm", 5.0)),
            "v2_max_expensive_checks": int(grasp_config.get("graspgenx_v2_max_expensive_checks", 80)),
            "v2_unsafe_enable_robot_commands": bool(grasp_config.get(
                "graspgenx_v2_unsafe_enable_robot_commands", False,
            )),
        } if is_v2 else {})
        return planner_class(
            **v2_options,
'''
if replacement not in source:
    if source.count(original) != 1:
        raise RuntimeError('Factory backend signature changed; refused to edit')
    source = source.replace(original, replacement, 1)
if "Supported backends are 'm2t2' and 'graspgenx'." in source:
    source = source.replace(
        "Supported backends are 'm2t2' and 'graspgenx'.",
        "Supported backends are 'm2t2', 'graspgenx', and 'graspgenx_v2'.",
    )
compile(source, str(path), 'exec')
path.write_text(source, encoding='utf-8')
print('Patched factory:', path)
