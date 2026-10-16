"""One-shot, idempotent integration of lightweight selection into graspgenx_v2.

Run from the repository copy of this file, *not* on a robot while it moves.
Edits existing V2 backend, worker, factory and YAML in place after checking
anchors and Python syntax. Saves pre-change versions under .lightweight_backup.
No branch checkout or GitHub write occurs.
"""
from __future__ import annotations

import ast
from datetime import datetime
from pathlib import Path


def replace_once(src: str, before: str, after: str, where: str) -> str:
    n = src.count(before)
    if n != 1:
        raise RuntimeError(f"{where}: expected one occurrence, found {n}: {before[:65]!r}")
    return src.replace(before, after, 1)


def make_backend(s: str) -> str:
    label = "graspgenx_v2/backend.py"
    s = replace_once(s,
        '        v2_max_expensive_checks: int = 80,\n',
        '        v2_max_expensive_checks: int = 0,\n'
        '        v2_validation_mode: str = "detailed",\n'
        '        v2_semantic_radius_px: float = 100.0,\n', label)
    s = replace_once(s,
        '        self.v2_max_expensive_checks = int(v2_max_expensive_checks)\n',
        '        self.v2_max_expensive_checks = int(v2_max_expensive_checks)\n'
        '        self.v2_validation_mode = str(v2_validation_mode).strip().lower()\n'
        '        if self.v2_validation_mode not in ("detailed", "lightweight"):\n'
        '            raise ValueError("V2 validation mode must be detailed/lightweight")\n'
        '        self.v2_semantic_radius_px = float(v2_semantic_radius_px)\n'
        '        if not (0 < self.v2_semantic_radius_px <= 1000):\n'
        '            raise ValueError("Invalid V2 semantic radius")\n', label)
    s = replace_once(s,
        '        self._v2_last_details = None\n\n    def _compute_plan',
        '        self._v2_last_details = None\n'
        '        if (self.v2_validation_mode == "lightweight"\n'
        '                and self.v2_unsafe_enable_robot_commands):\n'
        '            raise RuntimeError(\n'
        '                "Lightweight is offline-only: refuse robot-command enable"\n'
        '            )\n\n'
        '    def _compute_plan', label)
    s = replace_once(s,
        '                "status": "OFFLINE_GEOMETRIC_PLAN_NOT_ROBOT_SAFE",\n',
        '                "status": ("LIGHTWEIGHT_POSE_ONLY_NOT_ROBOT_SAFE"\n'
        '                           if self.v2_validation_mode == "lightweight"\n'
        '                           else "OFFLINE_GEOMETRIC_PLAN_NOT_ROBOT_SAFE"),\n', label)
    s = replace_once(s,
        '''        for path in (
            self.v2_driver_urdf,
            self.v2_worker_python,
        ):
            if not path.is_file():
                raise FileNotFoundError(
                    f"V2 dependency missing: {path}"
                )

        if not self.v2_robotiq_mesh_share.is_dir():
            raise FileNotFoundError(
                "V2 Robotiq package missing: "
                f"{self.v2_robotiq_mesh_share}"
            )
''',
        '''        dependencies = [self.v2_worker_python]
        if self.v2_validation_mode == "detailed":
            dependencies.append(self.v2_driver_urdf)
        for path in dependencies:
            if not path.is_file():
                raise FileNotFoundError(f"V2 dependency missing: {path}")

        if (self.v2_validation_mode == "detailed"
                and not self.v2_robotiq_mesh_share.is_dir()):
            raise FileNotFoundError(
                "V2 Robotiq package missing: "
                f"{self.v2_robotiq_mesh_share}"
            )
''', label)
    s = replace_once(s,
        '                "--max-expensive-checks", str(self.v2_max_expensive_checks),\n',
        '                "--max-expensive-checks", str(self.v2_max_expensive_checks),\n'
        '                "--validation-mode", self.v2_validation_mode,\n'
        '                "--semantic-radius-px", str(self.v2_semantic_radius_px),\n', label)
    return s


def make_worker(s: str) -> str:
    label = "graspgenx_v2/runtime_worker.py"
    s = replace_once(s,
        'from .runtime_core import GraspGenXV2Geometry, V2Config\n',
        'from .runtime_core import GraspGenXV2Geometry, V2Config\n'
        'from .lightweight_core import GraspGenXV2LightweightGeometry\n', label)
    s = replace_once(s,
        '    max_expensive_checks: int,\n) -> None:\n',
        '    max_expensive_checks: int,\n'
        '    validation_mode: str = "detailed",\n'
        '    semantic_radius_px: float = 100.0,\n) -> None:\n', label)
    s = replace_once(s,
        '        max_expensive_checks=int(max_expensive_checks),\n',
        '        max_expensive_checks=int(max_expensive_checks),\n'
        '        semantic_radius_px=float(semantic_radius_px),\n', label)
    s = replace_once(s,
        '''    selector = GraspGenXV2Geometry(
        urdf,
        mesh_share,
        cfg,
    )
''',
        '''    if validation_mode not in {"lightweight", "detailed"}:
        raise ValueError(f"Unknown V2 validation mode: {validation_mode}")
    selector_class = (
        GraspGenXV2LightweightGeometry
        if validation_mode == "lightweight"
        else GraspGenXV2Geometry
    )
    selector = selector_class(urdf, mesh_share, cfg)
''', label)
    s = replace_once(s,
        '    result = {\n        "index": int(choice.index),\n',
        '    result = {\n'
        '        "validation_mode": validation_mode,\n'
        '        "index": int(choice.index),\n', label)
    s = replace_once(s,
        '        "status": "GEOMETRY_PREFILTERED_NOT_ROBOT_SAFE",\n',
        '        "status": ("LIGHTWEIGHT_POSE_ONLY_NOT_ROBOT_SAFE"\n'
        '                   if validation_mode == "lightweight"\n'
        '                   else "GEOMETRY_PREFILTERED_NOT_ROBOT_SAFE"),\n', label)
    s = replace_once(s,
        '''    parser.add_argument(
        "--max-expensive-checks",
        type=int,
        default=80,
    )
''',
        '''    parser.add_argument(
        "--max-expensive-checks",
        type=int,
        default=0,
    )
    parser.add_argument(
        "--validation-mode", choices=("detailed", "lightweight"),
        default="detailed",
    )
    parser.add_argument(
        "--semantic-radius-px", type=float, default=100.0,
    )
''', label)
    s = replace_once(s,
        '        args.max_expensive_checks,\n    )\n',
        '        args.max_expensive_checks,\n'
        '        args.validation_mode,\n'
        '        args.semantic_radius_px,\n    )\n', label)
    return s


def make_factory(s: str) -> str:
    label = "grasp_planner_factory.py"
    s = replace_once(s,
        '            "v2_max_expensive_checks": int(grasp_config.get("graspgenx_v2_max_expensive_checks", 80)),\n',
        '            "v2_max_expensive_checks": int(grasp_config.get("graspgenx_v2_max_expensive_checks", 0)),\n'
        '            "v2_validation_mode": str(grasp_config.get("graspgenx_v2_validation_mode", "detailed")),\n'
        '            "v2_semantic_radius_px": float(grasp_config.get("semantic_radius_px", 100.0)),\n', label)
    before = '''            semantic_radius_px=float(
                grasp_config.get(
                    "semantic_radius_px",
                    50.0,
                )
            ),
        )

    raise ValueError('''
    after = '''            semantic_radius_px=float(
                grasp_config.get(
                    "semantic_radius_px",
                    50.0,
                )
            ),
            grasp_depth_offset_m=float(
                grasp_config.get(
                    "graspgenx_v2_depth_offset_m" if is_v2 else "graspgenx_depth_offset_m",
                    0.0 if is_v2 else 0.005,
                )
            ),
        )

    raise ValueError('''
    s = replace_once(s, before, after, label)
    return s


def make_yaml(s: str) -> str:
    label = "seedo_controller.yaml"
    s = replace_once(s,
        '  graspgenx_v2_max_expensive_checks: 80\n',
        '  graspgenx_v2_max_expensive_checks: 0  # exhaustive; 0=all\n'
        '  graspgenx_v2_validation_mode: lightweight\n'
        '  graspgenx_v2_depth_offset_m: 0.0\n', label)
    return s


def main():
    here = Path(__file__).resolve().parent
    repo_py = here.parents[2]  # ai_controller/ai_controller
    source = {
        here / "backend.py": make_backend,
        here / "runtime_worker.py": make_worker,
        here.parent / "grasp_planner_factory.py": make_factory,
        repo_py / "models/seedo_controller/config/seedo_controller.yaml": make_yaml,
    }
    if not (here / "lightweight_core.py").is_file():
        raise FileNotFoundError("Copy lightweight_core.py next to this installer")
    # All-or-nothing preparation: check every anchor and syntax before edits.
    updates = {}
    for path, patch in source.items():
        original = path.read_text(encoding="utf-8")
        if 'v2_validation_mode' in original and path.name != 'seedo_controller.yaml':
            print("Already patched:", path)
            continue
        if path.name == 'seedo_controller.yaml' and 'graspgenx_v2_validation_mode:' in original:
            print("Already patched:", path)
            continue
        updated = patch(original)
        if path.suffix == ".py":
            ast.parse(updated, filename=str(path))
        updates[path] = (original, updated)
    if not updates:
        print("No changes needed. Lightweight mode is already installed.")
        return

    backup_dir = here / ".lightweight_backup" / datetime.now().strftime('%Y%m%d_%H%M%S')
    backup_dir.mkdir(parents=True, exist_ok=False)
    for idx, (path, (old, new)) in enumerate(updates.items()):
        backup = backup_dir / f"{idx}_{path.name}"
        backup.write_text(old, encoding="utf-8")
        path.write_text(new, encoding="utf-8")
        print("Updated:", path)
    print("Originals backed up in:", backup_dir)
    print("NOTE: lightweight pose output is OFFLINE-ONLY and NOT robot-safe.")
    print("Run your existing offline replay test; verify module paths if using a non-symlink build.")


if __name__ == "__main__":
    main()
