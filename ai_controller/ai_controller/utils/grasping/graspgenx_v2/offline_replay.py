
"""GraspGen-X V2: offline selection and open/contact visual comparison.

Uses SAVED RGB, depth and grasp candidates. It does not rerun GraspMolmo,
SAM or GraspGen-X; it never commands a robot.
"""
from __future__ import annotations

import argparse
import json
import tempfile
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from .frames import fixed_chain_transform
from .gripper_model import RobotiqURDFModel
from .runtime_core import NoValidGrasp, V2Config, fit_table_plane
from .runtime_worker import run
from .scene_geometry import load_case_scene


STATUS_OK = "GEOMETRY_PREFILTERED_NOT_ROBOT_SAFE"


def _project(points, T_base_camera, K):
    camera = (
        points - T_base_camera[:3, 3]
    ) @ T_base_camera[:3, :3]

    z = camera[:, 2]
    valid = np.isfinite(camera).all(axis=1) & (z > 0.03)

    uv = np.full((len(points), 2), np.nan, dtype=float)

    uv[valid, 0] = (
        K[0, 0] * camera[valid, 0] / z[valid] + K[0, 2]
    )
    uv[valid, 1] = (
        K[1, 1] * camera[valid, 1] / z[valid] + K[1, 2]
    )

    return uv, valid


def _vertices_base(mesh, T):
    return (
        np.asarray(mesh.vertices) @ T[:3, :3].T
        + T[:3, 3]
    )


def _parts(model, q, include_all):
    parts = model.collision_parts(float(q))

    return {
        name: mesh
        for name, mesh in parts.items()
        if include_all or "finger_tip_link" in name
    }


def _color(name):
    if "left" in name:
        return (45, 215, 55)

    if "right" in name:
        return (0, 150, 255)

    return (215, 75, 195)


def _edges(
    canvas,
    uv,
    valid,
    faces,
    color,
    face_limit=320,
):
    stride = max(
        1,
        int(np.ceil(len(faces) / face_limit)),
    )

    h, w = canvas.shape[:2]

    for tri in faces[::stride]:
        if not valid[tri].all():
            continue

        xy = uv[tri]

        if (
            not np.isfinite(xy).all()
            or np.max(np.abs(xy)) > 10000
        ):
            continue

        if (
            xy[:, 0].max() < 0
            or xy[:, 0].min() >= w
            or xy[:, 1].max() < 0
            or xy[:, 1].min() >= h
        ):
            continue

        cv2.polylines(
            canvas,
            [
                np.rint(xy)
                .astype(np.int32)
                .reshape(-1, 1, 2)
            ],
            True,
            color,
            1,
            cv2.LINE_AA,
        )


def _header(image, *lines):
    height = 13 + 22 * len(lines)

    cv2.rectangle(
        image,
        (0, 0),
        (image.shape[1], height),
        (23, 23, 23),
        -1,
    )

    for i, line in enumerate(lines):
        cv2.putText(
            image,
            str(line)[:110],
            (9, 22 + 22 * i),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (245, 245, 245),
            1,
            cv2.LINE_AA,
        )


def _semantic(image, pixel, mask, radius_px):
    tint = image.copy()

    tint[mask, 1] = np.maximum(
        tint[mask, 1],
        160,
    )

    cv2.addWeighted(
        tint,
        0.22,
        image,
        0.78,
        0,
        image,
    )

    center = tuple(
        np.rint(pixel).astype(int)
    )

    cv2.circle(
        image,
        center,
        int(radius_px),
        (255, 0, 255),
        1,
        cv2.LINE_AA,
    )

    cv2.drawMarker(
        image,
        center,
        (255, 0, 255),
        cv2.MARKER_CROSS,
        18,
        2,
        cv2.LINE_AA,
    )


def _rgb_stage(
    rgb,
    scene,
    selected,
    T,
    model,
    q,
    text,
    all_links,
):
    image = rgb.copy()

    _semantic(
        image,
        selected["semantic_pixel"],
        scene.target_mask.astype(bool),
        radius_px=50,
    )

    if T is not None:
        for name, mesh in _parts(
            model,
            q,
            all_links,
        ).items():

            xyz = _vertices_base(mesh, T)

            uv, valid = _project(
                xyz,
                scene.T_base_camera,
                scene.K,
            )

            _edges(
                image,
                uv,
                valid,
                np.asarray(mesh.faces),
                _color(name),
            )

        # Direction of approach
        axis = T[:3, 2]

        xyz = np.stack((
            T[:3, 3] - 0.075 * axis,
            T[:3, 3],
        ))

        uv, valid = _project(
            xyz,
            scene.T_base_camera,
            scene.K,
        )

        if valid.all():
            start, end = [
                tuple(np.rint(p).astype(int))
                for p in uv
            ]

            cv2.arrowedLine(
                image,
                start,
                end,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
                tipLength=0.17,
            )

    _header(
        image,
        text,
        "Green=left, orange=right; purple=Molmo +/-50px",
    )

    return image


def _make_projector(scene, poses_meshes):
    target = np.asarray(
        scene.target_points_base,
        dtype=float,
    )

    focus = np.median(target, axis=0)

    forward = np.array(
        [-0.5, 0.65, -0.60],
        dtype=float,
    )
    forward /= np.linalg.norm(forward)

    right = np.cross(
        forward,
        [0.0, 0.0, 1.0],
    )
    right /= np.linalg.norm(right)

    up = np.cross(right, forward)
    up /= np.linalg.norm(up)

    cloud = [
        target[
            ::max(1, len(target) // 3500)
        ]
    ]

    cloud += [
        xyz
        for mesh_group in poses_meshes
        for xyz in mesh_group.values()
    ]

    all_pts = np.vstack(cloud)

    coords = np.column_stack((
        (all_pts - focus) @ right,
        (all_pts - focus) @ up,
    ))

    extent = max(
        0.09,
        float(np.percentile(
            np.max(np.abs(coords), axis=1),
            99,
        )),
    )

    scale = min(
        1100 * 0.42,
        760 * 0.38,
    ) / extent

    def project(xyz):
        shift = np.asarray(xyz) - focus

        return np.column_stack((
            550 + scale * (shift @ right),
            440 - scale * (shift @ up),
        ))

    return project, focus


def _view_3d(
    scene,
    table_point,
    table_normal,
    meshes,
    project,
    T,
    heading,
):
    width, height = 1100, 760

    canvas = np.full(
        (height, width, 3),
        248,
        dtype=np.uint8,
    )

    target = np.asarray(
        scene.target_points_base
    )

    center = np.median(target, axis=0)

    def table_z(x, y):
        return (
            table_point[2]
            - (
                table_normal[0] * (x - table_point[0])
                + table_normal[1] * (y - table_point[1])
            ) / table_normal[2]
        )

    for delta in np.linspace(-0.15, 0.15, 7):
        xs = np.array([
            center[0] - 0.15,
            center[0] + 0.15,
        ])

        ys = np.full(
            2,
            center[1] + delta,
        )

        line_a = np.column_stack((
            xs,
            ys,
            [
                table_z(x, y)
                for x, y in zip(xs, ys)
            ],
        ))

        ys2 = np.array([
            center[1] - 0.15,
            center[1] + 0.15,
        ])

        xs2 = np.full(
            2,
            center[0] + delta,
        )

        line_b = np.column_stack((
            xs2,
            ys2,
            [
                table_z(x, y)
                for x, y in zip(xs2, ys2)
            ],
        ))

        for line in (line_a, line_b):
            pt0, pt1 = np.rint(
                project(line)
            ).astype(np.int32)

            cv2.line(
                canvas,
                tuple(pt0),
                tuple(pt1),
                (210, 210, 210),
                1,
                cv2.LINE_AA,
            )

    points = target[
        ::max(1, len(target) // 4000)
    ]

    for pt in np.rint(
        project(points)
    ).astype(np.int32):

        if (
            0 <= pt[0] < width
            and 0 <= pt[1] < height
        ):
            cv2.circle(
                canvas,
                tuple(pt),
                2,
                (215, 155, 45),
                -1,
                cv2.LINE_AA,
            )

    for name, (mesh, xyz) in meshes.items():
        uv = project(xyz)

        valid = np.isfinite(
            uv
        ).all(axis=1)

        _edges(
            canvas,
            uv,
            valid,
            np.asarray(mesh.faces),
            _color(name),
            550,
        )

    if T is not None:
        axis = T[:3, 2]

        p0, p1 = np.rint(
            project(np.stack((
                T[:3, 3] - 0.08 * axis,
                T[:3, 3],
            )))
        ).astype(np.int32)

        cv2.arrowedLine(
            canvas,
            tuple(p0),
            tuple(p1),
            (0, 140, 240),
            2,
            cv2.LINE_AA,
            tipLength=0.14,
        )

    _header(
        canvas,
        heading,
        "Blue=observed target, gray=estimated table",
        "Wireframe=URDF collision geometry",
    )

    return canvas


def _save_image(path, image):
    if not cv2.imwrite(str(path), image):
        raise OSError(
            f"Cannot write {path}"
        )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--case-dir",
        required=True,
        type=Path,
    )

    parser.add_argument(
        "--urdf",
        type=Path,
        default=Path(
            "/test_real/urdf_audit/"
            "ur5e_driver_installed_audit.urdf"
        ),
    )

    parser.add_argument(
        "--mesh-share",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--open-q",
        type=float,
        default=0.1,
    )

    parser.add_argument(
        "--max-expensive-checks",
        type=int,
        default=80,
    )

    parser.add_argument(
        "--table-margin-mm",
        type=float,
        default=5.0,
    )

    parser.add_argument(
        "--current-quat",
        type=float,
        nargs=4,
        metavar=("QX", "QY", "QZ", "QW"),
    )

    parser.add_argument(
        "--out-dir",
        type=Path,
    )

    parser.add_argument(
        "--all-links",
        action="store_true",
        help=(
            "Draw all Robotiq collision links; "
            "default fingertips only"
        ),
    )

    args = parser.parse_args()

    case = args.case_dir.resolve()

    artifacts = (
        case
        / "grasp_planner"
        / "action_00"
    )

    diag = (
        artifacts
        / "graspgenx_diagnostics.npz"
    )

    rgb_path = (
        artifacts
        / "input_rgb.png"
    )

    if not diag.is_file() or not rgb_path.is_file():
        raise FileNotFoundError(
            f"Saved inputs missing: {diag} or {rgb_path}"
        )

    scene = load_case_scene(case)

    rgb = cv2.imread(
        str(rgb_path),
        cv2.IMREAD_COLOR,
    )

    if (
        rgb is None
        or rgb.shape[:2] != scene.target_mask.shape
    ):
        raise RuntimeError(
            "Saved RGB missing or not aligned "
            "with target depth mask"
        )

    with np.load(
        diag,
        allow_pickle=False,
    ) as data:

        payload = {
            key: data[key]
            for key in (
                "grasps_base",
                "confidences",
                "branch_tags",
                "semantic_point_px",
                "T_base_camera",
                "T_base_table",
            )
        }

    semantic = (
        payload["semantic_point_px"].tolist()
    )

    payload.update(
        camera_matrix=scene.K,
        target_points_base=scene.target_points_base,
        background_points_base=scene.background_points_base,
    )

    if args.current_quat is not None:
        quat = np.asarray(
            args.current_quat,
            dtype=float,
        )

        if (
            not np.isfinite(quat).all()
            or np.linalg.norm(quat) < 1e-9
        ):
            raise ValueError(
                "Invalid current TCP orientation"
            )

        Rotation.from_quat(quat)

        payload["current_tcp_orientation"] = quat

    else:
        print(
            "[REPLAY] No current TCP quaternion: "
            "rotation preference disabled"
        )

    dest = (
        args.out_dir
        or Path("/test_real/graspgenx_v2_replays") / case.name
    )

    dest.mkdir(
        parents=True,
        exist_ok=True,
    )

    with tempfile.TemporaryDirectory(
        prefix="graspgenx_v2_visual_"
    ) as tmp:

        inp = Path(tmp) / "input.npz"
        out = Path(tmp) / "output.json"

        np.savez_compressed(inp, **payload)

        try:
            run(
                inp,
                out,
                args.urdf,
                args.mesh_share,
                args.open_q,
                args.table_margin_mm,
                args.max_expensive_checks,
            )

            result = json.loads(
                out.read_text()
            )

        except NoValidGrasp as exc:
            result = {
                "status": "NO_VALID_GRASP",
                "reason": str(exc),
            }

    result.update(
        semantic_pixel=semantic,
        source_rgb=str(rgb_path),
        source_candidates=str(diag),
        current_tcp_quat_xyzw=args.current_quat,
        type="SAVED_CANDIDATE_REPLAY",
        interpretation=(
            "Geometric offline prefilter. Depth is partial; "
            "actual contact and grasp stability are unverified."
        ),
    )

    model = RobotiqURDFModel(
        args.urdf,
        args.mesh_share,
    )

    table_point, table_normal, table_fit = (
        fit_table_plane(
            scene.background_points_base,
            scene.T_base_table,
            V2Config(
                table_margin_m=(
                    args.table_margin_mm / 1000.0
                )
            ),
        )
    )

    result["table_fit"] = table_fit

    pose = None
    contact_q = None

    if result["status"] == STATUS_OK:
        pose = np.asarray(
            result["T_base_gripper"],
            dtype=float,
        )

        contact_q = result.get(
            "closure_first_contact_q_rad"
        )

        if contact_q is not None:
            contact_q = float(contact_q)

        T_tool_gripper = fixed_chain_transform(
            args.urdf,
            "tool0",
            "robotiq_85_base_link",
        )

        T_tool_tcp = fixed_chain_transform(
            args.urdf,
            "tool0",
            "tcp_link",
        )

        T_base_tcp = (
            pose
            @ np.linalg.inv(T_tool_gripper)
            @ T_tool_tcp
        )

        result["tcp_position_base"] = (
            T_base_tcp[:3, 3].tolist()
        )

        result["tcp_quat_xyzw"] = (
            Rotation.from_matrix(
                T_base_tcp[:3, :3]
            ).as_quat().tolist()
        )

        if args.current_quat is not None:
            d = (
                Rotation.from_quat(
                    args.current_quat
                ).inv()
                * Rotation.from_matrix(
                    T_base_tcp[:3, :3]
                )
            )

            result["rotation_deg"] = float(
                np.degrees(d.magnitude())
            )

        else:
            result["rotation_deg"] = None

        semantic_score = (
            1.0 - min(
                float(
                    result["semantic_3d_gap_m"]
                ) / 0.005,
                1.0,
            )
        )

        confidence_score = float(
            np.clip(
                result["confidence"],
                0.0,
                1.0,
            )
        )

        if result["rotation_deg"] is None:
            result["ranking_score_recomputed"] = (
                0.25 * confidence_score
                + 0.05 * semantic_score
            ) / 0.30

        else:
            rotation_score = (
                1.0
                - min(
                    result["rotation_deg"] / 180.0,
                    1.0,
                )
            )

            result["ranking_score_recomputed"] = (
                0.70 * rotation_score
                + 0.25 * confidence_score
                + 0.05 * semantic_score
            )

    original = rgb.copy()

    _header(
        original,
        "ORIGINAL RGB",
        "Saved frame (no new model inference)",
    )

    if pose is None:
        open_image = _rgb_stage(
            rgb,
            scene,
            result,
            None,
            model,
            args.open_q,
            "NO VALID GRASP",
            args.all_links,
        )

        contact_image = open_image.copy()

    else:
        rotation_text = result.get(
            "rotation_deg"
        )

        open_image = _rgb_stage(
            rgb,
            scene,
            result,
            pose,
            model,
            args.open_q,
            (
                f"OPEN q={args.open_q:.3f} "
                f"| {result['branch']} #{result['index']} "
                f"| rot={rotation_text}"
            ),
            args.all_links,
        )

        if contact_q is None:
            contact_image = _rgb_stage(
                rgb,
                scene,
                result,
                pose,
                model,
                args.open_q,
                "CONTACT NOT OBSERVED",
                args.all_links,
            )

        else:
            contact_image = _rgb_stage(
                rgb,
                scene,
                result,
                pose,
                model,
                contact_q,
                (
                    f"PREDICTED FIRST CONTACT "
                    f"q={contact_q:.4f} rad"
                ),
                args.all_links,
            )

    parts_open = (
        _parts(
            model,
            args.open_q,
            args.all_links,
        )
        if pose is not None
        else {}
    )

    parts_contact = (
        _parts(
            model,
            contact_q,
            args.all_links,
        )
        if pose is not None and contact_q is not None
        else parts_open
    )

    meshes_open = {
        name: (
            mesh,
            _vertices_base(mesh, pose),
        )
        for name, mesh in parts_open.items()
    }

    meshes_contact = {
        name: (
            mesh,
            _vertices_base(mesh, pose),
        )
        for name, mesh in parts_contact.items()
    }

    project, _ = _make_projector(
        scene,
        [
            {
                name: xyz
                for name, (_, xyz) in meshes_open.items()
            },
            {
                name: xyz
                for name, (_, xyz) in meshes_contact.items()
            },
        ],
    )

    view_open = _view_3d(
        scene,
        table_point,
        table_normal,
        meshes_open,
        project,
        pose,
        f"OPEN q={args.open_q:.3f} rad",
    )

    view_contact = _view_3d(
        scene,
        table_point,
        table_normal,
        meshes_contact,
        project,
        pose,
        (
            "PREDICTED FIRST CONTACT "
            + (
                f"q={contact_q:.4f} rad"
                if contact_q is not None
                else "NOT AVAILABLE"
            )
        ),
    )

    outputs = {
        "grasp_rgb_comparison.png": np.hstack((
            original,
            open_image,
        )),
        "grasp_rgb_contact.png": np.hstack((
            original,
            contact_image,
        )),
        "grasp_rgb_stages.png": np.hstack((
            original,
            open_image,
            contact_image,
        )),
        "grasp_3d_view.png": view_open,
        "grasp_3d_contact.png": view_contact,
        "grasp_3d_stages.png": np.hstack((
            view_open,
            view_contact,
        )),
    }

    for name, data in outputs.items():
        _save_image(
            dest / name,
            data,
        )

    result["render_open_q_rad"] = args.open_q
    result["render_contact_q_rad"] = contact_q

    result["render_links"] = (
        "all"
        if args.all_links
        else "fingertips_only"
    )

    result["output_files"] = [
        str(dest / name)
        for name in outputs
    ] + [
        str(dest / "v2_result.json")
    ]

    (
        dest / "v2_result.json"
    ).write_text(
        json.dumps(result, indent=2)
    )

    print("\nRESULT:", result["status"])

    for path in result["output_files"]:
        print(path)

    print(
        "NOTE: first contact is a geometric prediction, "
        "not a verified physical grasp."
    )


if __name__ == "__main__":
    main()
