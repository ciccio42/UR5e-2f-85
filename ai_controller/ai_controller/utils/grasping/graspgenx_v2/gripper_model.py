
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

from .frames import from_xyz_rpy, validate_transform


GRIPPER_BASE = "robotiq_85_base_link"
ACTUATED_JOINT = "robotiq_85_left_knuckle_joint"


def xml_origin(element):
    origin = element.find("origin")

    if origin is None:
        return np.eye(4)

    xyz = [
        float(x)
        for x in origin.get("xyz", "0 0 0").split()
    ]
    rpy = [
        float(x)
        for x in origin.get("rpy", "0 0 0").split()
    ]

    return from_xyz_rpy(xyz, rpy)


class RobotiqURDFModel:
    """
    Articulated Robotiq 2F-85 collision geometry.

    All geometry is expressed in robotiq_85_base_link.
    q is the left-knuckle joint position in radians.
    """

    def __init__(self, urdf_path, mesh_package_dir):
        self.urdf_path = Path(urdf_path)
        self.mesh_package_dir = Path(mesh_package_dir)

        root = ET.parse(self.urdf_path).getroot()

        self.links = {
            link.get("name"): link
            for link in root.findall("link")
        }

        self.by_child = {}

        for joint in root.findall("joint"):
            child = joint.find("child").get("link")
            axis_xml = joint.find("axis")
            mimic_xml = joint.find("mimic")

            axis = np.array(
                [
                    float(x)
                    for x in (
                        axis_xml.get("xyz", "1 0 0")
                        if axis_xml is not None
                        else "1 0 0"
                    ).split()
                ],
                dtype=float,
            )

            mimic = None

            if mimic_xml is not None:
                mimic = (
                    mimic_xml.get("joint"),
                    float(mimic_xml.get("multiplier", "1")),
                    float(mimic_xml.get("offset", "0")),
                )

            self.by_child[child] = {
                "name": joint.get("name"),
                "type": joint.get("type"),
                "origin": xml_origin(joint),
                "axis": axis,
                "mimic": mimic,
                "parent": joint.find("parent").get("link"),
            }

        if GRIPPER_BASE not in self.links:
            raise ValueError("Robotiq base link missing")

        if not self.mesh_package_dir.is_dir():
            raise FileNotFoundError(self.mesh_package_dir)

    def joint_position(self, joint, q):
        if joint["type"] == "fixed":
            return 0.0

        if joint["name"] == ACTUATED_JOINT:
            return float(q)

        mimic = joint["mimic"]

        if mimic is None:
            raise ValueError(
                f"Unresolved joint: {joint['name']}"
            )

        master, multiplier, offset = mimic

        if master != ACTUATED_JOINT:
            raise ValueError(
                f"Unsupported mimic master: {master}"
            )

        return multiplier * float(q) + offset

    def link_transform(self, link_name, q):
        """Return T_gripper_base_link for a specified q."""
        T = np.eye(4)
        current = link_name
        visited = set()

        while current != GRIPPER_BASE:
            if current in visited:
                raise ValueError("Cycle in joint hierarchy")

            visited.add(current)

            joint = self.by_child.get(current)

            if joint is None:
                raise ValueError(
                    f"No chain from {GRIPPER_BASE} to {link_name}"
                )

            kind = joint["type"]
            motion = np.eye(4)

            if kind in ("revolute", "continuous"):
                angle = self.joint_position(joint, q)

                axis = joint["axis"].copy()
                norm = np.linalg.norm(axis)

                if norm < 1e-12:
                    raise ValueError("Invalid joint axis")

                axis /= norm

                motion[:3, :3] = Rotation.from_rotvec(
                    axis * angle
                ).as_matrix()

            elif kind != "fixed":
                raise ValueError(
                    f"Unsupported joint type: {kind}"
                )

            T = joint["origin"] @ motion @ T
            current = joint["parent"]

        return validate_transform(T)

    def resolve_mesh(self, filename):
        prefix = "package://robotiq_description/"

        if not filename.startswith(prefix):
            raise ValueError(
                f"Unexpected mesh package: {filename}"
            )

        path = self.mesh_package_dir / filename[len(prefix):]

        if not path.is_file():
            raise FileNotFoundError(path)

        return path

    
    def collision_parts(self, q):
        """
        Return collision meshes separated by URDF link.

        Each mesh is expressed in robotiq_85_base_link.
        """
        q = float(q)

        if not np.isfinite(q) or not 0.0 <= q <= 0.8:
            raise ValueError("Robotiq q must be in [0, 0.8]")

        parts = {}

        for link_name, link in self.links.items():
            if not link_name.startswith("robotiq_85_"):
                continue

            T_base_link = self.link_transform(link_name, q)
            link_meshes = []

            for collision in link.findall("collision"):
                geometry = collision.find("geometry")

                if geometry is None:
                    continue

                mesh_xml = geometry.find("mesh")

                if mesh_xml is None:
                    raise ValueError(
                        f"Unsupported collision geometry: {link_name}"
                    )

                filename = mesh_xml.get("filename")
                path = self.resolve_mesh(filename)

                mesh = trimesh.load_mesh(
                    str(path),
                    process=False,
                )

                if not isinstance(mesh, trimesh.Trimesh):
                    raise ValueError(
                        f"Invalid triangle mesh: {path}"
                    )

                if len(mesh.faces) == 0:
                    raise ValueError(
                        f"Empty collision mesh: {path}"
                    )

                scale = np.array(
                    [
                        float(value)
                        for value in mesh_xml.get(
                            "scale", "1 1 1"
                        ).split()
                    ],
                    dtype=float,
                )

                mesh.apply_scale(scale)

                T_link_collision = xml_origin(collision)

                mesh.apply_transform(
                    T_base_link @ T_link_collision
                )

                link_meshes.append(mesh)

            if link_meshes:
                parts[link_name] = trimesh.util.concatenate(
                    link_meshes
                )

        if not parts:
            raise RuntimeError(
                "No Robotiq collision meshes found"
            )

        return parts

    def collision_mesh(self, q):
        """
        Return the full Robotiq collision mesh.

        Uses the same link geometry as collision_parts().
        """
        return trimesh.util.concatenate(
            list(self.collision_parts(q).values())
        )

