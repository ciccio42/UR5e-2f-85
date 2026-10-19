#!/usr/bin/env python3

import argparse
import json
import math
import sys
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# =============================================================================
# ROS 2 TOPICS
# =============================================================================

REQUEST_TOPIC = "/seedo/configure_scene_request"
RESPONSE_TOPIC = "/seedo/configure_scene_response"


# =============================================================================
# ENVIRONMENT TYPES
# =============================================================================

ENV_PICK_PLACE = "pick_place"
ENV_NUT_ASSEMBLY = "nut_assembly"

ALLOWED_ENV_TYPES = (
    ENV_PICK_PLACE,
    ENV_NUT_ASSEMBLY,
)


# =============================================================================
# GENERIC POSE UTILITIES
# =============================================================================

POSE_FIELDS = ("x", "y", "z", "rz")


def make_pose(x, y, z, rz=0.0):
    return {
        "x": float(x),
        "y": float(y),
        "z": float(z),
        "rz": float(rz),
    }


def copy_pose(pose):
    return {
        field: float(pose[field])
        for field in POSE_FIELDS
    }


# =============================================================================
# PICK-AND-PLACE CONFIGURATION
# =============================================================================

PICK_PLACE_CUBE_Z = 0.036

PICK_PLACE_CUBES = (
    "cube_red",
    "cube_green",
    "cube_yellow",
    "cube_blue",
)

# (row, column) -> (x, y)
PICK_PLACE_GRID = {
    (0, 0): (-0.23, 0.24),
    (0, 1): (-0.07, 0.24),
    (0, 2): (0.07, 0.24),
    (0, 3): (0.23, 0.24),

    (1, 0): (-0.23, 0.15),
    (1, 1): (-0.07, 0.15),
    (1, 2): (0.07, 0.15),
    (1, 3): (0.23, 0.15),

    (2, 0): (-0.23, 0.06),
    (2, 1): (-0.07, 0.06),
    (2, 2): (0.07, 0.06),
    (2, 3): (0.23, 0.06),
}

# Base trajectories 000-019.
# Trajectories 020-039 reuse the same layouts with rz=45 deg.
#
# NOTE: trajectory 003 uses (1, 3). The previous (0, 3) value was the
# alignment bug that put one cube one row behind the other three.
PICK_PLACE_TRAJECTORIES = {
    0: ((1, 0), [(1, 1), (1, 2), (1, 3)]),
    1: ((1, 1), [(1, 0), (1, 2), (1, 3)]),
    2: ((1, 2), [(1, 0), (1, 1), (1, 3)]),
    3: ((1, 3), [(1, 0), (1, 1), (1, 2)]),

    4: ((2, 0), [(2, 1), (2, 2), (2, 3)]),
    5: ((2, 1), [(2, 0), (2, 2), (2, 3)]),
    6: ((2, 2), [(2, 0), (2, 1), (2, 3)]),
    7: ((2, 3), [(2, 0), (2, 1), (2, 2)]),

    8: ((0, 0), [(0, 1), (0, 2), (0, 3)]),
    9: ((0, 1), [(0, 0), (0, 2), (0, 3)]),
    10: ((0, 2), [(0, 0), (0, 1), (0, 3)]),
    11: ((0, 3), [(0, 0), (0, 1), (0, 2)]),

    12: ((2, 0), [(0, 1), (2, 2), (0, 3)]),
    13: ((0, 1), [(2, 0), (2, 2), (0, 3)]),
    14: ((2, 2), [(2, 0), (0, 1), (0, 3)]),
    15: ((0, 3), [(2, 0), (0, 1), (2, 2)]),

    16: ((0, 0), [(2, 1), (0, 2), (2, 3)]),
    17: ((2, 1), [(0, 0), (0, 2), (2, 3)]),
    18: ((0, 2), [(0, 0), (2, 1), (2, 3)]),
    19: ((2, 3), [(0, 0), (2, 1), (0, 2)]),
}


def get_pick_place_target(task):
    if 0 <= task <= 3:
        return "cube_green"
    if 4 <= task <= 7:
        return "cube_yellow"
    if 8 <= task <= 11:
        return "cube_blue"
    if 12 <= task <= 15:
        return "cube_red"

    raise ValueError(
        f"Invalid Pick-and-Place task {task:02d}. Expected 00-15."
    )


def build_pick_place_configuration(task, trajectory):
    if not 0 <= task <= 15:
        raise ValueError(
            f"Invalid Pick-and-Place task {task:02d}. Expected 00-15."
        )

    if not 0 <= trajectory <= 39:
        raise ValueError(
            f"Invalid Pick-and-Place trajectory {trajectory:03d}. "
            "Expected 000-039."
        )

    target_cube = get_pick_place_target(task)

    # 020 -> 000, ..., 039 -> 019
    base_trajectory = trajectory % 20
    target_slot, other_slots = PICK_PLACE_TRAJECTORIES[base_trajectory]

    rotation_z = 45.0 if trajectory >= 20 else 0.0

    other_cubes = [
        cube
        for cube in PICK_PLACE_CUBES
        if cube != target_cube
    ]

    assignments = {target_cube: target_slot}

    for cube, slot in zip(other_cubes, other_slots):
        assignments[cube] = slot

    objects = {}

    for cube in PICK_PLACE_CUBES:
        slot = assignments[cube]
        x, y = PICK_PLACE_GRID[slot]

        objects[cube] = make_pose(
            x=x,
            y=y,
            z=PICK_PLACE_CUBE_Z,
            rz=rotation_z,
        )

    return {
        "env_type": ENV_PICK_PLACE,
        "task": int(task),
        "trajectory": int(trajectory),
        "target_object": target_cube,
        "target_destination": None,
        "objects": objects,
        "metadata": {
            "rotation_z": rotation_z,
            "assignments": assignments,
            "base_trajectory": base_trajectory,
        },
    }


# =============================================================================
# NUT-ASSEMBLY CONFIGURATION
# =============================================================================

NUT_RINGS = (
    "ring_gray",
    "ring_blue",
    "ring_yellow",
)

NUT_PEGS = (
    "peg_1",
    "peg_2",
    "peg_3",
)

# Final layout validated in Isaac Sim.
# Ring slot 1 is the left-most position.
# Ring X values are aligned one-to-one with the peg X values.
NUT_X_POSITIONS = (
    -0.203391,
    -0.001674,
    0.200044,
)

NUT_RING_Y = 0.150000
NUT_PEG_Y = -0.200000

NUT_RING_Z = 0.011000
NUT_PEG_Z = 0.011000

# The ring handle geometry points along local -Y. A 180 deg rotation therefore
# points every handle toward the robot-side table edge (+Y).
NUT_RING_RZ = 180.0
NUT_PEG_RZ = 0.0

NUT_RING_SLOT_POSES = {
    1: make_pose(NUT_X_POSITIONS[0], NUT_RING_Y, NUT_RING_Z, NUT_RING_RZ),
    2: make_pose(NUT_X_POSITIONS[1], NUT_RING_Y, NUT_RING_Z, NUT_RING_RZ),
    3: make_pose(NUT_X_POSITIONS[2], NUT_RING_Y, NUT_RING_Z, NUT_RING_RZ),
}

NUT_PEG_POSES = {
    "peg_1": make_pose(NUT_X_POSITIONS[0], NUT_PEG_Y, NUT_PEG_Z, NUT_PEG_RZ),
    "peg_2": make_pose(NUT_X_POSITIONS[1], NUT_PEG_Y, NUT_PEG_Z, NUT_PEG_RZ),
    "peg_3": make_pose(NUT_X_POSITIONS[2], NUT_PEG_Y, NUT_PEG_Z, NUT_PEG_RZ),
}

# The current Nut-Assembly dataset contains one trajectory per task.
# The target ring is always placed in slot 1 (left-most).
# This explicit table is intentionally future-proof: additional trajectories can
# later place the target in slot 2/3 without changing the request protocol.
NUT_TRAJECTORIES = {
    0: {
        "target_slot": 1,
        "other_slots": (2, 3),
    },
}


def get_nut_assembly_target(task):
    """
    Task mapping:

      0: ring_gray   -> peg_3
      1: ring_gray   -> peg_2
      2: ring_gray   -> peg_1

      3: ring_yellow -> peg_3
      4: ring_yellow -> peg_2
      5: ring_yellow -> peg_1

      6: ring_blue   -> peg_3
      7: ring_blue   -> peg_2
      8: ring_blue   -> peg_1
    """

    if not 0 <= task <= 8:
        raise ValueError(
            f"Invalid Nut-Assembly task {task:02d}. Expected 00-08."
        )

    target_ring_by_group = (
        "ring_gray",
        "ring_yellow",
        "ring_blue",
    )

    target_peg_by_offset = (
        "peg_3",
        "peg_2",
        "peg_1",
    )

    return (
        target_ring_by_group[task // 3],
        target_peg_by_offset[task % 3],
    )


def build_nut_assembly_configuration(task, trajectory):
    if not 0 <= task <= 8:
        raise ValueError(
            f"Invalid Nut-Assembly task {task:02d}. Expected 00-08."
        )

    if trajectory not in NUT_TRAJECTORIES:
        available = ", ".join(
            f"{value:03d}"
            for value in sorted(NUT_TRAJECTORIES)
        )

        raise ValueError(
            f"Invalid Nut-Assembly trajectory {trajectory:03d}. "
            f"Currently available: {available}."
        )

    target_ring, target_peg = get_nut_assembly_target(task)

    trajectory_layout = NUT_TRAJECTORIES[trajectory]
    target_slot = trajectory_layout["target_slot"]
    other_slots = trajectory_layout["other_slots"]

    other_rings = [
        ring
        for ring in NUT_RINGS
        if ring != target_ring
    ]

    assignments = {target_ring: target_slot}

    for ring, slot in zip(other_rings, other_slots):
        assignments[ring] = slot

    objects = {}

    # Reset all rings on every scene configuration.
    for ring in NUT_RINGS:
        slot = assignments[ring]
        objects[ring] = copy_pose(NUT_RING_SLOT_POSES[slot])

    # Pegs are dynamic rigid bodies too, so reset them on every run.
    for peg in NUT_PEGS:
        objects[peg] = copy_pose(NUT_PEG_POSES[peg])

    return {
        "env_type": ENV_NUT_ASSEMBLY,
        "task": int(task),
        "trajectory": int(trajectory),
        "target_object": target_ring,
        "target_destination": target_peg,
        "objects": objects,
        "metadata": {
            "assignments": assignments,
            "target_slot": target_slot,
        },
    }


# =============================================================================
# GENERIC CONFIGURATION DISPATCH + VALIDATION
# =============================================================================


def build_configuration(env_type, task, trajectory):
    if env_type == ENV_PICK_PLACE:
        configuration = build_pick_place_configuration(task, trajectory)
    elif env_type == ENV_NUT_ASSEMBLY:
        configuration = build_nut_assembly_configuration(task, trajectory)
    else:
        raise ValueError(
            f"Invalid env_type '{env_type}'. "
            f"Expected one of: {', '.join(ALLOWED_ENV_TYPES)}."
        )

    validate_configuration(configuration)
    return configuration


def validate_configuration(configuration):
    env_type = configuration.get("env_type")

    if env_type not in ALLOWED_ENV_TYPES:
        raise ValueError(
            f"Invalid configuration env_type: {env_type!r}."
        )

    objects = configuration.get("objects")

    if not isinstance(objects, dict):
        raise ValueError(
            "Configuration 'objects' must be a dictionary."
        )

    if env_type == ENV_PICK_PLACE:
        expected_objects = set(PICK_PLACE_CUBES)
    else:
        expected_objects = set(NUT_RINGS) | set(NUT_PEGS)

    received_objects = set(objects.keys())

    if received_objects != expected_objects:
        raise ValueError(
            f"{env_type}: invalid object set. "
            f"Expected {sorted(expected_objects)}, "
            f"received {sorted(received_objects)}."
        )

    for object_name, pose in objects.items():
        if not isinstance(pose, dict):
            raise ValueError(
                f"{object_name}: pose must be a dictionary."
            )

        missing = [
            field
            for field in POSE_FIELDS
            if field not in pose
        ]

        if missing:
            raise ValueError(
                f"{object_name}: missing pose fields {missing}."
            )

        for field in POSE_FIELDS:
            try:
                value = float(pose[field])
            except (TypeError, ValueError):
                raise ValueError(
                    f"{object_name}: invalid {field} value."
                )

            if not math.isfinite(value):
                raise ValueError(
                    f"{object_name}: non-finite {field} value."
                )

    target_object = configuration.get("target_object")

    if target_object not in objects:
        raise ValueError(
            f"Target object '{target_object}' is not present in the scene "
            "configuration."
        )

    target_destination = configuration.get("target_destination")

    if target_destination is not None and target_destination not in objects:
        raise ValueError(
            f"Target destination '{target_destination}' is not present in the "
            "scene configuration."
        )

    if env_type == ENV_PICK_PLACE:
        xy_positions = [
            (pose["x"], pose["y"])
            for pose in objects.values()
        ]

        if len(set(xy_positions)) != len(PICK_PLACE_CUBES):
            raise ValueError(
                "Two or more Pick-and-Place cubes share the same XY location."
            )

    elif env_type == ENV_NUT_ASSEMBLY:
        ring_x = [
            objects[ring]["x"]
            for ring in NUT_RINGS
        ]

        if len(set(ring_x)) != 3:
            raise ValueError(
                "Two or more Nut-Assembly rings share the same X slot."
            )


# =============================================================================
# ROS 2 CLIENT
# =============================================================================


class SceneConfigClient(Node):

    def __init__(self, request_id):
        super().__init__("seedo_isaac_scene_config_client")

        self.request_id = request_id
        self.response = None

        self.publisher = self.create_publisher(
            String,
            REQUEST_TOPIC,
            10,
        )

        self.subscription = self.create_subscription(
            String,
            RESPONSE_TOPIC,
            self.response_callback,
            10,
        )

    def response_callback(self, msg):
        try:
            response = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if response.get("request_id") != self.request_id:
            return

        self.response = response


def build_request_id(configuration):
    return (
        f"{configuration['env_type']}_"
        f"task_{configuration['task']:02d}_"
        f"traj_{configuration['trajectory']:03d}_"
        f"{uuid.uuid4().hex[:8]}"
    )


def build_request_payload(configuration, request_id):
    return {
        "request_id": request_id,
        "env_type": configuration["env_type"],
        "task": configuration["task"],
        "trajectory": configuration["trajectory"],
        "target_object": configuration["target_object"],
        "target_destination": configuration["target_destination"],
        "objects": configuration["objects"],
    }


def send_configuration(configuration, timeout=10.0):
    request_id = build_request_id(configuration)
    request = build_request_payload(configuration, request_id)

    node = SceneConfigClient(request_id)

    print("Waiting for Isaac scene server...")

    discovery_start = time.monotonic()

    while node.publisher.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)

        if time.monotonic() - discovery_start > 5.0:
            node.destroy_node()
            raise RuntimeError(
                "Isaac scene configuration server not found on topic "
                f"{REQUEST_TOPIC}."
            )

    msg = String()
    msg.data = json.dumps(request)

    print(
        f"Sending {configuration['env_type']} configuration to Isaac "
        f"(request {request_id})..."
    )

    node.publisher.publish(msg)

    start = time.monotonic()

    while node.response is None:
        rclpy.spin_once(node, timeout_sec=0.1)

        if time.monotonic() - start > timeout:
            node.destroy_node()
            raise TimeoutError(
                "Timed out waiting for Isaac response."
            )

    response = node.response
    node.destroy_node()
    return response


# =============================================================================
# OUTPUT
# =============================================================================


def print_configuration(configuration):
    print()
    print("=" * 76)
    print("SeeDo Isaac scene configuration")
    print("=" * 76)
    print(f"Environment: {configuration['env_type']}")
    print(f"Task:        {configuration['task']:02d}")
    print(f"Trajectory:  {configuration['trajectory']:03d}")
    print(f"Target:      {configuration['target_object']}")

    destination = configuration["target_destination"]

    if destination is not None:
        print(f"Destination: {destination}")

    print()

    if configuration["env_type"] == ENV_PICK_PLACE:
        assignments = configuration["metadata"]["assignments"]

        for cube in PICK_PLACE_CUBES:
            pose = configuration["objects"][cube]
            slot = assignments[cube]
            marker = (
                " <-- TARGET"
                if cube == configuration["target_object"]
                else ""
            )

            print(
                f"{cube:12s} "
                f"slot={slot} "
                f"xyz=({pose['x']:+.2f}, {pose['y']:+.2f}, {pose['z']:.3f}) "
                f"rz={pose['rz']:.1f}"
                f"{marker}"
            )

    else:
        assignments = configuration["metadata"]["assignments"]

        print("Rings:")

        for ring in NUT_RINGS:
            pose = configuration["objects"][ring]
            slot = assignments[ring]
            marker = (
                " <-- TARGET"
                if ring == configuration["target_object"]
                else ""
            )

            print(
                f"  {ring:12s} "
                f"slot={slot} "
                f"xyz=({pose['x']:+.6f}, {pose['y']:+.6f}, {pose['z']:.6f}) "
                f"rz={pose['rz']:.1f}"
                f"{marker}"
            )

        print()
        print("Pegs:")

        for peg in NUT_PEGS:
            pose = configuration["objects"][peg]
            marker = (
                " <-- DESTINATION"
                if peg == configuration["target_destination"]
                else ""
            )

            print(
                f"  {peg:12s} "
                f"xyz=({pose['x']:+.6f}, {pose['y']:+.6f}, {pose['z']:.6f}) "
                f"rz={pose['rz']:.1f}"
                f"{marker}"
            )

    print("=" * 76)
    print()


# =============================================================================
# CLI
# =============================================================================


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Configure an Isaac Sim scene for a SeeDo Pick-and-Place or "
            "Nut-Assembly test."
        )
    )

    parser.add_argument(
        "--env-type",
        type=str,
        choices=ALLOWED_ENV_TYPES,
        default=ENV_PICK_PLACE,
        help="Environment type (default: pick_place).",
    )

    parser.add_argument(
        "--task",
        type=int,
        default=None,
        help=(
            "Task number. Pick-and-Place: 00-15. "
            "Nut-Assembly: 00-08."
        ),
    )

    parser.add_argument(
        "--trajectory",
        type=int,
        default=None,
        help=(
            "Trajectory number. Pick-and-Place: 000-039. "
            "Nut-Assembly: currently 000 only."
        ),
    )

    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for the Isaac response.",
    )

    parser.add_argument(
        "--dry-run",
        action="store_true",
        help=(
            "Build and print the scene configuration without sending a ROS "
            "request."
        ),
    )

    return parser.parse_args()


def resolve_task_and_trajectory(env_type, task, trajectory):
    if task is None:
        prompt = (
            "Task [00-15]: "
            if env_type == ENV_PICK_PLACE
            else "Task [00-08]: "
        )
        task = int(input(prompt).strip())

    if trajectory is None:
        if env_type == ENV_PICK_PLACE:
            trajectory = int(
                input("Trajectory [000-039]: ").strip()
            )
        else:
            trajectory = 0
            print(
                "[INFO] Nut-Assembly trajectory not specified: using 000."
            )

    return task, trajectory


def main():
    print()
    print("=" * 100)
    print("START: CONFIGURE ISAAC SCENE")
    print("=" * 100)

    try:
        args = parse_args()

        task, trajectory = resolve_task_and_trajectory(
            env_type=args.env_type,
            task=args.task,
            trajectory=args.trajectory,
        )

        configuration = build_configuration(
            env_type=args.env_type,
            task=task,
            trajectory=trajectory,
        )

        print_configuration(configuration)

        if args.dry_run:
            print(
                "[PASS] Dry run completed. No ROS request was sent."
            )
            return 0

        rclpy.init()

        try:
            response = send_configuration(
                configuration,
                timeout=args.timeout,
            )
        finally:
            if rclpy.ok():
                rclpy.shutdown()

        print(
            f"Isaac response: {response.get('message')}"
        )

        if not response.get("success", False):
            print("Scene configuration FAILED.")
            return 1

        print("Scene configuration SUCCESS.")
        return 0

    except Exception as exc:
        print(
            f"ERROR: {exc}",
            file=sys.stderr,
        )
        return 1

    finally:
        print()
        print("=" * 100)
        print("END: CONFIGURE ISAAC SCENE")
        print("=" * 100)


if __name__ == "__main__":
    sys.exit(main())
