#!/usr/bin/env python3

import argparse
import json
import sys
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


REQUEST_TOPIC = "/seedo/configure_scene_request"
RESPONSE_TOPIC = "/seedo/configure_scene_response"

CUBE_Z = 0.036

# ============================================================
# CUBES
# ============================================================

CUBES = [
    "cube_red",
    "cube_green",
    "cube_yellow",
    "cube_blue",
]


# ============================================================
# GRID
#
# (row, column) -> (x, y)
# ============================================================

GRID = {
    (0, 0): (-0.23, 0.24),
    (0, 1): (-0.07, 0.24),
    (0, 2): ( 0.07, 0.24),
    (0, 3): ( 0.23, 0.24),

    (1, 0): (-0.23, 0.15),
    (1, 1): (-0.07, 0.15),
    (1, 2): ( 0.07, 0.15),
    (1, 3): ( 0.23, 0.15),

    (2, 0): (-0.23, 0.06),
    (2, 1): (-0.07, 0.06),
    (2, 2): ( 0.07, 0.06),
    (2, 3): ( 0.23, 0.06),
}


# ============================================================
# TRAJECTORIES 000-019
#
# 020-039 use the same layouts with rotation Z = 45 deg.
# ============================================================

TRAJECTORIES = {
    0:  ((1, 0), [(1, 1), (1, 2), (1, 3)]),
    1:  ((1, 1), [(1, 0), (1, 2), (1, 3)]),
    2:  ((1, 2), [(1, 0), (1, 1), (1, 3)]),
    3:  ((0, 3), [(1, 0), (1, 1), (1, 2)]),

    4:  ((2, 0), [(2, 1), (2, 2), (2, 3)]),
    5:  ((2, 1), [(2, 0), (2, 2), (2, 3)]),
    6:  ((2, 2), [(2, 0), (2, 1), (2, 3)]),
    7:  ((2, 3), [(2, 0), (2, 1), (2, 2)]),

    8:  ((0, 0), [(0, 1), (0, 2), (0, 3)]),
    9:  ((0, 1), [(0, 0), (0, 2), (0, 3)]),
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


# ============================================================
# TASK -> TARGET CUBE
# ============================================================

def get_target_cube(task):
    if 0 <= task <= 3:
        return "cube_green"

    if 4 <= task <= 7:
        return "cube_yellow"

    if 8 <= task <= 11:
        return "cube_blue"

    if 12 <= task <= 15:
        return "cube_red"

    raise ValueError(
        f"Invalid task {task:02d}. Expected 00-15."
    )


# ============================================================
# SCENE CONFIGURATION GENERATION
# ============================================================

def build_configuration(task, trajectory):

    if not 0 <= task <= 15:
        raise ValueError(
            f"Invalid task {task:02d}. Expected 00-15."
        )

    if not 0 <= trajectory <= 39:
        raise ValueError(
            f"Invalid trajectory {trajectory:03d}. Expected 000-039."
        )

    target_cube = get_target_cube(task)

    # 020 -> 000, ..., 039 -> 019
    base_trajectory = trajectory % 20

    target_slot, other_slots = TRAJECTORIES[
        base_trajectory
    ]

    rotation_z = 45.0 if trajectory >= 20 else 0.0

    # Deterministic assignment of non-target cubes.
    other_cubes = [
        cube
        for cube in CUBES
        if cube != target_cube
    ]

    assignments = {
        target_cube: target_slot,
    }

    for cube, slot in zip(
        other_cubes,
        other_slots,
    ):
        assignments[cube] = slot

    poses = {}

    for cube in CUBES:
        slot = assignments[cube]

        x, y = GRID[slot]

        poses[cube] = {
            "x": x,
            "y": y,
            "z": CUBE_Z,
            "rz": rotation_z,
        }

    return {
        "task": task,
        "trajectory": trajectory,
        "target_cube": target_cube,
        "rotation_z": rotation_z,
        "assignments": assignments,
        "poses": poses,
    }


# ============================================================
# ROS CLIENT
# ============================================================

class SceneConfigClient(Node):

    def __init__(self, request_id):
        super().__init__(
            "seedo_isaac_scene_config_client"
        )

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


def send_configuration(configuration, timeout=10.0):

    request_id = (
        f"task_{configuration['task']:02d}_"
        f"traj_{configuration['trajectory']:03d}_"
        f"{uuid.uuid4().hex[:8]}"
    )

    node = SceneConfigClient(request_id)

    # --------------------------------------------------------
    # Wait for the Isaac subscriber to be discovered.
    # --------------------------------------------------------

    print("Waiting for Isaac scene server...")

    discovery_start = time.monotonic()

    while node.publisher.get_subscription_count() == 0:

        rclpy.spin_once(
            node,
            timeout_sec=0.1,
        )

        if time.monotonic() - discovery_start > 5.0:
            node.destroy_node()

            raise RuntimeError(
                "Isaac scene configuration server not found "
                f"on topic {REQUEST_TOPIC}."
            )

    # --------------------------------------------------------
    # Send request
    # --------------------------------------------------------

    request = {
        "request_id": request_id,
        "task": configuration["task"],
        "trajectory": configuration["trajectory"],
        "target_cube": configuration["target_cube"],
        "cubes": configuration["poses"],
    }

    msg = String()
    msg.data = json.dumps(request)

    print(
        f"Sending configuration to Isaac "
        f"(request {request_id})..."
    )

    node.publisher.publish(msg)

    # --------------------------------------------------------
    # Wait for matching ACK.
    # --------------------------------------------------------

    start = time.monotonic()

    while node.response is None:

        rclpy.spin_once(
            node,
            timeout_sec=0.1,
        )

        if time.monotonic() - start > timeout:
            node.destroy_node()

            raise TimeoutError(
                "Timed out waiting for Isaac response."
            )

    response = node.response

    node.destroy_node()

    return response


# ============================================================
# OUTPUT
# ============================================================

def print_configuration(configuration):

    print()
    print("=================================================")
    print("SeeDo Isaac scene configuration")
    print("=================================================")

    print(
        f"Task:       "
        f"{configuration['task']:02d}"
    )

    print(
        f"Trajectory: "
        f"{configuration['trajectory']:03d}"
    )

    print(
        f"Target:     "
        f"{configuration['target_cube']}"
    )

    print(
        f"Rotation Z: "
        f"{configuration['rotation_z']:.1f} deg"
    )

    print()

    for cube in CUBES:

        slot = configuration[
            "assignments"
        ][cube]

        pose = configuration[
            "poses"
        ][cube]

        marker = (
            " <-- TARGET"
            if cube == configuration["target_cube"]
            else ""
        )

        print(
            f"{cube:12s} "
            f"slot={slot} "
            f"xyz=("
            f"{pose['x']:+.2f}, "
            f"{pose['y']:+.2f}, "
            f"{pose['z']:.3f}) "
            f"rz={pose['rz']:.1f}"
            f"{marker}"
        )

    print("=================================================")
    print()


# ============================================================
# MAIN
# ============================================================

def main():

    parser = argparse.ArgumentParser(
        description=(
            "Configure the Isaac Sim cube scene "
            "for a SeeDo test."
        )
    )

    parser.add_argument(
        "--task",
        type=int,
        default=None,
        help="Task number [00-15]",
    )

    parser.add_argument(
        "--trajectory",
        type=int,
        default=None,
        help="Trajectory number [000-039]",
    )

    args = parser.parse_args()

    task = args.task
    trajectory = args.trajectory

    if task is None:
        task = int(
            input("Task [00-15]: ").strip()
        )

    if trajectory is None:
        trajectory = int(
            input(
                "Trajectory [000-039]: "
            ).strip()
        )

    try:
        configuration = build_configuration(
            task,
            trajectory,
        )

        print_configuration(configuration)

        rclpy.init()

        try:
            response = send_configuration(
                configuration
            )
        finally:
            if rclpy.ok():
                rclpy.shutdown()

        print(
            f"Isaac response: "
            f"{response.get('message')}"
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


if __name__ == "__main__":
    sys.exit(main())
