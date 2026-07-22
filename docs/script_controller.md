# Script-Controller

`script_controller_node` (part of the `ai_controller` ROS2 package) is a non-learned
alternative to `ai_controller_node`: instead of running a policy, it asks the user to
click the target object and the target placing location on the frontal ZED image,
deprojects those two pixels to 3D using depth + camera intrinsics, and executes the
pick-place with six hand-coded linear-motion primitives (`reach`, `approaching`,
`pick`, `lift_up`, `moving`, `placing` — see
`ai_controller/ai_controller/script_controller/primitives.py`). Every micro-step is
recorded (RGB + depth from all 4 cameras, joint/eef state, commanded action) into a
rollout `.pkl`, using the same `savers.Trajectory` mechanism as `ai_controller_node.py`.

This assumes the Docker/container setup and UR-driver + `moveit_controller` launch
described in [AI-Controller](ai_controller.md) are already running.

## Launch

```bash
# Make sure base_link -> table_0 is being published in TF before starting script_controller_node
# (see "How target points are computed" below) - e.g.:
ros2 run tf2_ros static_transform_publisher \
    --x 0.00 --y 0.612 --z -0.120 --qx 0.000 --qy 0.000 --qz 1.000 --qw 0.000 \
    --frame-id base_link --child-frame-id table_0 &

# Run Script-Controller (scripted, click-to-target pick-place - no learned model)
# Leave move_robot:=False the first time to dry-run (logs the poses/primitives without moving the robot).
docker exec -it ur_robotiq_teleoperation_container  bash
source install/setup.bash
ros2 run ai_controller script_controller_node --ros-args \
    -p move_robot:=False \
    -p task_name:="pick_place" \
    -p click_camera_name:="zed_front" \
    -p camera_calibration_path:="/home/ros2_ws/src/zed_camera/zed_camera_calibration/estimated_camera_positions.yaml" \
    -p save_rollout_path:="/home/ros2_ws/src/ai_controller/saved_rollouts/script_controller"
```

## How target points are computed

1. The clicked pixel's depth (median over a small window, from `.../depth/depth_registered`)
   is deprojected to a 3D point in the `zed_front` camera's optical frame using that
   camera's intrinsics (`.../zed_node/rgb/color/rect/camera_info`).
2. That point is transformed into the **raw ArUco marker frame** (the marker glued at
   the table center, in cv2.aruco's own axis convention) using the extrinsics in
   `zed_camera/zed_camera_calibration/estimated_camera_positions.yaml` (produced by
   `interactive_aruco_calibration.py`).
3. A **fixed** offset converts the ArUco-frame point into the `table_0` frame: zero
   translation, quaternion (x, y, z, w) = (0, 0, 1, 0) — a 180° rotation about Z (X/Y
   flip, Z unchanged). This is baked into the code as `ARUCO_TO_TABLE0_ROTATION` in
   `ai_controller/ai_controller/script_controller/vision.py`.
4. The node looks up `base_link -> table_0` **via TF** to bring the point into
   `base_link`. **`script_controller_node` does not publish this transform itself** —
   something else must (e.g. a `static_transform_publisher`, or a URDF fixed joint);
   the node will error out at startup after a timeout if `table_0` isn't in TF yet.

## Required: base_link -> table_0 must be in TF before starting

The ZED cameras aren't part of the robot's kinematic tree, so their extrinsics are only
known relative to the ArUco marker (step 2/3 above) — getting from there into `base_link`
needs a `base_link -> table_0` transform published by something in your launch stack.
Quick way to publish it from a known/measured pose:
```bash
ros2 run tf2_ros static_transform_publisher \
    --x <x> --y <y> --z <z> --qx <qx> --qy <qy> --qz <qz> --qw <qw> \
    --frame-id base_link --child-frame-id table_0
```

## Simulate before you execute

Recommended progression to de-risk a new pick-place run: fake-hardware first, then a
plan-only preview on the real robot, then real execution.

**1. Waypoint plot only — zero ROS calls beyond a marker publish.** `script_controller_node`
always publishes the *fully computed* path to RViz as a `visualization_msgs/MarkerArray`
on `script_controller/planned_waypoints`, independently of `move_robot` — this is what
lets you inspect the plan even with `move_robot:=False` (no `GoToPose`/`GripperCommand`
calls at all). Add a `MarkerArray` display in RViz pointed at that topic (QoS is
Transient Local, so it shows the last plan even if RViz connects afterwards). You'll see:
- a big green sphere at the clicked **target object**, a big red sphere at the **target bin**,
- one line + point-strip per primitive, color-coded: `reach` blue, `approaching` cyan,
  `lift_up` green, `moving` yellow, `placing` orange (`pick` has no motion to plot).

Markers are cleared and republished at the start of every trajectory, and each segment is
published the instant it's computed — i.e. *before* it starts executing (if it executes at
all) — so this is the cheapest and safest first check, requiring no MoveIt/controller
involvement whatsoever.

**2. Fake hardware (no physical robot arm at all).** Launch the UR driver with
`use_fake_hardware:=true` (this is already the default baked into
`ur5e_2f_85_platform.urdf.xacro`):
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=${ROBOT_IP} \
  use_fake_hardware:=true
```
`moveit_controller_node` and `script_controller_node` (with `move_robot:=True`) behave
exactly as on the real robot — full MoveIt planning + execution, live TF/`/joint_states` —
just with no physics/collisions and a robot that can't actually hurt anything. Note the
ZED cameras are separate USB hardware and are **not** simulated by fake hardware: you
still need the real cameras plugged in and the Zed container running for the click UI.

**3. Plan-only preview on the real robot.** Launch `moveit_controller_node` with
`-p execute_trajectory:=False` (see [AI-Controller](ai_controller.md)'s **Launch**
section) and open RViz with a `MotionPlanning` display subscribed to
`/display_planned_path` (this is its default topic) alongside the `MarkerArray` display
from step 1. Run `script_controller_node` normally with `-p move_robot:=True`: every
`reach`/`approaching`/`pick`/`lift_up`/`moving`/`placing` waypoint gets planned and
drawn in RViz (both the full plotted path and, live, MoveIt's per-segment plan), but
`moveit_controller_node` never calls `ExecuteTrajectory`, so the arm never moves. This
is the way to sanity-check the *whole* pipeline — clicked pixels, table_0 calibration,
waypoint spacing — against the real robot's actual kinematics before it's allowed to move.

> **Gripper caveat:** `execute_trajectory:=False` only gates arm motion through
> `moveit_controller_node`. `script_controller_node`'s `pick`/`placing` primitives send
> `GripperCommand` goals straight to `/robotiq_gripper_controller/gripper_cmd`, bypassing
> moveit_controller entirely — so with `move_robot:=True` the **real gripper will still
> open/close** even while the arm motion is preview-only. Use `-p move_robot:=False` on
> `script_controller_node` instead (fully dry: no service/action calls at all, logs the
> intended poses/gripper commands, still plots the waypoints per step 1) if you want zero
> physical actuation.

**4. Real execution.** Drop `execute_trajectory:=False` (or launch `moveit_controller_node`
with the default `True`) and run against the real UR driver (no `use_fake_hardware`).

## Parameters (pass with `-p name:=value`, defaults shown)

| Parameter | Default | Meaning |
|---|---|---|
| `move_robot` | `False` | `False` = dry run (logs planned poses/gripper commands, no motion). |
| `click_camera_name` | `zed_front` | Camera used for the click UI (must exist in the calibration yaml). |
| `camera_calibration_path` | `/home/ros2_ws/src/zed_camera/zed_camera_calibration/estimated_camera_positions.yaml` | Camera-to-ArUco extrinsics (see mount added to the `UR-Container` run command in [AI-Controller](ai_controller.md)). |
| `table_frame_id` | `table_0` | TF frame the node looks up `base_link -> table_frame_id` in — must already be published, see above. |
| `grasp_orientation` | top-down quaternion (xyzw) | Fixed TCP orientation used for every primitive. |
| `min_step` | `0.02` | Minimum per-waypoint displacement (m) for the linear micro-stepping. |
| `reach_hover_height` | `0.15` | Height (m) above the clicked object used by `reach`. |
| `approach_z_offset` | `0.02` | Extra Z (m) added to the clicked object/bin depth before `approaching`/`placing` descend. |
| `lift_height` | `0.15` | How much `lift_up` raises the TCP after `pick` (m). |
| `gripper_open_position` / `gripper_closed_position` | `0.1` / `0.8` | Radians on `robotiq_85_left_knuckle_joint` (matches `teleoperator_node.py`). |
| `save_rollout_path` | `/home/ros2_ws/src/ai_controller/saved_rollouts/script_controller` | Where rollout `.pkl`/`.json` files are written. |

## Usage

1. Run the command in the **Launch** section above (`move_robot:=False` first to sanity-check
   the computed object/bin positions logged to the console).
2. A window pops up showing the `zed_front` RGB frame: left-click the **target object**,
   press `SPACE` to confirm (or `ESC` to abort); repeat for the **target placing** location.
3. The console prints the two computed `base_link` positions; press Enter to execute, or
   `Ctrl+C` to abort before any motion happens.
4. Once satisfied with the dry run, rerun with `-p move_robot:=True`.
