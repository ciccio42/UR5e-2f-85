# Isaac Sim + MoveIt 2 + ros2_control + SeeDo integration

## Conclusion

The objective is feasible without URSim, but it is **not yet implemented end to end** by the two repositories in their current state.

The recommended architecture is:

```text
AIControllerNode / SeeDo
        |
        | GoToPose service (PoseStamped in base_link)
        v
custom moveit_controller node
        |
        | MoveGroup + ExecuteTrajectory
        v
MoveIt 2 / move_group
        |
        | FollowJointTrajectory
        v
ros2_control controller_manager
        |
        | topic_based_ros2_control/TopicBasedSystem
        | publishes /isaac_joint_commands
        | consumes  /isaac_joint_states
        v
Isaac Sim ROS 2 Action Graph
        |
        v
UR5e + Robotiq articulation
```

This preserves the repository's intended control pipeline: SeeDo produces Cartesian TCP actions, the custom `moveit_controller` performs IK and MoveIt planning, MoveIt executes through a standard ros2_control trajectory controller, and Isaac Sim is the simulated hardware.

Do **not** use `joint_state_publisher_gui` as the state source in this architecture. Isaac Sim must be the authoritative joint-state source. Do **not** run the MoveIt demo with `mock_components/GenericSystem`, because that would move only MoveIt's internal fake robot and not the Isaac articulation.

## Repository audit and description confirmation

The Isaac repository contains a dedicated digital-twin setup:

- `docker/isaac_sim_ur5e.dockerfile` copies `robot/UR5e-2f-85/ur5e_2f_85/ur5e_2f_85_description` into the image.
- `docker/generate_ur5e_2f_85_urdf.sh` expands `ur5e_2f_85_platform.urdf.xacro` into `/workspace/isaac_assets/ur5e_2f_85/ur5e_2f_85_platform.urdf` and validates it with `check_urdf`.
- The custom Xacro includes the UR5e, Robotiq 2F-85, platform/table, gripper camera attachment, and `tcp_link`.
- `Digital_Twin_UR5e.md` describes importing that generated URDF into Isaac Sim 5.1.

The description tree in the Isaac repository was compared recursively against:

```text
/home/asus-mivia/Desktop/Angelo/UR5e-2f-85/ur5e_2f_85/ur5e_2f_85_description
```

The result is: **the Isaac environment does contain the requested custom description, but it is not an exact current copy**. All files match except:

```text
config/ur5e_2f_85_controllers.yaml
```

For the six joints under `scaled_joint_trajectory_controller.constraints`, the Angelo source uses trajectory tolerance `0.0`, while the Isaac copy uses `0.2`. The robot geometry, Xacro, meshes, and remaining description files match. Before rebuilding the Isaac image, synchronize the entire description directory from the Angelo repository and repeat the comparison. This prevents a silently stale simulation description.

The existing USD files also do not provide evidence of a completed UR5e control stage. In particular, the repository search did not find a saved UR5e articulation graph connected to joint command/state ROS topics. Importing and saving that stage is still required.

## Important current mismatches

Resolve these before attempting real execution:

1. `ur5e_2f_85_moveit_config/config/ur5e_2f_85.ros2_control.xacro` uses `mock_components/GenericSystem`. It must use the repository's included `topic_based_ros2_control/TopicBasedSystem` for Isaac execution.
2. `moveit_controllers.yaml` and the custom `moveit_controller` expect `scaled_joint_trajectory_controller`, while `ros2_controllers.yaml` defines `scaled_pos_joint_traj_controller`.
3. The custom node defaults to stopping `forward_position_controller`, but the generated MoveIt ros2_control configuration does not define that controller. For the Isaac pipeline, avoid switching back to it or define it consistently.
4. `AIControllerNode` expects a `control_msgs/action/GripperCommand` server at `/robotiq_gripper_controller/gripper_cmd`. The generated MoveIt configuration instead declares `gripper_controller` as `FollowJointTrajectory`. These are different names and action types.
5. The current Isaac image copies only the description package. It does not by itself build the Angelo MoveIt config, custom MoveIt services, or AI controller.
6. The current `demo.launch.py` starts fake hardware. A dedicated Isaac launch is needed so one and only one `robot_state_publisher`, `controller_manager`, and `/joint_states` authority exists.
7. SeeDo defaults to real ZED topic names and `table_0`; Isaac must publish matching RGB, registered depth, camera info, and TF, or the node parameters must be remapped.
8. The SeeDo configuration contains host/container-specific absolute paths for calibration and model checkpoints. Those paths must exist in the AI runtime container.
9. `ai_controller_config.yaml` uses `camera_topics`, but the node declares `camera_topic`. Use explicit command-line parameters or fix the eventual runtime YAML before relying on it.

## 1. Freeze compatible versions and networking

Use one ROS distribution and one DDS/RMW configuration across every process. The repositories are presently designed around:

- Isaac Sim 5.1.0;
- Ubuntu 24.04;
- ROS 2 Jazzy;
- `ROS_DOMAIN_ID=0`;
- `rmw_fastrtps_cpp` in the UR5e Isaac image, despite the base image initially declaring Cyclone DDS.

Pick either Fast DDS or Cyclone DDS and set the same `RMW_IMPLEMENTATION` in Isaac Sim and the external ROS/MoveIt/AI environment. The existing digital-twin run command uses Fast DDS, so that is the least surprising initial choice.

Use host networking for the containers:

```bash
--network=host \
-e ROS_DOMAIN_ID=0 \
-e RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Do not source two incompatible ROS installations into the same shell. Isaac's embedded Python and ROS bridge have stricter compatibility requirements than a normal ROS container; keep simulation and application nodes in separate containers/process environments unless the image is deliberately built to host both.

## 2. Synchronize and validate the custom description

From the Isaac repository, update its vendored robot copy from the Angelo source. The intended source and destination are:

```text
source:      /home/asus-mivia/Desktop/Angelo/UR5e-2f-85/ur5e_2f_85/ur5e_2f_85_description/
destination: /home/asus-mivia/Desktop/Isaac-Sim/Nvidia-Isaac-Sim-Env/robot/UR5e-2f-85/ur5e_2f_85/ur5e_2f_85_description/
```

After synchronization, verify that this command prints no differences:

```bash
diff -qr \
  /home/asus-mivia/Desktop/Angelo/UR5e-2f-85/ur5e_2f_85/ur5e_2f_85_description \
  /home/asus-mivia/Desktop/Isaac-Sim/Nvidia-Isaac-Sim-Env/robot/UR5e-2f-85/ur5e_2f_85/ur5e_2f_85_description
```

Rebuild the two Isaac images as described by the Isaac repository:

```bash
cd /home/asus-mivia/Desktop/Isaac-Sim/Nvidia-Isaac-Sim-Env

docker build -t isaac-sim-custom:5.1.0 \
  -f docker/isaac_sim.dockerfile .

docker build -t isaac-sim-ur5e:5.1.0 \
  -f docker/isaac_sim_ur5e.dockerfile .
```

Inside the resulting image, regenerate and validate the flattened URDF:

```bash
generate_ur5e_2f_85_urdf
check_urdf "$UR5E_2F_85_URDF"
```

Also inspect the generated joint names. They must include exactly these six arm joints, without an unexpected prefix:

```text
shoulder_pan_joint
shoulder_lift_joint
elbow_joint
wrist_1_joint
wrist_2_joint
wrist_3_joint
```

The commanded gripper joint expected by the Angelo code is:

```text
robotiq_85_left_knuckle_joint
```

## 3. Create and validate the Isaac articulation stage

Run `isaac-sim-ur5e:5.1.0` with the same mounts and GPU/X11 options documented in the Isaac repository, then launch:

```bash
/isaac-sim/isaac-sim.sh --allow-root \
  --enable isaacsim.asset.importer.urdf \
  --enable isaacsim.ros2.bridge
```

Import:

```text
/workspace/isaac_assets/ur5e_2f_85/ur5e_2f_85_platform.urdf
```

During import:

- fix the platform/world base;
- import movable joints as one articulation;
- preserve the source joint names;
- enable self-collision only after basic control works, because the custom SRDF contains a large collision-disable matrix that does not automatically configure Isaac physics;
- confirm that the gripper mimic joints behave correctly. If URDF mimic import is insufficient, configure the gripper coupling in Isaac or command all required gripper joints explicitly.

Before adding ROS, use Isaac's articulation inspector/controller to move every arm joint and the left knuckle joint. Check limits, axes, units (radians), and that the base remains fixed. Save the validated stage under the mounted `usd` directory, for example:

```text
/isaac-sim/usd/ur5e_2f_85/ur5e_2f_85_controlled.usd
```

## 4. Add the Isaac ROS 2 Action Graph

Add an On Playback Tick-driven graph to the saved stage. It needs:

- a ROS 2 Context node;
- a simulation-time publisher for `/clock` if the ROS nodes will use `use_sim_time:=true`;
- an articulation-state reader connected to a ROS 2 Publish Joint State node;
- a ROS 2 Subscribe Joint State node connected to an Articulation Controller.

Use dedicated hardware-interface topics to avoid feedback loops:

```text
Isaac publishes:   /isaac_joint_states
Isaac subscribes:  /isaac_joint_commands
message type:      sensor_msgs/msg/JointState
```

The state message must carry the exact joint names used by MoveIt/ros2_control. For the first arm-only milestone, publish and command the six UR joints in a consistent order. Add the gripper joint after arm trajectory execution works.

The graph's articulation target must be the imported UR5e articulation root, not the table/world prim. Enable the graph, press Play, and verify from an external ROS shell:

```bash
ros2 topic list | sort
ros2 topic echo /isaac_joint_states --once
ros2 topic info /isaac_joint_commands -v
```

Publish a very small manual `JointState` command and confirm physical movement in Isaac before introducing ros2_control. Stay well inside the imported joint limits.

## 5. Replace fake hardware with topic-based ros2_control

The required hardware plugin is already present at:

```text
IsaacSim-ros_workspaces/jazzy_ws/src/moveit/topic_based_ros2_control
```

Build/source that package in the ROS environment that will run `controller_manager`. Create an Isaac-specific ros2_control Xacro derived from the existing `ur5e_2f_85.ros2_control.xacro`. Its hardware block should be equivalent to:

```xml
<hardware>
  <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
  <param name="joint_commands_topic">/isaac_joint_commands</param>
  <param name="joint_states_topic">/isaac_joint_states</param>
  <param name="sum_wrapped_joint_states">true</param>
</hardware>
```

Keep position command interfaces and position/velocity state interfaces for the six arm joints. `sum_wrapped_joint_states` is appropriate because the included plugin documentation explicitly notes Isaac's `2*pi` to `-2*pi` wrapping.

Add `robotiq_85_left_knuckle_joint` to the same hardware system only if it is present in both Isaac command/state messages. For mimic joints, use the topic-based plugin's per-joint `mimic` and `multiplier` parameters if needed, matching the Robotiq URDF relationships.

Create an Isaac-specific controller YAML with consistent names:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    scaled_joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    robotiq_gripper_controller:
      type: position_controllers/GripperActionController

scaled_joint_trajectory_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    allow_partial_joints_goal: false

robotiq_gripper_controller:
  ros__parameters:
    joint: robotiq_85_left_knuckle_joint
    action_ns: gripper_cmd
    state_interfaces: [position]
    allow_stalling: true
    goal_tolerance: 0.02
```

The exact gripper plugin parameter schema must be checked against the installed Jazzy controller version. The required external contract is an action server at:

```text
/robotiq_gripper_controller/gripper_cmd
```

If the available Jazzy controller exposes a different namespace, remap the `AIControllerNode.gripper_action_topic` parameter rather than maintaining two aliases.

## 6. Create a dedicated MoveIt launch for Isaac

Do not use `demo.launch.py` unchanged. Create a launch description that starts:

1. `robot_state_publisher` using the custom robot description plus the topic-based ros2_control tag;
2. `ros2_control_node`/`controller_manager` using the Isaac controller YAML;
3. `joint_state_broadcaster`;
4. `scaled_joint_trajectory_controller`;
5. `robotiq_gripper_controller` when gripper integration is enabled;
6. `move_group` from `ur5e_2f_85_moveit_config`;
7. RViz optionally;
8. the repository's `moveit_controller_node`.

Use the same URDF geometry for Isaac, `robot_state_publisher`, and MoveIt. Use the existing SRDF planning group `arm_tcp`, whose chain is `base_link` to `tcp_link`.

Keep `moveit_controllers.yaml` aligned with the active ros2_control action server:

```yaml
moveit_simple_controller_manager:
  controller_names:
    - scaled_joint_trajectory_controller

  scaled_joint_trajectory_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
```

The resulting trajectory action must be:

```text
/scaled_joint_trajectory_controller/follow_joint_trajectory
```

For the custom `moveit_controller_node`, set:

```text
planning_component:=arm_tcp
controller_to_run:=scaled_joint_trajectory_controller
execute_trajectory:=false initially
```

The node currently tries to switch back to `forward_position_controller` after each request. For the Isaac launch, either configure a real controller with that name or, preferably, adjust the eventual implementation so the trajectory controller remains active and no unnecessary switch-back occurs. The latter requires a code change and is intentionally not performed by this document-only task.

## 7. Validate MoveIt in increasing-risk stages

### 7.1 ROS graph and state authority

Start Isaac, press Play, then start the Isaac-specific ROS/MoveIt launch. Verify:

```bash
ros2 node list
ros2 topic hz /isaac_joint_states
ros2 topic hz /joint_states
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 action list -t
```

Expected state:

- `/isaac_joint_states` comes from Isaac;
- `/joint_states` comes from `joint_state_broadcaster`;
- the six arm position command interfaces are claimed by `scaled_joint_trajectory_controller`;
- `joint_state_broadcaster` and `scaled_joint_trajectory_controller` are active;
- `/scaled_joint_trajectory_controller/follow_joint_trajectory` exists;
- `/move_action`, `/execute_trajectory`, and `/compute_ik` exist;
- `/set_robot_to_pose` exists after starting the custom controller node.

There must not be a second fake-hardware controller manager or a `joint_state_publisher_gui` publishing conflicting states.

### 7.2 Plan only

Run the custom MoveIt node with `execute_trajectory:=false`. Call `set_robot_to_pose` with a reachable pose expressed in `base_link`. Confirm:

- IK succeeds;
- the plan appears on `/display_planned_path`/RViz;
- Isaac does not move;
- the service reports that the trajectory was planned but not executed.

This validates the custom `GoToPose -> GetPositionIK -> MoveGroup` path without actuation.

### 7.3 Execute one arm trajectory

Set `execute_trajectory:=true`, lower the MoveIt velocity and acceleration scaling factors for the first test, and request a small motion. Observe:

```bash
ros2 topic echo /isaac_joint_commands
ros2 topic echo /isaac_joint_states
```

Confirm that the command changes, Isaac moves, state follows command, and `ExecuteTrajectory` returns success. If execution hangs, first check that simulation is playing, timestamps advance, joint names match, and the controller receives fresh state.

### 7.4 Gripper

Only after the arm works, test:

```bash
ros2 action info /robotiq_gripper_controller/gripper_cmd
```

Send conservative open/close values matching the imported joint limit. The SeeDo defaults are `0.1` open and `0.8` closed; confirm that `0.8` is legal for the actual Robotiq joint before using it. Verify all mimic fingers follow and collisions are reasonable.

## 8. Provide Isaac camera and TF inputs for SeeDo

SeeDo requires more than robot motion. In its current code path it waits for:

```text
/zed_front/zed_node/rgb/color/rect/image
/zed_front/zed_node/depth/depth_registered
/zed_front/zed_node/rgb/color/rect/camera_info
TF: base_link -> table_0
TF: base_link -> tcp_link
/joint_states
```

Create a simulated RGB-D camera in Isaac at the pose corresponding to `zed_front`, then add ROS 2 camera publishers for RGB image, depth image, and camera info. Prefer publishing directly with the names above; otherwise override `seedo_rgb_topic`, `seedo_depth_topic`, and `seedo_camera_info_topic`.

Requirements:

- RGB and depth timestamps must be close enough for the node's approximate synchronizer (`0.1` seconds).
- Depth encoding and units must match what `ScenePerceiver` expects.
- `CameraInfo.k` must describe the simulated camera, not the physical ZED calibration.
- The image frame and optical-axis convention must be correct.
- Publish or provide the static transform `base_link -> table_0`.
- `robot_state_publisher` must provide the chain to `tcp_link`.

The configured physical-camera file `estimated_camera_positions.yaml` should not be assumed valid for the simulated camera. Generate a simulation-specific calibration/extrinsic configuration from the Isaac camera pose and validate a known 3-D point before allowing motion.

## 9. Run the SeeDo controller safely

Build/source these Angelo packages in the AI/ROS workspace:

```text
moveit_controller/moveit_controller_srvs
moveit_controller/moveit_controller
ai_controller
dataset_collector/dataset_collector_pkg
ur5e_2f_85/ur5e_2f_85_description
ur5e_2f_85/ur5e_2f_85_moveit_config
```

Ensure the SeeDo Python/model dependencies, checkpoints, demonstration directory, calibration file, and any required API credentials are available inside that runtime. Then launch `AIControllerNode` explicitly with at least:

```text
ai_controller_target:=seedo_controller
model_config_path:=<container-visible path to seedo_controller.yaml>
frame_id:=base_link
eef_frame_name:=tcp_link
joint_states_topic:=/joint_states
set_pose_service:=set_robot_to_pose
move_robot:=false
seedo_execute_gripper:=false
```

First run with `move_robot:=false`. Confirm that the demonstration loads, RGB-D input arrives, `base_link -> table_0` resolves, perception produces plausible object coordinates, and a non-empty primitive plan/action sequence is generated.

Next use `move_robot:=true` while keeping:

```text
moveit_controller_node.execute_trajectory:=false
seedo_execute_gripper:=false
```

This exercises every SeeDo `GoToPose` call through MoveIt planning and publishes plans in RViz without moving Isaac.

Finally enable in this order:

1. `moveit_controller_node.execute_trajectory:=true` for arm motion;
2. keep `seedo_execute_gripper:=false` and validate a complete arm-only sequence;
3. set `seedo_execute_gripper:=true` after the gripper action server is proven independently.

The final expected SeeDo launch parameters are conceptually:

```bash
ros2 run ai_controller ai_controller_node --ros-args \
  -p ai_controller_target:=seedo_controller \
  -p model_config_path:=<path-to-seedo-controller-yaml> \
  -p frame_id:=base_link \
  -p eef_frame_name:=tcp_link \
  -p joint_states_topic:=/joint_states \
  -p set_pose_service:=set_robot_to_pose \
  -p move_robot:=true \
  -p seedo_execute_gripper:=true \
  -p seedo_rgb_topic:=/zed_front/zed_node/rgb/color/rect/image \
  -p seedo_depth_topic:=/zed_front/zed_node/depth/depth_registered \
  -p seedo_camera_info_topic:=/zed_front/zed_node/rgb/color/rect/camera_info \
  -p seedo_table_frame:=table_0
```

Supply array and path parameters through a validated YAML file when shell quoting becomes cumbersome.

## 10. End-to-end acceptance checklist

The integration is complete only when all of the following pass:

- [ ] Isaac's description copy is identical to the Angelo source.
- [ ] The imported stage visibly contains the UR5e, Robotiq 2F-85, platform/table, camera attachment, and `tcp_link` chain.
- [ ] Isaac alone publishes fresh `/isaac_joint_states` while simulation is playing.
- [ ] `topic_based_ros2_control` consumes those states and publishes `/isaac_joint_commands`.
- [ ] `joint_state_broadcaster` publishes `/joint_states` with all six arm joints and the gripper joint when enabled.
- [ ] Controller names match across ros2_control, MoveIt, and the custom MoveIt node.
- [ ] MoveIt plan-only succeeds for a reachable `base_link` TCP pose.
- [ ] A small MoveIt trajectory physically moves the Isaac articulation and returns success.
- [ ] `base_link -> tcp_link` matches the Isaac pose within a chosen tolerance.
- [ ] The gripper action moves the simulated Robotiq joint and its mimic fingers.
- [ ] Simulated RGB, depth, camera info, and `base_link -> table_0` are available and geometrically validated.
- [ ] SeeDo inference works with `move_robot:=false`.
- [ ] SeeDo plan-only works with `move_robot:=true` and MoveIt execution disabled.
- [ ] A low-speed, collision-free SeeDo sequence executes in Isaac.

## Practical recommendation

Implement this in two runtime containers:

- **Isaac container:** Isaac Sim, saved UR5e stage, ROS 2 bridge Action Graph, simulated RGB-D camera.
- **ROS application container:** topic-based ros2_control, controller manager, robot state publisher, MoveIt, custom `moveit_controller`, and `AIControllerNode`/SeeDo.

This separation avoids Isaac's embedded-Python constraints while retaining standard ROS 2 discovery over host networking. It also makes the replacement boundary explicit: Isaac Sim plus `topic_based_ros2_control` replaces URSim/UR driver as the hardware layer; MoveIt and the higher-level controller pipeline remain structurally unchanged.
