# SeeDo + Isaac Sim Integration — Technical Summary

_Last updated: 2026-08-23_

## 1. Objective

The work integrated **SeeDo** with an **Isaac Sim 5.1** simulation of a **UR5e + Robotiq 2F-85**, using Isaac Sim as the simulated hardware backend.

The resulting pipeline is:

```text
SeeDo
  ↓
AIControllerNode
  ↓
GoToPose / GripperCommand
  ↓
moveit_controller
  ↓
MoveIt 2
  ↓
ros2_control
  ↓
topic_based_ros2_control
  ↓
Isaac Sim
  ↓
UR5e + Robotiq 2F-85
```

The final setup does **not** use fake/mock hardware for the Isaac path.

---

# 2. Main repositories

## Main robotics repository

```text
/home/asus-mivia/Desktop/Angelo/UR5e-2f-85
```

Repository:

```text
https://github.com/ciccio42/UR5e-2f-85.git
```

Branch:

```text
a.infante32
```

## Isaac Sim repository

```text
/home/asus-mivia/Desktop/Isaac-Sim/Nvidia-Isaac-Sim-Env
```

Repository:

```text
https://github.com/ciccio42/Nvidia-Isaac-Sim-Env.git
```

Branch:

```text
ainfante32
```

Both repositories were committed and pushed after the final validation.

---

# 3. Isaac Sim scene and USD assets

Relevant USD files:

```text
usd/ur5e_2f_85/ur5e_2f_85_isaac.usd
usd/ur5e_2f_85/ur5e_2f_85_reimport_physics_validated.usd
usd/ur5e_2f_85/seedo_scene_v2.usd
usd/ur5e_2f_85/zed_camera_visual.usd
```

Final scene:

```text
/isaac-sim/usd/ur5e_2f_85/seedo_scene_v2.usd
```

Validated robot asset:

```text
/isaac-sim/usd/ur5e_2f_85/ur5e_2f_85_reimport_physics_validated.usd
```

---

# 4. ROS 2 configuration

The setup uses:

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

These values are shared by both the Isaac Sim container and the SeeDo ROS 2 container, allowing direct ROS 2 communication over `--network host`.

---

# 5. TABLE_0 transform

The transform between `base_link` and `table_0` was validated as:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.00 --y 0.612 --z -0.120 \
  --qx 0.000 --qy 0.000 --qz 1.000 --qw 0.000 \
  --frame-id base_link \
  --child-frame-id table_0
```

In the final Isaac scene this transform is published directly from the ActionGraph.

Therefore, in simulation, the old external `static_transform_publisher` must **not** be launched, otherwise duplicate TF publishers are created.

---

# 6. Camera integration

Three ZED cameras were reproduced in Isaac:

```text
/Environment/table_0/calibration_map
    ├── zed_front
    ├── zed_left
    └── zed_right
```

Resolution:

```text
672 × 376
```

Published topics:

```text
/zed_front/zed_node/rgb/color/rect/image
/zed_front/zed_node/depth/depth_registered
/zed_front/zed_node/rgb/color/rect/camera_info

/zed_left/zed_node/rgb/color/rect/image
/zed_left/zed_node/depth/depth_registered
/zed_left/zed_node/rgb/color/rect/camera_info

/zed_right/zed_node/rgb/color/rect/image
/zed_right/zed_node/depth/depth_registered
/zed_right/zed_node/rgb/color/rect/camera_info
```

Depth format:

```text
32FC1
```

The final camera transforms were numerically compared against:

```text
zed_camera/zed_camera_calibration/estimated_camera_positions.yaml
```

The comparison accounted for the OpenCV/ROS optical camera convention versus the USD camera convention.

Final result:

```text
PASS — camera poses match calibration YAML
```

The camera transforms must therefore be considered validated and should not be modified without a new calibration.

---

# 7. ZED visual housings

The ZED visual mesh:

```text
/workspace/build_ws/src/ur5e_2f_85_description/meshes/camera_gripper_attach/zed_camera.stl
```

was converted to:

```text
/isaac-sim/usd/ur5e_2f_85/zed_camera_visual.usd
```

Each camera has a visual-only child:

```text
visual_model
```

No rigid-body or collision physics were applied to the visual housing.

A final local transform was used to prevent the housing from appearing inside the RGB image.

The camera supports/poles were also added as purely visual geometry.

---

# 8. Robot initial Home configuration

The real-like Home configuration was measured as:

```text
shoulder_pan_joint   =  1.5609 rad
shoulder_lift_joint  = -2.3425 rad
elbow_joint          =  2.3534 rad
wrist_1_joint        = -1.5739 rad
wrist_2_joint        = -1.5755 rad
wrist_3_joint        = -0.0086 rad
```

Equivalent values in degrees:

```text
shoulder_pan_joint   ≈  89.433°
shoulder_lift_joint  ≈ -134.215°
elbow_joint          ≈ 134.840°
wrist_1_joint        ≈ -90.178°
wrist_2_joint        ≈ -90.270°
wrist_3_joint        ≈ -0.493°
```

A dedicated Isaac-only initial position file was created:

```text
ur5e_2f_85/ur5e_2f_85_moveit_config/config/initial_positions_isaac.yaml
```

Content:

```yaml
initial_positions:
  elbow_joint: 2.3534
  robotiq_85_left_knuckle_joint: 0.0
  shoulder_lift_joint: -2.3425
  shoulder_pan_joint: 1.5609
  wrist_1_joint: -1.5739
  wrist_2_joint: -1.5755
  wrist_3_joint: -0.0086
```

The Isaac Xacro was modified so that the Isaac path uses this file without affecting the real robot path.

The robot starts in Home when Isaac is put into **Play**.

The Stop appearance of the USD was intentionally left unchanged.

---

# 9. Isaac robot_state_publisher launch

A dedicated launch file was created:

```text
ur5e_2f_85/ur5e_2f_85_moveit_config/launch/isaac_robot_state_publisher.launch.py
```

It:

1. processes `ur5e_2f_85.isaac.urdf.xacro`;
2. uses `initial_positions_isaac.yaml`;
3. removes the legacy top-level `ros2_control name="RobotiqGripperHardwareInterface"`;
4. passes the resulting URDF directly to `robot_state_publisher`.

Typical launch:

```bash
source /home/ros2_ws/install/setup.bash
source /opt/topic_based_ws/install/setup.bash

ros2 launch \
  ur5e_2f_85_moveit_config \
  isaac_robot_state_publisher.launch.py
```

---

# 10. ros2_control / Isaac interface

The Isaac ROS 2 control path uses `TopicBasedSystem`.

Commanded joints:

```text
shoulder_pan_joint
shoulder_lift_joint
elbow_joint
wrist_1_joint
wrist_2_joint
wrist_3_joint
robotiq_85_left_knuckle_joint
```

Topics:

```text
/isaac_joint_commands
/isaac_joint_states
```

The configuration uses:

```text
sum_wrapped_joint_states=true
```

---

# 11. Controllers

The final idle controller configuration is:

```text
joint_state_broadcaster              active
hold_position_controller             active
robotiq_gripper_controller           active
scaled_joint_trajectory_controller   inactive
```

`hold_position_controller` is a `JointTrajectoryController`.

`forward_position_controller` must not be used for this integration.

Typical `moveit_controller` startup:

```bash
ros2 run moveit_controller moveit_controller_node \
  --ros-args \
  -p execute_trajectory:=true \
  -p controller_to_stop:=hold_position_controller
```

Go Home:

```bash
ros2 service call \
  /set_robot_to_home \
  moveit_controller_srvs/srv/GoHome \
  "{}"
```

---

# 12. Robotiq gripper physics

The final gripper physics configuration was tuned until a stable grasp-and-lift was obtained.

## Master drive

```text
joint: robotiq_85_left_knuckle_joint
maxForce: 16.5
stiffness: 1.0173239707946777
damping: 0.0004069295828230679
```

A previous test at `maxForce = 50` caused unstable/explosive contact.

Therefore `maxForce = 16.5` must be preserved.

## Fingertip weak angular drives

```text
type: force
targetPosition: 0
stiffness: 0.0002
damping: 0.00001
maxForce: 0.5
```

## Gripper action controller

```yaml
allow_stalling: true
stall_velocity_threshold: 0.55
stall_timeout: 3.0
```

Goal tolerance:

```text
0.01
```

The deprecated gripper action-controller warning was deliberately left untouched because the current setup is functional.

---

# 13. Cubes and physical properties

The scene contains:

```text
cube_green
cube_blue
cube_red
cube_yellow
```

All cubes were validated with identical physical settings:

```text
CollisionAPI:       True
RigidBodyAPI:       True
MassAPI:            True
collisionEnabled:   True
rigidBodyEnabled:   True
kinematicEnabled:   False
mass:               0.1 kg
density:            0.0
```

The target cube must remain a dynamic rigid body.

---

# 14. Scene layout for automated tests

A configurable 3×4 grid was defined.

All cubes always use:

```text
z = 0.036
```

| Row | Column 0 | Column 1 | Column 2 | Column 3 |
|---|---|---|---|---|
| 0 | `(-0.23, 0.24)` | `(-0.07, 0.24)` | `(0.07, 0.24)` | `(0.23, 0.24)` |
| 1 | `(-0.23, 0.15)` | `(-0.07, 0.15)` | `(0.07, 0.15)` | `(0.23, 0.15)` |
| 2 | `(-0.23, 0.06)` | `(-0.07, 0.06)` | `(0.07, 0.06)` | `(0.23, 0.06)` |

---

# 15. Task → target cube mapping

```text
Task 00–03 → cube_green
Task 04–07 → cube_yellow
Task 08–11 → cube_blue
Task 12–15 → cube_red
```

---

# 16. Trajectories

Each task has 40 trajectories.

Trajectories `000–019` use:

```text
rotation Z = 0°
```

Trajectories `020–039` reuse the same layouts as `000–019`, but apply:

```text
rotation Z = 45°
```

The target cube receives the trajectory-specific target slot.

The other three cubes are deterministically assigned to the remaining three slots to keep the test reproducible.

---

# 17. Automatic Isaac scene configuration — SeeDo-side client

File:

```text
ai_controller/ai_controller/models/seedo_controller/configure_isaac_scene.py
```

Responsibilities:

- accepts `--task`;
- accepts `--trajectory`;
- asks interactively for missing values;
- validates task and trajectory ranges;
- determines the target cube;
- generates the four cube poses;
- sends the configuration to Isaac over ROS 2;
- waits for an ACK from Isaac;
- exits with error if Isaac rejects the configuration.

Examples:

```bash
python3 configure_isaac_scene.py
```

Interactive:

```text
Task [00-15]:
Trajectory [000-039]:
```

Fully parameterized:

```bash
python3 configure_isaac_scene.py \
  --task 00 \
  --trajectory 020
```

Partially parameterized:

```bash
python3 configure_isaac_scene.py --task 00
```

asks only for the trajectory.

---

# 18. Isaac-side scene configuration server

File:

```text
docker/isaac_scene_config_server.py
```

ROS topics:

```text
/seedo/configure_scene_request
/seedo/configure_scene_response
```

ROS node:

```text
/seedo_isaac_scene_config_server
```

The server:

1. receives the four requested cube poses;
2. validates cube names;
3. validates `z = 0.036`;
4. validates `rz ∈ {0°, 45°}`;
5. checks that no two cubes occupy the same XY position;
6. finds the corresponding cube prims in the open Stage;
7. updates their transforms;
8. resets linear/angular velocity;
9. returns an ACK.

Validated communication:

```bash
ros2 node list | grep seedo_isaac_scene_config_server
```

Expected:

```text
/seedo_isaac_scene_config_server
```

And:

```bash
ros2 topic list | grep configure_scene
```

Expected:

```text
/seedo/configure_scene_request
/seedo/configure_scene_response
```

---

# 19. Important Stage vs Session Layer issue

Initially the scene server wrote cube transforms to the USD **Session Layer**.

This caused a critical regression:

```text
the gripper closed around the cube,
but the cube was not lifted
```

The cube was verified to still be:

```text
RigidBodyAPI: True
MassAPI: True
rigidBodyEnabled: True
kinematicEnabled: False
mass: 0.1
```

The issue disappeared when the server was changed to write directly to the Stage's normal edit target.

Therefore the final server must use normal Stage authoring and must **not** switch to `stage.GetSessionLayer()` for cube rigid-body transforms.

This is one of the most important implementation details discovered during testing.

Because the Stage is modified directly, it can become dirty after a scene configuration. Saving the USD will also save the current cube positions. This is acceptable because the automated configurator rewrites the positions whenever needed.

---

# 20. Automatic startup of the Isaac scene server

Initially the scene server had to be started manually from the Script Editor:

```python
exec(open(
    "/workspace/isaac_tools/isaac_scene_config_server.py"
).read())
```

This was replaced with an Isaac Sim extension.

Extension structure:

```text
docker/exts/seedo.scene_config/
├── config/
│   └── extension.toml
└── seedo/
    ├── __init__.py
    └── scene_config/
        ├── __init__.py
        └── extension.py
```

The extension automatically starts:

```text
/workspace/isaac_tools/isaac_scene_config_server.py
```

when Isaac Sim launches.

The Isaac startup command includes:

```bash
--ext-folder /workspace/isaac_tools/exts \
--enable seedo.scene_config
```

Therefore the manual Script Editor command is no longer required.

---

# 21. Isaac Docker container

```bash
docker run --name isaac-sim-ur5e \
  -it \
  --rm \
  --net=host \
  --ipc=host \
  --gpus all \
  -e ACCEPT_EULA=Y \
  -e ROS_DOMAIN_ID=42 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e PRIVACY_CONSENT=Y \
  -v "$HOME/.Xauthority:/isaac-sim/.Xauthority" \
  -e DISPLAY \
  -v "$HOME/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw" \
  -v "$HOME/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw" \
  -v "$HOME/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw" \
  -v "$HOME/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw" \
  -v "$HOME/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw" \
  -v "$HOME/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw" \
  -v "$PWD/usd:/isaac-sim/usd" \
  -v "$PWD/docker:/workspace/isaac_tools:ro" \
  isaac-sim-ur5e:5.1.0
```

The `docker` directory mount exposes the custom extension and scene server inside the container.

---

# 22. Isaac startup command

Inside the Isaac container:

```bash
cd /isaac-sim/usd/ur5e_2f_85

source /opt/ros/jazzy/setup.bash
source /workspace/jazzy_ws/install/setup.bash
source /workspace/build_ws/install/setup.bash

export ROS_PACKAGE_PATH="/workspace/build_ws/install/share:/workspace/jazzy_ws/install/share:/opt/ros/jazzy/share"

/isaac-sim/isaac-sim.sh --allow-root \
  --ext-folder /workspace/isaac_tools/exts \
  --enable seedo.scene_config \
  --enable isaacsim.asset.importer.urdf \
  --enable isaacsim.asset.exporter.urdf \
  --enable isaacsim.ros2.urdf \
  --enable isaacsim.ros2.bridge
```

---

# 23. SeeDo ROS 2 container

```bash
docker run --rm -it \
  --name seedo_ros2_container \
  --gpus all \
  --network host \
  --ipc=host \
  -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$(pwd)":/home/ros2_ws/src/UR5e-2f-85 \
  -v "$(pwd)/../test_dataset":/test_dataset:ro \
  -v "$(pwd)/../seedo_tests":/seedo_tests \
  -v "$(pwd)/../scene_capture":/scene_capture:ro \
  -v "$(pwd)/../test_isaac":/test_isaac \
  -v "$(pwd)/../test_real":/test_real \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e OPENAI_API_KEY="$OPENAI_API_KEY" \
  -e BUILD_SEEDO_ROS=1 \
  -e ROS_DOMAIN_ID=42 \
  seedo_ros2:latest
```

---

# 24. `run_seedo_isaac_test.sh`

The test launcher was extended to optionally configure the Isaac scene before launching SeeDo.

File:

```text
ai_controller/ai_controller/models/seedo_controller/run_seedo_isaac_test.sh
```

Desired behavior:

## No parameters

```bash
./run_seedo_isaac_test.sh
```

The script asks:

```text
Vuoi configurare la scena? [y/N]:
```

If the answer is no, scene configuration is skipped and the current scene is reused.

If the answer is yes, it asks:

```text
Task [00-15]:
Trajectory [000-039]:
```

## Both parameters

```bash
./run_seedo_isaac_test.sh \
  --task 00 \
  --trajectory 004
```

The scene is configured directly without asking whether it should be configured.

## Only task

```bash
./run_seedo_isaac_test.sh --task 00
```

The script asks only for:

```text
Trajectory [000-039]:
```

## Only trajectory

```bash
./run_seedo_isaac_test.sh --trajectory 004
```

The script asks only for:

```text
Task [00-15]:
```

The scene configuration is considered mandatory whenever at least one of the two parameters is supplied.

If Isaac responds with failure, SeeDo is **not** started.

---

# 25. Current demo/action-plan behavior

At the moment the automatic task/trajectory values are used to configure the simulated scene only.

The demo video and precomputed action plan are still fixed.

This was intentional because the complete set of test videos is not yet available.

Dynamic selection of `demo_path` and action plan based on task/trajectory is a future step.

---

# 26. End-to-end test status

The final integrated flow was tested as:

```text
run_seedo_isaac_test.sh
        ↓
optional task / trajectory selection
        ↓
configure_isaac_scene.py
        ↓
ROS 2 request
        ↓
Isaac scene configuration server
        ↓
cube transforms updated in Stage
        ↓
SUCCESS ACK
        ↓
SeeDo
        ↓
perception / planning
        ↓
MoveIt
        ↓
UR5e motion
        ↓
Robotiq close
        ↓
cube grasp
        ↓
cube lift
```

The grasp-and-lift regression caused by Session Layer authoring was resolved.

The complete integration was validated after switching back to normal Stage authoring.

---

# 27. Validated constraints — do not change casually

The following values/configurations are currently known-good and should be treated as checkpoints.

## Gripper

```text
master maxForce = 16.5
master stiffness = 1.0173239707946777
master damping = 0.0004069295828230679
```

Fingertip weak drive:

```text
stiffness = 0.0002
damping = 0.00001
maxForce = 0.5
```

Do not restore previous stronger values without a specific reason.

## Physics

- Do not recreate cube/gripper colliders.
- Do not modify the validated closed-loop/mimic topology casually.
- Do not reapply the previously tested unstable solver/velocity settings.

## Controllers

Do not use:

```text
forward_position_controller
```

Do not modify `moveit_controller` source code for this integration.

## Cameras

Camera extrinsics are validated against calibration and should not be changed.

## TF

Do not start the old external `table_0` static TF publisher in simulation.

## GroundingDINO

Do not change the validated GroundingDINO thresholds as part of unrelated Isaac/debugging work.

---

# 28. Suggested future work

1. Make the demo video path depend on `task` and `trajectory`.
2. Make the action-plan selection depend on the selected test.
3. Optionally save a `scene_setup.json` inside each run directory.
4. Optionally make the Docker launchers more portable.
5. Automate larger batches of the full `16 × 40 = 640` test combinations once the complete dataset is available.

---

# 29. Final status

The current system supports:

```text
✓ Isaac Sim 5.1 as simulated hardware
✓ UR5e + Robotiq 2F-85
✓ MoveIt 2 / ros2_control integration
✓ three calibrated ZED cameras
✓ RGB + depth + CameraInfo publication
✓ real-like robot Home configuration
✓ stable gripper grasp and cube lift
✓ task-dependent target cube
✓ 40 trajectory layouts
✓ 0° / 45° cube orientation tests
✓ automatic scene configuration over ROS 2
✓ automatic Isaac scene-server startup
✓ optional scene-configuration skip
✓ interactive or parameterized task/trajectory selection
✓ end-to-end SeeDo execution
```

The integration can therefore be considered a validated baseline for future automated SeeDo experiments in Isaac Sim.
