# Isaac Sim + ROS 2 Control + MoveIt Integration for UR5e + Robotiq 2F-85

## Overview

This document describes the integration that has been implemented and validated between:

- NVIDIA Isaac Sim 5.1
- ROS 2 Jazzy
- `ros2_control`
- `topic_based_ros2_control`
- MoveIt 2
- the existing `moveit_controller`
- the UR5e + Robotiq 2F-85 model used by the project

The validated execution chain is:

```text
MoveIt
  -> ExecuteTrajectory
  -> scaled_joint_trajectory_controller
  -> ros2_control
  -> topic_based_ros2_control/TopicBasedSystem
  -> /isaac_joint_commands
  -> NVIDIA Isaac Sim
  -> /isaac_joint_states
  -> joint_state_broadcaster
  -> /joint_states
```

A real simulated arm movement has been executed successfully through the full chain above.

---

## 1. Isaac Sim robot model

The UR5e + Robotiq 2F-85 model was imported into Isaac Sim from the project robot description.

The imported stage contains:

- UR5e arm
- Robotiq 2F-85 gripper
- platform/table geometry
- ZED camera attachment geometry
- `tcp_link`

The imported Isaac stage is persisted as USD and can be reopened in a fresh Isaac Sim container.

The main persistent stage is:

```text
/isaac-sim/usd/ur5e_2f_85/ur5e_2f_85_isaac.usd
```

An Isaac-specific URDF generation flow was also added in the Isaac environment so that `package://` mesh references are converted to paths that Isaac can resolve.

---

## 2. Robotiq import fixes in Isaac Sim

The imported Robotiq model required a correction to its mimic-joint configuration.

A patch was introduced for the left inner knuckle joint so that its limits and mimic relationship are valid after import.

The validated master joint for the gripper is:

```text
robotiq_85_left_knuckle_joint
```

The gripper was tested manually in Isaac using:

```text
0.0 rad -> open
0.8 rad -> closed
```

The initial mimic configuration also showed instability when rotating `wrist_3_joint`.

The five mimic joints were stabilized by changing their mimic natural frequency from:

```text
25 -> 100
```

while keeping the original damping value.

After this change:

- wrist rotation no longer caused large mimic-joint distortions;
- opening and closing the gripper remained functional;
- the setting persisted after reopening the USD stage.

---

## 3. Isaac ROS 2 Action Graph

A persistent Isaac ROS 2 Action Graph was created in the USD stage.

It contains:

```text
On Playback Tick
Isaac Read Simulation Time
ROS2 Publish Joint State
ROS2 Subscribe Joint State
Articulation Controller
```

The graph publishes robot state on:

```text
/isaac_joint_states
```

and receives commands on:

```text
/isaac_joint_commands
```

The publisher exposes the 12 simulated articulation joints:

```text
shoulder_pan_joint
shoulder_lift_joint
elbow_joint
wrist_1_joint
wrist_2_joint
wrist_3_joint
robotiq_85_left_inner_knuckle_joint
robotiq_85_left_knuckle_joint
robotiq_85_right_inner_knuckle_joint
robotiq_85_right_knuckle_joint
robotiq_85_left_finger_tip_joint
robotiq_85_right_finger_tip_joint
```

Direct ROS 2 commands to `/isaac_joint_commands` were validated for both the arm and the gripper.

Example:

```bash
ros2 topic pub --once /isaac_joint_commands sensor_msgs/msg/JointState \
"{name: ['wrist_3_joint'], position: [1.0]}"
```

---

## 4. ROS 2 communication between containers

Isaac Sim and the SeeDo ROS 2 container are isolated using:

```text
ROS_DOMAIN_ID=42
```

Both containers use Fast DDS:

```text
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

For ROS 2 data transport between the two Docker containers, both containers use:

```text
--network host
--ipc=host
```

Without `--ipc=host` on the SeeDo container, ROS 2 discovery could see the Isaac topics but messages from `/isaac_joint_states` were not received.

After adding `--ipc=host`, the SeeDo container successfully received the full Isaac `JointState` stream.

---

## 5. `topic_based_ros2_control`

The package:

```text
PickNikRobotics/topic_based_ros2_control
```

was added to the SeeDo Docker image and built in:

```text
/opt/topic_based_ws
```

The installed hardware plugin is:

```text
topic_based_ros2_control/TopicBasedSystem
```

The runtime environment must source the workspaces in this order:

```bash
source /home/ros2_ws/install/setup.bash
source /opt/topic_based_ws/install/setup.bash
```

This order is required so that `controller_manager` can discover:

```text
topic_based_ros2_control/TopicBasedSystem
```

The SeeDo entrypoint was updated accordingly.

---

## 6. SeeDo entrypoint build changes

The ROS build executed by `BUILD_SEEDO_ROS=1` includes the packages required for the Isaac/MoveIt integration.

The relevant packages include:

```text
moveit_controller_srvs
moveit_controller
ur5e_2f_85_teleoperation_msg
dataset_collector_pkg
ai_controller
ur5e_2f_85_description
ur5e_2f_85_moveit_config
```

The final overlay order used by the entrypoint is:

```bash
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# build /home/ros2_ws

source "/home/ros2_ws/install/setup.bash"

if [ -f "/opt/topic_based_ws/install/setup.bash" ]; then
    source "/opt/topic_based_ws/install/setup.bash"
fi
```

When opening a new shell with:

```bash
docker exec -it seedo_ros2_container bash
```

the two workspace setup files are sourced manually before starting ROS nodes:

```bash
source /home/ros2_ws/install/setup.bash
source /opt/topic_based_ws/install/setup.bash
```

---

## 7. Isaac-specific `ros2_control` description

A dedicated Isaac `ros2_control` xacro was added:

```text
ur5e_2f_85/ur5e_2f_85_moveit_config/config/
ur5e_2f_85.isaac_ros2_control.xacro
```

It uses:

```xml
<plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
```

with:

```xml
<param name="joint_commands_topic">/isaac_joint_commands</param>
<param name="joint_states_topic">/isaac_joint_states</param>
<param name="sum_wrapped_joint_states">true</param>
```

The Isaac hardware system currently exports the six UR5e arm joints:

```text
shoulder_pan_joint
shoulder_lift_joint
elbow_joint
wrist_1_joint
wrist_2_joint
wrist_3_joint
```

The existing project robot description already contains a separate legacy Robotiq `ros2_control` block based on:

```text
mock_components/GenericSystem
```

During arm integration, the Robotiq master joint was therefore removed from `IsaacSystem` to avoid duplicate hardware interfaces.

This resolved errors of the form:

```text
ResourceStorage: Tried to insert StateInterface with already existing key
ResourceStorage: Tried to insert CommandInterface with already existing key
```

---

## 8. Isaac-specific robot description

The dedicated MoveIt/Isaac robot xacro is:

```text
ur5e_2f_85/ur5e_2f_85_moveit_config/config/
ur5e_2f_85.isaac.urdf.xacro
```

It includes the existing project platform robot description and instantiates:

```text
IsaacSystem
```

instead of the default fake arm system.

The generated URDF was validated with:

```bash
xacro \
  /home/ros2_ws/src/UR5e-2f-85/ur5e_2f_85/ur5e_2f_85_moveit_config/config/ur5e_2f_85.isaac.urdf.xacro \
  -o /tmp/ur5e_2f_85_isaac.urdf

check_urdf /tmp/ur5e_2f_85_isaac.urdf
```

---

## 9. Isaac controller configuration

A dedicated controller configuration was added:

```text
ur5e_2f_85/ur5e_2f_85_moveit_config/config/
ros2_controllers_isaac.yaml
```

The validated controller configuration contains:

```text
joint_state_broadcaster
scaled_joint_trajectory_controller
forward_position_controller
gripper_controller
```

For the arm:

```text
scaled_joint_trajectory_controller
```

uses the six UR5e joints with a `position` command interface.

The initial controller state used by the existing `moveit_controller` is:

```text
forward_position_controller          active
joint_state_broadcaster              active
scaled_joint_trajectory_controller   inactive
```

The switch:

```text
forward_position_controller
    ->
scaled_joint_trajectory_controller
```

was validated with `STRICT` switching.

The reverse switch was also validated.

---

## 10. Controller Manager validation

`controller_manager` successfully loads:

```text
RobotiqGripperHardwareInterface
  -> mock_components/GenericSystem

IsaacSystem
  -> topic_based_ros2_control/TopicBasedSystem
```

The final initialization reaches:

```text
Resource Manager has been successfully initialized.
Starting Controller Manager services...
```

The six UR5e command interfaces are exposed as:

```text
shoulder_pan_joint/position
shoulder_lift_joint/position
elbow_joint/position
wrist_1_joint/position
wrist_2_joint/position
wrist_3_joint/position
```

with corresponding position and velocity state interfaces.

---

## 11. Joint-state bridge validation

`joint_state_broadcaster` was loaded and activated.

The resulting flow is:

```text
/isaac_joint_states
    ->
TopicBasedSystem
    ->
ros2_control state interfaces
    ->
joint_state_broadcaster
    ->
/joint_states
```

The six arm joint values published on `/joint_states` were verified to match the values published by Isaac on `/isaac_joint_states`.

Example validated values after trajectory execution:

```text
shoulder_pan_joint   0.0058
shoulder_lift_joint  0.0215
elbow_joint         -0.0082
wrist_1_joint        0.0016
wrist_2_joint       -0.0003
wrist_3_joint        0.0235
```

These values were identical between `/isaac_joint_states` and `/joint_states`.

---

## 12. Isaac-specific MoveIt launch

A dedicated MoveIt launch file was added:

```text
ur5e_2f_85/ur5e_2f_85_moveit_config/launch/
move_group_isaac.launch.py
```

It builds the MoveIt configuration using:

```text
config/ur5e_2f_85.isaac.urdf.xacro
```

and:

```text
config/moveit_controllers.yaml
```

The existing legacy 3D sensor configuration caused a headless MoveIt crash in the occupancy-map initialization path.

For the Isaac arm test, the dedicated launch disables that legacy sensor configuration with:

```python
moveit_config.sensors_3d = {}
```

After this change, `move_group` starts correctly and reaches:

```text
MoveGroup context initialization complete
You can start planning now!
```

MoveIt also detects the configured FollowJointTrajectory controllers, including:

```text
scaled_joint_trajectory_controller
```

---

## 13. MoveIt plan-only validation

The existing `moveit_controller` supports:

```text
execute_trajectory:=false
```

In this mode it:

- requests a MoveIt plan;
- publishes the planned trajectory to `/display_planned_path`;
- does not switch controllers;
- does not call `ExecuteTrajectory`.

The node was launched with:

```bash
ros2 run moveit_controller moveit_controller_node \
  --ros-args \
  -p execute_trajectory:=false
```

A plan-only request was executed with:

```bash
ros2 service call /set_robot_to_home \
  moveit_controller_srvs/srv/GoHome \
  "{}"
```

The validated response was:

```text
success=True
message='Home trajectory planned (not executed).'
```

The node log confirmed:

```text
execute_trajectory:=False - skipping controller switch (plan-only).
Planning trajectory to specified joint positions...
Published planned trajectory to /display_planned_path.
execute_trajectory:=False - plan published for RViz, skipping execution.
```

---

## 14. Full MoveIt-to-Isaac execution validation

After validating plan-only mode, `moveit_controller` was launched with trajectory execution enabled.

A small joint-space target was used to validate the complete execution chain while keeping the requested motion small.

The test target was:

```text
[0.0004, 0.0111, 0.0, 0.0001, -0.0004, 0.02]
```

for the joint order:

```text
elbow_joint
shoulder_lift_joint
shoulder_pan_joint
wrist_1_joint
wrist_2_joint
wrist_3_joint
```

The controller was launched with:

```bash
ros2 run moveit_controller moveit_controller_node \
  --ros-args \
  -p execute_trajectory:=true \
  -p home_joint_positions:="[0.0004, 0.0111, 0.0, 0.0001, -0.0004, 0.02]"
```

The trajectory was executed with:

```bash
ros2 service call /set_robot_to_home \
  moveit_controller_srvs/srv/GoHome \
  "{}"
```

The service returned:

```text
success=True
message='Robot moved to home position successfully.'
```

The simulated UR5e moved in Isaac.

The resulting `wrist_3_joint` state was approximately:

```text
0.0235 rad
```

for a requested target of:

```text
0.02 rad
```

The final position was observed both on:

```text
/isaac_joint_states
```

and:

```text
/joint_states
```

This validates the complete path:

```text
MoveIt
  -> ExecuteTrajectory
  -> scaled_joint_trajectory_controller
  -> ros2_control
  -> TopicBasedSystem
  -> /isaac_joint_commands
  -> Isaac articulation
  -> /isaac_joint_states
  -> /joint_states
```

---

## 15. Cartesian IK test performed

A Cartesian test was also performed using the current TCP transform:

```text
Translation:
x = 0.816
y = 0.380
z = 0.054

Quaternion:
x = 0.004
y = 0.707
z = 0.707
w = -0.004
```

A target with `z = 0.064` was submitted to `/set_robot_to_pose`.

The IK service returned:

```text
Error code: -31
```

and no trajectory was executed.

The controller correctly preserved the original controller state and the simulated robot did not move as a result of the failed IK request.

This test confirmed that failed IK requests do not propagate into trajectory execution.

---

## 16. Validated startup sequence

### Terminal 1 - robot state publisher

```bash
xacro \
  /home/ros2_ws/src/UR5e-2f-85/ur5e_2f_85/ur5e_2f_85_moveit_config/config/ur5e_2f_85.isaac.urdf.xacro \
  -o /tmp/ur5e_2f_85_isaac.urdf \
&& ros2 run robot_state_publisher robot_state_publisher \
  /tmp/ur5e_2f_85_isaac.urdf
```

### Terminal 2 - controller manager

```bash
source /home/ros2_ws/install/setup.bash
source /opt/topic_based_ws/install/setup.bash

ros2 run controller_manager ros2_control_node \
  --ros-args \
  --params-file /home/ros2_ws/src/UR5e-2f-85/ur5e_2f_85/ur5e_2f_85_moveit_config/config/ros2_controllers_isaac.yaml
```

### Terminal 3 - controller spawners

```bash
ros2 run controller_manager spawner joint_state_broadcaster \
  --param-file /home/ros2_ws/src/UR5e-2f-85/ur5e_2f_85/ur5e_2f_85_moveit_config/config/ros2_controllers_isaac.yaml
```

```bash
ros2 run controller_manager spawner forward_position_controller \
  --param-file /home/ros2_ws/src/UR5e-2f-85/ur5e_2f_85/ur5e_2f_85_moveit_config/config/ros2_controllers_isaac.yaml
```

```bash
ros2 run controller_manager spawner scaled_joint_trajectory_controller \
  --inactive \
  --param-file /home/ros2_ws/src/UR5e-2f-85/ur5e_2f_85/ur5e_2f_85_moveit_config/config/ros2_controllers_isaac.yaml
```

### Terminal 4 - MoveIt

```bash
source /home/ros2_ws/install/setup.bash
source /opt/topic_based_ws/install/setup.bash

ros2 launch ur5e_2f_85_moveit_config move_group_isaac.launch.py
```

### Terminal 5 - moveit_controller

Plan-only:

```bash
ros2 run moveit_controller moveit_controller_node \
  --ros-args \
  -p execute_trajectory:=false
```

Execution enabled:

```bash
ros2 run moveit_controller moveit_controller_node \
  --ros-args \
  -p execute_trajectory:=true
```

---

## 17. Validated integration status

The following functionality has been implemented and validated:

```text
[OK] UR5e + Robotiq model imported into Isaac Sim
[OK] Persistent Isaac USD stage
[OK] Robotiq mimic-joint import correction
[OK] Stable Robotiq mimic behavior with naturalFrequency = 100
[OK] Isaac ROS 2 Action Graph persisted in USD
[OK] /isaac_joint_states publication
[OK] /isaac_joint_commands subscription
[OK] Direct ROS 2 articulation commands in Isaac
[OK] ROS_DOMAIN_ID=42 isolation
[OK] Fast DDS cross-container communication
[OK] --ipc=host requirement validated
[OK] topic_based_ros2_control built into SeeDo image
[OK] TopicBasedSystem loaded by controller_manager
[OK] Isaac-specific ros2_control xacro
[OK] Isaac-specific robot xacro
[OK] Isaac-specific controller YAML
[OK] joint_state_broadcaster
[OK] forward_position_controller
[OK] scaled_joint_trajectory_controller
[OK] strict controller switching in both directions
[OK] /isaac_joint_states -> /joint_states bridge
[OK] Isaac-specific MoveIt launch
[OK] MoveIt headless startup
[OK] MoveIt plan-only through moveit_controller
[OK] ExecuteTrajectory through moveit_controller
[OK] Physical simulated movement of the UR5e in Isaac
[OK] State feedback from Isaac after MoveIt trajectory execution
```
