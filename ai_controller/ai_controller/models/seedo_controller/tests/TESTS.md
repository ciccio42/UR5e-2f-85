# SeeDo Controller Tests

The following tests validate the modular SeeDo controller pipeline.

Each stage can be executed independently or as part of the complete end-to-end pipeline.

Artifact storage supports two modes:

- **Persistent mode**: when `--artifacts-dir` is provided, generated artifacts are stored in the specified directory and preserved after execution.
- **Temporary mode**: when `--artifacts-dir` is omitted, `SeeDoController` automatically creates a temporary directory under `/tmp`. The temporary artifacts remain available during execution and are automatically removed when the controller is reset.

---

# 1. Keyframe Selection

Runs the keyframe selection module and saves the extracted keyframes.

```bash
python -m ai_controller.models.seedo_controller.tests \
  --stage keyframe \
  --video /test_dataset/pick_place/human_rgb_pick_place/task_00/traj000/converted/traj000-h264-30fps_modified.mp4 \
  --artifacts-dir /seedo_tests/keyframe_selection
```

Outputs:

- selected keyframes
- CSV diagnostics
- optional preview

---

# 2. Visual Prompting

Runs the SeeDo visual prompting stage (GroundingDINO + SAM + SAM2).

```bash
python -m ai_controller.models.seedo_controller.tests \
  --stage visual_prompting \
  --video /test_dataset/pick_place/human_rgb_pick_place/task_00/traj000/converted/traj000-h264-30fps_modified.mp4 \
  --expected-keyframes 25 51 \
  --artifacts-dir /seedo_tests/visual_prompting
```

Outputs:

- annotated tracking video
- track ID map
- keyframe object coordinates
- bounding box summary
- count diagnostics

---

# 3. Action Planning

Runs the VLM action planner.

```bash
python -m ai_controller.models.seedo_controller.tests \
  --stage action_planning \
  --video /test_dataset/pick_place/human_rgb_pick_place/task_00/traj000/converted/traj000-h264-30fps_modified.mp4 \
  --expected-keyframes 25 51 \
  --artifacts-dir /seedo_tests/action_planning
```

Outputs:

- natural-language action plan
- structured action plan
- OpenAI prompt
- raw OpenAI response
- JSON outputs

---

# 4. Scene Perceiver

Runs the runtime geometric perception pipeline starting from a captured RGB-D scene.

```bash
python -m ai_controller.models.seedo_controller.tests \
  --stage scene_perceiver \
  --scene-dir /scene_capture \
  --base-to-table-transform /scene_capture/base_to_table_transform.yaml \
  --model-config /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/config/seedo_controller.yaml \
  --artifacts-dir /seedo_tests/scene_perceiver
```

Outputs:

- annotated RGB image
- raw geometric scene
- raw scene JSON

---

# 5. Scene Interpreter

Runs the semantic interpretation stage on top of the runtime perception result.

```bash
python -m ai_controller.models.seedo_controller.tests \
  --stage scene_interpreter \
  --scene-dir /scene_capture \
  --base-to-table-transform /scene_capture/base_to_table_transform.yaml \
  --model-config /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/config/seedo_controller.yaml \
  --artifacts-dir /seedo_tests/scene_interpreter
```

Outputs:

- semantic scene
- semantic object names
- interpreted scene JSON

---

# 6. LMP Generator

Runs the complete planning pipeline from the demonstration video and captured runtime scene to CAP primitive generation.

```bash
python -m ai_controller.models.seedo_controller.tests \
  --stage lmp_generator \
  --video /test_dataset/pick_place/human_rgb_pick_place/task_00/traj000/converted/traj000-h264-30fps_modified.mp4 \
  --expected-keyframes 25 51 \
  --scene-dir /scene_capture \
  --base-to-table-transform /scene_capture/base_to_table_transform.yaml \
  --model-config /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/config/seedo_controller.yaml \
  --artifacts-dir /seedo_tests/lmp_generator
```

Pipeline:

```text
Demonstration Video
    │
    ▼
KeyframeSelector
    │
    ▼
VisualPrompter
    │
    ▼
ActionPlanner
    │
    ▼
ActionPlanningResult

Runtime RGB-D Scene
    │
    ▼
ScenePerceiver
    │
    ▼
RawSceneState
    │
    ▼
SceneInterpreter
    │
    ▼
SceneState

ActionPlanningResult + SceneState
    │
    ▼
LMPGenerator
    │
    ▼
PrimitivePlan
```

Outputs:

- generated CAP program
- primitive plan
- intermediate artifacts from all upstream modules

---

# 7. Motion Layer

Runs the SeeDo Motion Layer independently from the perception and planning pipeline.

The test loads an existing `SceneState` and `PrimitivePlan`, initializes the Motion Layer from a simulated end-effector pose, and translates each symbolic `PrimitiveStep` into low-level robot actions.

Each low-level action follows the format:

```text
[x, y, z, qx, qy, qz, qw, gripper_position]
```

A single symbolic primitive may generate multiple low-level actions. For example, `reach`, `approaching`, `lift_up`, `moving`, and `placing` generate linear waypoint sequences, while `pick` generates a gripper action at the current planned pose.

Example:

```text
PrimitiveStep("reach")
    │
    ▼
SeeDoMotionLayer
    │
    ├── low-level action #1
    ├── low-level action #2
    ├── ...
    └── low-level action #N
```

Run the test with:

```bash
python -m ai_controller.models.seedo_controller.tests.test_motion_layer \
  --scene-state /seedo_tests/ai_controller_node_ros/scene_interpreter/scene_state.json \
  --primitive-plan /seedo_tests/ai_controller_node_ros/lmp_generator/primitive_plan.json
```

The test validates:

- `SceneState` loading
- `PrimitivePlan` loading
- primitive target resolution through `SceneObject.object_id`
- fixed grasp orientation
- object Y offset
- reach hover height
- approach offset
- lift height
- release height
- gripper open/closed commands
- linear waypoint generation
- planned-pose chaining between primitives
- low-level action format
- final planned pose
- final gripper state
- Motion Layer artifact generation

For the current pick-and-place example, the generated primitive sequence is:

```text
reach("green cube")
approaching("green cube")
pick("green cube")
lift_up("green cube")
moving("first bin from the left")
placing("first bin from the left")
```

The corresponding test currently generates:

```text
reach       -> 19 low-level actions
approaching ->  7 low-level actions
pick        ->  1 low-level action
lift_up     ->  7 low-level actions
moving      -> 19 low-level actions
placing     ->  5 low-level actions
```

for a total of:

```text
58 low-level actions
```

The Motion Layer artifact is written to:

```text
/seedo_tests/motion_layer/
└── motion_plan.json
```

`motion_plan.json` stores:

- each symbolic primitive
- primitive arguments
- low-level actions generated for that primitive
- total number of generated actions
- final planned end-effector pose
- final gripper state

The test does not execute physical robot motion.

---

# 8. Complete SeeDo Controller

Runs the complete pipeline through the public `SeeDoController` interface.

The test first processes the demonstration video through `load_command()`.

It then provides the captured RGB-D runtime scene to `inference(t=0)` to generate:

- the semantic `SceneState`
- the CAP-generated `PrimitivePlan`

Subsequent calls to `inference(t)` consume the symbolic primitive plan one primitive at a time.

Each primitive is internally translated by the SeeDo Motion Layer into one or more executable low-level actions.

Therefore:

```text
1 SeeDo inference timestep = 1 PrimitiveStep
1 PrimitiveStep            = N low-level actions
```

For example:

```text
inference(t=1)
    │
    ▼
PrimitiveStep("reach")
    │
    ▼
SeeDoMotionLayer
    │
    ▼
list[np.ndarray]
    │
    ├── action #1
    ├── action #2
    ├── ...
    └── action #N
```

The first motion timestep also initializes the Motion Layer from the current end-effector position and orientation contained in `robot_state`.

After initialization, the Motion Layer maintains the planned pose internally so that each primitive starts from the final planned pose of the previous primitive.

## Persistent artifact mode

Providing `--artifacts-dir` stores and preserves all generated artifacts.

```bash
python -m ai_controller.models.seedo_controller.tests \
  --stage seedo_controller \
  --video /test_dataset/pick_place/human_rgb_pick_place/task_00/traj000/converted/traj000-h264-30fps_modified.mp4 \
  --scene-dir /scene_capture \
  --base-to-table-transform /scene_capture/base_to_table_transform.yaml \
  --model-config /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/config/seedo_controller.yaml \
  --artifacts-dir /seedo_tests/seedo_controller
```

The resulting artifact structure is:

```text
/seedo_tests/seedo_controller/
├── keyframe_selection/
├── visual_prompting/
├── action_planning/
├── scene_perceiver/
├── scene_interpreter/
├── lmp_generator/
└── motion_layer/
    └── motion_plan.json
```

Calling `controller.reset()` clears the controller state but does not remove these files.

## Temporary artifact mode

Omitting `--artifacts-dir` enables automatic temporary artifact storage.

```bash
python -m ai_controller.models.seedo_controller.tests \
  --stage seedo_controller \
  --video /test_dataset/pick_place/human_rgb_pick_place/task_00/traj000/converted/traj000-h264-30fps_modified.mp4 \
  --scene-dir /scene_capture \
  --base-to-table-transform /scene_capture/base_to_table_transform.yaml \
  --model-config /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/config/seedo_controller.yaml
```

During execution, the controller automatically creates a directory similar to:

```text
/tmp/seedo_xxxxxxxx/
```

All pipeline stages use this directory exactly as in persistent mode, including the Motion Layer.

When `controller.reset()` is called, the temporary directory and its contents are automatically removed.

## Controller pipeline

```text
Demonstration Video
    │
    ▼
SeeDoController.load_command()
    │
    ├── KeyframeSelector
    ├── VisualPrompter
    └── ActionPlanner
    │
    ▼
ActionPlanningResult

Runtime RGB-D Scene
    │
    ▼
SeeDoController.inference(t=0)
    │
    ├── ScenePerceiver
    ├── SceneInterpreter
    └── LMPGenerator
    │
    ▼
PrimitivePlan
    │
    ├── inference(t=1)
    │       │
    │       ▼
    │   PrimitiveStep #1
    │       │
    │       ▼
    │   MotionLayer
    │       │
    │       ▼
    │   list[low-level action]
    │
    ├── inference(t=2)
    │       │
    │       ▼
    │   PrimitiveStep #2
    │       │
    │       ▼
    │   MotionLayer
    │       │
    │       ▼
    │   list[low-level action]
    │
    ├── ...
    │
    └── inference(t=N)
            │
            ▼
        PrimitiveStep #N
            │
            ▼
        MotionLayer
            │
            ▼
        list[low-level action]
    │
    ▼
inference(t=N+1)
    │
    ▼
Completed
```

The test validates:

- YAML configuration loading
- controller initialization
- demonstration processing through `load_command()`
- runtime RGB-D input loading
- runtime geometric perception
- semantic scene interpretation
- CAP program generation
- `PrimitivePlan` generation
- primitive sequence generated by CAP
- Motion Layer initialization from end-effector state
- step-by-step primitive translation through `inference(t)`
- low-level action list generation
- low-level action shape `[x, y, z, qx, qy, qz, qw, gripper_position]`
- finite low-level action values
- planned-pose chaining across primitive steps
- transition to the `completed` execution state
- Motion Layer artifact generation
- persistent artifact generation and preservation
- temporary artifact generation and automatic cleanup
- controller reset

The test does not execute physical robot motion.

The complete controller currently generates the symbolic primitive plan and translates each primitive into low-level actions. Actual transmission of these actions to the robot is performed by the ROS2 `AIControllerNode` execution layer.

---

# 9. AIControllerNode Offline Integration

Runs the real `AIControllerNode.control_loop()` through an offline integration test.

The test validates the SeeDo integration at node level while keeping robot motion disabled.

It exercises:

- the real `control_loop()`
- SeeDo controller reset and demo loading
- `inference(t=0)` runtime planning
- primitive-by-primitive execution
- iteration over every low-level action returned by the Motion Layer
- SeeDo completion detection
- rollout control-flow integration

Physical robot motion is disabled with `move_robot=False`.

The test is intended to validate the node integration without requiring live ROS sensor topics or robot hardware.

---

# 10. AIControllerNode ROS Integration

Runs the real `AIControllerNode.control_loop()` with a synthetic ROS2 runtime.

The test publishes simulated RGB-D data, `CameraInfo`, and the required TF transform so that the node consumes the runtime scene through its normal ROS interfaces.

Run with:

```bash
python -m ai_controller.models.seedo_controller.tests.test_seedo_node_ros \
  --video /test_dataset/pick_place/human_rgb_pick_place/task_00/traj000/converted/traj000-h264-30fps_modified.mp4 \
  --scene-dir /scene_capture \
  --base-to-table-transform /scene_capture/base_to_table_transform.yaml \
  --model-config /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/config/seedo_controller.yaml \
  --artifacts-dir /seedo_tests/ai_controller_node_ros
```

The test validates:

- `AIControllerNode` initialization with `seedo_controller`
- ROS RGB-D publication and subscription
- `CameraInfo` publication and subscription
- TF lookup for `base_link <- table_0`
- SeeDo demo processing
- runtime perception and semantic interpretation
- CAP primitive generation
- Motion Layer translation
- iteration over every generated low-level action
- SeeDo completion handling inside `control_loop()`
- Motion Layer artifact generation

For the current pick-and-place example, the CAP plan is:

```text
reach("green cube")
approaching("green cube")
pick("green cube")
lift_up("green cube")
moving("first bin from the left")
placing("first bin from the left")
```

and the current Motion Layer output is:

```text
reach        -> 19 low-level actions
approaching  ->  7 low-level actions
pick         ->  1 low-level action
lift_up      ->  7 low-level actions
moving       -> 19 low-level actions
placing      ->  5 low-level actions
```

for a total of:

```text
58 low-level actions
```

The test does not execute physical robot motion.

---

# 11. Interactive ROS Dry-Run

Runs a production-like interactive dry-run of the real `AIControllerNode.control_loop()`.

Unlike the previous ROS integration test, this test keeps the normal interactive workflow and the real rollout-saving logic.

The synthetic ROS runtime publishes:

- SeeDo RGB image
- depth image
- `CameraInfo`
- legacy camera images used by `get_synced_images()`
- `/joint_states`
- TF `base_link -> table_0`
- TF `base_link -> tcp_link`

Therefore the following node functions are exercised without replacing them with mocked values:

```text
get_synced_images()
_capture_robot_state()
```

The test keeps:

```text
move_robot=False
```

so no `GoToPose` or gripper command is physically sent.

Run with:

```bash
python -m ai_controller.models.seedo_controller.tests.test_seedo_node_interactive \
  --video /test_dataset/pick_place/human_rgb_pick_place/task_00/traj000/converted/traj000-h264-30fps_modified.mp4 \
  --scene-dir /scene_capture \
  --base-to-table-transform /scene_capture/base_to_table_transform.yaml \
  --model-config /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/config/seedo_controller.yaml \
  --artifacts-dir /seedo_tests/ai_controller_node_interactive \
  --rollouts-dir /seedo_tests/ai_controller_node_interactive/rollouts
```

The normal `control_loop()` interaction is preserved:

```text
trajectory count
    │
    ▼
Press Enter to start
    │
    ▼
task ID
    │
    ▼
SeeDo execution
    │
    ▼
save_rollout()
    │
    ▼
rollout outcome questions
```

After one trajectory has completed and the rollout outcome questions have been answered, the control loop asks again:

```text
Press Enter to start the control loop. Make sure the robot is in a safe position.
```

At that point the dry-run can be stopped with `Ctrl+C`.

The test then validates that the rollout and SeeDo artifacts were generated successfully.

For task `00` and trajectory `000`, the rollout files are stored under:

```text
/seedo_tests/ai_controller_node_interactive/rollouts/
└── seedo_controller/
    └── pick_place/
        └── task_00/
            ├── traj_000.pkl
            └── traj_000.json
```

The Motion Layer artifact is stored at:

```text
/seedo_tests/ai_controller_node_interactive/
└── motion_layer/
    └── motion_plan.json
```

## Rollout recording behavior

For SeeDo, the `Trajectory` now records every low-level action produced by the Motion Layer.

Therefore:

```text
1 PrimitiveStep = N low-level actions = N Trajectory entries
```

For the current pick-and-place example:

```text
6 symbolic primitives
        ↓
58 low-level actions
        ↓
58 saved Trajectory entries
```

Each saved action follows:

```text
[x, y, z, qx, qy, qz, qw, gripper_position]
```

Only the final low-level action of the final primitive is marked as:

```text
done=True
reward=1
```

All previous low-level actions are stored with:

```text
done=False
reward=0
```

The observation stored with each rollout entry contains:

```text
joint_pos
joint_vel
gripper_qpos
gripper_qvel
eef_pos
eef_quat
camera_front_image
```

In the current synthetic dry-run, the robot state remains constant because the simulated robot does not physically move.

---

# 12. Saved Rollout Inspection

The saved SeeDo rollout can be inspected independently with:

```bash
python -m ai_controller.models.seedo_controller.tests.inspect_seedo_rollout
```

The default inspected trajectory is:

```text
/seedo_tests/ai_controller_node_interactive/rollouts/
└── seedo_controller/
    └── pick_place/
        └── task_00/
            └── traj_000.pkl
```

The inspector validates:

- top-level rollout metadata
- `savers.Trajectory` deserialization
- number of recorded trajectory entries
- observation keys
- robot-state array shapes
- front-camera image shape
- low-level action shape `(8,)`
- finite action values
- rollout termination flags
- final reward

For the current pick-and-place test, the expected result is:

```text
Number of recorded steps: 58
Valid 8D actions: 58/58
First step done: False
First step reward: 0
Last step done: True
Last step reward: 1

ROLLOUT INSPECTION PASSED
```

This verifies that every low-level action generated by the SeeDo Motion Layer is preserved in the saved `.pkl` rollout.