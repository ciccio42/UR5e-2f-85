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