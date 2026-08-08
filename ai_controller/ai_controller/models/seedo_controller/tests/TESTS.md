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

# 7. Complete SeeDo Controller

Runs the complete pipeline through the public `SeeDoController` interface.

The test first processes the demonstration video through `load_command()`. It then provides the captured RGB-D runtime scene to `inference(t=0)` to generate the semantic scene and primitive plan.

Finally, the generated primitive plan is consumed step by step through subsequent calls to `inference(t)`.

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
└── lmp_generator/
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

All pipeline stages use this directory exactly as in persistent mode.

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
    ├── inference(t=1) → PrimitiveStep #1
    ├── inference(t=2) → PrimitiveStep #2
    ├── ...
    └── inference(t=N) → PrimitiveStep #N
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
- step-by-step primitive retrieval through `inference(t)`
- consistency between returned primitive steps and the generated `PrimitivePlan`
- transition to the `completed` execution state
- persistent artifact generation and preservation
- temporary artifact generation and automatic cleanup
- controller reset

The test does not execute physical robot motion. The primitive sequence is generated and consumed step by step through the controller interface. Actual robot execution is left to the motion/execution layer.