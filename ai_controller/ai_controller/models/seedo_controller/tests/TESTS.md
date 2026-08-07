# SeeDo Controller Tests

The following tests validate the modular SeeDo controller pipeline.

Each stage can be executed independently or as part of the complete end-to-end pipeline.

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

Runs the runtime perception pipeline starting from a captured RGB-D scene.

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
- scene JSON

---

# 5. Scene Interpreter

Runs the semantic interpretation stage.

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

Runs the complete planning pipeline from video to CAP primitive generation.

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

```
Video
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
Natural-language plan
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

Runs the complete SeeDo controller through the public `SeeDoController` interface.

```bash
python -m ai_controller.models.seedo_controller.tests \
  --stage seedo_controller \
  --video /test_dataset/pick_place/human_rgb_pick_place/task_00/traj000/converted/traj000-h264-30fps_modified.mp4 \
  --artifacts-dir /seedo_tests/seedo_controller \
  --model-config /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/config/seedo_controller.yaml
```

Pipeline:

```
Video
    │
    ▼
SeeDoController
    │
    ▼
PrimitivePlan
```

The test validates:

- YAML configuration loading
- controller initialization
- complete modular pipeline execution
- artifact generation
- primitive plan generation
- controller reset