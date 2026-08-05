# SeeDo Controller Tests

The following tests validate the complete SeeDo integration as independent reusable modules.

---

# 1. Keyframe Selection

Runs the keyframe selection module and saves the extracted keyframes.

```bash
python -m ai_controller.models.seedo_controller.test \
  --stage keyframe \
  --video /test_dataset/pick_place/human_rgb_pick_place/task_00/traj000/converted/traj000-h264-30fps_modified.mp4 \
  --artifacts-dir /seedo_tests/keyframe_selection
```

Output:

- selected keyframes
- CSV diagnostics
- optional preview

---

# 2. Visual Prompting

Runs GroundingDINO + SAM2 starting from the selected keyframes.

```bash
python -m ai_controller.models.seedo_controller.test \
  --stage visual_prompting \
  --video /test_dataset/pick_place/human_rgb_pick_place/task_00/traj000/converted/traj000-h264-30fps_modified.mp4 \
  --artifacts-dir /seedo_tests/visual_prompting \
  --expected-keyframes 25 51
```

Output:

- annotated tracking video
- tracked objects
- keyframe coordinates
- bounding box summary
- count diagnostics

---

# 3. Action Planning

Runs the VLM planner using the annotated video and detected objects.

```bash
python -m ai_controller.models.seedo_controller.test \
  --stage action_planning \
  --video /test_dataset/pick_place/human_rgb_pick_place/task_00/traj000/converted/traj000-h264-30fps_modified.mp4 \
  --artifacts-dir /seedo_tests/action_planning \
  --expected-keyframes 25 51
```

Output:

- natural-language action plan
- structured action plan
- prompt
- raw OpenAI response
- JSON outputs

---

# 4. Complete SeeDo Controller

Runs the complete SeeDo pipeline through `SeeDoController`.

```bash
python -m ai_controller.models.seedo_controller.test \
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
```

The test validates:

- YAML configuration loading
- controller initialization
- complete SeeDo pipeline execution
- artifact generation
- ActionPlanningResult generation
- controller reset