# AI Controller Architecture

This package contains the ROS 2 AI-control layer used to run learned or programmatic manipulation policies on the UR5e + Robotiq 2F-85 setup.

The package supports multiple controller backends through a shared `AIController` interface. The current branch also integrates **SeeDo** as a complete demonstration-to-robot-action pipeline, including demonstration understanding, runtime RGB-D perception, semantic scene interpretation, CAP/LMP primitive generation, geometric motion translation, ROS 2 execution, rollout recording, artifact generation, and a layered test suite.

> Repository: `ciccio42/UR5e-2f-85`  
> Branch: `a.infante32`  
> Package root: `ai_controller/ai_controller`

---

## 1. Package overview

```text
ai_controller/ai_controller/
├── README.md
├── ai_controller_node.py
├── replicate_rollout.py
├── script_controller/
├── script_controller_node.py
├── utils/
│   ├── ai_controller.py
│   └── utils.py
└── models/
    ├── cod_controller/
    ├── openvla_controller/
    ├── tinyvla_controller/
    ├── requirements/
    └── seedo_controller/
        ├── __init__.py
        ├── README.md
        ├── seedo_controller.py
        ├── keyframe_selector.py
        ├── visual_prompter.py
        ├── action_planner.py
        ├── scene_perceiver.py
        ├── scene_interpreter.py
        ├── scene_interpreter_prompts.py
        ├── lmp_generator.py
        ├── lmp_prompts.py
        ├── motion_layer.py
        ├── config/
        │   └── seedo_controller.yaml
        ├── models/
        │   └── seedo/
        │       ├── bert-base-uncased/
        │       ├── groundingdino/
        │       ├── sam/
        │       └── sam2/
        └── tests/
            ├── TESTS.md
            ├── cli.py
            ├── common.py
            ├── inspect_seedo_rollout.py
            ├── test_keyframe.py
            ├── test_action_planning.py
            ├── test_scene_interpreter.py
            ├── test_lmp_generator.py
            ├── test_motion_layer.py
            └── ...
```

The main ROS 2 entry point is `ai_controller_node.py`. It selects the requested controller through the `ai_controller_target` ROS parameter and normalizes the controller output into executable robot actions.

The currently supported controller targets are:

```text
cod_controller
openvla_controller
tinyvla_controller
seedo_controller
```

---

## 2. Shared `AIController` abstraction

Every controller follows the common interface defined in:

```text
utils/ai_controller.py
```

The main methods are:

| Method | Purpose |
|---|---|
| `load_model(model_config)` | Parse configuration and initialize the controller/model. |
| `move_model_to_device(device)` | Move model components to the desired execution device when required. |
| `reset()` | Clear per-trajectory state. |
| `load_command(demo_path, task_id, **kwargs)` | Load the task command, demonstration, or conditioning data. |
| `pre_process(input_data)` | Normalize raw controller input. |
| `inference(input_data, t, save_path=None)` | Perform one controller inference step. |
| `post_process(output_data)` | Validate or normalize controller output. |

`ai_controller_node.py` owns the ROS-facing execution loop. Controllers do not directly send robot commands.

---

# 3. SeeDo integration

The SeeDo integration is intentionally split into two major phases:

1. **demonstration understanding**, executed once for a task;
2. **runtime scene understanding and execution planning**, executed from the robot workspace.

The key architectural choice is that the runtime scene is perceived once at `t=0`. The resulting scene and primitive plan are then kept fixed while the primitive sequence is translated into low-level robot actions.

---

## 3.1 Complete SeeDo pipeline

```text
                         DEMONSTRATION PHASE
                         ===================

Demonstration video
        │
        ▼
KeyframeSelector
        │
        ▼
FrameExtractorResult
        │
        ▼
VisualPrompter
  GroundingDINO
  SAM
  SAM2
        │
        ▼
VisualPromptingResult
        │
        ▼
ActionPlanner
        │
        ▼
ActionPlanningResult
        │
        │
        │       RUNTIME PHASE
        │       =============
        │
        │       RGB + Depth + CameraInfo
        │                │
        │                ▼
        │         ScenePerceiver
        │                │
        │                ▼
        │        RawSceneState
        │                │
        │                ▼
        │       SceneInterpreter
        │                │
        │                ▼
        │          SceneState
        │                │
        └──────────┬─────┘
                   ▼
              LMPGenerator
                  / CAP
                   │
                   ▼
             PrimitivePlan
                   │
                   ▼
             Motion Layer
                   │
                   ▼
       list of low-level actions
                   │
                   ▼
          AIControllerNode
                   │
                   ▼
       GoToPose + gripper command
```

The `SeeDoController` orchestrates these modules but keeps them individually testable.

---

# 4. Demonstration understanding

The demonstration side is executed by:

```python
SeeDoController.load_command(...)
```

It runs the modules below once per trajectory/task.

---

## 4.1 Keyframe Selector

File:

```text
models/seedo_controller/keyframe_selector.py
```

The selector processes the human demonstration video and identifies relevant manipulation keyframes from hand-motion dynamics.

Its main output is conceptually:

```text
FrameExtractorResult
├── keyframes
└── keyframe_images
```

The current configuration contains:

```yaml
keyframe_selector:
  gaussian_sigma: 5.0
  prominence: 0.8
  expected_keyframes: 2
  save_preview: false
```

---

## 4.2 Visual Prompter

File:

```text
models/seedo_controller/visual_prompter.py
```

The visual-prompting stage performs object discovery, detection, segmentation, and tracking on the demonstration.

The integrated stack uses:

```text
GroundingDINO
    │
    ▼
SAM
    │
    ▼
SAM2
```

The stage produces information such as:

```text
VisualPromptingResult
├── annotated_video_path
├── track_id_map
├── key_frame_coordinates
├── bounding_box_summary
└── count_diagnostics
```

`track_id_map` keeps persistent visual identities across the demonstration so that the action planner can reason about the manipulated object and destination.

The `objects` field in the YAML configuration can be left `null` to use automatic object recognition through the OpenAI-backed pipeline, or it can be explicitly provided.

---

## 4.3 Action Planner

File:

```text
models/seedo_controller/action_planner.py
```

The action planner interprets the demonstration artifacts and produces a structured task description.

Conceptually:

```text
VisualPromptingResult
        │
        ▼
ActionPlanner
        │
        ├── structured ActionStep sequence
        ├── ambiguity information
        └── natural-language task
        │
        ▼
ActionPlanningResult
```

An `ActionStep` describes information such as:

```text
pick keyframe
place keyframe
picked track ID
picked category
picked color
destination track ID
destination category
destination ordinal
relation
action description
```

Example natural-language output validated during integration:

```text
Pick the green cube and place it into the first storage bin from the left.
```

The resulting `ActionPlanningResult` is stored by `SeeDoController` and reused during runtime planning.

---

# 5. Runtime perception

Runtime perception is executed only at:

```text
inference(t=0)
```

The ROS node supplies:

```text
RGB image
depth image
CameraInfo
base_link -> table_0 transform
```

These inputs are passed to the runtime SeeDo pipeline.

---

## 5.1 Scene Perceiver

File:

```text
models/seedo_controller/scene_perceiver.py
```

The Scene Perceiver performs geometric workspace perception using:

```text
RGB
 │
 ├── GroundingDINO detection
 │
 └── SAM segmentation
          │
          ▼
        mask
          │
Depth + CameraInfo
          │
          ▼
3D camera coordinates
          │
base_link/table transform
          │
          ▼
3D robot-base coordinates
```

Its output is a raw geometric scene without task-specific semantic interpretation.

Conceptually:

```text
RawSceneObject
├── object_id
├── raw label
├── pixel_coordinates
├── position_camera
├── position_base
├── mask
└── confidence
```

and:

```text
RawSceneState
└── tuple[RawSceneObject, ...]
```

The configured detector classes are currently:

```yaml
detector_labels:
  - cube
  - storage bin
```

---

## 5.2 Scene Interpreter

File:

```text
models/seedo_controller/scene_interpreter.py
```

The Scene Interpreter converts raw geometric detections into semantically usable objects.

The separation is intentional:

```text
RawSceneState
    │
    │ geometric information only
    ▼
SceneInterpreter
    │
    │ semantic naming/reasoning
    ▼
SceneState
```

A final `SceneObject` contains:

```text
object_id
label
pixel_coordinates
position_camera
position_base
```

`object_id` is the stable identifier used later by the generated primitive plan and by the Motion Layer for target resolution.

The resulting `SceneState` is frozen for the remainder of the current execution.

---

# 6. CAP / LMP primitive generation

Files:

```text
models/seedo_controller/lmp_generator.py
models/seedo_controller/lmp_prompts.py
```

The LMP generator combines:

```text
ActionPlanningResult
+
SceneState
+
workspace bounds
```

and produces executable symbolic code through CAP.

The output is represented as:

```text
PrimitivePlan
├── steps: tuple[PrimitiveStep, ...]
└── source_code
```

Each `PrimitiveStep` contains:

```text
name
arguments
source_code
```

The primitive order is not hardcoded in `AIControllerNode`. It comes from the generated `PrimitivePlan`.

A validated pick-and-place plan is:

```python
reach("green cube")
approaching("green cube")
pick("green cube")
lift_up("green cube")
moving("first bin from the left")
placing("first bin from the left")
```

The currently supported primitive names in the Motion Layer are:

```text
reach
approaching
pick
lift_up
moving
placing
```

---

# 7. Motion Layer

File:

```text
models/seedo_controller/motion_layer.py
```

The Motion Layer is deliberately ROS-independent. It does not call services and does not publish robot commands.

Its responsibility is:

```text
PrimitiveStep
+
SceneState
+
current/planned end-effector pose
        │
        ▼
SeeDoMotionLayer
        │
        ▼
list[np.ndarray]
```

Every generated low-level action has the format:

```text
[x, y, z, qx, qy, qz, qw, gripper_position]
```

The Motion Layer is initialized from the real end-effector state when translating the first primitive:

```text
robot_state[eef_pos]
robot_state[eef_quat]
```

After initialization it uses **planned-pose chaining**: every primitive begins from the final planned pose of the previous primitive.

This is important because the symbolic timestep remains independent from the number of generated Cartesian waypoints:

```text
1 SeeDo inference timestep = 1 PrimitiveStep
1 PrimitiveStep            = N low-level actions
```

For the validated pick-and-place example:

```text
reach        -> 19 low-level actions
approaching  ->  7 low-level actions
pick         ->  1 low-level action
lift_up      ->  7 low-level actions
moving       -> 19 low-level actions
placing      ->  5 low-level actions
-----------------------------------
total        -> 58 low-level actions
```

---

## 7.1 Motion parameters

The current YAML configuration is:

```yaml
motion_layer:
  grasp_orientation:
    - 0.9994452044624775
    - 0.03161651380119412
    - 0.0021438049655468088
    - 0.010251021036213035

  min_step: 0.02
  reach_hover_height: 0.15
  approach_z_offset: 0.0
  release_height_offset: 0.10
  lift_height: 0.15

  gripper_open_position: 0.1
  gripper_closed_position: 0.8

  object_y_offset: -0.06
```

Meaning:

| Parameter | Role |
|---|---|
| `grasp_orientation` | Fixed quaternion used for the configured grasp orientation. |
| `min_step` | Approximate Cartesian interpolation step. |
| `reach_hover_height` | Height above the object used by `reach`. |
| `approach_z_offset` | Vertical offset used by `approaching`. |
| `release_height_offset` | Release height offset used by `placing`. |
| `lift_height` | Vertical lift after grasping. |
| `gripper_open_position` | Physical open-gripper command. |
| `gripper_closed_position` | Physical closed-gripper command. |
| `object_y_offset` | Y correction applied to object grasp targets. |

The current physical gripper convention for SeeDo is therefore:

```text
open   = 0.1
closed = 0.8
```

---

# 8. `SeeDoController` execution semantics

File:

```text
models/seedo_controller/seedo_controller.py
```

The controller is stateful over one trajectory.

Its main states include:

```text
action_plan
scene_state
primitive_plan
execution_status
execution_error
```

The execution flow is:

```text
reset()
   │
   ▼
load_command(...)
   │
   ├── KeyframeSelector
   ├── VisualPrompter
   └── ActionPlanner
   │
   ▼
ActionPlanningResult
   │
   ▼
inference(t=0)
   │
   ├── ScenePerceiver
   ├── SceneInterpreter
   └── LMPGenerator
   │
   ▼
PrimitivePlan ready
   │
   ▼
inference(t=1)
   │
   ├── initialize Motion Layer
   └── translate primitive #1
   │
   ▼
list[low-level actions]
   │
   ▼
inference(t=2)
   │
   └── translate primitive #2
   │
   ▼
...
   │
   ▼
inference(t=N)
   │
   └── translate primitive #N
   │
   ▼
inference(t=N+1)
   │
   ▼
execution_status = "completed"
```

Runtime perception is **not repeated between primitives**. The initial `SceneState` remains fixed.

---

# 9. ROS 2 integration

File:

```text
ai_controller_node.py
```

`AIControllerNode` integrates SeeDo into the same execution framework used by the other controllers.

SeeDo-specific ROS parameters currently include:

```text
seedo_rgb_topic
seedo_depth_topic
seedo_camera_info_topic
seedo_table_frame
seedo_artifacts_dir
```

Default runtime interfaces include:

```text
RGB:
  /zed_front/zed_node/rgb/color/rect/image

Depth:
  /zed_front/zed_node/depth/depth_registered

CameraInfo:
  /zed_front/zed_node/rgb/color/rect/camera_info

Joint state:
  /joint_states

EEF frame:
  tcp_link

Table frame:
  table_0
```

The node also maintains the existing synchronized camera interface:

```text
zed_front
zed_left
zed_right
zed_gripper
```

---

## 9.1 Robot-state capture

The generic robot-state dictionary recorded by the node contains:

```text
joint_pos
joint_vel
gripper_qpos
gripper_qvel
eef_pos
eef_quat
```

For the end-effector pose, TF is used to resolve:

```text
base_link -> tcp_link
```

This state is used both to initialize the SeeDo Motion Layer and to record rollout observations.

---

## 9.2 Execution layer

The Motion Layer only computes desired actions.

The ROS node is responsible for converting those actions into robot-side commands:

```text
[x, y, z, qx, qy, qz, qw, gripper]
               │
               ▼
        AIControllerNode
          │           │
          ▼           ▼
      GoToPose    GripperCommand
```

Development and validation should always begin with:

```text
move_robot=False
```

The current SeeDo test suite validates the planning and execution-control path without commanding physical motion.

> **Current integration note:** in the branch documented here, pose/gripper service clients are initialized in the legacy non-SeeDo setup path. Before enabling physical SeeDo motion with `move_robot=True`, verify or extend the SeeDo service-client initialization path and validate it with the real robot interfaces.

---

# 10. Rollout recording

`AIControllerNode` saves executed trajectories through the existing `Trajectory` infrastructure.

Rollouts are stored under:

```text
<save_rollout_path>/
└── <controller>/
    └── <task>/
        └── task_XX/
            ├── traj_NNN.pkl
            └── traj_NNN.json
```

For SeeDo, every low-level Motion Layer action is now preserved as a separate trajectory entry.

Therefore:

```text
1 PrimitiveStep
       │
       ├── action 1 -> Trajectory entry
       ├── action 2 -> Trajectory entry
       ├── ...
       └── action N -> Trajectory entry
```

For the validated six-primitive example:

```text
58 Motion Layer actions
        =
58 saved Trajectory entries
```

Each entry stores:

```text
obs
├── joint_pos
├── joint_vel
├── gripper_qpos
├── gripper_qvel
├── eef_pos
├── eef_quat
└── camera_front_image

action
done
reward
```

Only the final low-level action of the final primitive is marked:

```text
done=True
reward=1
```

The result metadata entered through `save_rollout()` is stored in the companion JSON file.

---

# 11. Artifact system

SeeDo can preserve the intermediate results of every stage.

When an explicit artifact directory is provided:

```text
<artifacts_dir>/
├── keyframe_selection/
├── visual_prompting/
├── action_planning/
├── scene_perceiver/
├── scene_interpreter/
├── lmp_generator/
└── motion_layer/
    └── motion_plan.json
```

When no explicit directory is supplied, `SeeDoController` can create temporary storage under `/tmp`.

`motion_plan.json` records the generated primitive/motion history, including the low-level actions and final planned state.

Artifacts make each stage independently inspectable and are heavily used by the test suite.

---

# 12. SeeDo configuration

Main configuration:

```text
models/seedo_controller/config/seedo_controller.yaml
```

Current structure:

```yaml
keyframe_selector:
  gaussian_sigma: 5.0
  prominence: 0.8
  expected_keyframes: 2
  save_preview: false

visual_prompter:
  grounding_config: /opt/checkpoints/seedo/groundingdino/GroundingDINO_SwinB.cfg.py
  grounding_checkpoint: /opt/checkpoints/seedo/groundingdino/groundingdino_swinb_cogcoor.pth
  bert_model: /opt/checkpoints/seedo/bert-base-uncased
  sam_checkpoint: /opt/checkpoints/seedo/sam/sam_vit_h_4b8939.pth
  sam2_checkpoint: /opt/checkpoints/seedo/sam2/sam2_hiera_large.pt
  objects: null

action_planner:
  model: gpt-4o-2024-08-06

scene_perceiver:
  camera_calibration_path: /home/ros2_ws/src/UR5e-2f-85/zed_camera/zed_camera_calibration/estimated_camera_positions.yaml
  camera_name: zed_front
  workspace_bottom_left: [0.0, 0.0]
  workspace_top_right: [1.0, 1.0]
  detector_labels:
    - cube
    - storage bin

lmp_generator:
  model: gpt-4o-2024-08-06

scene_interpreter:
  model: gpt-4o-2024-08-06

motion_layer:
  grasp_orientation:
    - 0.9994452044624775
    - 0.03161651380119412
    - 0.0021438049655468088
    - 0.010251021036213035
  min_step: 0.02
  reach_hover_height: 0.15
  approach_z_offset: 0.0
  release_height_offset: 0.10
  lift_height: 0.15
  gripper_open_position: 0.1
  gripper_closed_position: 0.8
  object_y_offset: -0.06
```

---

# 13. OpenAI configuration

The action-planning, scene-interpretation, and LMP/CAP stages use the OpenAI-backed models configured in the YAML file.

Set the API key in the environment before running these stages:

```bash
export OPENAI_API_KEY="YOUR_KEY"
```

For a persistent Bash configuration:

```bash
echo 'export OPENAI_API_KEY="YOUR_KEY"' >> ~/.bashrc
source ~/.bashrc
```

Do not commit API keys to Git.

---

# 14. SeeDo checkpoint setup

The large SeeDo model files are intentionally excluded from Git.

The expected local directory is:

```text
ai_controller/ai_controller/models/seedo_controller/models/seedo/
├── bert-base-uncased/
├── groundingdino/
├── sam/
└── sam2/
```

During the Docker build, this directory is copied to:

```text
/opt/checkpoints/seedo/
```

The current configuration expects:

```text
/opt/checkpoints/seedo/groundingdino/GroundingDINO_SwinB.cfg.py
/opt/checkpoints/seedo/groundingdino/groundingdino_swinb_cogcoor.pth
/opt/checkpoints/seedo/bert-base-uncased
/opt/checkpoints/seedo/sam/sam_vit_h_4b8939.pth
/opt/checkpoints/seedo/sam2/sam2_hiera_large.pt
```

---

## 14.1 Create the checkpoint directories

From:

```text
UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/models
```

run:

```bash
mkdir -p \
    seedo/groundingdino \
    seedo/bert-base-uncased \
    seedo/sam \
    seedo/sam2
```

---

## 14.2 Install Hugging Face CLI

```bash
python3 -m pip install -U huggingface_hub
```

Verify:

```bash
hf --help
```

---

## 14.3 Download GroundingDINO SwinB

```bash
hf download ShilongLiu/GroundingDINO \
    groundingdino_swinb_cogcoor.pth \
    GroundingDINO_SwinB.cfg.py \
    --local-dir seedo/groundingdino
```

Expected:

```text
seedo/groundingdino/
├── GroundingDINO_SwinB.cfg.py
└── groundingdino_swinb_cogcoor.pth
```

Do not replace this checkpoint with `groundingdino_swint_ogc.pth`: the current SeeDo configuration uses the SwinB model.

---

## 14.4 Download BERT

```bash
hf download google-bert/bert-base-uncased \
    --local-dir seedo/bert-base-uncased
```

Expected files include:

```text
seedo/bert-base-uncased/
├── config.json
├── model.safetensors
├── tokenizer_config.json
├── tokenizer.json
├── vocab.txt
└── ...
```

The complete directory is required because SeeDo loads BERT from the local directory path.

---

## 14.5 Download SAM ViT-H

```bash
wget \
    https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth \
    -O seedo/sam/sam_vit_h_4b8939.pth
```

Expected:

```text
seedo/sam/
└── sam_vit_h_4b8939.pth
```

---

## 14.6 Download SAM2 Hiera Large

```bash
wget \
    https://dl.fbaipublicfiles.com/segment_anything_2/072824/sam2_hiera_large.pt \
    -O seedo/sam2/sam2_hiera_large.pt
```

Expected:

```text
seedo/sam2/
└── sam2_hiera_large.pt
```

Do not replace it with `sam2.1_hiera_large.pt` unless the SeeDo configuration and SAM2 code version are intentionally updated together.

---

## 14.7 Complete checkpoint setup

```bash
mkdir -p \
    seedo/groundingdino \
    seedo/bert-base-uncased \
    seedo/sam \
    seedo/sam2

python3 -m pip install -U huggingface_hub

hf download ShilongLiu/GroundingDINO \
    groundingdino_swinb_cogcoor.pth \
    GroundingDINO_SwinB.cfg.py \
    --local-dir seedo/groundingdino

hf download google-bert/bert-base-uncased \
    --local-dir seedo/bert-base-uncased

wget \
    https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth \
    -O seedo/sam/sam_vit_h_4b8939.pth

wget \
    https://dl.fbaipublicfiles.com/segment_anything_2/072824/sam2_hiera_large.pt \
    -O seedo/sam2/sam2_hiera_large.pt
```

---

## 14.8 Verify checkpoint files

```bash
find seedo -maxdepth 2 -type f -printf '%p  %k KB\n' | sort
```

Important structure:

```text
seedo/
├── bert-base-uncased/
│   ├── config.json
│   ├── model.safetensors
│   ├── tokenizer_config.json
│   ├── tokenizer.json
│   ├── vocab.txt
│   └── ...
├── groundingdino/
│   ├── GroundingDINO_SwinB.cfg.py
│   └── groundingdino_swinb_cogcoor.pth
├── sam/
│   └── sam_vit_h_4b8939.pth
└── sam2/
    └── sam2_hiera_large.pt
```

These files are intentionally kept outside Git because of their size.

---

# 15. Testing

The complete SeeDo test documentation is available in:

```text
models/seedo_controller/tests/TESTS.md
```

The suite is organized from isolated modules to full ROS integration.

It covers:

```text
Keyframe Selection
        │
        ▼
Visual Prompting
        │
        ▼
Action Planning
        │
        ▼
Scene Perceiver
        │
        ▼
Scene Interpreter
        │
        ▼
LMP Generator
        │
        ▼
Motion Layer
        │
        ▼
Complete SeeDoController
        │
        ▼
AIControllerNode offline integration
        │
        ▼
AIControllerNode synthetic ROS integration
        │
        ▼
Interactive ROS dry-run
        │
        ▼
Saved rollout inspection
```

The strongest current dry-run validates the real node control loop while synthetic ROS publishers provide:

```text
RGB
Depth
CameraInfo
legacy synchronized camera topics
/joint_states
TF base_link -> table_0
TF base_link -> tcp_link
```

The test keeps:

```text
move_robot=False
```

while using the real:

```text
control_loop()
get_synced_images()
_capture_robot_state()
Trajectory
save_rollout()
```

The saved `.pkl` can be inspected with:

```bash
python -m ai_controller.models.seedo_controller.tests.inspect_seedo_rollout
```

For the validated example the rollout contains:

```text
Number of recorded steps: 58
Valid 8D actions: 58/58
First step done: False
First step reward: 0
Last step done: True
Last step reward: 1
```

---

# 16. Example SeeDo test configuration

A typical test invocation uses:

```bash
python -m ai_controller.models.seedo_controller.tests \
    --stage seedo_controller \
    --video /test_dataset/pick_place/human_rgb_pick_place/task_00/traj000/converted/traj000-h264-30fps_modified.mp4 \
    --scene-dir /scene_capture \
    --base-to-table-transform /scene_capture/base_to_table_transform.yaml \
    --model-config /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/config/seedo_controller.yaml \
    --artifacts-dir /seedo_tests/seedo_controller
```

For exact stage-specific commands and ROS integration tests, use `tests/TESTS.md`.

---

# 17. Design principles of the integration

The SeeDo integration follows several explicit design decisions:

- **Modular pipeline:** every SeeDo stage can be tested independently.
- **Frozen runtime scene:** perception and semantic interpretation are performed at `t=0`, not before every primitive.
- **CAP-generated execution sequence:** primitive order is generated from the task and scene rather than hardcoded in the controller.
- **Symbolic timestep semantics:** one SeeDo timestep means one symbolic primitive.
- **Motion Layer separation:** symbolic planning is separated from Cartesian motion generation.
- **ROS-independent geometry:** the Motion Layer does not execute ROS commands.
- **Planned-pose chaining:** low-level waypoint generation continues from the previously planned pose.
- **Explicit artifacts:** intermediate results are persisted for debugging and reproducibility.
- **Full rollout preservation:** every generated low-level action is recorded in the trajectory.
- **Backwards-compatible controller framework:** COD, OpenVLA, and TinyVLA retain their existing execution paths.
- **Hardware-safe development:** integration testing is performed with `move_robot=False` before physical execution.

---

# 18. Current validated scope

The integrated software path has been validated through:

```text
human demonstration
        │
        ▼
demonstration understanding
        │
        ▼
RGB-D runtime perception
        │
        ▼
semantic scene construction
        │
        ▼
CAP primitive generation
        │
        ▼
Motion Layer waypoint generation
        │
        ▼
AIControllerNode action iteration
        │
        ▼
Trajectory recording
```

The synthetic ROS tests additionally validate the ROS-facing data flow for camera data, robot joint state, and TF.

Physical robot execution should remain a separate validation stage and should first be tested conservatively with low-speed motion and `move_robot=False` inspection before enabling command transmission.
