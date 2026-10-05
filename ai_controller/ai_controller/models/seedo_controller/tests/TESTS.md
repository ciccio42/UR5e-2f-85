# SeeDo Controller Tests

This document describes the automated tests for the modular SeeDo controller pipeline.

The test suite is organized into two levels:

- **Unit tests**: validate each component in isolation using deterministic inputs, mocks, and fake dependencies.

- **Integration tests**: validate interactions between multiple pipeline stages and the ROS2 execution flow.

The **unit test suite and the generalized integration/end-to-end suite are complete and validated**. The integration section documents the execution order, persistent handoff artifacts, and the test that produces each artifact.

---

# Table of Contents

- [1. Test Layout](#1-test-layout)
- [2. Running the Unit Test Suite](#2-running-the-unit-test-suite)
  - [2.1 Run all unit tests](#21-run-all-unit-tests)
  - [2.2 Run all unit tests without suppressing warnings](#22-run-all-unit-tests-without-suppressing-warnings)
  - [2.3 Run a single unit test file](#23-run-a-single-unit-test-file)
  - [2.4 Run a single test function](#24-run-a-single-test-function)
- [3. Unit Test Summary](#3-unit-test-summary)
- [4. Utility Tests](#4-utility-tests)
  - [4.1 Spatial Utilities](#41-spatial-utilities)
  - [4.2 GroundingDINO Preprocessing Utilities](#42-groundingdino-preprocessing-utilities)
- [5. Demonstration Structured Scene Builder](#5-demonstration-structured-scene-builder)
- [6. Runtime Structured Scene Builder](#6-runtime-structured-scene-builder)
- [7. Structural Matcher](#7-structural-matcher)
- [8. Replicability Checker](#8-replicability-checker)
- [9. Action Planner](#9-action-planner)
- [10. Scene Interpreter](#10-scene-interpreter)
  - [Generalized mode](#generalized-mode)
  - [Prior-guided mode](#prior-guided-mode)
- [11. LMP Generator](#11-lmp-generator)
  - [LMPSceneWrapper](#lmpscenewrapper)
  - [LMPGenerator generalized mode](#lmpgenerator-generalized-mode)
  - [Prior-guided mode](#prior-guided-mode-1)
- [12. Motion Layer](#12-motion-layer)
- [13. Complete SeeDo Controller Unit Tests](#13-complete-seedo-controller-unit-tests)
  - [Configuration loading](#configuration-loading)
  - [Precomputed demonstration artifacts](#precomputed-demonstration-artifacts)
  - [Demonstration pipeline](#demonstration-pipeline)
  - [Runtime generalized pipeline](#runtime-generalized-pipeline)
  - [Runtime prior-guided pipeline](#runtime-prior-guided-pipeline)
  - [Motion execution](#motion-execution)
- [14. Current Unit-Test Result](#14-current-unit-test-result)
- [15. Integration Tests](#15-integration-tests)
  - [15.1 Canonical integration inputs](#151-canonical-integration-inputs)
  - [15.2 Artifact dependency summary](#152-artifact-dependency-summary)
  - [15.3 Keyframe Selector integration](#153-keyframe-selector-integration)
  - [15.4 Visual Prompter integration](#154-visual-prompter-integration)
  - [15.5 Demonstration Structured Scene Builder integration](#155-demonstration-structured-scene-builder-integration)
  - [15.6 Action Planner integration](#156-action-planner-integration)
  - [15.7 Scene Perceiver integration](#157-scene-perceiver-integration)
  - [15.8 Scene Interpreter integration](#158-scene-interpreter-integration)
  - [15.9 Runtime Structured Scene Builder integration](#159-runtime-structured-scene-builder-integration)
  - [15.10 Structural Matcher integration](#1510-structural-matcher-integration)
  - [15.11 Replicability Checker integration](#1511-replicability-checker-integration)
  - [15.12 LMP Generator integration](#1512-lmp-generator-integration)
  - [15.13 Motion Layer integration](#1513-motion-layer-integration)
  - [15.14 SeeDoController end-to-end integration](#1514-seedocontroller-end-to-end-integration)
  - [15.15 SeeDoController prior-guided end-to-end integration](#1515-seedocontroller-prior-guided-end-to-end-integration)
  - [15.16 AIControllerNode offline end-to-end](#1515-aicontrollernode-offline-end-to-end)
  - [15.17 AIControllerNode ROS end-to-end](#1516-aicontrollernode-ros-end-to-end)
  - [15.18 AIControllerNode interactive ROS dry-run](#1517-aicontrollernode-interactive-ros-dry-run)
  - [15.19 Recommended execution order](#1518-recommended-execution-order)
  - [15.20 Current validated integration status](#1519-current-validated-integration-status)

---

# 1. Test Layout

The test directory is organized as follows:

```text
ai_controller/ai_controller/models/seedo_controller/tests/
├── __init__.py
├── common.py
├── __main__.py
├── cli.py
├── TEST.md
├── inspect_seedo_rollout.py
├── unit/
│   ├── __init__.py
│   ├── test_spatial_utils.py
│   ├── test_grounding_dino_utils.py
│   ├── test_demo_structured_scene_builder.py
│   ├── test_runtime_structured_scene_builder.py
│   ├── test_structural_matcher.py
│   ├── test_replicability_checker.py
│   ├── test_action_planner.py
│   ├── test_scene_interpreter.py
│   ├── test_lmp_generator.py
│   ├── test_motion_layer.py
│   └── test_seedo_controller.py
└── integration/
    ├── __init__.py
    ├── test_keyframe.py
    ├── test_visual_prompting.py
    ├── test_demo_structured_scene_builder.py
    ├── test_action_planning.py
    ├── test_scene_perceiver.py
    ├── test_scene_interpreter.py
    ├── test_runtime_structured_scene_builder.py
    ├── test_structural_matcher.py
    ├── test_replicability_checker.py
    ├── test_lmp_generator.py
    ├── test_motion_layer.py
    ├── test_seedo_controller.py
    ├── test_seedo_controller_prior_guided.py
    ├── test_seedo_node_offline.py
    ├── test_seedo_node_ros.py
    └── test_seedo_node_interactive.py
```

All unit tests are written with `pytest`.

The tests are executed inside the SeeDo Docker environment from:

```text
/home/ros2_ws
```

No additional `PYTHONPATH` configuration is required.

---

# 2. Running the Unit Test Suite

## 2.1 Run all unit tests

Recommended command:

```bash
PYTHONWARNINGS=ignore python3 -m pytest /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/tests/unit -v
```

This command:

- runs every unit test;

- prints every test name and its result;

- suppresses warnings generated by external dependencies during import and collection.

The current expected result is:

```text
389 passed
```

## 2.2 Run all unit tests without suppressing warnings

```bash
python3 -m pytest /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/tests/unit -v
```

Warnings currently originate from external dependencies such as protobuf, Matplotlib, PyTorch, timm, and GroundingDINO. They are not failures in the SeeDo unit test suite.

## 2.3 Run a single unit test file

General form:

```bash
PYTHONWARNINGS=ignore python3 -m pytest /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/tests/unit/<test_file>.py -v
```

Example:

```bash
PYTHONWARNINGS=ignore python3 -m pytest /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/tests/unit/test_structural_matcher.py -v
```

## 2.4 Run a single test function

General form:

```bash
PYTHONWARNINGS=ignore python3 -m pytest /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/tests/unit/<test_file>.py::<test_name> -v
```

Example:

```bash
PYTHONWARNINGS=ignore python3 -m pytest /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/tests/unit/test_scene_interpreter.py::test_prior_guided_vlm_returns_valid_mapping -v
```

---

# 3. Unit Test Summary

Current validated unit suite:

| Test file | Tests | Main component |
|---|---:|---|
| `test_spatial_utils.py` | 35 | Spatial relation utilities |
| `test_grounding_dino_utils.py` | 26 | GroundingDINO crop and bounding-box remapping utilities |
| `test_demo_structured_scene_builder.py` | 28 | Demonstration structured-scene construction |
| `test_runtime_structured_scene_builder.py` | 24 | Runtime structured-scene construction |
| `test_structural_matcher.py` | 16 | Structural graph matching |
| `test_replicability_checker.py` | 23 | Runtime task replicability |
| `test_action_planner.py` | 37 | Action-planner wrapper |
| `test_scene_interpreter.py` | 36 | Runtime semantic interpretation |
| `test_lmp_generator.py` | 51 | CAP/LMP generation and scene wrapper |
| `test_motion_layer.py` | 49 | Symbolic-to-low-level motion translation |
| `test_seedo_controller.py` | 64 | Complete controller orchestration |
| **Total** | **389** | |

---

# 4. Utility Tests

## 4.1 Spatial Utilities

File:

```text
unit/test_spatial_utils.py
```

Run:

```bash
PYTHONWARNINGS=ignore python3 -m pytest /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/tests/unit/test_spatial_utils.py -v
```

Expected result:

```text
35 passed
```

The spatial utilities define the qualitative relations used by the structured-scene representation.

The relation between two objects is derived from the vector joining their 2D centers in image coordinates.

Supported direction modes are:

```text
4 directions:
left
right
above
below
```

and:

```text
8 directions:
left
right
above
below
upper-left
upper-right
lower-left
lower-right
```

The tests validate:

- 4-direction relation quantization;

- 8-direction relation quantization;

- diagonal collapse in 4-direction mode;

- invalid direction modes;

- coincident object centers;

- pairwise relation generation;

- the expected `N(N-1)` directed relation count;

- empty and single-object scenes;

- duplicate object IDs;

- relation direction consistency.

The structured-scene representation uses object centers only. Bounding boxes are not used for structural matching.

---


## 4.2 GroundingDINO Preprocessing Utilities

File:

```text
unit/test_grounding_dino_utils.py
```

Run:

```bash
PYTHONWARNINGS=ignore python3 -m pytest /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/tests/unit/test_grounding_dino_utils.py -v
```

Expected result:

```text
26 passed
```

This test file validates the shared GroundingDINO preprocessing utilities used by both demonstration and runtime perception.

The tested utilities are:

```text
crop_image_for_grounding_dino()
remap_grounding_dino_boxes_to_full_frame()
```

`crop_image_for_grounding_dino()` validates and applies a configurable image crop before GroundingDINO inference while preserving the complete image for the rest of the perception pipeline.

The tests cover:

- top, bottom, left, and right crop margins;

- zero-crop behavior;

- invalid and negative margins;

- crops that would produce an empty image;

- preservation of contiguous NumPy image data.

`remap_grounding_dino_boxes_to_full_frame()` converts normalized GroundingDINO `cxcywh` bounding boxes from cropped-image coordinates back to normalized full-frame coordinates before SAM and downstream processing use them.

For the current canonical configuration:

```text
full image = 672 x 376

crop:
top    = 80 px
bottom = 0 px
left   = 0 px
right  = 0 px

GroundingDINO input = 672 x 296
```

Only the vertical reference system changes. Therefore:

```text
cx     -> unchanged
cy     -> remapped to full-frame coordinates
width  -> unchanged
height -> remapped to full-frame coordinates
```

The implementation also supports future cropping on all four image sides.

The tests additionally validate:

- preservation of the original GroundingDINO tensor;

- exact identity behavior when no crop is applied;

- empty `(0, 4)` detection tensors;

- rejection of invalid bounding-box shapes;

- rejection of incomplete crop metadata;

- rejection of non-positive image dimensions.

The same shared crop/remapping logic is used by both the demonstration `VisualPrompter` and the runtime `ScenePerceiver`.

---

# 5. Demonstration Structured Scene Builder

File:

```text
unit/test_demo_structured_scene_builder.py
```

Run:

```bash
PYTHONWARNINGS=ignore python3 -m pytest /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/tests/unit/test_demo_structured_scene_builder.py -v
```

Expected result:

```text
28 passed
```

`DemoStructuredSceneBuilder` converts the tracked demonstration objects into a `StructuredScene`.

For each relevant destination object, the builder stores:

```text
object_id
category
center
```

and computes the pairwise qualitative spatial relations.

The demonstration center is obtained from the tracked object's initial SAM-derived center.

The tests validate:

- valid 4- and 8-direction configurations;

- invalid direction modes;

- empty `track_id_map`;

- missing object categories;

- filtering of non-place objects;

- place-category normalization;

- missing and invalid object centers;

- conversion of center coordinates to floats;

- pairwise spatial relation generation;

- coincident centers;

- artifact generation.

The generated artifact is:

```text
demo_structured_scene.json
```

---

# 6. Runtime Structured Scene Builder

File:

```text
unit/test_runtime_structured_scene_builder.py
```

Run:

```bash
PYTHONWARNINGS=ignore python3 -m pytest /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/tests/unit/test_runtime_structured_scene_builder.py -v
```

Expected result:

```text
24 passed
```

`RuntimeStructuredSceneBuilder` converts a runtime `SceneState` into the same structural representation used for the demonstration.

Each destination object contributes:

```text
object_id
category
pixel center
```

The tests validate:

- supported direction modes;

- invalid direction modes;

- empty runtime scenes;

- missing categories;

- filtering of non-place objects;

- category normalization;

- preservation of runtime object IDs;

- conversion of pixel centers to floats;

- 4-direction relations;

- 8-direction relations;

- coincident centers;

- artifact generation.

The generated artifact is:

```text
runtime_structured_scene.json
```

---

# 7. Structural Matcher

File:

```text
unit/test_structural_matcher.py
```

Run:

```bash
PYTHONWARNINGS=ignore python3 -m pytest /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/tests/unit/test_structural_matcher.py -v
```

Expected result:

```text
16 passed
```

`StructuralMatcher` compares the demonstration and runtime structured scenes.

Matching is based on the graph of qualitative spatial relations rather than semantic category equality.

The matcher:

1\. checks that both scenes use the same direction mode;

2\. checks compatible object counts;

3\. validates the relation graphs;

4\. enumerates candidate demonstration-to-runtime permutations;

5\. keeps every mapping that preserves all qualitative relations.

A mapping is therefore structural:

```text
demo object ID -> runtime object ID
```

The tests validate:

- direction-mode mismatch;

- object-count mismatch;

- unique mappings;

- preservation of demonstration object ordering;

- category-independent matching;

- scenes with no valid mapping;

- symmetric scenes with multiple valid mappings;

- 4-direction structural matching;

- incomplete relation graphs;

- unknown relation subjects;

- unknown relation references;

- self-relations;

- duplicate relation pairs;

- artifact generation.

The generated artifact is:

```text
structural_matching_result.json
```

A structural mapping does not need to be globally unique for task execution. Ambiguity is evaluated later with respect to the specific demonstrated destination.

---

# 8. Replicability Checker

File:

```text
unit/test_replicability_checker.py
```

Run:

```bash
PYTHONWARNINGS=ignore python3 -m pytest /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/tests/unit/test_replicability_checker.py -v
```

Expected result:

```text
23 passed
```

`ReplicabilityChecker` verifies whether the demonstrated action can be reproduced in the runtime scene.

It combines:

```text
ActionPlanningResult
\+
StructuralMatchingResult
\+
SceneState
```

and produces:

```text
ReplicabilityResult
```

For each demonstrated action step, it resolves:

```text
runtime_pick_object_id
runtime_place_object_id
```

The pick object is matched using the authoritative detector label.

The place object is resolved through the valid structural mappings.

If several globally valid structural mappings exist, the action is still considered unambiguous when all valid mappings send the demonstrated destination to the same runtime object.

The tests validate:

- valid pick-and-place replication;

- task type represented as enum or string;

- missing or unsupported task types;

- empty action plans;

- missing structural mappings;

- missing pick objects;

- ambiguous pick objects;

- detector-label normalization;

- missing destination mappings;

- multiple valid mappings resolving to the same destination;

- multiple valid mappings resolving to different destinations;

- missing runtime destinations;

- destination-category compatibility;

- pick-and-place destination categories;

- nut-assembly destination categories;

- pick and destination resolving to the same object;

- multi-step action plans;

- failure behavior;

- successful resolved targets;

- artifact generation.

---

# 9. Action Planner

File:

```text
unit/test_action_planner.py
```

Run:

```bash
PYTHONWARNINGS=ignore python3 -m pytest /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/tests/unit/test_action_planner.py -v
```

Expected result:

```text
37 passed
```

`ActionPlanner` is the validation and orchestration wrapper around the VLM action-planning function.

The unit tests do not perform real OpenAI calls. The underlying `generate_action_plan()` function is replaced with a deterministic fake.

The tests validate:

- `generalized` perception mode normalization;

- `prior_guided` perception mode normalization;

- invalid perception modes;

- demonstration bin-order normalization;

- invalid bin-order values;

- model configuration;

- missing demonstration video;

- empty demonstration video;

- requirement for exactly two keyframes;

- chronological pick/place keyframe ordering;

- keyframe conversion to integers;

- empty `track_id_map`;

- empty key-frame coordinates;

- artifact-directory creation;

- path normalization;

- forwarding of all validated arguments to the VLM layer;

- propagation of VLM errors;

- rejection of a missing VLM result.

---

# 10. Scene Interpreter

File:

```text
unit/test_scene_interpreter.py
```

Run:

```bash
PYTHONWARNINGS=ignore python3 -m pytest /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/tests/unit/test_scene_interpreter.py -v
```

Expected result:

```text
36 passed
```

`SceneInterpreter` converts the perceived runtime scene into a semantic `SceneState`.

The behavior depends on `perception_mode`.

## Generalized mode

The generalized pipeline does not use a VLM for runtime semantic renaming.

Instead:

```text
RawSceneObject.object_id
        ->
SceneObject.object_id
```

is preserved directly.

Category and attribute metadata are also propagated to `SceneObject`.

Spatial ordering is intentionally not encoded in semantic names.

For example, runtime destinations are not renamed as:

```text
first storage bin
second storage bin
```

Structural relationships are handled separately by the structured-scene and structural-matching pipeline.

## Prior-guided mode

The original semantic interpretation behavior is preserved.

The VLM can assign semantic names, while detector labels remain authoritative for non-bin objects.

Storage bins may still receive semantic names such as ordinal descriptions.

The tests validate:

- mode normalization;

- invalid modes;

- empty raw scenes;

- missing overlay image;

- missing overlay files;

- raw-ID preservation in generalized mode;

- metadata preservation in generalized mode;

- confirmation that generalized mode bypasses VLM semantic naming;

- missing categories;

- invalid attributes;

- duplicate runtime object IDs;

- generalized artifacts;

- prior-guided semantic naming;

- prior-guided removal of generalized-only metadata;

- incomplete semantic mappings;

- prior-guided artifacts;

- missing OpenAI API key;

- mocked valid VLM responses;

- empty VLM responses;

- invalid VLM object mappings;

- unauthorized detector-label changes;

- storage-bin semantic renaming;

- duplicate semantic names.

No real OpenAI request is performed by the unit suite.

---

# 11. LMP Generator

File:

```text
unit/test_lmp_generator.py
```

Run:

```bash
PYTHONWARNINGS=ignore python3 -m pytest /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/tests/unit/test_lmp_generator.py -v
```

Expected result:

```text
51 passed
```

This test file validates both:

```text
LMPSceneWrapper
LMPGenerator
```

No real CAP/OpenAI generation is executed during the unit tests. `_setup_lmp()` is replaced with a fake LMP that records the received instruction and produces deterministic symbolic primitives.

## LMPSceneWrapper

The wrapper exposes the frozen runtime `SceneState` to Code-as-Policies and records symbolic robot primitive calls.

The tests validate:

- workspace bounds;

- object lookup;

- object visibility;

- 2D and 3D object positions;

- normalized workspace coordinates;

- corner positions;

- side positions;

- corner names;

- side names;

- batched object positions;

- empty object lists;

- invalid input types;

- symbolic primitive recording.

Supported symbolic primitives currently include:

```text
reach
approaching
pick
lift_up
moving
placing
aligning
inserting
```

## LMPGenerator generalized mode

Generalized execution requires a successful `ReplicabilityResult`.

The final CAP instruction is reconstructed using runtime object IDs.

Pick-and-place example:

```text
Pick 'runtime_pick' and place it in 'runtime_place'.
```

Nut-assembly example:

```text
Pick 'runtime_pick' and assemble it onto 'runtime_place'.
```

For multi-step plans, resolved instructions are joined with:

```text
and then
```

The tests validate:

- missing `ReplicabilityResult`;

- non-replicable tasks;

- incorrect resolved-target count;

- missing action-step indices;

- runtime pick/place target resolution;

- pick-and-place instruction construction;

- nut-assembly instruction construction;

- multi-action instruction construction.

## Prior-guided mode

Prior-guided execution preserves the existing behavior and sends:

```text
action_plan.natural_language_plan
```

directly to CAP.

The tests additionally validate:

- invalid action-plan status;

- unsupported task types;

- empty natural-language plans;

- empty generated primitive plans;

- `PrimitivePlan` construction;

- generated CAP source-code storage;

- artifact generation.

Generated artifacts are:

```text
generated_program.py
primitive_plan.json
```

---

# 12. Motion Layer

File:

```text
unit/test_motion_layer.py
```

Run:

```bash
PYTHONWARNINGS=ignore python3 -m pytest /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/tests/unit/test_motion_layer.py -v
```

Expected result:

```text
49 passed
```

`SeeDoMotionLayer` converts symbolic `PrimitiveStep` objects into low-level actions.

Each generated low-level action has the format:

```text
[x, y, z, qx, qy, qz, qw, gripper_position]
```

The unit tests validate the symbolic-to-geometric translation without commanding a physical robot.

The tested primitives are:

```text
reach
approaching
pick
lift_up
moving
placing
aligning
inserting
```

The tests validate:

- grasp-orientation shape;

- initial state;

- reset position and orientation validation;

- reset behavior;

- requirement to initialize the Motion Layer before translation;

- `PrimitiveStep` validation;

- `SceneState` validation;

- unsupported primitives;

- primitive-name normalization;

- reach hover target;

- approach target;

- gripper closing during `pick`;

- lift height;

- destination XY motion;

- place height;

- gripper opening during `placing`;

- nut-assembly alignment offset;

- nut-assembly insertion offset;

- gripper opening after insertion;

- linear waypoint conversion;

- planned-pose update;

- behavior with no generated waypoints;

- 8D action format;

- target resolution;

- case-insensitive runtime IDs;

- `target`, `object`, and `destination` aliases;

- missing targets;

- ambiguous runtime object IDs;

- invalid target arguments;

- motion history;

- initial motion artifact;

- updated motion artifact.

The Motion Layer artifact is:

```text
motion_plan.json
```

and contains:

```text
primitives
total_actions
final_planned_state
```

The Motion Layer itself only performs motion translation. It does not execute ROS services or robot commands.

---

# 13. Complete SeeDo Controller Unit Tests

File:

```text
unit/test_seedo_controller.py
```

Run:

```bash
PYTHONWARNINGS=ignore python3 -m pytest /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/tests/unit/test_seedo_controller.py -v
```

Expected result:

```text
64 passed
```

This file validates the orchestration performed by `SeeDoController` while replacing expensive or external pipeline stages with deterministic fakes.

It covers:

```text
load_model()
load_command()
inference()
pre_process()
post_process()
reset()
```

as well as the precomputed execution path.

## Configuration loading

The tests validate:

- missing configuration files;

- perception-mode validation;

- structured-scene direction validation;

- camera-pose noise-level validation;

- propagation of `perception_mode`;

- propagation of structured-scene direction mode;

- workspace configuration;

- construction of all SeeDo pipeline components.

## Precomputed demonstration artifacts

The controller can load:

```text
precomputed ActionPlanningResult
precomputed demonstration StructuredScene
```

Generalized execution requires both.

Prior-guided execution requires only the precomputed action plan.

The tests validate:

- missing files;

- empty files;

- generalized ordinal removal;

- prior-guided ordinal preservation;

- missing prior-guided ordinal;

- direction-mode validation;

- direction-mode/configuration mismatch;

- structured-scene object loading;

- structured-scene relation loading.

## Demonstration pipeline

The current demonstration flows are intentionally different.

Generalized:

```text
Demonstration video
        |
        v
KeyframeSelector
        |
        v
VisualPrompter
        |
        v
DemoStructuredSceneBuilder
        |
        v
ActionPlanner
        |
        v
ActionPlanningResult
```

Prior-guided:

```text
Demonstration video
        |
        v
KeyframeSelector
        |
        v
VisualPrompter
        |
        v
ActionPlanner
        |
        v
ActionPlanningResult
```

`DemoStructuredSceneBuilder` is therefore not executed in prior-guided mode.

## Runtime generalized pipeline

At `inference(t=0)`:

```text
Runtime RGB-D scene
        |
        v
ScenePerceiver
        |
        v
SceneInterpreter
        |
        v
RuntimeStructuredSceneBuilder
        |
        v
StructuralMatcher
        |
        v
ReplicabilityChecker
        |
        v
LMPGenerator
        |
        v
PrimitivePlan
```

The tests validate:

- required runtime inputs;

- scene perception;

- scene interpretation;

- runtime structured-scene generation;

- structural matching;

- replicability checking;

- rejection of non-replicable tasks;

- propagation of the resolved `ReplicabilityResult` into the LMP generator;

- failure-state handling.

## Runtime prior-guided pipeline

At `inference(t=0)`:

```text
Runtime RGB-D scene
        |
        v
ScenePerceiver
        |
        v
SceneInterpreter
        |
        v
LMPGenerator
        |
        v
PrimitivePlan
```

Prior-guided mode does not execute:

```text
RuntimeStructuredSceneBuilder
StructuralMatcher
ReplicabilityChecker
```

and passes:

```text
replicability_result=None
```

to the LMP generator.

## Motion execution

After planning:

```text
inference(t=1)
    -> PrimitiveStep #1
inference(t=2)
    -> PrimitiveStep #2
...
inference(t=N)
    -> PrimitiveStep #N
inference(t=N+1)
    -> completed
```

The first motion timestep initializes the Motion Layer from:

```text
robot_state[eef_pos]
robot_state[eef_quat]
```

The tests validate:

- missing primitive plans;

- missing `SceneState`;

- missing robot state;

- missing end-effector position;

- missing end-effector orientation;

- Motion Layer reset at `t=1`;

- no repeated reset at later timesteps;

- primitive-by-primitive translation;

- completion after the last primitive;

- propagation of Motion Layer errors;

- controller execution status;

- controller execution errors.

---

# 14. Current Unit-Test Result

The current full unit suite contains:

```text
389 tests
```

All unit tests currently pass:

```text
389 passed
```

Breakdown:

```text
Spatial utilities                     35
GroundingDINO preprocessing utilities  26
Demo structured-scene builder         28
Runtime structured-scene builder      24
Structural matcher                    16
Replicability checker                 23
Action planner                        37
Scene interpreter                     36
LMP generator                         51
Motion layer                          49
SeeDo controller                      64
----------------------------------------
TOTAL                                389
```

The full suite should be executed before moving to integration testing:

```bash
PYTHONWARNINGS=ignore python3 -m pytest /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/tests/unit -v
```

---

# 15. Integration Tests

The integration suite validates the real generalized SeeDo pipeline progressively, from individual expensive components to the complete ROS2 execution flow.

Unlike the unit suite, integration tests may use real OpenAI requests, GroundingDINO/SAM/SAM2, CUDA, the canonical demonstration video, the canonical RGB-D runtime scene, ROS2 publishers/subscribers and TF, and manual input in the final interactive dry-run.

The modular integration tests use persisted handoff artifacts where this helps isolate one stage. The final controller and node tests **do not reuse those handoffs as inputs**: they rerun the complete pipeline from the original video and runtime scene.

All commands below are intended to be executed from:

```text
/home/ros2_ws
```

No additional `PYTHONPATH` is required.

## 15.1 Canonical integration inputs

Demonstration video:

```text
/test_dataset/pick_place/human_rgb_pick_place/task_00/traj000/converted/traj000-h264-30fps.mp4
```

Runtime scene:

```text
/scene_capture/without_distractors/scene_1_no_distractors
```

Transform:

```text
/scene_capture/without_distractors/scene_1_no_distractors/base_to_table_transform.yaml
```

Controller configuration:

```text
/home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/config/seedo_controller.yaml
```

Canonical keyframes:

```text
pick  = 20
place = 35
```

Canonical generalized runtime resolution:

```text
pick  = green_block_0
place = storage_bin_0
```

Expected primitive sequence:

```text
reach       -> green_block_0
approaching -> green_block_0
pick        -> green_block_0
lift_up     -> green_block_0
moving      -> storage_bin_0
placing     -> storage_bin_0
```

Tests that perform real OpenAI calls require the API configuration to be available in the environment. GroundingDINO/SAM/SAM2 tests require the configured checkpoints under `/opt/checkpoints/seedo`.

---

## 15.2 Artifact dependency summary

| Integration test | Required upstream artifact(s) | Produced handoff artifact(s) |
|---|---|---|
| `test_keyframe.py` | None | Keyframe previews; validates frames `20` and `35` |
| `test_visual_prompting.py` | No persisted artifact; uses keyframes `20 35` validated by `test_keyframe.py` | `/seedo_tests/visual_prompting/visual_prompting_result.json` plus visual/debug artifacts |
| `test_demo_structured_scene_builder.py` | `visual_prompting_result.json` from `test_visual_prompting.py` | `/seedo_tests/demo_structured_scene_builder/demo_structured_scene.json` |
| `test_action_planning.py` | `visual_prompting_result.json` from `test_visual_prompting.py` | `/seedo_tests/action_planning/action_plan.json` |
| `test_scene_perceiver.py` | None; reads the canonical runtime scene directly | `/seedo_tests/scene_perceiver/raw_scene_state.json`, `raw_scene_overlay.png`, `camera_pose_noise.json` |
| `test_scene_interpreter.py` | `raw_scene_state.json` + `raw_scene_overlay.png` from `test_scene_perceiver.py` | `/seedo_tests/scene_interpreter/scene_state.json`, `scene_interpretation.json` |
| `test_runtime_structured_scene_builder.py` | `scene_state.json` from `test_scene_interpreter.py` | `/seedo_tests/runtime_structured_scene_builder/runtime_structured_scene.json` |
| `test_structural_matcher.py` | Demo structured scene + runtime structured scene | `/seedo_tests/structural_matcher/structural_matching_result.json` |
| `test_replicability_checker.py` | `action_plan.json` + `structural_matching_result.json` + `scene_state.json` | `/seedo_tests/replicability_checker/replicability_result.json` |
| `test_lmp_generator.py` | `action_plan.json` + `scene_state.json` + `replicability_result.json` | `/seedo_tests/lmp_generator/generated_program.py`, `primitive_plan.json` |
| `test_motion_layer.py` | `primitive_plan.json` + `scene_state.json` | `/seedo_tests/motion_layer/motion_plan.json` |
| `test_seedo_controller.py` | None | Regenerates the complete controller artifact tree |
| `test_seedo_controller_prior_guided.py` | None | Regenerates the complete prior-guided controller artifact tree |
| `test_seedo_node_offline.py` | None | Regenerates the complete node/controller artifact tree |
| `test_seedo_node_ros.py` | None | Regenerates the complete node/controller artifact tree through ROS scene messages |
| `test_seedo_node_interactive.py` | None | Regenerates the full artifact tree and saves a rollout `.pkl` + outcome `.json` |

Recommended modular dependency chain:

```text
test_keyframe.py
        |
        v
keyframes 20, 35
        |
        v
test_visual_prompting.py
        |
        +------------------------------+
        |                              |
        v                              v
visual_prompting_result.json     visual_prompting_result.json
        |                              |
        v                              v
DemoStructuredSceneBuilder       ActionPlanner
        |                              |
        v                              v
demo_structured_scene.json       action_plan.json

ScenePerceiver
        |
        v
raw_scene_state.json + raw_scene_overlay.png
        |
        v
SceneInterpreter
        |
        v
scene_state.json
        |
        v
RuntimeStructuredSceneBuilder
        |
        v
runtime_structured_scene.json

demo_structured_scene.json
        +
runtime_structured_scene.json
        |
        v
StructuralMatcher
        |
        v
structural_matching_result.json

action_plan.json
        +
structural_matching_result.json
        +
scene_state.json
        |
        v
ReplicabilityChecker
        |
        v
replicability_result.json

action_plan.json
        +
scene_state.json
        +
replicability_result.json
        |
        v
LMPGenerator
        |
        v
primitive_plan.json

primitive_plan.json
        +
scene_state.json
        |
        v
MotionLayer
        |
        v
motion_plan.json
```

---

## 15.3 Keyframe Selector integration

File:

```text
integration/test_keyframe.py
```

Run:

```bash
PYTHONWARNINGS=ignore python3 -m ai_controller.models.seedo_controller.tests \
  --stage keyframe \
  --video /test_dataset/pick_place/human_rgb_pick_place/task_00/traj000/converted/traj000-h264-30fps.mp4 \
  --expected-keyframes 20 35 \
  --artifacts-dir /seedo_tests/keyframe
```

Required upstream artifacts:

```text
None
```

Expected result:

```text
pick_frame  = 20
place_frame = 35
```

The selected indices are passed explicitly to the VisualPrompter test. No persisted keyframe result file is required as a downstream handoff.

---

## 15.4 Visual Prompter integration

File:

```text
integration/test_visual_prompting.py
```

Run:

```bash
PYTHONWARNINGS=ignore python3 -m ai_controller.models.seedo_controller.tests \
  --stage visual_prompting \
  --video /test_dataset/pick_place/human_rgb_pick_place/task_00/traj000/converted/traj000-h264-30fps.mp4 \
  --expected-keyframes 20 35 \
  --artifacts-dir /seedo_tests/visual_prompting
```

Required upstream artifacts:

```text
None
```

Logical upstream dependency:

```text
test_keyframe.py -> validated keyframes 20 and 35
```

The test performs real generalized VLM object discovery, GroundingDINO, SAM and SAM2.

Expected canonical scene content:

```text
4 colored blocks
4 storage bins
8 tracked objects total
```

Essential handoff:

```text
/seedo_tests/visual_prompting/visual_prompting_result.json
```

It contains the annotated-video path, `track_id_map`, key-frame coordinates, bounding-box summary and count diagnostics.

The annotated tracked video referenced by the JSON must remain available for downstream demonstration tests.

---

## 15.5 Demonstration Structured Scene Builder integration

File:

```text
integration/test_demo_structured_scene_builder.py
```

Run:

```bash
PYTHONWARNINGS=ignore python3 -m ai_controller.models.seedo_controller.tests \
  --stage demo_structured_scene_builder \
  --visual-prompting-result /seedo_tests/visual_prompting/visual_prompting_result.json \
  --artifacts-dir /seedo_tests/demo_structured_scene_builder
```

Required upstream artifact:

```text
/seedo_tests/visual_prompting/visual_prompting_result.json
```

Produced by:

```text
integration/test_visual_prompting.py
```

Expected structured destinations:

```text
0 -> bin -> (203.0, 298.0)
1 -> bin -> (305.0, 296.0)
2 -> bin -> (408.0, 293.0)
3 -> bin -> (511.0, 290.0)
```

Expected graph:

```text
directions = 8
objects    = 4
relations  = 12
```

Produced handoff:

```text
/seedo_tests/demo_structured_scene_builder/demo_structured_scene.json
```

---

## 15.6 Action Planner integration

File:

```text
integration/test_action_planning.py
```

Run:

```bash
PYTHONWARNINGS=ignore python3 -m ai_controller.models.seedo_controller.tests \
  --stage action_planning \
  --visual-prompting-result /seedo_tests/visual_prompting/visual_prompting_result.json \
  --artifacts-dir /seedo_tests/action_planning
```

Required upstream artifact:

```text
/seedo_tests/visual_prompting/visual_prompting_result.json
```

Produced by:

```text
integration/test_visual_prompting.py
```

Expected generalized result:

```text
task_type            = pick_and_place
pick_keyframe         = 20
place_keyframe        = 35
picked_track_id       = 5
picked_detector_label = green block
destination_track_id  = 0
destination_category  = bin
relation              = in
```

Produced handoff:

```text
/seedo_tests/action_planning/action_plan.json
```

This artifact is later required by the ReplicabilityChecker and LMPGenerator integration tests.

---

## 15.7 Scene Perceiver integration

File:

```text
integration/test_scene_perceiver.py
```

Run:

```bash
PYTHONWARNINGS=ignore python3 -m ai_controller.models.seedo_controller.tests \
  --stage scene_perceiver \
  --scene-dir /scene_capture/without_distractors/scene_1_no_distractors \
  --base-to-table-transform /scene_capture/without_distractors/scene_1_no_distractors/base_to_table_transform.yaml \
  --model-config /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/config/seedo_controller.yaml \
  --artifacts-dir /seedo_tests/scene_perceiver
```

Required upstream artifacts:

```text
None
```

The test reads the canonical runtime `rgb.png`, `depth.npy`, `camera_info.yaml` and transform directly and runs the real generalized runtime perception stage.

Expected runtime IDs:

```text
storage_bin_0
storage_bin_1
storage_bin_2
storage_bin_3
green_block_0
yellow_block_0
blue_block_0
red_block_0
```

Produced handoffs:

```text
/seedo_tests/scene_perceiver/raw_scene_state.json
/seedo_tests/scene_perceiver/raw_scene_overlay.png
```

Additional artifact:

```text
/seedo_tests/scene_perceiver/camera_pose_noise.json
```

---

## 15.8 Scene Interpreter integration

File:

```text
integration/test_scene_interpreter.py
```

Run:

```bash
PYTHONWARNINGS=ignore python3 -m ai_controller.models.seedo_controller.tests \
  --stage scene_interpreter \
  --model-config /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/config/seedo_controller.yaml \
  --artifacts-dir /seedo_tests/scene_interpreter
```

Required upstream artifacts:

```text
/seedo_tests/scene_perceiver/raw_scene_state.json
/seedo_tests/scene_perceiver/raw_scene_overlay.png
```

Produced by:

```text
integration/test_scene_perceiver.py
```

The test reconstructs the `ScenePerceptionResult` from persisted handoffs and runs only the real `SceneInterpreter`.

Generalized mode must preserve neutral runtime IDs exactly and does not perform VLM semantic renaming.

Produced artifacts:

```text
/seedo_tests/scene_interpreter/scene_interpretation.json
/seedo_tests/scene_interpreter/scene_state.json
```

Essential downstream handoff:

```text
/seedo_tests/scene_interpreter/scene_state.json
```

---

## 15.9 Runtime Structured Scene Builder integration

File:

```text
integration/test_runtime_structured_scene_builder.py
```

Run:

```bash
PYTHONWARNINGS=ignore python3 -m ai_controller.models.seedo_controller.tests \
  --stage runtime_structured_scene_builder \
  --artifacts-dir /seedo_tests/runtime_structured_scene_builder
```

Required upstream artifact:

```text
/seedo_tests/scene_interpreter/scene_state.json
```

Produced by:

```text
integration/test_scene_interpreter.py
```

Expected destinations:

```text
storage_bin_0 -> bin -> (194.0, 316.0)
storage_bin_1 -> bin -> (304.0, 314.0)
storage_bin_2 -> bin -> (412.0, 311.0)
storage_bin_3 -> bin -> (521.0, 309.0)
```

Expected graph:

```text
directions = 8
objects    = 4
relations  = 12
```

Produced handoff:

```text
/seedo_tests/runtime_structured_scene_builder/runtime_structured_scene.json
```

---

## 15.10 Structural Matcher integration

File:

```text
integration/test_structural_matcher.py
```

Run:

```bash
PYTHONWARNINGS=ignore python3 -m ai_controller.models.seedo_controller.tests \
  --stage structural_matcher \
  --artifacts-dir /seedo_tests/structural_matcher
```

Required upstream artifacts:

```text
/seedo_tests/demo_structured_scene_builder/demo_structured_scene.json
/seedo_tests/runtime_structured_scene_builder/runtime_structured_scene.json
```

Produced by:

```text
integration/test_demo_structured_scene_builder.py
integration/test_runtime_structured_scene_builder.py
```

The matcher uses the qualitative relation graph only.

Expected unique mapping:

```text
0 -> storage_bin_0
1 -> storage_bin_1
2 -> storage_bin_2
3 -> storage_bin_3
```

Produced handoff:

```text
/seedo_tests/structural_matcher/structural_matching_result.json
```

---

## 15.11 Replicability Checker integration

File:

```text
integration/test_replicability_checker.py
```

Run:

```bash
PYTHONWARNINGS=ignore python3 -m ai_controller.models.seedo_controller.tests \
  --stage replicability_checker \
  --artifacts-dir /seedo_tests/replicability_checker
```

Required upstream artifacts:

```text
/seedo_tests/action_planning/action_plan.json
/seedo_tests/structural_matcher/structural_matching_result.json
/seedo_tests/scene_interpreter/scene_state.json
```

Produced by:

```text
integration/test_action_planning.py
integration/test_structural_matcher.py
integration/test_scene_interpreter.py
```

Expected result:

```text
replicable = True

step 0:
    runtime_pick_object_id  = green_block_0
    runtime_place_object_id = storage_bin_0

failure_reasons = ()
```

Produced handoff:

```text
/seedo_tests/replicability_checker/replicability_result.json
```

---

## 15.12 LMP Generator integration

File:

```text
integration/test_lmp_generator.py
```

Run:

```bash
PYTHONWARNINGS=ignore python3 -m ai_controller.models.seedo_controller.tests \
  --stage lmp_generator \
  --model-config /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/config/seedo_controller.yaml \
  --artifacts-dir /seedo_tests/lmp_generator
```

Required upstream artifacts:

```text
/seedo_tests/action_planning/action_plan.json
/seedo_tests/scene_interpreter/scene_state.json
/seedo_tests/replicability_checker/replicability_result.json
```

Produced by:

```text
integration/test_action_planning.py
integration/test_scene_interpreter.py
integration/test_replicability_checker.py
```

The generalized CAP instruction must use resolved runtime IDs:

```text
Pick 'green_block_0' and place it in 'storage_bin_0'.
```

Expected primitive sequence:

```text
reach       -> green_block_0
approaching -> green_block_0
pick        -> green_block_0
lift_up     -> green_block_0
moving      -> storage_bin_0
placing     -> storage_bin_0
```

Produced artifacts:

```text
/seedo_tests/lmp_generator/generated_program.py
/seedo_tests/lmp_generator/primitive_plan.json
```

---

## 15.13 Motion Layer integration

File:

```text
integration/test_motion_layer.py
```

Run:

```bash
PYTHONWARNINGS=ignore python3 -m \
  ai_controller.models.seedo_controller.tests.integration.test_motion_layer
```

Required upstream artifacts:

```text
/seedo_tests/lmp_generator/primitive_plan.json
/seedo_tests/scene_interpreter/scene_state.json
```

Produced by:

```text
integration/test_lmp_generator.py
integration/test_scene_interpreter.py
```

The test validates conversion of the six real symbolic primitives into 8D low-level actions:

```text
[x, y, z, qx, qy, qz, qw, gripper_position]
```

Produced artifact:

```text
/seedo_tests/motion_layer/motion_plan.json
```

The exact total number of interpolated actions is not treated as a global end-to-end invariant, because small upstream geometric changes can change the number of interpolation steps. The test instead validates primitive semantics and consistency with `motion_plan.json`.

---

## 15.14 SeeDoController end-to-end integration

File:

```text
integration/test_seedo_controller.py
```

Run:

```bash
PYTHONWARNINGS=ignore python3 -m ai_controller.models.seedo_controller.tests \
  --stage seedo_controller \
  --video /test_dataset/pick_place/human_rgb_pick_place/task_00/traj000/converted/traj000-h264-30fps.mp4 \
  --scene-dir /scene_capture/without_distractors/scene_1_no_distractors \
  --base-to-table-transform /scene_capture/without_distractors/scene_1_no_distractors/base_to_table_transform.yaml \
  --model-config /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/config/seedo_controller.yaml \
  --artifacts-dir /seedo_tests/seedo_controller
```

Required upstream integration artifacts:

```text
None
```

The test intentionally reruns the full controller pipeline from the original video and runtime scene:

```text
KeyframeSelector
-> VisualPrompter
-> DemoStructuredSceneBuilder
-> ActionPlanner
-> ScenePerceiver
-> SceneInterpreter
-> RuntimeStructuredSceneBuilder
-> StructuralMatcher
-> ReplicabilityChecker
-> LMPGenerator
-> MotionLayer
```

It verifies the structural mapping, runtime target resolution, six primitive steps, low-level actions, final completion state and controller reset.

It regenerates its own artifact tree under:

```text
/seedo_tests/seedo_controller
```

---

## 15.15 SeeDoController prior-guided end-to-end integration

File:

```text
integration/test_seedo_controller_prior_guided.py
```

Run:

```bash
PYTHONWARNINGS=ignore python3 -m ai_controller.models.seedo_controller.tests --stage seedo_controller_prior_guided --video /test_dataset/pick_place/human_rgb_pick_place/task_00/traj000/converted/traj000-h264-30fps.mp4 --scene-dir /scene_capture/without_distractors/scene_1_no_distractors --base-to-table-transform /scene_capture/without_distractors/scene_1_no_distractors/base_to_table_transform.yaml --model-config /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/config/seedo_controller.yaml --artifacts-dir /seedo_tests/seedo_controller_prior_guided
```

Required upstream integration artifacts:

```text
None
```

This test independently reruns the complete legacy prior-guided SeeDo pipeline using the canonical demonstration video and runtime scene.

The test derives a temporary prior-guided configuration from the normal SeeDo model configuration, without requiring the repository configuration to be changed manually.

The validated demonstration path is:

```text
KeyframeSelector
        |
        v
VisualPrompter
[prior_guided]
        |
        v
ActionPlanner
[prior_guided]
        |
        v
ActionPlanningResult
```

The validated runtime path is:

```text
ScenePerceiver
[prior_guided]
        |
        v
SceneInterpreter
[prior_guided]
        |
        v
LMPGenerator
[prior_guided]
        |
        v
PrimitivePlan
        |
        v
MotionLayer
        |
        v
Low-level robot actions
```

The test explicitly verifies that the generalized structural pipeline is not used:

```text
demo_structured_scene        = None
runtime_structured_scene     = None
structural_matching_result   = None
replicability_result         = None
```

The prior-guided ActionPlanner must preserve the legacy destination representation, including a non-null:

```text
destination_ordinal_from_left
```

and a non-empty natural-language action plan.

The runtime SceneInterpreter must preserve the legacy semantic naming scheme used by CAP, including semantic object names such as coloured cubes and ordinal storage-bin descriptions.

The LMPGenerator must execute the prior-guided path based on:

```text
action_plan.natural_language_plan
```

rather than the generalized runtime targets produced by structural matching.

The generated primitive targets must correspond to valid objects in the runtime `SceneState`, and every primitive must be successfully translated by the Motion Layer into finite low-level robot actions.

The test finally verifies that the complete primitive plan is consumed and that the controller reaches:

```text
execution_status = completed
```

This integration test exists specifically to demonstrate that the original prior-guided execution path remains functional after the introduction of the generalized structural-matching pipeline.

---

## 15.16 AIControllerNode offline end-to-end

File:

```text
integration/test_seedo_node_offline.py
```

Run:

```bash
PYTHONWARNINGS=ignore python3 -m \
  ai_controller.models.seedo_controller.tests.integration.test_seedo_node_offline \
  --video /test_dataset/pick_place/human_rgb_pick_place/task_00/traj000/converted/traj000-h264-30fps.mp4 \
  --scene-dir /scene_capture/without_distractors/scene_1_no_distractors \
  --base-to-table-transform /scene_capture/without_distractors/scene_1_no_distractors/base_to_table_transform.yaml \
  --model-config /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/config/seedo_controller.yaml \
  --artifacts-dir /seedo_tests/seedo_node_offline \
  --task-id 1
```

Required upstream integration artifacts:

```text
None
```

This test executes the real `AIControllerNode.control_loop()` while replacing only the physical/ROS environment required to run offline.

It validates:

- the complete generalized pipeline;
- all primitive timesteps and final completion inference;
- one trajectory entry per low-level action;
- four-camera RGB/depth rollout structure;
- object bounding boxes;
- dataset statuses;
- final `done=True`, `reward=1`, `status=end`;
- `timings.json`;
- consistency with `motion_plan.json`.

---

## 15.17 AIControllerNode ROS end-to-end

File:

```text
integration/test_seedo_node_ros.py
```

Run:

```bash
PYTHONWARNINGS=ignore python3 -m \
  ai_controller.models.seedo_controller.tests.integration.test_seedo_node_ros \
  --video /test_dataset/pick_place/human_rgb_pick_place/task_00/traj000/converted/traj000-h264-30fps.mp4 \
  --scene-dir /scene_capture/without_distractors/scene_1_no_distractors \
  --base-to-table-transform /scene_capture/without_distractors/scene_1_no_distractors/base_to_table_transform.yaml \
  --model-config /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/config/seedo_controller.yaml \
  --artifacts-dir /seedo_tests/seedo_node_ros \
  --task-id 1
```

Required upstream integration artifacts:

```text
None
```

This test sends the canonical runtime scene through real ROS2 publisher/subscriber paths.

The test publisher provides:

```text
4 x RGB
4 x depth
front CameraInfo
base_link -> table_0 TF
```

The real `AIControllerNode` callbacks must populate the runtime input and recording-camera state before the real `control_loop()` executes the full generalized pipeline.

---

## 15.18 AIControllerNode interactive ROS dry-run

File:

```text
integration/test_seedo_node_interactive.py
```

Run:

```bash
PYTHONWARNINGS=ignore python3 -m \
  ai_controller.models.seedo_controller.tests.integration.test_seedo_node_interactive \
  --video /test_dataset/pick_place/human_rgb_pick_place/task_00/traj000/converted/traj000-h264-30fps.mp4 \
  --scene-dir /scene_capture/without_distractors/scene_1_no_distractors \
  --base-to-table-transform /scene_capture/without_distractors/scene_1_no_distractors/base_to_table_transform.yaml \
  --model-config /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/config/seedo_controller.yaml \
  --artifacts-dir /seedo_tests/seedo_node_interactive \
  --rollouts-dir /seedo_tests/seedo_node_interactive/rollouts
```

Required upstream integration artifacts:

```text
None
```

This is the closest validated test to normal execution without commanding the physical robot.

The simulated ROS runtime publishes:

```text
4 x RGB
4 x depth
CameraInfo
/joint_states
base_link -> table_0 TF
base_link -> EEF TF
```

The test forces:

```text
move_robot=False
seedo_execute_gripper=False
```

and validates the real `_capture_robot_state()` path.

Validated interactive success sequence:

```text
Write the current trajectory count to the console: 0
Press Enter to start the control loop.
Enter task ID: 0
```

At the outcome prompts:

```text
Did the robot successfully reach the target object? 1
Did the robot successfully pick the target object?  1
Did the robot successfully place the target object? 1
```

When the control loop asks again:

```text
Press Enter to start the control loop...
```

press `Ctrl+C`.

The test then verifies exactly one new rollout pair:

```text
/seedo_tests/seedo_node_interactive/rollouts/
└── seedo_controller/
    └── pick_place/
        └── task_00/
            ├── traj_000.pkl
            └── traj_000.json
```

It also validates the outcome metadata schema, complete generalized artifact tree, `timings.json`, resolved runtime targets, and final controller state.

---

## 15.19 Recommended execution order

Run modular integration tests in this order:

```text
1.  test_keyframe.py
2.  test_visual_prompting.py
3.  test_demo_structured_scene_builder.py
4.  test_action_planning.py
5.  test_scene_perceiver.py
6.  test_scene_interpreter.py
7.  test_runtime_structured_scene_builder.py
8.  test_structural_matcher.py
9.  test_replicability_checker.py
10. test_lmp_generator.py
11. test_motion_layer.py
```

This guarantees that every persisted handoff exists before a downstream modular test is run.

Then validate complete orchestration in increasing order of realism:

```text
12. test_seedo_controller.py
13. test_seedo_node_offline.py
14. test_seedo_node_ros.py
15. test_seedo_node_interactive.py
```

The four end-to-end tests above are intentionally independent of the modular handoff chain.

---

## 15.20 Current validated integration status

```text
KeyframeSelector                                 PASS
VisualPrompter                                   PASS
DemoStructuredSceneBuilder                       PASS
ActionPlanner                                    PASS
ScenePerceiver                                   PASS
SceneInterpreter                                 PASS
RuntimeStructuredSceneBuilder                    PASS
StructuralMatcher                                PASS
ReplicabilityChecker                             PASS
LMPGenerator                                     PASS
MotionLayer                                      PASS
SeeDoController end-to-end                       PASS
AIControllerNode offline end-to-end              PASS
AIControllerNode ROS end-to-end                  PASS
AIControllerNode interactive ROS dry-run         PASS
```

Together with the unit suite:

```text
Unit tests: 389 passed
Integration/end-to-end stages: all validated PASS
```

The unit suite remains independent of real OpenAI calls, ROS2 runtime data, physical robot motion, external runtime scenes and manual interaction. Those behaviors are intentionally covered only by the integration/end-to-end layers.
