# GraspMolmo + GraspGen-X — Quick Setup

This guide describes the current project setup used to run **GraspMolmo** and **GraspGen-X** as separate services and use them together from the SeeDo grasp-planning pipeline.

The GraspGen-X integration is intentionally isolated from the SeeDo Python environment:

- GraspMolmo runs as an HTTP service on port `8780`;
- GraspGen-X runs as a ZMQ service on port `8782`;
- SeeDo calls GraspGen-X through the project `GraspGenXRemoteClient`;
- the bridge worker is executed with the dedicated GraspGen-X Python environment;
- the active project backend is named `graspgenx` — there is no separate `graspgenx_v2` backend.

---

## Expected structure

Repository inside the container:

```text
/home/ros2_ws/src/UR5e-2f-85
```

GraspMolmo environment and source:

```text
/opt/graspmolmo_venv
/opt/GraspMolmo
```

GraspGen-X environment and source:

```text
/opt/graspgenx_venv
/opt/GraspGenX
```

Model/checkpoint data:

```text
/home/ros2_ws/src/UR5e-2f-85/.runtime/huggingface

/opt/graspgenx_checkpoints
/opt/graspgenx_gripper_descriptions
```

Relevant project modules:

```text
ai_controller/ai_controller/utils/grasping/
├── graspmolmo/
│   ├── client.py
│   ├── runtime.py
│   └── server.py
│
├── graspgenx/
│   ├── artifacts.py
│   ├── client.py
│   ├── geometry_selector.py
│   ├── geometry_utils.py
│   ├── inference_core.py
│   ├── planner.py
│   ├── runtime_worker.py
│   └── worker.py
│
├── grasp_planner_factory.py
└── replay_graspgenx_runtime_case.py
```

The project also requires the SAM checkpoint used by the GraspGen-X inference core:

```text
/opt/checkpoints/seedo/sam/sam_vit_h_4b8939.pth
```

---

## Runtime overview

The current pipeline is:

```text
RGB-D eye-in-hand frame
        |
        v
GraspMolmo
semantic grasp point
        |
        v
SAM
target mask
        |
        v
target/background point clouds
        |
        v
GraspGen-X
candidate 6-DoF grasps
        |
        v
GraspGenXGeometrySelector
table + semantic + symmetry evaluation
        |
        v
selected grasp pose
        |
        v
TCP grasp pose
        |
        v
Motion Layer / MoveIt
```

The two external services are:

```text
GraspMolmo  -> 127.0.0.1:8780
GraspGen-X  -> 127.0.0.1:8782
```

---

# 1. GraspMolmo

## Environment installation

The project installation script prepares:

```text
/opt/graspmolmo_venv
/opt/GraspMolmo
```

From the repository:

```bash
cd /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/utils/grasping/graspmolmo/scripts
bash install_graspmolmo.sh
```

Official repository:

```text
https://github.com/abhaybd/GraspMolmo.git
```

Commit used by the project setup:

```text
73990b5c5121ac5ab5fd402453bfeeefd270d153
```

The runtime loads:

```text
allenai/GraspMolmo
```

## Hugging Face cache

The project uses:

```text
/home/ros2_ws/src/UR5e-2f-85/.runtime/huggingface
```

Before starting the service:

```bash
export HF_HOME=/home/ros2_ws/src/UR5e-2f-85/.runtime/huggingface
```

## Start the GraspMolmo server

```bash
cd /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/utils/grasping/graspmolmo

HF_HOME=/home/ros2_ws/src/UR5e-2f-85/.runtime/huggingface \
PYTHONPATH=/home/ros2_ws/src/UR5e-2f-85/ai_controller \
PYTHONNOUSERSITE=1 \
/opt/graspmolmo_venv/bin/python server.py \
    --config config/graspmolmo_config.yaml
```

Server:

```text
127.0.0.1:8780
```

Health check:

```bash
curl -s --max-time 5 http://127.0.0.1:8780/health
```

---

# 2. GraspGen-X

## Source repository

Official repository:

```text
https://github.com/NVlabs/GraspGenX.git
```

The project expects the source tree at:

```text
/opt/GraspGenX
```

For a fresh installation:

```bash
git clone https://github.com/NVlabs/GraspGenX.git /opt/GraspGenX
```

No exact GraspGen-X git revision was recorded in the current project notes.  
For reproducibility, record the revision after installation with:

```bash
cd /opt/GraspGenX
git rev-parse HEAD
```

## Python environment

The dedicated environment is:

```text
/opt/graspgenx_venv
```
uv installation:

```bash
cd /opt/GraspGenX

curl -LsSf https://astral.sh/uv/install.sh | sh
```
The setup used by the project is:

```bash
cd /opt/GraspGenX

export PATH="/root/.local/bin:$PATH"

export UV_PROJECT_ENVIRONMENT=/opt/graspgenx_venv

unset PYTHONPATH
export PYTHONNOUSERSITE=1

sed -i \
  's/scene-synthesizer\[recommend\]/scene-synthesizer/' \
  pyproject.toml

uv pip install \
  --python /opt/graspgenx_venv/bin/python \
  "git+https://github.com/drufat/triangle.git@595b43eb6682992a0b1012b9671bf860c0e6ae56"

UV_LINK_MODE=copy \
uv sync \
  --extra serve \
  --inexact \
  --no-install-package triangle
```

Quick import check:

```bash
cd /opt/GraspGenX

/opt/graspgenx_venv/bin/python -c \
'import triangle; print("triangle:", triangle.__version__)'

PYTHONNOUSERSITE=1 \
/opt/graspgenx_venv/bin/python -c \
"import graspgenx; print('GraspGen-X dependencies ready')"
```

The SeeDo environment itself does **not** need the GraspGen-X Python dependencies.  
`GraspGenXRemoteClient` starts the project worker using:

```text
/opt/graspgenx_venv/bin/python
```

with working directory:

```text
/opt/GraspGenX
```

---

## Download the GraspGen-X checkpoints

The project uses:

```text
/opt/graspgenx_checkpoints
```

The checkpoint repository used during setup is:

```text
https://huggingface.co/adithyamurali/GraspGenXModel
```

Fresh download:

```bash
apt update
apt install -y git-lfs

git lfs install

git clone --depth 1 \
    https://huggingface.co/adithyamurali/GraspGenXModel \
    /opt/graspgenx_checkpoints
```

The server is started with the checkpoint root:

```text
/opt/graspgenx_checkpoints/release
```

This directory must contain the GraspGen-X release checkpoint structure expected by the official server.

---

## Download the gripper descriptions

The project keeps GraspGen-X gripper descriptions separately at:

```text
/opt/graspgenx_gripper_descriptions
```

The dataset repository used during setup is:

```text
https://huggingface.co/datasets/adithyamurali/gripper_descriptions
```

Fresh download:

```bash
git lfs install

git clone --depth 1 \
    https://huggingface.co/datasets/adithyamurali/gripper_descriptions \
    /opt/graspgenx_gripper_descriptions
```

The active gripper is:

```text
robotiq_2f_85
```

The server uses the assets directory:

```text
/opt/graspgenx_gripper_descriptions/gripper_descriptions/assets
```

---

## Start the GraspGen-X server

Run GraspGen-X in its dedicated environment:

```bash
cd /opt/GraspGenX

env -u PYTHONPATH \
PYTHONNOUSERSITE=1 \
GRASPGENX_CHECKPOINT_DIR=/opt/graspgenx_checkpoints \
GRASPGENX_GRIPPER_CFG_DIR=/opt/graspgenx_gripper_descriptions \
/opt/graspgenx_venv/bin/python \
    client-server/graspgenx_server.py \
    --config /opt/graspgenx_checkpoints/release \
    --assets_dir /opt/graspgenx_gripper_descriptions/gripper_descriptions/assets \
    --default_gripper robotiq_2f_85 \
    --host 0.0.0.0 \
    --port 8782
```

Server:

```text
127.0.0.1:8782
```

The server process loads the GraspGen-X model.  
The project-side worker communicates with it through the official ZMQ client.

---

## GraspGen-X health check through the project bridge

With the GraspGen-X server already running:

```bash
cd /home/ros2_ws/src/UR5e-2f-85

PYTHONPATH=/home/ros2_ws/src/UR5e-2f-85/ai_controller:$PYTHONPATH \
python3 - <<'PY'
from ai_controller.utils.grasping.graspgenx.client import (
    GraspGenXRemoteClient,
)

client = GraspGenXRemoteClient(
    host="127.0.0.1",
    port=8782,
    gripper_name="robotiq_2f_85",
)

print("GraspGen-X health:", client.health())
PY
```

Expected result:

```text
GraspGen-X health: True
```

---

# 3. SeeDo configuration

The active backend is:

```yaml
grasp_planner:
  enabled: true
  backend: graspgenx

  seed: 42
  semantic_radius_px: 50.0

  graspgenx_server_host: 127.0.0.1
  graspgenx_server_port: 8782
  graspgenx_gripper_name: robotiq_2f_85

  graspgenx_num_grasps: 200
  graspgenx_grasp_threshold: -1.0
  graspgenx_topk_num_grasps: -1

  graspgenx_worker_python: /opt/graspgenx_venv/bin/python

  # Margin applied by the current point-cloud geometry selector.
  graspgenx_table_margin_mm: 0.0

  # Positive offset moves the GraspGen-X grasp deeper along
  # the candidate local +Z approach direction.
  graspgenx_depth_offset_m: 0.005
```

The following legacy configuration is **not part of the current setup**:

```text
graspgenx_v2
graspgenx_v2_*
v2_unsafe_enable_robot_commands
```

There is only one public GraspGen-X backend:

```text
graspgenx
```

---

# 4. Current GraspGen-X selection logic

Candidate generation and candidate selection are separate.

The inference core performs:

```text
GraspMolmo semantic point
        ->
SAM segmentation
        ->
target/background point clouds
        ->
GraspGen-X candidate generation
```

The current `GraspGenXGeometrySelector` then evaluates the candidates using the observed RGB-D geometry.

Main checks include:

```text
- fit the tabletop from the observed background point cloud;
- reject candidates whose conservative Robotiq envelope violates the table margin;
- require observed target points inside the intended gripper sweep region;
- compare the candidate contact region with the GraspMolmo semantic point;
- evaluate both parallel-jaw symmetries:
    original
    pi_z
- use confidence, semantic consistency and current TCP orientation for ranking;
- optionally prefer a better-centered candidate when the baseline grasp is
  significantly off-center.
```

The selected GraspGen-X gripper pose is converted to the TCP grasp pose used by the Motion Layer.

The active geometric selection path does **not** use the old mesh-based `collision_filter` worker action.

---

# 5. Unit tests

The intentionally small GraspGen-X unit-test suite is:

```text
graspgenx/test_graspgenx_client.py
graspgenx/test_graspgenx_planner.py
```

Run:

```bash
cd /home/ros2_ws/src/UR5e-2f-85

PYTHONPATH=/home/ros2_ws/src/UR5e-2f-85/ai_controller:$PYTHONPATH \
pytest -q \
  ai_controller/ai_controller/utils/grasping/graspgenx/test_graspgenx_client.py \
  ai_controller/ai_controller/utils/grasping/graspgenx/test_graspgenx_planner.py
```

Current expected result:

```text
5 passed
```

These unit tests do not require a real GraspMolmo or GraspGen-X server.

---

# 6. Offline replay of a real GraspGen-X case

The project keeps a manual replay tool:

```text
ai_controller/ai_controller/utils/grasping/replay_graspgenx_runtime_case.py
```

This is intentionally separate from the normal pytest suite.

## Required saved data

A real-case directory must contain:

```text
input_rgb.png
input_depth.npy
```

Normally it also contains:

```text
grasp_plan.json
```

The source rollout `.pkl` is used to recover the TCP pose and must provide:

```text
eef_pos
eef_quat
```

at the selected step.

The replay script can automatically match `input_rgb.png` against the eye-in-hand images in the rollout if `--step` is omitted.

---

## Replay using the semantic point saved during the real run

Example:

```bash
cd /home/ros2_ws/src/UR5e-2f-85

PYTHONPATH=/home/ros2_ws/src/UR5e-2f-85/ai_controller:$PYTHONPATH \
python3 ai_controller/ai_controller/utils/grasping/replay_graspgenx_runtime_case.py \
    --case-dir /test_real/test_task00_23 \
    --pkl /path/to/source_rollout.pkl \
    --instruction "Pick up the cube." \
    --server-host 127.0.0.1 \
    --server-port 8782 \
    --gripper-name robotiq_2f_85 \
    --num-grasps 200 \
    --grasp-threshold -1.0 \
    --semantic-radius-px 50 \
    --seed 42
```

When `grasp_plan.json` contains `semantic_point_px`, the replay reuses that point.

In this mode:

```text
GraspMolmo server: NOT required
GraspGen-X server: required
```

This mode is useful for regression testing changes downstream of GraspMolmo while keeping the semantic target identical to the real run.

---

## Replay with live GraspMolmo

Add:

```text
--live-molmo
```

Example:

```bash
cd /home/ros2_ws/src/UR5e-2f-85

PYTHONPATH=/home/ros2_ws/src/UR5e-2f-85/ai_controller:$PYTHONPATH \
python3 ai_controller/ai_controller/utils/grasping/replay_graspgenx_runtime_case.py \
    --case-dir /test_real/test_task00_23 \
    --pkl /path/to/source_rollout.pkl \
    --instruction "Pick up the cube." \
    --server-host 127.0.0.1 \
    --server-port 8782 \
    --gripper-name robotiq_2f_85 \
    --semantic-radius-px 50 \
    --seed 42 \
    --live-molmo
```

In this mode both services must be running:

```text
GraspMolmo -> 127.0.0.1:8780
GraspGen-X -> 127.0.0.1:8782
```

---

## Default table transform used by the replay

The current replay defaults correspond to the production `table_0` transform:

```text
base_link -> table_0

x       = 0.000 m
y       = 0.612 m
z       = -0.120 m
yaw     = 180 deg
```

They can be overridden with:

```text
--table-x
--table-y
--table-z
--table-yaw-deg
```

The old `table_surface_offset_m` parameter is no longer part of the current GraspGen-X pipeline.

---

# 7. Full GraspMolmo + GraspGen-X runtime

For the real SeeDo pipeline, start both servers.

Terminal 1:

```text
GraspMolmo
127.0.0.1:8780
```

Terminal 2:

```text
GraspGen-X
127.0.0.1:8782
```

Then configure:

```yaml
grasp_planner:
  enabled: true
  backend: graspgenx
```

The SeeDo process remains in its normal environment.

GraspGen-X-specific Python dependencies remain isolated in:

```text
/opt/graspgenx_venv
```

The project bridge invokes the GraspGen-X worker in that environment and communicates with the ZMQ server.

---

# 8. Saved diagnostics

Depending on the execution path and artifact directory, GraspGen-X runs can save data such as:

```text
input_rgb.png
input_depth.npy
target_mask.png
selected_grasp_overlay.png
graspgenx_diagnostics.npz
grasp_plan.json
graspgenx_geometry_selection.json
```

These artifacts are useful for reproducing a real case with:

```text
replay_graspgenx_runtime_case.py
```

without moving the robot.

---

# 9. Geometry limitations

The current GraspGen-X selector is a point-cloud geometric selector.  
A successful selection means that the candidate passed the implemented geometric and semantic checks; it is **not** a complete proof of robot-safe execution.

The selector does not provide:

```text
- full fingertip/target mesh collision verification;
- force or force-closure verification;
- full scene-obstacle collision verification;
- full robot-arm collision verification.
```

Final trajectory feasibility and collision checking remain responsibilities of the robot motion-planning/execution layer.

---

# Official references

```text
GraspMolmo:
https://github.com/abhaybd/GraspMolmo
https://huggingface.co/allenai/GraspMolmo

GraspGen-X:
https://github.com/NVlabs/GraspGenX
https://huggingface.co/adithyamurali/GraspGenXModel

Gripper descriptions:
https://huggingface.co/datasets/adithyamurali/gripper_descriptions
```
