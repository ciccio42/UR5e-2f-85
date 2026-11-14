# GraspMolmo + GraspGen-X — Quick Setup

This guide documents the **current optimized GraspMolmo + GraspGen-X setup** used by the SeeDo grasp-planning pipeline, in both Isaac Sim and real-robot operation. It complements the project launch guides `start_seedo_isaac_sim.md` and `start_seedo_real_test.md`.

The services are deliberately isolated from the SeeDo Python environment:

- **GraspMolmo** runs as an HTTP service at `127.0.0.1:8780`, using the optimized launch options documented below;
- **GraspGen-X** runs as a GPU-backed ZMQ service at `127.0.0.1:8782`;
- a **persistent GraspGen-X worker** listens on an AF_UNIX socket (`/tmp/seedo_graspgenx_worker.sock`), holds the heavy Python imports in memory, and forwards individual inference requests to the GPU server;
- SeeDo uses `GraspGenXRemoteClient` to communicate with that worker when `SEEDO_GRASPGENX_PERSISTENT=1`;
- the combined GraspGen-X launcher runs **one inference warm-up before `[READY]`** when `SEEDO_GRASPGENX_WARMUP=1`;
- the active project backend remains `graspgenx` (there is no separate `graspgenx_v2` backend).

**Environment scope:** setting variables before `./start_graspgenx_services.sh` configures that launcher and its children only. A separate SeeDo / AI Controller / GUI process must export the persistent-worker variables in its **own** environment.

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
│   ├── persistent_worker.py
│   ├── runtime_worker.py
│   ├── worker.py
│   └── scripts/
│       └── start_graspgenx_services.sh
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

Service communication during grasp candidate generation:

```text
SeeDo / GraspGenXRemoteClient
         |
         | AF_UNIX: /tmp/seedo_graspgenx_worker.sock
         v
GraspGen-X persistent worker (dedicated Python environment)
         |
         | ZMQ: tcp://127.0.0.1:8782
         v
GraspGen-X GPU server (loaded model)
```

GraspMolmo is an independent HTTP service at `127.0.0.1:8780`. The persistent worker and the ZMQ server are started and stopped together by the combined launcher. Grasp candidate generation remains separate from the SeeDo-side geometric selection.

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

## Start the optimized GraspMolmo server

Open a dedicated terminal inside `seedo_ros2_container` (from the host, use `docker exec -it seedo_ros2_container bash`) and run the **current project launch command**:

```bash
cd /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/utils/grasping/graspmolmo

SEEDO_GRASPMOLMO_GQA=1 \
SEEDO_GRASPMOLMO_PROFILE_DECODER=1 \
SEEDO_GRASPMOLMO_FP8_TEST_LAYER=all \
SEEDO_GRASPMOLMO_FP8_COMPILE_MODE=default \
SEEDO_GRASPMOLMO_ATTN_COMPILE_LAYERS=all \
SEEDO_GRASPMOLMO_ATTN_COMPILE_PARTS=out \
TRITON_PTXAS_PATH=/usr/local/cuda/bin/ptxas \
HF_HOME=/home/ros2_ws/src/UR5e-2f-85/.runtime/huggingface \
PYTHONPATH=/home/ros2_ws/src/UR5e-2f-85/ai_controller \
PYTHONNOUSERSITE=1 \
/opt/graspmolmo_venv/bin/python server.py \
  --config config/graspmolmo_config.yaml
```

These are the flags used by the project's optimized configuration (GQA, FP8/attention compile-related settings and decoder profiling). They do not change the GraspMolmo endpoint consumed by SeeDo. Keep this process alive throughout the experiments.

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

The SeeDo environment itself does **not** need the GraspGen-X Python dependencies. The dedicated worker uses:

```text
/opt/graspgenx_venv/bin/python
```

The persistent worker imports the GraspGen-X integration once, then handles successive requests over the UNIX socket; each inference still goes to the official GraspGen-X GPU server over ZMQ. The legacy subprocess-per-call path is retained only for compatibility when `SEEDO_GRASPGENX_PERSISTENT` is disabled. **There is no automatic fallback** to that path if persistent mode is enabled but the worker is unavailable.

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

## Start the GraspGen-X server + persistent worker + warm-up (recommended)

From a new **host terminal**, enter `seedo_ros2_container` and start the combined launcher:

```bash
docker exec -it seedo_ros2_container bash

cd /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/utils/grasping/graspgenx/scripts

SEEDO_GRASPGENX_PERSISTENT=1 \
SEEDO_GRASPGENX_SOCKET=/tmp/seedo_graspgenx_worker.sock \
SEEDO_GRASPGENX_WARMUP=1 \
GRASPGENX_CHECKPOINT_DIR=/opt/graspgenx_checkpoints \
GRASPGENX_GRIPPER_CFG_DIR=/opt/graspgenx_gripper_descriptions \
PYTHONNOUSERSITE=1 \
./start_graspgenx_services.sh
```

The launcher performs the following steps, **in order**:

1. Starts the GraspGen-X GPU ZMQ server at `tcp://0.0.0.0:8782` and loads the checkpoints and default `robotiq_2f_85` gripper.
2. Starts `persistent_worker.py` in `/opt/graspgenx_venv`, listening on `/tmp/seedo_graspgenx_worker.sock`.
3. Checks that the worker and the server can communicate.
4. With `SEEDO_GRASPGENX_WARMUP=1`, sends one real `infer_object` request through the persistent worker, using a **synthetic 5-cm cuboid point cloud (2,048 points)** and the project's `graspmoe` / `dense` OBB configuration (200 diffusion grasps requested). Generated candidates are discarded; **no robot command is issued**.
5. Prints `[READY]` after the checks and warm-up have succeeded.

Expected startup log excerpt (timings and candidate counts can vary):

```text
Starting GraspGenX GPU server (PID=...)
Starting persistent worker (PID=...)
[GRASPGENX PERSISTENT] Ready | PID=... | socket=/tmp/seedo_graspgenx_worker.sock
[WARMUP] Running one GraspGenX infer_object through the persistent worker...
[WARMUP] Completed in ... s | grasp candidates=... (discarded)
[READY] GraspGenX server PID=...; worker PID=...
```

**Keep the launcher running** while SeeDo is active. Stop both services with `Ctrl+C` in this terminal; do not restart them between consecutive experiments. `SEEDO_GRASPGENX_WARMUP=1` controls the launcher warm-up, **not** the SeeDo process; the warm-up cost is incurred before the SeeDo end-to-end timer starts. A health check alone would not execute the GPU inference path.

The ZMQ service uses TCP port `8782`, but SeeDo in persistent mode connects to the **UNIX socket**, not directly to the ZMQ server. The socket and SeeDo process must be accessible in the same filesystem/container namespace (as in the documented `seedo_ros2_container` setup).

## Health and persistence checks

From another **host terminal**, enter the same container and use the project's client in persistent mode:

```bash
docker exec -it seedo_ros2_container bash

cd /home/ros2_ws/src/UR5e-2f-85

SEEDO_GRASPGENX_PERSISTENT=1 \
SEEDO_GRASPGENX_SOCKET=/tmp/seedo_graspgenx_worker.sock \
PYTHONPATH=/home/ros2_ws/src/UR5e-2f-85/ai_controller \
python3 - <<'PY'
from ai_controller.utils.grasping.graspgenx.client import GraspGenXRemoteClient

client = GraspGenXRemoteClient(
    host="127.0.0.1",
    port=8782,
    gripper_name="robotiq_2f_85",
)
print("GraspGen-X persistent health:", client.health())
PY
```

Expected result: `GraspGen-X persistent health: True`.

Check that the **same worker PID** remains alive across experiments:

```bash
ps -ef | grep '[p]ersistent_worker.py'
```

A responsive health check establishes connectivity, not grasp correctness. Use the saved artifacts and Isaac Sim tests for the latter.

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

## Persistent-worker environment in the SeeDo process

**Required for the optimized path:** set these exports in the same terminal/process environment that starts the SeeDo AI Controller (or the GUI backend):

```bash
export SEEDO_GRASPGENX_PERSISTENT=1
export SEEDO_GRASPGENX_SOCKET=/tmp/seedo_graspgenx_worker.sock
```

**Do not assume** that variables provided to the GraspGen-X launcher in Terminal 2 are automatically visible in another terminal. If the SeeDo process is launched by a script, ensure it inherits the variables, for example:

```bash
export SEEDO_GRASPGENX_PERSISTENT=1
export SEEDO_GRASPGENX_SOCKET=/tmp/seedo_graspgenx_worker.sock

bash /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/launch/run_seedo_isaac_test.sh \
  --config /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/config/seedo_controller.yaml
```

The full command, including any precomputed-plan arguments, is documented in `start_seedo_isaac_sim.md`; for the robot use `start_seedo_real_test.md`. Both launch guides also export the persistent-worker variables for the experiment configurations and GUI, even where the grasp module is not used, to keep the launch environment consistent. Those exports **do not force the grasp module to run** when the selected YAML/configuration or execution mode does not use it.

### Isaac Sim versus real robot

For the **currently documented Isaac validation setup**, the launch guide applies a *temporary post-planning* offset:

```bash
export SEEDO_ISAAC_GRASP_Z_OFFSET_M=0.020
export QT_QPA_PLATFORM=offscreen
```

This `+20 mm` Isaac-only test correction is **not** the same as `graspgenx_depth_offset_m: 0.005` in the planner configuration, and it is **not part of the GraspGen-X warm-up**. It must not silently carry over to physical robot execution.

For the **real robot** the launch guide explicitly clears it:

```bash
unset SEEDO_ISAAC_GRASP_Z_OFFSET_M
export QT_QPA_PLATFORM=offscreen
export SEEDO_GRASPGENX_PERSISTENT=1
export SEEDO_GRASPGENX_SOCKET=/tmp/seedo_graspgenx_worker.sock
```

Keep the robot's normal collision, trajectory and execution safety checks active. The temporary Isaac offset does not validate a grasp as safe for the physical UR5e.

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

Recorded expectation for the two original unit-test files:

```text
5 passed
```

This number is historical and is **not** a verified result for every subsequent persistent-worker change. The unit tests listed above do not require a real GraspMolmo or GraspGen-X server; separately validate persistent-worker connectivity, warm-up startup and an actual Isaac/real task before a release.

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

The examples below assume the GraspGen-X GPU server and persistent worker are already running; they pass the same socket environment variables as the SeeDo process. The replay does **not** execute physical robot commands.

---

## Replay using the semantic point saved during the real run

Example:

```bash
cd /home/ros2_ws/src/UR5e-2f-85

SEEDO_GRASPGENX_PERSISTENT=1 \
SEEDO_GRASPGENX_SOCKET=/tmp/seedo_graspgenx_worker.sock \
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

SEEDO_GRASPGENX_PERSISTENT=1 \
SEEDO_GRASPGENX_SOCKET=/tmp/seedo_graspgenx_worker.sock \
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

Run both services and the AI Controller in **separate terminals**, all inside `seedo_ros2_container`:

| Terminal | Process | Interface |
| --- | --- | --- |
| 1 | Optimized GraspMolmo server | HTTP `127.0.0.1:8780` |
| 2 | Combined GraspGen-X GPU server + persistent worker + warm-up | ZMQ `127.0.0.1:8782`; AF_UNIX `/tmp/seedo_graspgenx_worker.sock` |
| 3 | SeeDo AI Controller / Isaac or real launch | `SEEDO_GRASPGENX_PERSISTENT=1`, matching socket path |

Enable `grasp_planner.enabled: true` and `grasp_planner.backend: graspgenx` only in configurations intended to run this grasp pipeline. A demo-only or baseline configuration that does not invoke grasping does not use the worker just because the variables are set.

The SeeDo process stays in its normal Python environment. GraspGen-X dependencies remain isolated in `/opt/graspgenx_venv`. The persistent bridge processes sequential inference requests without repeating Python/PyTorch imports; the GPU model remains loaded in the ZMQ server across experiments.

For the **full startup order**, including ZED cameras, Isaac Sim, MoveIt, robot controllers, GUI and experiment YAMLs, consult the matching project guide:

- `start_seedo_isaac_sim.md` — Isaac Sim 5.1.0 startup, `SEEDO_ISAAC_GRASP_Z_OFFSET_M=0.020` only for the documented simulation tests;
- `start_seedo_real_test.md` — ZED + UR5e real setup, with `SEEDO_ISAAC_GRASP_Z_OFFSET_M` unset.

## Measured optimization (single-run evidence, not a benchmark distribution)

Measurements from the SeeDo/Isaac validation sessions of **14 November 2026**:

| Execution | `grasp.graspgenx_inference` |
| --- | ---: |
| Earlier legacy subprocess baseline | ~10.246 s |
| First persistent execution, without GPU warm-up | 1.632 s |
| Subsequent persistent execution, same service session | 0.412 s |
| First persistent execution after server restart **with automatic warm-up** | 0.497 s |

The new startup warm-up itself took **1.706 s** in the recorded launcher log (the inference produced and discarded 1,064 candidates). It is outside the SeeDo run timer. The first post-warm-up inference is consistent with the earlier steady-state measurement (~0.4–0.5 s), but point-cloud content, GPU load and run conditions can vary. These results **support**, but do not statistically prove, warm-up efficacy; evaluate repeated matched inputs for a controlled performance claim.

`grasp.graspgenx_inference` is a **subphase** of `grasp_planning`; do not add it again to `grasp_planning` or the end-to-end time in presentation tables. The geometry-selection time is a separate part of the grasp-planning phase.

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
