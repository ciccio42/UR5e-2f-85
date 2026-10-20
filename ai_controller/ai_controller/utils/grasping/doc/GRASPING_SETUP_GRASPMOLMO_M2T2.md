# GraspMolmo + M2T2 — Quick Setup

This guide describes the project setup used to run **GraspMolmo** and **M2T2** as two isolated services and use them together from the SeeDo grasp-planning pipeline.

The two models run in dedicated Python environments so that their dependencies remain isolated from the ROS / SeeDo environment.

The current service layout is:

```text
GraspMolmo -> HTTP -> 127.0.0.1:8780
M2T2      -> HTTP -> 127.0.0.1:8781
```

The combined SeeDo grasp planner uses:

```text
RGB-D eye-in-hand frame
        |
        v
GraspMolmo
semantic grasp point
        |
        v
RGB-D / geometric preprocessing
        |
        v
M2T2
6-DoF grasp candidates
        |
        v
geometric + semantic selection
        |
        v
selected grasp pose
```

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

M2T2 environment and source:

```text
/opt/m2t2_venv
/opt/M2T2
```

Runtime model data:

```text
/home/ros2_ws/src/UR5e-2f-85/.runtime/huggingface
/home/ros2_ws/src/UR5e-2f-85/.runtime/m2t2_checkpoints/m2t2.pth
```

Relevant project modules:

```text
ai_controller/ai_controller/utils/grasping/
├── graspmolmo/
│   ├── client.py
│   ├── runtime.py
│   ├── server.py
│   ├── test_client_server.py
│   ├── config/
│   │   └── graspmolmo_config.yaml
│   └── scripts/
│       ├── install_graspmolmo.sh
│       └── download_graspmolmo_checkpoint.py
│
├── m2t2/
│   ├── client.py
│   ├── runtime.py
│   ├── server.py
│   ├── test_client_server.py
│   ├── config/
│   │   └── m2t2_config.yaml
│   └── scripts/
│       ├── install_m2t2.sh
│       └── download_m2t2_checkpoint.py
│
└── test_molmo_m2t2.py
```

---

# 1. GraspMolmo

## Environment installation

The project installation script creates:

```text
/opt/graspmolmo_venv
```

and clones GraspMolmo into:

```text
/opt/GraspMolmo
```

The project uses the official repository:

```text
https://github.com/abhaybd/GraspMolmo.git
```

at commit:

```text
73990b5c5121ac5ab5fd402453bfeeefd270d153
```

Run:

```bash
cd /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/utils/grasping/graspmolmo/scripts

bash install_graspmolmo.sh
```

The installation keeps the environment isolated from ROS / SeeDo and installs the CUDA-compatible PyTorch stack required by the project.

---

## Download the GraspMolmo checkpoint

The runtime loads:

```text
allenai/GraspMolmo
```

The project stores the Hugging Face cache under:

```text
/home/ros2_ws/src/UR5e-2f-85/.runtime/huggingface
```

Run:

```bash
cd /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/utils/grasping/graspmolmo/scripts

/opt/graspmolmo_venv/bin/python \
    download_graspmolmo_checkpoint.py
```

The download script configures the project-local Hugging Face cache automatically.

---

## Start the GraspMolmo server

Run:

```bash
cd /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/utils/grasping/graspmolmo

HF_HOME=/home/ros2_ws/src/UR5e-2f-85/.runtime/huggingface \
PYTHONPATH=/home/ros2_ws/src/UR5e-2f-85/ai_controller \
PYTHONNOUSERSITE=1 \
/opt/graspmolmo_venv/bin/python server.py \
    --config config/graspmolmo_config.yaml
```

The default configuration is:

```yaml
server_host: "127.0.0.1"
server_port: 8780
```

Therefore the service is available at:

```text
127.0.0.1:8780
```

The model is loaded once when the server starts and remains resident for subsequent requests.

---

## GraspMolmo health check

With the server running:

```bash
curl -s --max-time 5 \
    http://127.0.0.1:8780/health
```

The server should return a response indicating that the backend is ready.

---

## Standalone GraspMolmo client/server test

The project provides:

```text
ai_controller/ai_controller/utils/grasping/graspmolmo/test_client_server.py
```

This test requires only the GraspMolmo server.

Example:

```bash
cd /home/ros2_ws/src/UR5e-2f-85

PYTHONPATH=/home/ros2_ws/src/UR5e-2f-85/ai_controller:$PYTHONPATH \
python3 \
    ai_controller/ai_controller/utils/grasping/graspmolmo/test_client_server.py \
    --image /scene_capture/nut/scena_1/rgb.png \
    --task "Grasp the protruding handle of the blue ring. Do not grasp the circular ring body or through the hole."
```

The test:

```text
1. loads the RGB image;
2. sends it to the GraspMolmo server together with the task instruction;
3. receives the predicted semantic grasp point;
4. verifies that the point lies inside the image;
5. saves a visualization of the prediction.
```

The default visualization output is:

```text
/home/ros2_ws/src/UR5e-2f-85/.runtime/graspmolmo_tests/test_client_server_prediction.png
```

---

# 2. M2T2

## Environment installation

The M2T2 environment is isolated from ROS / SeeDo and is located at:

```text
/opt/m2t2_venv
```

The official M2T2 source is cloned into:

```text
/opt/M2T2
```

Official repository:

```text
https://github.com/NVlabs/M2T2.git
```

The project uses commit:

```text
be2e5f6feae2f961615ab0f7b643f0f98b0c3a50
```

Run:

```bash
cd /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/utils/grasping/m2t2/scripts

bash install_m2t2.sh
```

The installation script is configured for the project CUDA environment:

```text
CUDA_HOME=/usr/local/cuda-13.0
TORCH_CUDA_ARCH_LIST=12.1
```

and compiles the PointNet++ CUDA extension required by M2T2.

---

## Download the M2T2 checkpoint

The project uses the generic M2T2 checkpoint:

```text
m2t2.pth
```

stored at:

```text
/home/ros2_ws/src/UR5e-2f-85/.runtime/m2t2_checkpoints/m2t2.pth
```

Run:

```bash
cd /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/utils/grasping/m2t2/scripts

/opt/m2t2_venv/bin/python \
    download_m2t2_checkpoint.py
```

The checkpoint is downloaded from the official Hugging Face distribution used by the project.

---

## M2T2 configuration

The configuration file is:

```text
ai_controller/ai_controller/utils/grasping/m2t2/config/m2t2_config.yaml
```

Current configuration:

```yaml
server_host: "127.0.0.1"
server_port: 8781

checkpoint_path: "/home/ros2_ws/src/UR5e-2f-85/.runtime/m2t2_checkpoints/m2t2.pth"
model_config_path: "/opt/M2T2/config.yaml"
```

---

## Start the M2T2 server

Run:

```bash
cd /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/utils/grasping/m2t2

PYTHONPATH=/home/ros2_ws/src/UR5e-2f-85/ai_controller \
PYTHONNOUSERSITE=1 \
/opt/m2t2_venv/bin/python server.py \
    --config config/m2t2_config.yaml
```

The service is available at:

```text
127.0.0.1:8781
```

The M2T2 model is loaded once at startup and remains resident on the GPU.

---

## M2T2 health check

With the server running:

```bash
curl -s --max-time 5 \
    http://127.0.0.1:8781/health
```

The response should indicate that the M2T2 backend is ready.

---

## Standalone M2T2 client/server test

The standalone M2T2 integration test is:

```text
ai_controller/ai_controller/utils/grasping/m2t2/test_client_server.py
```

This test does **not** run GraspMolmo.

It operates on a captured scene directory containing:

```text
rgb.png
depth.npy
camera_info.yaml
base_to_table_transform.yaml
```

A typical scene directory is:

```text
/scene_capture/nut/scena_1
```

Run:

```bash
cd /home/ros2_ws/src/UR5e-2f-85

PYTHONPATH=/home/ros2_ws/src/UR5e-2f-85/ai_controller:$PYTHONPATH \
python3 \
    ai_controller/ai_controller/utils/grasping/m2t2/test_client_server.py \
    --scene-dir /scene_capture/nut/scena_1 \
    --num-runs 10
```

The default camera calibration file is:

```text
/home/ros2_ws/src/UR5e-2f-85/zed_camera/zed_camera_calibration/estimated_camera_positions.yaml
```

The default camera is:

```text
zed_front
```

The standalone test performs:

```text
RGB-D loading
    ->
depth backprojection
    ->
camera -> ArUco transform
    ->
ArUco -> table_0 transform
    ->
table workspace crop
    ->
M2T2 inference
    ->
grasp candidate visualization
```

The test is intended to verify the M2T2 client/server and geometric preprocessing independently of GraspMolmo.

---

# 3. Combined GraspMolmo + M2T2 test

The combined offline test is:

```text
ai_controller/ai_controller/utils/grasping/test_molmo_m2t2.py
```

Both servers must already be running:

```text
GraspMolmo -> 127.0.0.1:8780
M2T2      -> 127.0.0.1:8781
```

---

## Required rollout data

The test receives a rollout `.pkl`.

Example:

```text
/scene_capture/traj_001.pkl
```

The file must contain:

```text
traj
```

The selected trajectory step must contain an eye-in-hand RGB-D pair using either:

```text
eye_in_hand_image
eye_in_hand_depth
```

or:

```text
camera_gripper_image
camera_gripper_depth
```

The observation must also contain:

```text
eef_pos
eef_quat
```

These values are used to reconstruct the recorded TCP pose and the corresponding eye-in-hand camera pose.

---

## Required table transform

The combined test requires a table-to-base transform file through:

```text
--base-to-table
```

Example:

```text
/home/ros2_ws/src/UR5e-2f-85/.runtime/eye_in_hand_tests/ring/base_to_table_transform.yaml
```

The transform file must contain:

```yaml
rotation: ...
translation: ...
```

---

## Camera intrinsics

If `--camera-info` is not supplied, the test uses the built-in eye-in-hand calibration:

```text
fx = 363.8071594238281
fy = 363.8071594238281
cx = 335.86553955078125
cy = 183.81210327148438
```

A camera-info YAML can instead be supplied with:

```text
--camera-info <path>
```

---

## Run the combined test

Example:

```bash
cd /home/ros2_ws/src/UR5e-2f-85

PYTHONPATH=/home/ros2_ws/src/UR5e-2f-85/ai_controller:$PYTHONPATH \
python3 \
    ai_controller/ai_controller/utils/grasping/test_molmo_m2t2.py \
    --pkl /scene_capture/traj_001.pkl \
    --step 16 \
    --base-to-table /home/ros2_ws/src/UR5e-2f-85/.runtime/eye_in_hand_tests/ring/base_to_table_transform.yaml \
    --task "Pick up the gray ring by grasping its handle, not the circular ring body." \
    --num-runs 20 \
    --seed 42 \
    --semantic-radius-px 50 \
    --confidence-threshold 0.4 \
    --max-approach-tilt-deg 45 \
    --max-wrist-rotation-deg 45
```

The test requires `--task`.

The task text is passed directly to GraspMolmo.

---

## Finger collision check

The finger collision check is **enabled by default**.

To disable it explicitly:

```text
--disable-finger-collision-check
```

Therefore, no `--finger-collision-check` argument is required.

---

## Combined pipeline

The combined test performs:

```text
rollout .pkl
    |
    v
selected trajectory step
    |
    v
eye-in-hand RGB-D
    |
    +--------------------------+
    |                          |
    v                          |
GraspMolmo                     |
    |                          |
semantic grasp point           |
    |                          |
    +------------+-------------+
                 |
                 v
        RGB-D geometry
                 |
                 v
              M2T2
                 |
                 v
        grasp candidates
                 |
                 v
 semantic + geometric filtering
                 |
                 v
       selected grasp pose
                 |
                 v
          base_link pose
```

The test validates that the final grasp result contains:

```text
grasp_pose_base
grasp_position_base
grasp_orientation_base
semantic_point_px
```

and checks that the values are finite and have the expected dimensions.

---

# 4. Main combined-test parameters

The relevant CLI parameters are:

```text
--pkl
--step
--base-to-table
--camera-info
--task
--output-dir
--num-runs
--seed
--confidence-threshold
--semantic-radius-px
--max-approach-tilt-deg
--max-wrist-rotation-deg
--disable-finger-collision-check
```

Validated/default values used by the test include:

```text
num-runs                 = 20
seed                     = 42
confidence-threshold     = 0.4
semantic-radius-px       = 50.0
max-approach-tilt-deg    = 45.0
max-wrist-rotation-deg   = 45.0
finger-collision-check   = enabled
```

These values can be overridden from the command line.

---

# 5. Combined-test outputs

The default combined-test output directory is:

```text
/home/ros2_ws/src/UR5e-2f-85/.runtime/grasp_planner_test
```

The test saves the input data used for the replay:

```text
input_rgb.png
input_depth.npy
```

The grasp planner must also generate:

```text
grasp_plan.json
```

The test fails if the grasp plan is missing, invalid or contains non-finite pose values.

---

# 6. Service startup summary

For the complete GraspMolmo + M2T2 pipeline, use three terminals.

## Terminal 1 — GraspMolmo

```bash
cd /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/utils/grasping/graspmolmo

HF_HOME=/home/ros2_ws/src/UR5e-2f-85/.runtime/huggingface \
PYTHONPATH=/home/ros2_ws/src/UR5e-2f-85/ai_controller \
PYTHONNOUSERSITE=1 \
/opt/graspmolmo_venv/bin/python server.py \
    --config config/graspmolmo_config.yaml
```

## Terminal 2 — M2T2

```bash
cd /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/utils/grasping/m2t2

PYTHONPATH=/home/ros2_ws/src/UR5e-2f-85/ai_controller \
PYTHONNOUSERSITE=1 \
/opt/m2t2_venv/bin/python server.py \
    --config config/m2t2_config.yaml
```

## Terminal 3 — Combined test

```bash
cd /home/ros2_ws/src/UR5e-2f-85

PYTHONPATH=/home/ros2_ws/src/UR5e-2f-85/ai_controller:$PYTHONPATH \
python3 \
    ai_controller/ai_controller/utils/grasping/test_molmo_m2t2.py \
    --pkl /scene_capture/traj_001.pkl \
    --step 16 \
    --base-to-table /home/ros2_ws/src/UR5e-2f-85/.runtime/eye_in_hand_tests/ring/base_to_table_transform.yaml \
    --task "Pick up the gray ring by grasping its handle, not the circular ring body." \
    --num-runs 20 \
    --seed 42
```

---

# 7. Troubleshooting

## GraspMolmo server does not start

Check:

```bash
/opt/graspmolmo_venv/bin/python --version
```

and verify that the model cache exists:

```bash
ls -lah \
    /home/ros2_ws/src/UR5e-2f-85/.runtime/huggingface
```

Use the same `HF_HOME` when starting the server.

---

## M2T2 server does not start

Check that the checkpoint exists:

```bash
ls -lh \
    /home/ros2_ws/src/UR5e-2f-85/.runtime/m2t2_checkpoints/m2t2.pth
```

Check CUDA from the M2T2 environment:

```bash
/opt/m2t2_venv/bin/python - <<'PY'
import torch
import m2t2
import pointnet2_ops

print("PyTorch:", torch.__version__)
print("CUDA available:", torch.cuda.is_available())
print("GPU:", torch.cuda.get_device_name(0))
print("Capability:", torch.cuda.get_device_capability(0))

print("M2T2 import: OK")
print("pointnet2_ops import: OK")
PY
```

---

## Test cannot connect to a backend

Check the two health endpoints:

```bash
curl -s --max-time 5 http://127.0.0.1:8780/health
curl -s --max-time 5 http://127.0.0.1:8781/health
```

The expected mapping is:

```text
8780 -> GraspMolmo
8781 -> M2T2
```

---

## Combined test cannot find the RGB-D data

The selected rollout observation must contain one of:

```text
eye_in_hand_image + eye_in_hand_depth
```

or:

```text
camera_gripper_image + camera_gripper_depth
```

It must additionally contain:

```text
eef_pos
eef_quat
```

---

## Combined test cannot load the table transform

Verify the path passed to:

```text
--base-to-table
```

The file is mandatory for `test_molmo_m2t2.py`.

---

# Official references

GraspMolmo:

```text
https://github.com/abhaybd/GraspMolmo
https://huggingface.co/allenai/GraspMolmo
```

M2T2:

```text
https://github.com/NVlabs/M2T2
https://huggingface.co/wentao-yuan/m2t2
```