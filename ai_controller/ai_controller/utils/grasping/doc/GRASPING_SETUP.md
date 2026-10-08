# GraspMolmo + M2T2 — Quick Setup

This guide describes the project setup used to run GraspMolmo and M2T2 as two separate servers and test them on a `.pkl` rollout.

## Expected structure

Repository inside the container:

```text
/home/ros2_ws/src/UR5e-2f-85
```

Environments and external sources:

```text
/opt/graspmolmo_venv
/opt/GraspMolmo

/opt/m2t2_venv
/opt/M2T2
```

Checkpoints/cache:

```text
<repo>/.runtime/huggingface
<repo>/.runtime/m2t2_checkpoints/m2t2.pth
```

## Requirement: `.pkl` rollout

The tests described below assume a rollout file, for example:

```text
/scene_capture/traj_011.pkl
```

The file must contain `traj`. The selected step must contain the following fields inside `obs`:

```text
camera_gripper_image + camera_gripper_depth
```

or:

```text
eye_in_hand_image + eye_in_hand_depth
```

The geometric part also requires:

```text
eef_pos
eef_quat
```

The examples below use `--step 18`.

---

## 1. GraspMolmo

### Environment installation

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

Commit used by the project:

```text
73990b5c5121ac5ab5fd402453bfeeefd270d153
```

The runtime loads:

```text
allenai/GraspMolmo
```

### Download the checkpoint

Place the download script inside the `scripts` folder, for example as `download_checkpoint.py`, then run:

```bash
cd /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/utils/grasping/graspmolmo/scripts
/opt/graspmolmo_venv/bin/python download_checkpoint.py
```

The model is stored in the project Hugging Face cache:

```text
/home/ros2_ws/src/UR5e-2f-85/.runtime/huggingface
```

Before starting GraspMolmo, use the same cache:

```bash
export HF_HOME=/home/ros2_ws/src/UR5e-2f-85/.runtime/huggingface
```

### Start the GraspMolmo server

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

### Standalone GraspMolmo test from `.pkl`

With the GraspMolmo server already running:

```bash
cd /home/ros2_ws/src/UR5e-2f-85

PKL=/scene_capture/traj_011.pkl \
STEP=18 \
TASK="Pick up the blue cube." \
PYTHONPATH=/home/ros2_ws/src/UR5e-2f-85/ai_controller:$PYTHONPATH \
python3 - <<'PY'
import os
import cv2
import numpy as np
from pathlib import Path

from ai_controller.utils.grasping.test_grasp_pipeline import (
    GRASPMOLMO_CONFIG,
    get_gripper_rgbd,
    get_trajectory_step,
    load_pickle,
)
from ai_controller.utils.grasping.graspmolmo.client import (
    GraspMolmoClient,
)

pkl_path = Path(os.environ["PKL"])
step_index = int(os.environ["STEP"])
task = os.environ["TASK"]

data = load_pickle(pkl_path)
step = get_trajectory_step(
    data["traj"],
    step_index,
)

image_bgr, _, _, _ = get_gripper_rgbd(
    step["obs"]
)

image_rgb = cv2.cvtColor(
    np.asarray(image_bgr),
    cv2.COLOR_BGR2RGB,
)

client = GraspMolmoClient(
    config_path=GRASPMOLMO_CONFIG
)

point = client.predict_point(
    image_rgb,
    task,
    verbosity=1,
    timeout=60.0,
    seed=42,
)

print("GraspMolmo point:", point)
PY
```

---

## 2. M2T2

### Environment installation

The project installation script prepares:

```text
/opt/m2t2_venv
/opt/M2T2
```

From the repository:

```bash
cd /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/utils/grasping/m2t2/scripts
bash install_m2t2.sh
```

Official repository:

```text
https://github.com/NVlabs/M2T2.git
```

Commit used by the project:

```text
be2e5f6feae2f961615ab0f7b643f0f98b0c3a50
```

### Download the checkpoint

Place the download script inside the `scripts` folder, for example as `download_checkpoint.py`, then run:

```bash
cd /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/utils/grasping/m2t2/scripts
/opt/m2t2_venv/bin/python download_checkpoint.py
```

The generic checkpoint is stored at:

```text
/home/ros2_ws/src/UR5e-2f-85/.runtime/m2t2_checkpoints/m2t2.pth
```

### Start the M2T2 server

```bash
cd /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/utils/grasping/m2t2

PYTHONPATH=/home/ros2_ws/src/UR5e-2f-85/ai_controller \
PYTHONNOUSERSITE=1 \
/opt/m2t2_venv/bin/python server.py \
    --config config/m2t2_config.yaml
```

Server:

```text
127.0.0.1:8781
```

### Standalone M2T2 test from `.pkl`

With the M2T2 server already running, do not pass `--task`. In this case, the test runs the RGB-D/geometric preprocessing and M2T2 only, without running GraspMolmo.

```bash
cd /home/ros2_ws/src/UR5e-2f-85

PYTHONPATH=/home/ros2_ws/src/UR5e-2f-85/ai_controller:$PYTHONPATH \
python3 ai_controller/ai_controller/utils/grasping/test_grasp_pipeline.py \
  --pkl /scene_capture/traj_011.pkl \
  --step 18 \
  --num-runs 10 \
  --seed 42
```

---

## 3. Combined GraspMolmo + M2T2 test

Both servers must be running at the same time:

```text
GraspMolmo -> 127.0.0.1:8780
M2T2      -> 127.0.0.1:8781
```

Then run:

```bash
cd /home/ros2_ws/src/UR5e-2f-85

PYTHONPATH=/home/ros2_ws/src/UR5e-2f-85/ai_controller:$PYTHONPATH \
python3 ai_controller/ai_controller/utils/grasping/test_grasp_pipeline.py \
    --pkl /scene_capture/traj_001.pkl \
    --step 16 \
    --base-to-table /home/ros2_ws/src/UR5e-2f-85/.runtime/eye_in_hand_tests/ring/base_to_table_transform.yaml \
    --num-runs 20 \
    --seed 42 \
    --task "Pick up the gray ring by grasping its handle, not the circular ring body." \
    --semantic-radius-px 50 \
    --confidence-threshold 0.4 \
    --max-approach-tilt-deg 45 \
    --max-wrist-rotation-deg 45 \
    --finger-collision-check
```

Configuration used in the recent tests:

```text
num-runs                 = 10
confidence-threshold     = 0.50
semantic-radius-px       = 50
max-approach-tilt-deg    = 20
max-wrist-rotation-deg   = 20
```

Test outputs are saved to:

```text
/home/ros2_ws/src/UR5e-2f-85/.runtime/grasp_pipeline_test
```

## Official references

```text
GraspMolmo:
https://github.com/abhaybd/GraspMolmo
https://huggingface.co/allenai/GraspMolmo

M2T2:
https://github.com/NVlabs/M2T2
https://huggingface.co/wentao-yuan/m2t2
```
