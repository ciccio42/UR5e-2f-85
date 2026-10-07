# Video-Captioning + VLA-JEPA (Cosmos-Reason2 bridge)

This page covers two things that share the same dependency stack and the
same GPU-architecture fix:

1. **Cosmos-Reason2** (`video_captioning/`) - NVIDIA's video-captioning VLM,
   used standalone (`cosmos-reason2/scripts/inference_sample.py`) or as a
   once-per-episode task-instruction generator for VLA-JEPA (`cosmos_captioner.py`),
   reproducing the pattern in `Multi-Task-LFD/repo/VLA-Bench/robosuite_test/run_robosuite_eval.py`
   + `vllm_utils.py` (there: an external vLLM server + CLI subprocess; here:
   in-process via `transformers`).
2. **VLA-JEPA** (`vla_jepa_controller/`) - the `lerobot`-based real-robot
   policy, ported from `Alex/UR5e-2f-85/.../vla_jepa_controller` (checkpoint
   and human-demo data were already present in this repo, no transfer
   needed).

## Why a separate venv, not `--break-system-packages` into the ROS python

`lerobot`/`transformers`/`scipy` require **numpy>=2**, whose ABI is
incompatible with `cv_bridge`'s compiled C++ extension (built against numpy
1.x) that `ai_controller_node.py` depends on for *all* ROS image decoding -
verified on this host: with numpy>=2 installed system-wide, `cv_bridge.imgmsg_to_cv2`
crashes on every call (even a same-encoding rgb8->rgb8 passthrough), and
with numpy<2, `lerobot`/`scipy` crash on import (`AttributeError: module
'numpy' has no attribute 'long'`). The two stacks cannot share a Python
interpreter. So:

- **`ai_controller_node.py`'s own python** (`pip3 ... --break-system-packages`,
  the convention used by every other model in `ai_controller_models/`) stays
  on **numpy<2** - untouched by anything in this page.
- **Cosmos + VLA-JEPA** run in a **separate venv** (`/opt/vla_jepa_venv` below)
  with **numpy>=2**, isolated from ROS entirely.
- `ai_controller_node.py` talks to VLA-JEPA over local HTTP
  (`vla_jepa_client.py`, numpy<2, no torch) instead of importing
  `VLAJEPAController` (numpy>=2) directly - see `server.py` and
  `vla_jepa_client.py`'s module docstrings. This mirrors the split-process
  pattern the cluster eval harness already uses
  (`models/lerobot_policy.py` <-> `lerobot_policy_server.py`), just over a
  local loopback port instead of a Slurm-allocated node.

## GB10 (DGX Spark) NVRTC fix

This host's GPU is an NVIDIA GB10 (compute capability 12.1 / `sm_121`). The
`cu128` (CUDA 12.8) PyTorch wheels' precompiled kernels stop at `sm_120`, and
CUDA 12.8's NVRTC doesn't recognize `sm_121` either, so any JIT-compiled CUDA
kernel crashes with `nvrtc: error: invalid value for --gpu-architecture
(-arch)`. CUDA 13.0's NVRTC does support `sm_121` - install the **`cu130`**
wheels everywhere below. `lerobot` also pins `torch>=2.7,<2.12.0`, so pin the
exact version `2.11.0+cu130` (both constraints are satisfiable together;
confirmed the same reduction-kernel JIT smoke test - `torch.prod` on a CUDA
tensor - still passes at this exact version).

## Installation

All of this goes in a dedicated venv inside the container (`docker exec -it
ur_robotiq_teleoperation_container bash`), not the system python:

```bash
apt-get update && apt-get install -y python3.12-venv ffmpeg
cd /home/ros2_ws/src/ai_controller/ai_controller/models/vla_jepa_controller
python3 -m venv ./venv/vla_jepa_venv

# activate venv
source ./venv/vla_jepa_venv/bin/activate
pip install --upgrade pip
pip install pyyaml typeguard
pip install 'torch==2.11.0+cu130' 'torchvision==0.26.0+cu130' \
  --index-url https://download.pytorch.org/whl/cu130

pip install \
  'numpy>=2.0.0,<2.3.0' \
  'opencv-python>=4.9.0,<4.14.0' \
  'Pillow>=10.0.0,<13.0.0' \
  'einops>=0.8.0,<0.9.0' \
  'draccus==0.10.0' \
  'huggingface-hub>=1.0.0,<2.0.0' \
  'requests>=2.32.0,<3.0.0' \
  'gymnasium>=1.1.1,<2.0.0' \
  'safetensors>=0.4.3,<1.0.0' \
  'packaging>=24.2,<26.0' \
  'termcolor>=2.4.0,<4.0.0' \
  'tqdm>=4.66.0,<5.0.0' \
  'setuptools>=71.0.0,<81.0.0' \
  'transformers>=5.4.0,<5.6.0' \
  'diffusers>=0.27.2,<0.36.0' \
  'qwen-vl-utils>=0.0.11,<0.1.0' \
  'peft>=0.18.0,<1.0.0' \
  gguf omegaconf scipy accelerate pyyaml scikit-learn matplotlib flask torchcodec

# lerobot (bundled fork, VLA-JEPA policy code under src/lerobot/policies/vla_jepa/)
# --no-deps: its pyproject.toml deps are already covered above one by one -
# letting pip re-resolve here is what silently upgraded torch to a plain
# (non-cu130) PyPI build earlier and broke CUDA; --no-deps avoids that.
cd /home/ros2_ws/src/ai_controller/ai_controller/models/vla_jepa_controller/external/lerobot
pip install -e . --no-deps
```

`vla_jepa_config.yaml`'s `checkpoint_path` already points at
`ai_controller/checkpoint_folder/lerobot/vla-jepa/016000/pretrained_model`
(present in this repo, no transfer needed). Human-demo trajectories (for
Cosmos captioning and for `load_command`'s static per-task prompt fallback)
are expected under `demo_path` (`/dataset/pick_place/human_rgb_pick_place`
by default, same convention as every other controller here).

## Running

**1. Start the VLA-JEPA + Cosmos server** (separate venv, loads the real
checkpoint - keep this running in its own terminal/session):
```bash
docker exec -it ur_robotiq_teleoperation_container bash
export HF_HOME="/home/ros2_ws/src/ai_controller/ai_controller/models/video_captioning/Video-Captioning-Human-Demo"
cd /home/ros2_ws/src/ai_controller/ai_controller/models/vla_jepa_controller
python3 server.py --config vla_jepa_config.yaml
# -> "[vla_jepa_server] Listening on 127.0.0.1:8770"
```
Cosmos itself is loaded lazily here, on the first `/caption_task` request -
not at server startup, so a run without Cosmos doesn't pay that extra load
time/VRAM. If you *will* run with `use_cosmos_task_description:=true`, add
`--use-cosmos-task-description` to preload it up front instead:
```bash
python3 server.py --config vla_jepa_config.yaml --use-cosmos-task-description
# -> "[vla_jepa_server] Preloading Cosmos-Reason2 for task captioning..."
# -> "[vla_jepa_server] Cosmos-Reason2 ready."
# -> "[vla_jepa_server] Listening on 127.0.0.1:8770"
```
Do this - lazily loading Cosmos-Reason2-2B (`from_pretrained` alone, on a
cold HF cache) can take longer than `vla_jepa_client.py`'s `/caption_task`
timeout (180s), timing out the *first* Cosmos-generated-instruction request
of a session. Preloading moves that cost to server startup (before `/health`
responds - the node's connection retries just wait longer), so every
`/caption_task` call afterwards only pays for actual inference.

**2. Run `ai_controller_node`** (normal ROS system python, separate
terminal) - static per-task prompt from `vla_jepa_config.yaml`'s `tasks:` map:
```bash
docker exec -it ur_robotiq_teleoperation_container bash
source install/setup.bash
ros2 run ai_controller ai_controller_node --ros-args \
    -p move_robot:=True \
    -p ai_controller_target:="vla_jepa_controller" \
    -p model_config_path:="/home/ros2_ws/src/ai_controller/ai_controller/models/vla_jepa_controller/vla_jepa_config.yaml"
```

To use a **Cosmos-generated** instruction instead (captions that task's
human demo once, at episode start, instead of the static prompt - same
config, same checkpoint, just add one flag):
```bash
ros2 run ai_controller ai_controller_node --ros-args \
    -p move_robot:=True \
    -p ai_controller_target:="vla_jepa_controller" \
    -p model_config_path:="/home/ros2_ws/src/ai_controller/ai_controller/models/vla_jepa_controller/vla_jepa_config.yaml" \
    -p use_cosmos_task_description:=true
```

## Unit tests

```bash
# Cosmos alone (standalone smoke test, no VLA-JEPA): venv
export HF_TOKEN="" # Insert Secret
export HF_HOME="/home/ros2_ws/src/ai_controller/ai_controller/models/video_captioning/Video-Captioning-Human-Demo"
cd /home/ros2_ws/src/ai_controller/ai_controller/models/video_captioning
/opt/vla_jepa_venv/bin/python3 cosmos-reason2/scripts/inference_sample.py

# render_demo_clip() + CosmosCaptioner on a real task demo: venv
/opt/vla_jepa_venv/bin/python3 test_cosmos_captioner.py --task-id 10

# VLAJEPAController in isolation (no HTTP): venv
cd /home/ros2_ws/src/ai_controller/ai_controller/models/vla_jepa_controller
/opt/vla_jepa_venv/bin/python3 test_vla_jepa_controller.py --task-id 10

# VLAJEPAControllerClient <-> server.py HTTP round-trip: system python
# (requires the server from step 1 above to already be running)
python3 test_vla_jepa_client_server.py --task-id 10
```

## Debug windows

When `caption_task_with_cosmos`/`CosmosCaptioner.caption_video` runs, two
`cv2` debug windows pop up on whatever `DISPLAY` the process inherits (the
`ur_robotiq_teleoperation` container is already run with `-e DISPLAY` + the
X11 socket mounted):

1. **`Cosmos: input video`** - the rendered demo clip plays back live, frame
   by frame, while `render_demo_clip()` is loading/preparing it (this is
   also why the venv needs the GUI `opencv-python` build above, not
   `opencv-python-headless` - the latter can never open a window, on any
   `DISPLAY`).
2. Once inference finishes, that window closes and **`Cosmos: computed
   prompt`** opens instead: the clip's first and last frame side by side,
   with the generated caption wrapped underneath - a quick sanity check that
   the instruction actually matches what happened in the demo.

Both are best-effort: if no `DISPLAY`/X server is reachable, `cv2.imshow`
raises `cv2.error` (the "GTK+/Cocoa support" message), which is caught and
logged once (same pattern as `osvi_awda_controller`'s waypoint-overlay debug
window) rather than crashing the caption call.

Separately, `ai_controller_node.py` itself (system python, any
`ai_controller_target`) opens a third window, **`AI Controller: gripper |
third view`**, updated every control-loop step: `self.camera_topic[3]`
(gripper/wrist camera) side by side with `self.camera_topic[0]` (front
camera) - the same two cameras `front_camera_index`/`gripper_camera_index`
in `vla_jepa_config.yaml` select for VLA-JEPA's own inference input, so this
preview shows exactly what the model sees, not an extra unused viewpoint.
Same best-effort try/except-`cv2.error` pattern as the two windows above.

## Saved intermediate results

Everything is written next to the rollout itself, per task and trajectory
count:

```
<save_rollout_path>/vla_jepa_controller/pick_place/{cosmos|static_prompt}/
└── task_<id>/
    ├── traj_<cnt>.pkl / traj_<cnt>.json   # rollout + outcome (save_rollout)
    └── traj_<cnt>/                        # intermediate results
        ├── step_<n>/camera_image_<i>.png  # every camera, every step
        └── cosmos/                        # only with use_cosmos_task_description
            ├── task_<id>_demo.mp4         # square-padded clip Cosmos saw
            ├── caption_result.png         # first/last frame + caption
            └── caption.json               # caption, raw_caption, prompts,
                                           # demo_file, fps, token count, sampling params
```

This applies to every `ai_controller_target`, not just VLA-JEPA: per-step
images used to go to a single `saved_images/task_<task_name>/` folder that
was overwritten on every step and every trajectory.

## Parity with VLA-Bench's Cosmos pipeline

`cosmos_captioner.py` is meant to reproduce `Multi-Task-LFD/repo/VLA-Bench/
robosuite_test/vllm_utils.py::run_vllm_server`'s Cosmos call as closely as
the in-process `transformers` path allows (that script drives an external
`cosmos-reason2-inference` CLI/vLLM server instead):

- **Video preprocessing**: `render_demo_clip()` pads every frame to a black
  square (`_pad_frames_to_square`) before encoding, matching
  `vllm_utils.py::pad_video_to_square` - otherwise the vision processor's
  own resize-to-token-budget step would distort a non-square frame's aspect
  ratio.
- **Sampling params**: `CosmosCaptioner.caption_video()`'s `model.generate()`
  call uses `do_sample=True, temperature=0.7, top_p=0.8, top_k=20,
  repetition_penalty=1.0` - the same defaults VLA-Bench gets from its
  `--no-reasoning` CLI flag (`cosmos_reason2_utils.script.inference.
  SamplingOverrides.get_defaults(reasoning=False)`). That default set also
  has `presence_penalty=1.5`, which `transformers.generate()` has no
  equivalent for, so it's omitted. `max_new_tokens=64` matches VLA-Bench's
  `--max-tokens 64`; sampling fps deliberately differs (`fps=2` here vs.
  VLA-Bench's `--fps 4`) to halve the number of video tokens and speed up
  prefill.
- **Caption post-processing**: `_postprocess_caption()` applies the exact
  same normalization `run_vllm_server` does to Cosmos's raw text before
  it's used as a task instruction - digits to ordinal words ("1"->"first",
  ..., "4"->"fourth"), "three"/"four" -> "third"/"fourth", a redundant
  "compartment box" -> "box", and stripped trailing periods. This matters
  regardless of which `cosmos_prompt_yaml` is active, since it normalizes
  whatever numeral/wording Cosmos happens to produce into the same
  ordinal-word convention `vla_jepa_config.yaml`'s `tasks:` map uses.

Not replicated: VLA-Bench's actual production prompt
(`robosuite_test/prompt/human_task_description_prompt.yaml`) is a generic
"What happened in the video?" prompt with no bin/color template - the
post-processing above exists specifically to clean up *that* prompt's raw
output. This deployment instead defaults to the more structured
`fixed_prompt.yaml` (see below), which needs less normalization but still
benefits from the same post-processing pass.

## `cosmos_prompt_yaml`

`vla_jepa_config.yaml`'s `cosmos_prompt_yaml` key selects which
`system_prompt`/`user_prompt` pair (under `video_captioning/Video-Captioning-
Human-Demo/prompts/`) `CosmosCaptioner.caption_video` uses when
`use_cosmos_task_description:=true`. Defaults to `fixed_prompt.yaml`, whose
"4 colored boxes, 4 numbered bins" template matches this deployment's actual
task convention (`tasks:` map above) exactly - unlike the more open-ended
`generalist_task_description.yaml` used by the standalone
`cosmos-reason2/scripts/inference_sample.py`/`test_cosmos_captioner.py`
smoke tests. Read server-side (`server.py`'s `/caption_task` handler, same
config file as `checkpoint_path` - no client-side change needed); a relative
path resolves against `video_captioning/`, matching `cosmos_captioner.py`'s
`DEFAULT_PROMPT_YAML`.

## Files

- `video_captioning/cosmos_captioner.py` - `CosmosCaptioner` (loads
  `nvidia/Cosmos-Reason2-2B` once, `caption_video()`) + `render_demo_clip()`
  (renders an mp4 from a human-demo trajectory's `camera_front_image` frames
  for Cosmos to caption - see its docstring for the JPEG color-order fix
  this needed).
- `vla_jepa_controller/vla_jepa_controller.py`, `vla_jepa.py`,
  `vla_jepa_utils.py` - the ported controller (unchanged from
  `Alex/UR5e-2f-85`, ROS-agnostic already).
- `vla_jepa_controller/server.py` - the HTTP bridge server (runs in
  `/opt/vla_jepa_venv`).
- `vla_jepa_controller/vla_jepa_client.py` - the HTTP client
  `ai_controller_node.py` actually imports for `ai_controller_target:=vla_jepa_controller`.
