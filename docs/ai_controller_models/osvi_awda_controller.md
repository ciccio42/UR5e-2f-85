# OSVI-AWDA-Controller

`osvi_awda_controller` (`ai_controller/ai_controller/models/osvi_awda_controller/`)
runs the OSVI-AWDA policy: a single forward pass on the frontal camera image
(conditioned on human-demo context frames, sampled once per task via
`load_command`) predicts a set of 5 normalized image-space waypoints
(`u, v, depth, grasp_attr`), which get projected to `base_link` and expanded
into a fixed `free_space -> grasp -> carry -> carry -> drop` primitive
sequence. Execution stops at the grasp-hover waypoint; the next control-loop
call resumes with wrist/eye-in-hand depth-based grasp refinement
(`build_post_hover_grasp_actions`) before continuing to carry/drop. The
controller itself has no ROS dependency - `ai_controller_node.py` supplies
synchronized RGB+depth images, the gripper camera intrinsics and the
depth-camera -> `base_link` transform via `inference()`'s `input_data`,
mirroring how `current_eef_pos`/`current_eef_quat` are passed to
`cod_controller`. The model code and configs are bundled directly under the
model folder (`models/`, `datasets/`, `configs/`, `osvi_awda_config.yaml`) -
no external repo to clone.

This assumes the Docker/container setup and UR-driver + `moveit_controller`
launch described in [AI-Controller](../ai_controller.md) are already running.

## Installation

Install the following inside the container (`docker exec -it
ur_robotiq_teleoperation_container bash`):

```bash
cd /home/ros2_ws
colcon build --packages-select ai_controller --symlink-install
source install/setup.bash
python3 -m pip install einops hydra-core omegaconf torchsummary tqdm pyyaml matplotlib --break-system-packages
```

`osvi_awda_config.yaml`'s `checkpoint_dir` / `checkpoint_step` must point at
an AWDA checkpoint trained with `image_waypoints=true` (e.g. under
`ai_controller/checkpoint_folder/osvi-awda/`). The gripper depth stream used
for grasp refinement is configured under `grasp_refinement:` in that same
file (`depth_source_frame_override`, `hover_height_m`, workspace-clamp
bounds, etc.).

## Running

```bash
docker exec -it ur_robotiq_teleoperation_container bash
cd /home/ros2_ws
source install/setup.bash

ros2 run ai_controller ai_controller_node --ros-args \
    -p move_robot:=True \
    -p ai_controller_target:="osvi_awda_controller" \
    -p model_config_path:="/home/ros2_ws/src/ai_controller/ai_controller/models/osvi_awda_controller/osvi_awda_config.yaml"
```

## Debug / offline tools

These run outside Docker on the host, in a Python environment with the
OSVI-AWDA torch stack (e.g. `conda activate osvi_awda`):

- `debug/test_osvi_awda_scene_offline.py` - runs the real checkpoint on a
  single saved image + human demo folder, asserts the fixed primitive
  schedule, and dumps `osvi_awda_raw_waypoints_t000.json` (all 15 predicted
  waypoints, the selected 5, projected base waypoints, gripper decisions and
  execution actions) plus overlay PNGs.
- `run_inference_waypoints_overlay.py` - same real-checkpoint forward pass,
  but reads its context from an already-saved rollout's
  `osvi_awda_context_NNN/context_raw_*.png` and its first frame from that
  rollout's `traj_XXX.pkl`, then draws the 5 predicted waypoints (labeled by
  primitive) on the observation frame. See its module docstring for usage.
- `plot_traj_waypoints.py` - plots the waypoints actually *executed* and
  recorded in a saved `traj_XXX.pkl` (3D + top-down views next to the
  frontal camera frame). Note this reflects one Trajectory sample per
  control-loop step, not one per individual AWDA waypoint - see the script's
  module docstring for the caveat.
