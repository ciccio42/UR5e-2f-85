# TinyVLA-Controller

`tinyvla_controller` (`ai_controller/ai_controller/models/tinyvla_controller/`)
wraps [TinyVLA](https://github.com/ciccio42/TinyVLA) (a LoRA-fine-tuned
Llava-Pythia VLM with an ACT/diffusion action head) for closed-loop control.
Reference (validated) inference code this controller is ported from lives at
`~/Desktop/Multi-Task-LFD/repo/VLA-Bench/robosuite_test/models/tinyvla.py`; the
original TinyVLA training repo is at `~/Desktop/Multi-Task-LFD/repo/TinyVLA`.

This assumes the Docker/container setup and UR-driver + `moveit_controller`
launch described in [AI-Controller](../ai_controller.md) are already running.

## Installation

Install the following inside the container (`docker exec -it
ur_robotiq_teleoperation_container bash`).

### Required
```bash
cd /home/ros2_ws/src/ai_controller/ai_controller/models/tinyvla_controller
git clone https://github.com/ciccio42/TinyVLA.git

# llava_pythia (model/tokenizer/image-processor code) and policy_heads (action
# head implementations: act / droid_diffusion / transformer_diffusion)
cd TinyVLA/llava-pythia && pip install -e . --break-system-packages
cd ../policy_heads && pip install -e . --break-system-packages

pip uninstall torch torchvision --break-system-packages
pip install "torch==2.7.0" "torchvision==0.22.0" --index-url https://download.pytorch.org/whl/cu128 --break-system-packages
pip install ipython --break-system-packages --ignore-installed psutil
pip install "diffusers==0.39.0" --break-system-packages
pip uninstall flash-attn -y --break-system-packages

export PYTHONPATH=$PYTHONPATH:/home/ros2_ws/src/ai_controller/ai_controller/models/tinyvla_controller/TinyVLA
export PYTHONPATH=$PYTHONPATH:/home/ros2_ws/src/ai_controller/ai_controller/models/tinyvla_controller/TinyVLA/llava-pythia

# unit test
cd /home/ros2_ws/src/ai_controller/ai_controller/models/tinyvla_controller
python3 test.py \
        --config /home/ros2_ws/src/ai_controller/ai_controller/models/tinyvla_controller/tinyvla_config.yaml
```

### Checkpoint layout

`model_path` / `model_base` in `tinyvla_config.yaml` expect: a LoRA (or
merged) checkpoint directory, its Llava-Pythia-1.3B base, and a
`dataset_stats.pkl` (qpos/action normalization stats) one level above
`model_path` - see the comments in `tinyvla_config.yaml` and
`TinyVLAPolicy.__init__` in `tinyvla.py`.

To turn a raw DeepSpeed training run into that layout (extracting
`non_lora_trainables.bin` from the ZeRO shards and dropping the multi-GB
`global_step*/` directories), use
`TinyVLA/scripts/postprocess_checkpoint.sh <source_dir> <target_dir> [min_step]`.
`source_dir` must already contain `config.json` and `dataset_stats.pkl`
alongside the `checkpoint-*` folders (copy them in from an already
post-processed reference run if the training run doesn't have them).

## Running

```bash
docker exec -it ur_robotiq_teleoperation_container bash
source install/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/ros2_ws/src/ai_controller/ai_controller/models/tinyvla_controller/TinyVLA
export PYTHONPATH=$PYTHONPATH:/home/ros2_ws/src/ai_controller/ai_controller/models/tinyvla_controller/TinyVLA/llava-pythia

ros2 run ai_controller ai_controller_node --ros-args \
  -p move_robot:=True \
  -p ai_controller_target:=tinyvla_controller \
  -p model_config_path:=/home/ros2_ws/src/ai_controller/ai_controller/models/tinyvla_controller/tinyvla_config.yaml \
  -p task_name:=pick_place
```
