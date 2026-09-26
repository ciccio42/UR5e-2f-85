# ai_controller

ROS 2 (`ament_python`) package that runs a policy ("controller") in a closed
loop against the real UR5e + Robotiq 2F-85 setup: it pulls synchronized
camera frames (and optionally robot state) from ROS topics, feeds them to a
model, and sends the predicted end-effector pose + gripper command back to
the robot via the `moveit_controller` services.

This file documents the package layout and, in particular, **how to plug in
a new controller** (a new policy/model) without touching the control loop
itself.

For docker build/launch instructions (containers, camera driver, existing
OpenVLA setup) see [`docs/ai_controller.md`](../../docs/ai_controller.md) at
the repo root.

## Package layout

```
ai_controller/
├── ai_controller_node.py       # ROS2 node: control loop, robot I/O, controller dispatch
├── replicate_rollout.py        # Replays a saved rollout .pkl back through the robot
├── utils/
│   ├── ai_controller.py        # AIController abstract base class (the contract every controller implements)
│   └── utils.py                # Shared math helpers + robot-state field name constants
└── models/
    ├── cod_controller/         # Conditioned-Object-Detector controller (imitation learning, demo-conditioned)
    ├── openvla_controller/     # OpenVLA-OFT (VLA) controller
    ├── requirements/           # Per-model heavy-dependency install scripts
    └── <your_new_controller>/  # <- what you are about to add
```

Each subfolder under `models/` is self-contained: its own model code,
config file(s), and any vendored upstream repo it depends on (e.g.
`openvla_controller/openvla-oft/`). `ai_controller_node.py` never imports a
concrete model directly at module load time — it lazily imports whichever
one is selected by the `ai_controller_target` ROS parameter.

## The `AIController` contract

Every controller subclasses [`utils/ai_controller.py`](utils/ai_controller.py)'s
`AIController` (ABC) and implements:

| Method | Called from | Responsibility |
|---|---|---|
| `load_model(model_config)` | `AIController.__init__` | Parse the config, build the model, load weights, return the model object (assigned to `self.model`). |
| `move_model_to_device(device)` | your `__init__`, after `super().__init__()` | Move `self.model` (and any submodules) to CPU/GPU. |
| `reset()` | start of every new trajectory (`step == 0` in the control loop) | Clear any per-episode state (e.g. `gripper_closed`). |
| `load_command(demo_path, task_id, **kwargs)` | once per trajectory, right after `reset()` | Load whatever "task command" your model conditions on — demo frames, a language prompt, etc. |
| `pre_process(input_data)` | inside `inference()` | Turn raw `[images, states]` into whatever tensor/dict shape the model needs. |
| `inference(input_data, t, save_path=None)` | every control-loop step | Run the forward pass; return the **action(s)** to execute (see format below). |
| `post_process(output_data)` | inside `inference()` | Turn raw model output into the action format the node expects. |

`AIController.__init__(self, model_config: str)` already stores
`self.model_config_path` and calls `self.load_model()` for you — call
`super().__init__(model_config)` from your `__init__` and do device
placement / seeding / extra setup afterward (see both existing controllers).

### `input_data` format (what `inference()` receives)

`ai_controller_node.control_loop()` calls:

```python
out = self.controller.inference(input_data=[images, states], t=step, save_path=step_save_path)
```

- `images`: list of `numpy` `uint8` RGB arrays `(H, W, 3)`, one per topic in
  the `camera_topic` ROS parameter, in that same order (default:
  `[front, left, right, gripper]`).
- `states`: `None`, unless your controller needs proprioception. If it does,
  add a branch in `ai_controller_node.py` (see `_build_openvla_state` for an
  example) that builds the state vector your controller expects from the
  generic robot-state dict returned by `_capture_robot_state()` (keys:
  `eef_pos`, `eef_quat`, `joint_pos`, `joint_vel`, `gripper_qpos`,
  `gripper_qvel` — see `utils/utils.py`).

### Action format (what `inference()` must return)

The control loop branches on `self.ai_controller_target` to interpret the
return value of `inference()`, so a new controller needs a matching branch.
Two conventions already exist — pick whichever is more natural for your
model, or add a new one:

- **`cod_controller` convention** — `inference()` returns a *single* action
  `[x, y, z, qx, qy, qz, qw, gripper]` (world-frame position + quaternion +
  gripper position in `{0, 255}`). The node wraps it as `actions = [pred_action]`.
- **`openvla_controller` convention** — `inference()` returns a *list* of
  open-loop actions, each `[x, y, z, roll, pitch, yaw, gripper]`; the node
  converts roll/pitch/yaw to a quaternion for every action before sending it
  (`_euler2quat`).

Whatever convention you pick, each action sent to the robot must end up as
`[x, y, z, qx, qy, qz, qw, gripper]` by the time it reaches the
`for indx, action in enumerate(actions):` loop in `control_loop()`.

## Adding a new controller: step by step

1. **Create the model folder**

   ```bash
   mkdir -p ai_controller/ai_controller/models/my_controller
   touch ai_controller/ai_controller/models/my_controller/__init__.py
   ```

   If your model needs a vendored upstream repo or a heavy/awkward install
   (extra CUDA wheels, a forked training repo, etc.), mirror the
   `openvla_controller` pattern: vendor the repo under this folder and add an
   install script under `models/requirements/` (see
   `models/requirements/openvla_oft_installation.sh`), then document the
   install commands in `docs/ai_controller.md`.

2. **Add a config file** (optional but recommended)

   A YAML file (see `models/openvla_controller/openvla_config.yaml`) keeps
   model/runtime knobs out of code and out of ROS parameters. Load it in
   `load_model()`. The path to this file is passed in via the
   `model_config_path` ROS parameter.

3. **Implement the controller**

   `ai_controller/ai_controller/models/my_controller/my_controller.py`:

   ```python
   from ai_controller.utils.ai_controller import AIController


   class MyController(AIController):
       def __init__(self, model_config: str, task_name: str = "pick_place"):
           self.task_name = task_name
           super().__init__(model_config)          # calls self.load_model(model_config)

           self.device = ...                        # e.g. torch.device("cuda" if torch.cuda.is_available() else "cpu")
           self.move_model_to_device(self.device)

       def load_model(self, model_config: str):
           # parse config, build model, load weights
           # return the model object
           ...

       def move_model_to_device(self, device):
           self.model.to(device)

       def load_command(self, demo_path: str, task_id: str, **kwargs):
           # load demo frames / language prompt / whatever your model conditions on
           ...

       def reset(self):
           # clear per-episode state
           ...

       def pre_process(self, input_data):
           images, states = input_data[0], input_data[1]
           ...
           return processed_input

       def post_process(self, output_data):
           # convert raw model output -> action format (see "Action format" above)
           ...

       def inference(self, input_data, t: int, save_path=None):
           processed = self.pre_process(input_data)
           raw_output = self.model(processed)        # or self.model.get_action(...), etc.
           return self.post_process(raw_output)
   ```

4. **Register it in `ai_controller_node.py`**

   Add a branch in `AIControllerNode.__init__` next to the existing
   `cod_controller` / `openvla_controller` ones:

   ```python
   elif self.ai_controller_target == 'my_controller':
       from ai_controller.models.my_controller.my_controller import MyController
       self.controller = MyController(self.model_config_path, self.task_name)
   ```

   If your controller needs proprioception, add a matching branch where
   `states` is built (next to `_build_openvla_state`'s call site), and if it
   returns actions in a shape other than the two already handled, add a
   branch next to the `pred_action, predicted_bb, ... = out` /
   `actions = out` logic in `control_loop()` to normalize it to
   `[x, y, z, qx, qy, qz, qw, gripper]`.

5. **Update `setup.py` / dependencies**

   - Model-specific Python deps: install them inside the running container
     (see `docs/ai_controller.md`'s "OpenVLA Dependencies" section for the
     pattern) rather than adding them to `ai_controller/setup.py`'s
     `install_requires` — this package is built with `ament_python` and
     heavy ML deps (torch, etc.) are installed separately in the image.
   - No new console-script entry point is needed — every controller is
     reached through the existing `ai_controller_node` entry point via the
     `ai_controller_target` parameter.

6. **(Recommended) Add a standalone smoke test**

   Both existing controllers can be exercised outside ROS to catch loading /
   shape errors quickly:
   - `cod_controller.py` has a `if __name__ == "__main__":` block.
   - `openvla_controller/test.py` loads the controller directly and runs one
     `inference()` call on a saved test image.

   Add an equivalent `test.py` (or `__main__` block) for your controller
   before wiring it into the live robot loop.

7. **Run it**

   ```bash
   docker exec -it ur_robotiq_teleoperation_container bash
   source install/setup.bash
   colcon build --packages-select ai_controller && source install/setup.bash

   ros2 run ai_controller ai_controller_node --ros-args \
       -p ai_controller_target:="my_controller" \
       -p model_config_path:="/home/ros2_ws/src/ai_controller/ai_controller/models/my_controller/my_config.yaml" \
       -p task_name:="pick_place" \
       -p move_robot:=False   # keep False until you've verified predicted actions look sane
   ```

   Start with `move_robot:=False` and `debug_mode:=True` (+
   `debug_trajectory_path:=<path to a saved .pkl>`) to replay recorded
   camera frames through your controller without touching the real robot or
   requiring live camera topics — check the logged `Computed Action`
   values before ever setting `move_robot:=True`.

## Reference implementations

- [`models/cod_controller/cod_controller.py`](models/cod_controller/cod_controller.py) —
  demo-conditioned imitation-learning controller (Hydra/OmegaConf config,
  loads model weights from a checkpoint folder, outputs a single
  world-frame action + predicted bounding box).
- [`models/openvla_controller/openvla_controller.py`](models/openvla_controller/openvla_controller.py) —
  language-conditioned VLA controller (dataclass config loaded from YAML,
  outputs an open-loop chunk of delta-actions integrated against the
  current end-effector pose).
- [`models/tinyvla_controller/tinyvla_controller.py`](models/tinyvla_controller/tinyvla_controller.py) —
  language-conditioned TinyVLA (Llava-Pythia) controller, ported from
  VLA-Bench's validated test-time inference path. Unlike OpenVLA's open-loop
  chunk execution, it re-queries the model every control-loop step and relies
  on ACT-style temporal aggregation (see `models/tinyvla_controller/tinyvla.py`),
  so `inference()` always returns a single action.
